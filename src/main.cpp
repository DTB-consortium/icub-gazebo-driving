#include <cstdlib>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <cmath>
#include <limits>
#include <random>
#include <fstream>
#include <iomanip>

#include <glog/logging.h>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Value.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Property.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/PointCloud.h>
#include <yarp/math/Math.h>
#include <yarp/os/PeriodicThread.h>

#include "rpc_IDL.h"
#include "viewer.h"
#include "segmentation.h"
#include "cardinal_points_grasp.h"

#include <yarp/cv/Cv.h>
#include <opencv2/opencv.hpp>
#include <yarp/sig/ImageFile.h>

#include <pcl/point_types.h>
#include <yarp/os/Node.h>
#include <yarp/os/Publisher.h>
#include <yarp/os/Subscriber.h>
//##include <chrono> //sleep and time
#include <yarp/rosmsg/std_msgs/Float32.h>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>
#include <gazebo/gazebo.hh>

#define ICUBSIM_OPTION 1
#define RADIUS              0.135    // [m]
#define MAX_TORSO_PITCH     30.0    // [deg]

using namespace std;
using namespace google;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace viewer;
using namespace segmentation;
using namespace cardinal_points_grasp;

using yarp::os::Network;
using yarp::os::Node;
using yarp::os::Publisher;
using yarp::os::Subscriber;

yarp::dev::ICartesianControl *iarm_l, *iarm_r;
PolyDriver arm_r,arm_l;
vector<pair<yarp::sig::Vector, yarp::sig::Vector>> targetPos_r, targetPos_l;
double current_degree_r, current_degree_l;
double steer_joint_offset;

class MyThread : public yarp::os::PeriodicThread {
	yarp::os::BufferedPort<yarp::os::Bottle> port;
	double  control_steer;
	double  current_offset, target_offset, integral;
	double  output, pre_output;
	double  Kp, Ki, Kd, dt, past_steering;
	yarp::os::Subscriber<yarp::rosmsg::std_msgs::Float32> subscriber;
	yarp::os::Subscriber<yarp::rosmsg::std_msgs::Float32> subscriber2;
	yarp::os::Publisher<yarp::rosmsg::std_msgs::Float32> publisher;
	queue<double> err_buffer;
public:
    MyThread() : yarp::os::PeriodicThread(0.1)   /* Period */
    {
        // Constructor
		Kp = 0.075;
		Ki = 0.005;
		Kd = 0.01;
		dt = 0.02;
		target_offset = 0;
		integral = 0;
		steer_joint_offset = 0;
		LOG(INFO) << "MyThread default" << std::endl;
		port.open("/steer_joint_angle_receiver:i");

		if (!publisher.topic("/steer_cmd_main")) {
		   LOG(INFO) << "Failed to create publisher to /steer_cmd_main" << std::endl;
		   //return -1;
		} else {
			LOG(INFO) << "Succeed to publisher to /steer_cmd_main";
		}

		if (!subscriber2.topic("/offset")) {
			LOG(INFO) << "Failed to subscriber to /offset";
			// return -1;
		} else {
			LOG(INFO) << "Succeed to subscriber to /offset";
		}
		google::FlushLogFiles(INFO);
    }

	void publishTopic(double degree) {
		yarp::rosmsg::std_msgs::Float32 msg;
		msg.data = degree;
		/* publish it to the topic */
		publisher.write(msg);
	}

	void run() override
    {
        // Perform the task here
        // This will be executed every 100ms
    	static unsigned long long updateCount = 0; // 静态变量，只会初始化一次

        yarp::rosmsg::std_msgs::Float32 msg2;
        subscriber2.read(msg2);
        current_offset = msg2.data;
        LOG(INFO) << "updateCount: " << updateCount<< " offset Received From Carla:" << current_offset;

        output = current_offset;
        int degree = (output - pre_output) * 50;
        // int degree = output * 50;
        if (updateCount) {
			if (degree > 0) {
				turnright(degree);
			} else if (degree < 0) {
				turnleft(-degree);
			}
	        LOG(INFO) << "degree " << degree << " current_degree_r "<< current_degree_r;
        }

        pre_output = output;

        yarp::os::Bottle* b = port.read(false);
        if (b != nullptr) {
            // Extract the steer_joint angle from the message
            double steer_joint_angle = b->get(0).asFloat32();
            steer_joint_offset = steer_joint_angle;
            LOG(INFO) << "Received steer_joint angle: " << steer_joint_angle << std::endl;
            publishTopic(steer_joint_angle);
        }

    	google::FlushLogFiles(INFO);
        if (updateCount >= std::numeric_limits<unsigned long long>::max() - 10) {
            updateCount = 0;
        }
        updateCount++;
    }

    bool turnleft(std::int32_t delta) {

    	LOG(INFO) << "turnleft  delta: "<< delta << " current_degree_r: "<< current_degree_r;;

        ICartesianControl* iarm;
        arm_r.view(iarm);
		while (current_degree_r < 75 && delta > 0) {
			int idx = 75 + current_degree_r;
			auto& x = targetPos_r[idx].first;
			auto& o = targetPos_r[idx].second;
			iarm->goToPose(x, o);
			delta--;
			current_degree_r++;
			// Time::delay(0.3);
		}
    	return true;
    }

    bool turnright(std::int32_t delta) {

    	LOG(INFO) << "turnright  delta: "<< delta << " current_degree_r: "<< current_degree_r;;

        ICartesianControl* iarm;
        arm_r.view(iarm);
    	while (current_degree_r > -75 && delta > 0) {
    		int idx = 75 + current_degree_r;
    		auto& x = targetPos_r[idx].first;
    		auto& o = targetPos_r[idx].second;
    		iarm->goToPose(x, o);
    		delta--;
    		current_degree_r--;
    		// Time::delay(0.3);
    	}
		return true;
    }
};


/******************************************************************************/
class GrasperModule : public RFModule, public rpc_IDL {
    // PolyDriver arm_r,arm_l;
    PolyDriver hand_r,hand_l;
    PolyDriver gaze;

    // yarp::dev::ICartesianControl *iarm_l, *iarm_r;
    vector<pair<string, shared_ptr<BufferedPort<Bottle>>>> objMoverPorts;

    RpcServer rpcPort;
    BufferedPort<ImageOf<PixelRgb>> rgb_rPort;
    BufferedPort<ImageOf<PixelRgb>> rgb_lPort;
    BufferedPort<ImageOf<PixelFloat>> depthPort;
    RpcClient sqPort;

    bool is_grasp {false};
    int startup_context_id_l, startup_context_id_r;
    double deg_delta, degree = 0;
    // double current_degree_r, current_degree_l;
	yarp::sig::Vector xd_l, xd_r;
	yarp::sig::Vector od_l, od_r;
	yarp::sig::Vector pregrasp_pos_x, pregrasp_pos_o;
	// yarp::sig::Vector center;// {-0.6, 0, 0.12};
    static constexpr double wait_ping{.1};
    static constexpr double wait_tmo{3.};
    yarp::sig::Matrix Rx = yarp::math::zeros(3, 3);
    yarp::sig::Matrix Ry = yarp::math::zeros(3, 3);
    yarp::os::BufferedPort<yarp::os::Bottle> targetPort_l, targetPort_r;

    // vector<pair<yarp::sig::Vector, yarp::sig::Vector>> targetPos_r, targetPos_l;

	vector<int> fingers = {7, 8, 9, 10, 11, 12, 13, 14, 15};

    shared_ptr<yarp::sig::PointCloud<DataXYZRGBA>> pc_scene{nullptr};
    shared_ptr<yarp::sig::PointCloud<DataXYZRGBA>> pc_table{nullptr};
    shared_ptr<yarp::sig::PointCloud<DataXYZRGBA>> pc_object{nullptr};
    shared_ptr<yarp::sig::PointCloud<DataXYZRGBA>> pc_object_boundary{nullptr};

    Matrix Teye;
    double table_height{numeric_limits<double>::quiet_NaN()};
    Bottle sqParams;
    
    unique_ptr<Viewer> viewer;

    cv::Mat left_image, right_image, disparity, depth_image, depth_image_normalized;
    cv::Ptr<cv::StereoBM> stereo;// = cv::StereoBM::create(16, 15);  // numDisparities=16, blockSize=15
    double focal_length = 0.00125;  // 焦距，需要根据相机参数进行调整
    double baseline = 0.007;      // 基线长度，需要根据相机参数进行调整
/*
    left_image = cv::imread("left_image.png", cv::IMREAD_GRAYSCALE);
    right_image = cv::imread("right_image.png", cv::IMREAD_GRAYSCALE);
    stereo->compute(left_image, right_image, disparity);
    cv::Mat depth_image = focal_length * baseline / disparity;
    // 对深度图像进行归一化，使其取值范围在0-255之间
    cv::Mat depth_image_normalized;
    cv::normalize(depth_image, depth_image_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
*/
    /**************************************************************************/
    bool attach(RpcServer& source) override {
        return this->yarp().attachAsServer(source);
    }

    /**************************************************************************/
    auto savePCL(const string& filename, shared_ptr<yarp::sig::PointCloud<DataXYZRGBA>> pc) {
        ofstream fout(filename);
        if (fout.is_open()) {
            fout << "COFF" << endl;
            fout << pc->size() << " 0 0" << endl;
            for (size_t i = 0; i < pc->size(); i++) {
                const auto& p = (*pc)(i);
                fout << p.x << " " << p.y << " " << p.z << " "
                     << (int)p.r << " " << (int)p.g << " "
                     << (int)p.b << " " << (int)p.a << endl;
            }
            return true;
        } else {
            yError() << "Unable to write to" << filename;
            return false;
        }
    }

    bool savePLY(const std::string& filename, shared_ptr<yarp::sig::PointCloud<DataXYZRGBA>> pc) {
        std::ofstream fout(filename);
        if (fout.is_open()) {
            fout << "ply" << std::endl;
            fout << "format ascii 1.0" << std::endl;
            fout << "element vertex " << pc->size() << std::endl;
            fout << "property float x" << std::endl;
            fout << "property float y" << std::endl;
            fout << "property float z" << std::endl;
            fout << "end_header" << std::endl;

            for (size_t i = 0; i < pc->size(); i++) {
                const auto& p = (*pc)(i);
                fout << p.x << " " << p.y << " " << p.z << std::endl;
            }
            return true;
        } else {
            std::cerr << "Unable to write to " << filename << std::endl;
            return false;
        }
    }

    /**************************************************************************/
    auto helperArmDriverOpener(PolyDriver& arm, const Property& options) {
        const auto t0 = Time::now();
        while (Time::now() - t0 < 10.) {
            // this might fail if controller is not connected to solver yet
            if (arm.open(const_cast<Property&>(options))) {
                return true;
            }
            Time::delay(1.);
        }
        return false;
    }

    /**************************************************************************/
    bool lookAtDeveloper() {
        IGazeControl* igaze;
        gaze.view(igaze);
        igaze->lookAtAbsAnglesSync({60., 15., 5.});
        igaze->waitMotionDone();
        return true;
    }

    /**************************************************************************/
    bool shrug() {
        Vector x{-.1, .3, .15};
        Vector o{0., 0., 1., M_PI / 2.};
        vector<PolyDriver*> polys({&arm_r, &arm_l});
        ICartesianControl* iarm;
        for (size_t i = 0; i < polys.size(); i++) {
            polys[i]->view(iarm);
            iarm->setPosePriority("orientation");
            iarm->goToPoseSync(x, o);

            x[1] = -x[1];
            o = dcm2axis(axis2dcm(o) * axis2dcm({0., 1., 0., M_PI}));
        }

        // wait only for the last arm
        iarm->waitMotionDone(.1, 5.);
        return true;
    }

    /**************************************************************************/
    auto angle(const Vector& o1, const Vector& o2) const {
        auto R1 = axis2dcm(o1);
        auto R2 = axis2dcm(o2);
        auto Re = R1 * R2.transposed();

        return (180./M_PI) * acos((Re(1, 1) + Re(2, 2) + Re(3, 3) - 1.) / 2.);
    }

    /**************************************************************************/
    bool reach(ICartesianControl* iarm, const Vector& x, const Vector& o,
               const string& pose_tag) {
        iarm->goToPoseSync(x, o);
        // iarm->waitMotionDone(.1, 5.);
        iarm->waitMotionDone(.1, 3.);

        Vector _x, _o;
        iarm->getPose(_x, _o);
        LOG(INFO) << "Reached " << pose_tag << " position: " << _x.toString(3, 3) << "; error (m) = " << norm(x - _x);
        LOG(INFO) << "Reached " << pose_tag << " orientation: " << _o.toString(3, 3) << "; error (deg) = " << angle(o, _o);
        return true;
    }

    bool gotopose(ICartesianControl* iarm, const Vector& x, const Vector& o) {
        iarm->goToPose(x, o);
        //iarm->waitMotionDone(.1, 5.);
        return true;
    }

    /**************************************************************************/
    bool configure(ResourceFinder& rf) override {
        const string name = "icub-grasp";
        Property arm_r_options;
        arm_r_options.put("device", "cartesiancontrollerclient");
        arm_r_options.put("local", "/"+name+"/arm_r");
        arm_r_options.put("remote", "/icubSim/cartesianController/right_arm");
        if (!helperArmDriverOpener(arm_r, arm_r_options)) {
            yError() << "Unable to open right arm driver!";
            return false;
        }

        Property arm_l_options;
        arm_l_options.put("device", "cartesiancontrollerclient");
        arm_l_options.put("local", "/"+name+"/arm_l");
        arm_l_options.put("remote", "/icubSim/cartesianController/left_arm");
        if (!helperArmDriverOpener(arm_l, arm_l_options)) {
            yError() << "Unable to open left arm driver!";
            arm_r.close();
            return false;
        }

        Property hand_r_options;
        hand_r_options.put("device", "remote_controlboard");
        hand_r_options.put("local", "/"+name+"/hand_r");
        hand_r_options.put("remote", "/icubSim/right_arm");
        if (!hand_r.open(hand_r_options)) {
            yError() << "Unable to open right hand driver!";
            arm_r.close();
            arm_l.close();
            return false;
        }

        Property hand_l_options;
        hand_l_options.put("device", "remote_controlboard");
        hand_l_options.put("local", "/"+name+"/hand_l");
        hand_l_options.put("remote", "/icubSim/left_arm");
        if (!hand_l.open(hand_l_options)) {
            yError() << "Unable to open left hand driver!";
            arm_r.close();
            arm_l.close();
            hand_r.close();
            return false;
        }

        Property gaze_options;
        gaze_options.put("device", "gazecontrollerclient");
        gaze_options.put("local", "/"+name+"/gaze");
        gaze_options.put("remote", "/iKinGazeCtrl");
        if (!gaze.open(gaze_options)) {
            yError() << "Unable to open gaze driver!";
            arm_r.close();
            arm_l.close();
            hand_r.close();
            hand_l.close();
            return false;
        }

        deg_delta = 1.0;
        targetPort_l.open("/wheel/arm-l");
        targetPort_r.open("/wheel/arm-r");
        arm_l.view(iarm_l);
        arm_r.view(iarm_r);
        iarm_l->storeContext(&startup_context_id_l);
        iarm_r->storeContext(&startup_context_id_r);
        iarm_l->setTrajTime(1.0);
        iarm_r->setTrajTime(1.0);
        double tempd = -15.5;
	    Ry(0, 0) =  cos(tempd/180*M_PI); Ry(0, 2) = sin(tempd/180*M_PI);
	    Ry(1, 1) = 1;
	    Ry(2, 0) =  sin(tempd/180*M_PI); Ry(2, 2) = cos(tempd/180*M_PI);
        xd_l.resize(3);        od_l.resize(4);
        xd_r.resize(3);        od_r.resize(4);
        // set up velocity of arms' movements
        {
            vector<PolyDriver*> polys({&arm_r, &arm_l});
            for (auto poly:polys) {
                ICartesianControl* iarm;
                poly->view(iarm);
                iarm->setTrajTime(2.);
            }
        }

        // enable position control of the fingers
        {
            vector<PolyDriver*> polys({&hand_r, &hand_l});
            for (auto poly:polys) {
                IControlMode* imod;
                poly->view(imod);
                imod->setControlModes(fingers.size(), fingers.data(), vector<int>(fingers.size(), VOCAB_CM_POSITION).data());
            }
        }

        vector<string> objects_names{"mustard_bottle", "g29"};
        for (const auto& object_name:objects_names) {
            objMoverPorts.push_back(make_pair(object_name, make_shared<BufferedPort<Bottle>>()));
            objMoverPorts.back().second->open("/"+name+"/"+object_name+"/mover:o");
        }

        rgb_rPort.open("/"+name+"/r_rgb:i");
        rgb_lPort.open("/"+name+"/rgb:i");
        depthPort.open("/"+name+"/depth:i");
        sqPort.open("/"+name+"/sq:rpc");
        rpcPort.open("/"+name+"/rpc");
        auto result = attach(rpcPort);
        LOG(INFO) << "attach(rpcPort) " << result;
        viewer = make_unique<Viewer>(10, 370, 350, 350);
        viewer->start();

        google::FlushLogFiles(INFO);

        return true;
    }

    /**************************************************************************/
    bool go(const string& random_pose) override {
        if (random_pose == "on") {
            if (!randomize()) {
                return false;
            }
        }
        if (home()) {
        	if (segment()) {
                if (fit()) {
                	yError() <<  "fit && sqParams";
                	yError() << sqParams.toString();
                	yError() << sqParams.get(0).asFloat64();
                	yError() << sqParams.get(1).asFloat64();
                	yError() << sqParams.get(2).asFloat64();
                	if (grasp()) {
                    	yError() <<  "grasp";
                    	is_grasp = true;
                        return true;
                    }
                }
            }
        }
        return false;
    }

    /**************************************************************************/
    bool randomize() override {
        random_device rnd_device;
        mt19937 mersenne_engine(rnd_device());
        uniform_real_distribution<double> dist_p(0., (double)(objMoverPorts.size() - 1));
        // const auto i = (size_t)round(dist_p(mersenne_engine));
        const auto i = 0;
        yError() << "i == " << i << " objMoverPorts.size() == " << objMoverPorts.size();
        auto port = objMoverPorts[i].second;
        if (port->getOutputCount() > 0) {
            uniform_real_distribution<double> dist_r(0., .05);
            uniform_real_distribution<double> dist_ang(90., 270.);
            uniform_real_distribution<double> dist_rot(-180., 180.);

            Bottle delta_pose;
            auto r = dist_r(mersenne_engine);
            auto ang = dist_ang(mersenne_engine) * (M_PI / 180.);
            delta_pose.addFloat64(-.35 + r * cos(ang));
            delta_pose.addFloat64(r * sin(ang));
            delta_pose.addFloat64(dist_rot(mersenne_engine) * (M_PI / 180.));
            port->prepare() = delta_pose;
            port->writeStrict();

            return true;
        }
        
        return false;
    }

    /**************************************************************************/
    bool home() override {
        // home gazing
        IGazeControl* igaze;
        gaze.view(igaze);
        igaze->lookAtAbsAnglesSync({0., -50., 10.});

        // home arms
        {
            Vector x{-.25, .3, .1};
            vector<PolyDriver*> polys({&arm_r, &arm_l});
            ICartesianControl* iarm;
            for (auto poly:polys) {
                poly->view(iarm);
                iarm->goToPositionSync(x);
                x[1] = -x[1];
            }
            // wait only for the last arm
            iarm->waitMotionDone();
        }

        igaze->waitMotionDone();
        return true;
    }

    /**************************************************************************/
    bool segment() override {
        // get image data
    	auto* rgb_rImage = rgb_rPort.read();
        auto* rgbImage = rgb_lPort.read();
        auto* depthImage = depthPort.read();
		// ImageOf<PixelFloat> depthImage;

        // cv::imwrite("depthImage.ppm", yarp::cv::toCvMat(*depthImage));
#if 0
        {
        	//cv::imwrite("depthImage.ppm", yarp::cv::toCvMat(*depthImage));

        	left_image = yarp::cv::toCvMat(*rgbImage);
			right_image = yarp::cv::toCvMat(*rgb_rImage);
			yError() << "left_image right_image toCvMat";

		    cv::Mat grayLImage, grayRImage;
		    cv::cvtColor(left_image, grayLImage, cv::COLOR_RGB2GRAY);
		    cv::cvtColor(right_image, grayRImage, cv::COLOR_RGB2GRAY);
		    cv::imwrite("left.png", grayLImage);
		    cv::imwrite("right.png", grayRImage);

		    yarp::sig::file::write(*rgbImage, "left_image.ppm");
			yarp::sig::file::write(*rgb_rImage, "right_image.ppm");

			stereo = cv::StereoBM::create(16, 15);
			stereo->compute(grayLImage, grayRImage, disparity);
			yError() << "compute disparity";

		    double minVal, maxVal;
		    cv::minMaxLoc(disparity, &minVal, &maxVal);
		    cv::Mat scaledDisparity;
		    disparity.convertTo(scaledDisparity, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
		    // 保存视差图
		    cv::imwrite("scaledDisparity.png", scaledDisparity);

			depth_image = focal_length * baseline / disparity;
			cv::normalize(depth_image, depth_image_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
			yError() << "depth_image depth_image_normalized";
#ifndef ICUBSIM_OPTION
			// depthImage = yarp::cv::fromCvMat<yarp::sig::PixelFloat>(depth_image_normalized);
#endif
			/*
			yarp::sig::ImageOf<yarp::sig::PixelRgb> yarpImage;
		    // yarp::cv::fromCvMat(depth_image_normalized, yarpImage);
		    cv::Mat cvImage2=depth_image_normalized.clone();
		    yarpImage=yarp::cv::fromCvMat<yarp::sig::PixelRgb>(cvImage2);
		    yarp::sig::file::write(yarpImage, "test-2.ppm");
		    */
        }
#endif

        if ((rgbImage == nullptr) || (depthImage == nullptr)) {
            yError() << "Unable to receive image data!";
            return false;
        }

        if ((rgbImage->width() != depthImage->width()) ||
            (rgbImage->height() != depthImage->height()) ) {
            yError() << "Received image data with wrong size!";
            return false;
        }

        const auto w = rgbImage->width();
        const auto h = rgbImage->height();
        LOG(INFO) << "w = " << w << " h = " << h;

        // get camera extrinsics
        IGazeControl* igaze;
        gaze.view(igaze);
        Vector cam_x, cam_o;
        igaze->getLeftEyePose(cam_x, cam_o);
        Teye = axis2dcm(cam_o);
        Teye.setSubcol(cam_x, 0, 3);

        // get camera intrinsics
        Bottle info;
        igaze->getInfo(info);
        const auto fov_h = info.find("camera_intrinsics_left").asList()->get(0).asFloat64();
        const auto view_angle = 2. * std::atan((w / 2.) / fov_h) * (180. / M_PI);

        // aggregate image data in the point cloud of the whole scene
        pc_scene = make_shared<yarp::sig::PointCloud<DataXYZRGBA>>();
        Vector x{0., 0., 0., 1.};
        for (int v = 0; v < h; v++) {
            for (int u = 0; u < w; u++) {
                const auto rgb = (*rgbImage)(u, v);
                const auto depth = (*depthImage)(u, v);
                
                if (depth > 0.F) {
                    x[0] = depth * (u - .5 * (w - 1)) / fov_h;
                    x[1] = depth * (v - .5 * (h - 1)) / fov_h;
                    x[2] = depth;
                    x = Teye * x;
                
                    pc_scene->push_back(DataXYZRGBA());
                    auto& p = (*pc_scene)(pc_scene->size() - 1);
                    p.x = (float)x[0];
                    p.y = (float)x[1];
                    p.z = (float)x[2];
                    p.r = rgb.r;
                    p.g = rgb.g;
                    p.b = rgb.b;
                    p.a = 255;
                }
            }
        }

        savePCL("/home/icub/workspace/icub-gazebo-grasping-sandbox/pc_scene.off", pc_scene);

        // segment out the table and the object
        pc_table = make_shared<yarp::sig::PointCloud<DataXYZRGBA>>();
        pc_object = make_shared<yarp::sig::PointCloud<DataXYZRGBA>>();
        table_height = Segmentation::RANSAC(pc_scene, pc_table, pc_object);
        if (isnan(table_height)) {
            yError() << "Segmentation failed! isnan(table_height)";
            return false;
        }

        //savePCL("/home/icub/workspace/icub-gazebo-grasping-sandbox/pc_table.off", pc_table);
        //savePCL("/home/icub/workspace/icub-gazebo-grasping-sandbox/pc_object_steer.off", pc_object);
        //savePLY("/home/icub/workspace/icub-gazebo-grasping-sandbox/pc_object_steer.ply", pc_object);

        // update viewer
        Vector cam_foc;
        igaze->get3DPointOnPlane(0, {w/2., h/2.}, {0., 0., 1., -table_height}, cam_foc);
        viewer->addTable({cam_foc[0], cam_foc[1], cam_foc[2]}, {0., 0., 1.});
        viewer->addObject(pc_object);
        viewer->addCamera({cam_x[0], cam_x[1], cam_x[2]}, {cam_foc[0], cam_foc[1], cam_foc[2]},
                          {0., 0., 1.}, view_angle);

        if (pc_object->size() > 0) {
            return true;
        } else {
            yError() << "Unable to segment any object!";
            return false;
        }
    }

    /**************************************************************************/
    bool fit() override {
        if (pc_object) {
            if (pc_object->size() > 0) {
                if (sqPort.getOutputCount() > 0) {
                    // offload object fitting to the external solver
                    const auto ret = sqPort.write(*pc_object, sqParams);
                    if (ret) {
                        viewer->addSuperquadric(sqParams);
                        LOG(INFO) << "Superquadric parameters: " << sqParams.toString();
                        yError() << "Superquadric parameters: " << sqParams.toString();
                        return true;
                    } else {
                        yError() << "Unable to fit the object!";
                        return false;
                    }
                } else {
                    yError() << "Not connected to the solver!";
                    return false;
                }
            }
        }
        
        yError() << "No object to fit!";
        return false;
    }

    /**************************************************************************/
    bool grasp() override {
        if (sqParams.size() == 0) {
            yError() << "No object to grasp!";
            return false;
        }

        viewer->focusOnSuperquadric();

        const Vector sqCenter{sqParams.get(0).asFloat64(),
                              sqParams.get(1).asFloat64(),
                              sqParams.get(2).asFloat64()};
        // center = sqCenter;
        LOG(INFO) << "Superquadric center: " << sqCenter.toString(3, 3);

        // keep gazing at the object
        IGazeControl* igaze;
        gaze.view(igaze);
        igaze->setTrackingMode(true);
        igaze->lookAtFixationPoint(sqCenter);

        // set up the hand pre-grasp configuration
        IControlLimits* ilim;
        hand_r.view(ilim);
        double pinkie_min, pinkie_max;
        ilim->getLimits(15, &pinkie_min, &pinkie_max);
        // const vector<double> pregrasp_fingers_posture{60., 80., 0., 0., 0., 0., 0., 0., pinkie_max};
        const vector<double> pregrasp_fingers_posture{45., 80., 0., 0., 0., 0., 0., 0., 0};

        // apply cardinal points grasp algorithm
        ICartesianControl* iarm;
        arm_r.view(iarm);
        auto grasper_r = make_shared<CardinalPointsGrasp>(CardinalPointsGrasp("right", pregrasp_fingers_posture));
        const auto candidates_r = grasper_r->getCandidates(sqParams, iarm);

        arm_l.view(iarm);
        auto grasper_l = make_shared<CardinalPointsGrasp>(CardinalPointsGrasp("left", pregrasp_fingers_posture));
        const auto candidates_l = grasper_l->getCandidates(sqParams, iarm);

        // aggregate right and left arms candidates
        auto candidates = candidates_r.first;
        candidates.insert(candidates.end(), candidates_l.first.begin(), candidates_l.first.end());
        std::sort(candidates.begin(), candidates.end(), CardinalPointsGrasp::compareCandidates);

        // some safety checks
        if (candidates.empty()) {
            yError() << "No good grasp candidates found! candidates.empty()";
            // lookAtDeveloper();
            shrug();
            return false;
        }

        // extract relevant info
        const auto& best = candidates[0];
        const auto& type = get<0>(best);
        const auto& T = get<2>(best);

        viewer->showCandidates(candidates);

        // select arm corresponding to the best candidate
        shared_ptr<CardinalPointsGrasp> grasper;
        IPositionControl* ihand;
        int context;
        if (type == "right") {
             grasper = grasper_r;
             hand_r.view(ihand);
             arm_r.view(iarm);
             context = candidates_r.second;
        } else {
             grasper = grasper_l;
             hand_l.view(ihand);
             arm_l.view(iarm);
             context = candidates_l.second;
        }
        LOG(INFO) << "Selected arm: \"" << type << "\"";

        // target pose that allows grasping the object
        const auto x = T.getCol(3).subVector(0, 2);
        const auto o = dcm2axis(T);

        LOG(INFO) << "Best grasp position: " << x.toString(3, 3);
        LOG(INFO) << "Best grasp orientation: " << o.toString(3, 3);
        LOG(INFO) << "T: " << T.toString(3, 3);

        // enable the context used by the algorithm
        iarm->stopControl();
        iarm->restoreContext(context);
        iarm->setInTargetTol(.001);

        // put the hand in the pre-grasp configuration
        ihand->setRefAccelerations(fingers.size(), fingers.data(), vector<double>(fingers.size(), numeric_limits<double>::infinity()).data());
        ihand->setRefSpeeds(fingers.size(), fingers.data(), vector<double>{60., 60., 60., 60., 60., 60., 60., 60., 200.}.data());
        ihand->positionMove(fingers.size(), fingers.data(), pregrasp_fingers_posture.data());
        auto done = false;
        while (!done) {
            Time::delay(1.);
            ihand->checkMotionDone(fingers.size(), fingers.data(), &done);
        };

        // reach for the pre-grasp pose
        const auto dir = x - sqCenter;
        // const auto pre_x = x + .06 * dir / norm(dir);
        const auto pre_x = x + .02 * dir / norm(dir);
        LOG(INFO) << "Pre-grasp position: " << pre_x.toString(3, 3);
        reach(iarm, pre_x, o, "pre-grasp");
        pregrasp_pos_x = pre_x;
        pregrasp_pos_o = o;

        // reach for the object
        reach(iarm, x, o, "grasp");

        // close fingers
        //ihand->positionMove(fingers.size(), fingers.data(), vector<double>{60., 80., 40., 35., 40., 35., 40., 35., pinkie_max}.data());
        ihand->positionMove(fingers.size(), fingers.data(), vector<double>{60., 80., 40., 35., 65., 50., 60., 50., 50}.data());

        // give enough time to adjust the contacts
        Time::delay(1.);

        /*
        // lift up the object
        const auto lift = x + Vector{0., 0., .1};
        igaze->lookAtFixationPoint(lift);
        iarm->goToPoseSync(lift, o);
        iarm->waitMotionDone(.1, 5.);

        lookAtDeveloper();
        */

		for (degree = -75; degree < 75; degree += 1.) {
    		// LOG(INFO) << "degree: "<<degree;
	        Rx = yarp::math::zeros(3, 3);
	        Rx(0, 0) = 1;
			Rx(1, 1) = cos(degree/180*M_PI); Rx(1, 2) = -sin(degree/180*M_PI);
			Rx(2, 1) = sin(degree/180*M_PI); Rx(2, 2) = cos(degree/180*M_PI);
			xd_r = Rx * x;

	        Rx = yarp::math::zeros(3, 3);
	        Rx(0, 0) = -1;
			Rx(1, 1) = sin(degree/180*M_PI); Rx(1, 2) = -cos(degree/180*M_PI);
			Rx(2, 1) = -cos(degree/180*M_PI); Rx(2, 2) = -sin(degree/180*M_PI);
			od_r = dcm2axis(Rx);

			targetPos_r.emplace_back(make_pair(xd_r, od_r));
			// degree = degree + deg_delta;
		}
        current_degree_r = 0.;
        // righthandmove(iarm, x, o, true, "is_clockwise");
	    google::FlushLogFiles(INFO);
        return true;
    }

    void limitTorsoPitch(ICartesianControl* iarm)
    {
        int axis = 0; // pitch joint
        double min, max;
        iarm->getLimits(axis,&min,&max);
        iarm->setLimits(axis,min,MAX_TORSO_PITCH);
    }

    bool release() override {
        ICartesianControl* iarm;
        arm_r.view(iarm);
        iarm->goToPoseSync(pregrasp_pos_x, pregrasp_pos_o);

        IPositionControl* ihand;
        int context;
		 // grasper = grasper_r;
		 hand_r.view(ihand);
		 // arm_r.view(iarm);
		 // context = candidates_r.second;
		 const vector<double> pregrasp_fingers_posture{0., 0., 0., 0., 0., 0., 0., 0., 0};
    	// put the hand in the pre-grasp configuration
        ihand->setRefAccelerations(fingers.size(), fingers.data(), vector<double>(fingers.size(), numeric_limits<double>::infinity()).data());
        ihand->setRefSpeeds(fingers.size(), fingers.data(), vector<double>{60., 60., 60., 60., 60., 60., 60., 60., 200.}.data());
        ihand->positionMove(fingers.size(), fingers.data(), pregrasp_fingers_posture.data());
        auto done = false;
        while (!done) {
            Time::delay(1.);
            ihand->checkMotionDone(fingers.size(), fingers.data(), &done);
        };
        return true;
    }

    bool turnleft(const std::int32_t delta) override {
    	int degree = delta;
    	LOG(INFO) << "turnleft  delta: "<< delta << " current_degree_r: "<< current_degree_r;;

        ICartesianControl* iarm;
        arm_r.view(iarm);
		while (current_degree_r < 75 && degree > 0) {
			int idx = 75 + current_degree_r;
			auto& x = targetPos_r[idx].first;
			auto& o = targetPos_r[idx].second;
			//LOG(INFO) << "xd_r: " << x.toString(3, 3);
			gotopose(iarm, x, o);
			degree--;
			current_degree_r++;
			Time::delay(0.3);
		}
		iarm->waitMotionDone(.1, 3);
		LOG(INFO) << " current_degree_r: "<< current_degree_r;
    	return true;
    }

    bool turnright(const std::int32_t delta) override {
    	int degree = delta;
    	LOG(INFO) << "turnright  degree: "<< degree << " current_degree_r: "<< current_degree_r;;

        ICartesianControl* iarm;
        arm_r.view(iarm);
    	while (current_degree_r > -75 && degree > 0) {
    		int idx = 75 + current_degree_r;
    		auto& x = targetPos_r[idx].first;
    		auto& o = targetPos_r[idx].second;
    		gotopose(iarm, x, o);
    		degree--;
    		current_degree_r--;
    		Time::delay(0.3);
    	}
    	iarm->waitMotionDone(.1, 3);
    	LOG(INFO) << " current_degree_r: "<< current_degree_r;
		return true;
    }

    /**************************************************************************/
    double getPeriod() override {
        return 1.0;
    }

    /**************************************************************************/
    bool updateModule() override {
        if (auto* b = targetPort_l.read(false)) {
            // const auto p = (b->get(0).asFloat64() * (y_max_l - y_min_l) + (y_max_l + y_min_l)) / 2.;
        	if (b->get(0).asString() == "clock") {
        		yError() << "targetPort_l clockwise";
        		// fingerOperation(1, "grasp");
        	} else if (b->get(0).asString() == "counterclockwise") {
        		yError() << "targetPort_l counterclockwise";
        		//handmove("counterclockwise", 20);
        		// fingerOperation(1, "release");
            } else {
        		const auto p = b->get(0).asFloat64();
        		// target_l[0] = std::max(std::min(p, y_max_l), y_min_l);
        	}
        }
        // reference_l->computeNextValues(target_l);

        if (auto* b = targetPort_r.read(false)) {
        	LOG(INFO) << b->get(0).asString();
        	if (b->get(0).asString() == "clockwise") {
        		LOG(INFO) << "targetPort_r clockwise";
        		//fingerOperation(2, "grasp");
        		//handmove("clockwise", 20);
        	} else if (b->get(0).asString() == "counterclockwise") {
        		LOG(INFO) << "targetPort_r counterclockwise";
        		//fingerOperation(2, "release");
        		//handmove("counterclockwise", 20);
            } else {
            	const auto p = b->get(0).asFloat64();
            	LOG(INFO) << "targetPort_r invalid parameter";
            }
        }
        google::FlushLogFiles(INFO);
        return true;
    }

    bool interruptModule() override {
        viewer->stop();
        // interrupt blocking read
        sqPort.interrupt();
        depthPort.interrupt();
        rgb_lPort.interrupt();
        rgb_rPort.interrupt();
        return true;
    }

    /**************************************************************************/
    bool close() override {
        // restore default contexts
        IGazeControl* igaze;
        gaze.view(igaze);
        igaze->stopControl();
        igaze->restoreContext(0);

        vector<PolyDriver*> polys({&arm_r, &arm_l});
        for (auto poly:polys) {
            ICartesianControl* iarm;
            poly->view(iarm);
            iarm->stopControl();
            iarm->restoreContext(0);
        }

        rpcPort.close();
        sqPort.close();
        depthPort.close();
        rgb_lPort.close();
        rgb_rPort.close();
        for (auto port:objMoverPorts) {
            port.second->close();
        }
        gaze.close();
        arm_r.close();
        arm_l.close();
        hand_r.close();
        hand_l.close();

        targetPort_l.close();
        targetPort_r.close();
        iarm_l->stopControl();
        iarm_r->stopControl();
        iarm_l->restoreContext(startup_context_id_l);
        iarm_r->restoreContext(startup_context_id_r);

        return true;
    }
};


/******************************************************************************/
int main(int argc, char *argv[]) {
	google::InitGoogleLogging(argv[0]);
	google::SetLogDestination(google::INFO, "/home/icub/workspace/icub-gazebo-grasping-sandbox/glog");

    Network yarp;
    /* creates a node called /yarp/talker */
	Node node("/yarp/maintalker");

	// Network yarp;
    if (!yarp.checkNetwork()) {
        yError() << "Unable to find YARP server!";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    // LOG(INFO) << "RosTopicThread";
    // RosTopicThread rosthread;
    // rosthread.start();

    LOG(INFO) << "MyThread";
    MyThread thread;
    thread.start();

    GrasperModule module;
    return module.runModule(rf);
}