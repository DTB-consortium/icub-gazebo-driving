/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include <functional>
#include <cmath>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/common/Events.hh>
#include <glog/logging.h>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>
#include <gazebo/gazebo.hh>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>

using namespace google;

namespace gazebo {
/******************************************************************************/
class ReadJoint : public gazebo::ModelPlugin
{
    gazebo::physics::ModelPtr model;
    gazebo::event::ConnectionPtr renderer_connection;
    yarp::os::BufferedPort<yarp::os::Bottle> port;

    /**************************************************************************/
    void onWorldFrame() {
    	static unsigned long long updateCount = 0;
		gazebo::physics::JointPtr joint = this->model->GetJoint("steer_joint");
		if (joint) {
			double degree = joint->Position(0) * 180 / M_PI;
			if (updateCount % 10 == 0) {
				LOG(INFO) << "onWorldFrame updateCount: "<< updateCount << " joint->Position(0): " << joint->Position(0) << " joint->GetForce(0): "<< joint->GetForce(0) << " degree: " << degree ;
			}
			yarp::os::Bottle& msg = port.prepare();
			msg.clear();
            msg.addFloat32(degree);
			port.writeStrict();
		}
        if (updateCount >= std::numeric_limits<unsigned long long>::max() - 10) {
            updateCount = 0;
        }
        updateCount++;
    }

public:
    /**************************************************************************/
    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr) override {
    	google::InitGoogleLogging("");
    	google::SetLogDestination(google::INFO, "/home/icub/workspace/icub-gazebo-grasping-sandbox/build/glog");

    	this->model = model;
        // Open YARP port for sending steer_joint angle
        port.open("/steer_joint_angle:o");

        // Connect onWorldFrame callback to Gazebo's WorldUpdateBegin event
        auto bind = std::bind(&ReadJoint::onWorldFrame, this);
        renderer_connection = gazebo::event::Events::ConnectWorldUpdateBegin(bind);
    	google::FlushLogFiles(INFO);
    }

    /**************************************************************************/
    virtual ~ReadJoint() {
        if (!port.isClosed()) {
            port.close();
        }
    }


    // double predegree;
    // if (updateCount && abs(degree - predegree) < 1e-2) {
    // 	msg.addFloat32(predegree);
    // } else {
    // 	msg.addFloat32(degree);
    // 	predegree = degree;
    // }


//    void OnUpdate() {
//    	static unsigned long long updateCount = 0; // 静态变量，只会初始化一次
//        updateCount++;
//        if (updateCount % 1000 == 0) {
//			gazebo::physics::JointPtr joint = this->model->GetJoint("steer_joint");
//			if (joint)
//			{
//				double degree = joint->Position(0) * 180 / M_PI; // 获取关节角度并转换为度数
//		    	LOG(INFO) << "updateCount: "<< updateCount << " joint->Position(0): " << joint->Position(0) << " M_PI: "<< M_PI << " degree: " << degree ;
//				yarp::rosmsg::std_msgs::Float32 msg;
//				msg.data = degree;
//				publisher.write(msg);
//			}
//        }
//        if (updateCount >= std::numeric_limits<unsigned long long>::max() - 10) {
//            // 计数器接近最大值时，重置为0
//            updateCount = 0;
//        }
//    }
};
}

GZ_REGISTER_MODEL_PLUGIN(gazebo::ReadJoint)
