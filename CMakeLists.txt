cmake_minimum_required(VERSION 3.12)
project(icub-gazebo-driving VERSION 1.0.0)

find_package(YARP 3.5.1 REQUIRED)
find_package(YCM REQUIRED)
find_package(ICUBcontrib REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(ICUBcontribOptions)
include(ICUBcontribHelpers)

set(ALLOW_IDL_GENERATION 1)

if(GLOG_FOUND)
    # 由于glog在连接时将头文件直接链接到了库里面，所以这里不用调用target_include_directories
    target_link_libraries(glogtest glog::glog)
else(GLOG_FOUND)
    message("GLOG library not found")
endif(GLOG_FOUND)

icubcontrib_set_default_prefix()
add_subdirectory(src)
add_subdirectory(gazebo)
add_subdirectory(scripts)
include(AddUninstallTarget)
