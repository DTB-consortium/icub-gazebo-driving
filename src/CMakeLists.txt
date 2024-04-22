find_package(YARP REQUIRED COMPONENTS os dev sig math cv rosmsg)
#find_package(YARP 3.5.1 REQUIRED)
find_package(ICUB REQUIRED COMPONENTS iKin)
find_package(PkgConfig REQUIRED)
pkg_check_modules(glog REQUIRED libglog)

pkg_check_modules(SDF sdformat9)
include_directories(${SDF_INCLUDE_DIRS})
link_directories(${SDF_LIBRARY_DIRS})
message(STATUS "SDF_INCLUDE_DIRS: ${SDF_INCLUDE_DIRS}")

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(GAZEBO REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
message(STATUS "GAZEBO_INCLUDE_DIRS: ${GAZEBO_INCLUDE_DIRS}")
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(${YARP_LIBRARIES})
if (YARP_FOUND)
    message(STATUS "YARP libraries: ${YARP_LIBRARIES}")
else()
    message(FATAL_ERROR "YARP not found")
endif()

# Get around conflicts between VTK and Gazebo in Ubuntu 2022.04
set(CMAKE_MODULE_PATH_BAK ${CMAKE_MODULE_PATH})
find_package(Gazebo REQUIRED)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH_BAK})

find_package(VTK REQUIRED)
message (STATUS "VTK_VERSION: ${VTK_VERSION}")
if (VTK_VERSION VERSION_LESS "8.90.0")
  include(${VTK_USE_FILE})
endif()

find_package(OpenCV REQUIRED)

yarp_add_idl(IDL_GEN_FILES ${CMAKE_CURRENT_SOURCE_DIR}/rpc.thrift)

add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp ${IDL_GEN_FILES} ${glog_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES)
target_link_libraries(${PROJECT_NAME} PRIVATE ${YARP_LIBRARIES} ICUB::iKin ${VTK_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${glog_LIBRARIES} ${ROS_LIBRARIES} ${SDF_LIBRARIES} ${GAZEBO_LIBRARIES})

if (VTK_VERSION VERSION_GREATER_EQUAL "8.90.0")
  vtk_module_autoinit(TARGETS ${PROJECT_NAME} MODULES ${VTK_LIBRARIES})
endif()

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

set(PLUGIN_MOVER ${PROJECT_NAME}-model-mover)
add_library(${PLUGIN_MOVER} SHARED model_mover.cpp)
target_include_directories(${PLUGIN_MOVER} PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(${PLUGIN_MOVER} PRIVATE ${GAZEBO_LIBRARIES} YARP::YARP_os)
install(TARGETS ${PLUGIN_MOVER} LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib)

set(PLUGIN_READ ${PROJECT_NAME}-read-joint)
add_library(${PLUGIN_READ} SHARED read_joint_port.cpp)
target_include_directories(${PLUGIN_READ} PRIVATE ${GAZEBO_INCLUDE_DIRS} ${glog_INCLUDE_DIRS})
target_link_libraries(${PLUGIN_READ} PRIVATE ${GAZEBO_LIBRARIES} YARP::YARP_os ${glog_LIBRARIES})
install(TARGETS ${PLUGIN_READ} LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib)
