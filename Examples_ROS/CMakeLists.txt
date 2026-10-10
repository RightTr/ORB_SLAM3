cmake_minimum_required(VERSION 3.10)
project(orb_slam3)

set(ROS_EDITION "ROS2" CACHE STRING "Build ROS1 or ROS2 wrapper")
set_property(CACHE ROS_EDITION PROPERTY STRINGS ROS1 ROS2)
if(NOT ROS_EDITION STREQUAL "ROS1" AND NOT ROS_EDITION STREQUAL "ROS2")
  message(FATAL_ERROR "ROS_EDITION must be ROS1 or ROS2")
endif()

set(ORB_SLAM3_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." CACHE PATH "Path to the native ORB-SLAM3 source tree")
get_filename_component(ORB_SLAM3_ROOT "${ORB_SLAM3_ROOT}" REALPATH)
if(NOT EXISTS "${ORB_SLAM3_ROOT}/include/System.h")
  message(FATAL_ERROR "ORB_SLAM3_ROOT does not contain include/System.h: ${ORB_SLAM3_ROOT}")
endif()

find_package(OpenCV 4.4 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_library(ORB_SLAM3_LIBRARY NAMES ORB_SLAM3 PATHS "${ORB_SLAM3_ROOT}/lib" NO_DEFAULT_PATH)
find_library(DBOW2_LIBRARY NAMES DBoW2 PATHS "${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib" NO_DEFAULT_PATH)
find_library(G2O_LIBRARY NAMES g2o PATHS "${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib" NO_DEFAULT_PATH)
if(NOT ORB_SLAM3_LIBRARY OR NOT DBOW2_LIBRARY OR NOT G2O_LIBRARY)
  message(FATAL_ERROR "Build the native ORB-SLAM3 library and Thirdparty libraries first (run ../build.sh)")
endif()

set(ORB_SLAM3_INCLUDE_DIRS
  "${ORB_SLAM3_ROOT}"
  "${ORB_SLAM3_ROOT}/include"
  "${ORB_SLAM3_ROOT}/include/CameraModels"
  "${ORB_SLAM3_ROOT}/Thirdparty/Sophus"
  "${ORB_SLAM3_ROOT}/Thirdparty/DBoW2"
  "${ORB_SLAM3_ROOT}/Thirdparty/g2o"
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
)
set(ORB_SLAM3_LINK_LIBRARIES
  "${ORB_SLAM3_LIBRARY}" "${DBOW2_LIBRARY}" "${G2O_LIBRARY}"
  ${OpenCV_LIBS} ${EIGEN3_LIBS} ${Pangolin_LIBRARIES} -lboost_serialization -lcrypto
)
set(WRAPPER_RPATH "${ORB_SLAM3_ROOT}/lib;${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib;${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib")

if(ROS_EDITION STREQUAL "ROS1")
  set(CMAKE_CXX_STANDARD 11)
  find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cv_bridge message_filters nav_msgs geometry_msgs std_msgs tf2_ros)
  catkin_package(CATKIN_DEPENDS roscpp sensor_msgs cv_bridge message_filters nav_msgs geometry_msgs std_msgs tf2_ros)

  foreach(node mono stereo rgbd mono_inertial stereo_inertial rgbd_inertial)
    add_executable(ros_${node} "${CMAKE_CURRENT_SOURCE_DIR}/src/${node}.cpp")
    target_include_directories(ros_${node} PRIVATE ${ORB_SLAM3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} "${CMAKE_CURRENT_SOURCE_DIR}/include")
    target_link_libraries(ros_${node} ${ORB_SLAM3_LINK_LIBRARIES} ${catkin_LIBRARIES})
    target_compile_definitions(ros_${node} PRIVATE USE_ROS1)
    set_target_properties(ros_${node} PROPERTIES INSTALL_RPATH "${WRAPPER_RPATH}" BUILD_RPATH "${WRAPPER_RPATH}")
    add_dependencies(ros_${node} ${catkin_EXPORTED_TARGETS})
    install(TARGETS ros_${node} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  endforeach()
elseif(ROS_EDITION STREQUAL "ROS2")
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(message_filters REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)

  foreach(node mono stereo rgbd mono_inertial stereo_inertial rgbd_inertial)
    add_executable(ros_${node} "${CMAKE_CURRENT_SOURCE_DIR}/src/${node}.cpp")
    target_compile_definitions(ros_${node} PRIVATE USE_ROS2)
    target_include_directories(ros_${node} PRIVATE ${ORB_SLAM3_INCLUDE_DIRS} "${CMAKE_CURRENT_SOURCE_DIR}/include")
    ament_target_dependencies(ros_${node} rclcpp sensor_msgs builtin_interfaces cv_bridge message_filters nav_msgs geometry_msgs std_msgs tf2_ros)
    target_link_libraries(ros_${node} ${ORB_SLAM3_LINK_LIBRARIES})
    set_target_properties(ros_${node} PROPERTIES INSTALL_RPATH "${WRAPPER_RPATH}" BUILD_RPATH "${WRAPPER_RPATH}")
    install(TARGETS ros_${node} DESTINATION lib/${PROJECT_NAME})
  endforeach()
  install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
  ament_package()
endif()
