cmake_minimum_required(VERSION 2.8.3)
project(dji_sdk)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  opencv_apps
  tf
)
find_package(DJIOSDK REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
catkin_python_setup()

add_message_files(
  FILES
  Gimbal.msg
  Waypoint.msg
  WaypointList.msg
  MobileData.msg
  MissionWaypointAction.msg
  MissionWaypoint.msg
  MissionWaypointTask.msg
  MissionHotpointTask.msg
)

add_service_files(
  FILES
  Activation.srv
  CameraAction.srv
  DroneTaskControl.srv
  SDKControlAuthority.srv
  SetLocalPosRef.srv
  MFIOConfig.srv
  MFIOSetValue.srv
  DroneArmControl.srv
  MissionStatus.srv
  MissionWpAction.srv
  MissionHpAction.srv
  MissionWpUpload.srv
  MissionWpSetSpeed.srv
  MissionWpGetSpeed.srv
  MissionWpGetInfo.srv
  MissionHpUpload.srv
  MissionHpGetInfo.srv
  MissionHpUpdateYawRate.srv
  MissionHpUpdateRadius.srv
  MissionHpResetYaw.srv
  SendMobileData.srv
  SetHardSync.srv
  QueryDroneVersion.srv
  Stereo240pSubscription.srv
  StereoVGASubscription.srv
  StereoDepthSubscription.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
        message_runtime
        geometry_msgs
        nav_msgs
        std_msgs
)

include_directories(
  include
  ${DJIOSDK_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(dji_sdk_node
  src/main.cpp
  src/modules/dji_sdk_node_hover.cpp
  src/modules/dji_sdk_node_services.cpp
  src/modules/dji_sdk_node.cpp
  src/modules/dji_sdk_node_mission_services.cpp
  src/modules/dji_sdk_node_subscriber.cpp
  src/modules/dji_sdk_node_publisher.cpp
  src/modules/dji_sdk_node_mobile_comm.cpp
  src/modules/dji_sdk_node_control.cpp
)

add_dependencies(dji_sdk_node
  dji_sdk_generate_messages_cpp)

target_link_libraries(dji_sdk_node
  ${catkin_LIBRARIES}
  ${DJIOSDK_LIBRARIES}
  pthread
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS dji_sdk_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

