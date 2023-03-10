cmake_minimum_required(VERSION 3.0.2)
project(l3cam_ros)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  message_generation
  dynamic_reconfigure
  sensor_msgs
  cv_bridge
)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Sensor.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetSensorsAvaliable.srv
  GetVersion.srv
  Initialize.srv
  Terminate.srv
  FindDevices.srv
  GetLocalServerAddress.srv
  GetDeviceStatus.srv
  GetNetworkConfiguration.srv
  ChangeNetworkConfiguration.srv
  PowerOffDevice.srv
  StartDevice.srv
  StopDevice.srv
  StartStream.srv
  StopStream.srv
  ChangePointcloudColor.srv
  ChangePointcloudColorRange.srv
  ChangeDistanceRange.srv
  SetPolarimetricCameraDefaultSettings.srv
  ChangePolarimetricCameraBrightness.srv
  ChangePolarimetricCameraBlackLevel.srv
  EnablePolarimetricCameraAutoGain.srv
  ChangePolarimetricCameraAutoGainRange.srv
  ChangePolarimetricCameraGain.srv
  EnablePolarimetricCameraAutoExposureTime.srv
  ChangePolarimetricCameraAutoExposureTimeRange.srv
  ChangePolarimetricCameraExposureTime.srv
  SetRgbCameraDefaultSettings.srv
  ChangeRgbCameraBrightness.srv
  ChangeRgbCameraContrast.srv
  ChangeRgbCameraSaturation.srv
  ChangeRgbCameraSharpness.srv
  ChangeRgbCameraGamma.srv
  ChangeRgbCameraGain.srv
  EnableRgbCameraAutoWhiteBalance.srv
  ChangeRgbCameraWhiteBalance.srv
  EnableRgbCameraAutoExposureTime.srv
  ChangeRgbCameraExposureTime.srv
  ChangeRgbCameraResolution.srv
  ChangeRgbCameraFrameRate.srv
  ChangeThermalCameraColormap.srv
  ChangeThermalCameraTemperatureFilter.srv
  EnableThermalCameraTemperatureFilter.srv
  ChangeAlliedCameraBlackLevel.srv
  ChangeAlliedCameraExposureTime.srv
  EnableAlliedCameraAutoExposureTime.srv
  ChangeAlliedCameraAutoExposureTimeRange.srv
  ChangeAlliedCameraGain.srv
  EnableAlliedCameraAutoGain.srv
  ChangeAlliedCameraAutoGainRange.srv
  ChangeAlliedCameraGamma.srv
  ChangeAlliedCameraSaturation.srv
  ChangeAlliedCameraSharpness.srv
  ChangeAlliedCameraHue.srv
  ChangeAlliedCameraIntensityAutoPrecedence.srv
  EnableAlliedCameraAutoWhiteBalance.srv
  ChangeAlliedCameraBalanceRatioSelector.srv
  ChangeAlliedCameraBalanceRatio.srv
  ChangeAlliedCameraBalanceWhiteAutoRate.srv
  ChangeAlliedCameraBalanceWhiteAutoTolerance.srv
  ChangeAlliedCameraAutoModeRegion.srv
  ChangeAlliedCameraIntensityControllerRegion.srv
  ChangeAlliedCameraIntensityControllerTarget.srv
  ChangeAlliedCameraMaxDriverBuffersCount.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/General.cfg
  cfg/Pointcloud.cfg
  cfg/PolarimetricCamera.cfg
  cfg/RgbCamera.cfg
  cfg/ThermalCamera.cfg
  cfg/AlliedCamera.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

include_directories(
  include/${PROJECT_NAME}
  /usr/local/lib/libl3cam
  /usr/local/include/libl3cam
  ${catkin_INCLUDE_DIRS}
)

## Declare C++ executables
add_executable(${PROJECT_NAME}_node src/l3cam_ros_node.cpp)
add_executable(general_configuration src/general_configuration.cpp)
add_executable(pointcloud_configuration src/pointcloud_configuration.cpp)
add_executable(polarimetric_camera_configuration src/polarimetric_camera_configuration.cpp)
add_executable(rgb_camera_configuration src/rgb_camera_configuration.cpp)
add_executable(thermal_camera_configuration src/thermal_camera_configuration.cpp)
add_executable(allied_camera_configuration src/allied_camera_configuration.cpp)
add_executable(pointcloud_stream src/pointcloud_stream.cpp)
add_executable(polarimetric_stream src/polarimetric_stream.cpp)
add_executable(rgb_stream src/rgb_stream.cpp)
add_executable(thermal_stream src/thermal_stream.cpp)

## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
add_dependencies(general_configuration ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg) 
add_dependencies(pointcloud_configuration ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg) 
add_dependencies(polarimetric_camera_configuration ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg) 
add_dependencies(rgb_camera_configuration ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg) 
add_dependencies(thermal_camera_configuration ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg) 
add_dependencies(allied_camera_configuration ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg) 
add_dependencies(pointcloud_stream ${PROJECT_NAME}_gencpp) 
add_dependencies(polarimetric_stream ${PROJECT_NAME}_gencpp) 
add_dependencies(rgb_stream ${PROJECT_NAME}_gencpp) 
add_dependencies(thermal_stream ${PROJECT_NAME}_gencpp) 

## Specify libraries to link a library or executable target against
FIND_LIBRARY(l3cam_LIBRARIES libL3Cam.a /usr/local/lib/libl3cam/)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${l3cam_LIBRARIES})
target_link_libraries(general_configuration ${catkin_LIBRARIES} ${l3cam_LIBRARIES})
target_link_libraries(pointcloud_configuration ${catkin_LIBRARIES} ${l3cam_LIBRARIES})
target_link_libraries(polarimetric_camera_configuration ${catkin_LIBRARIES} ${l3cam_LIBRARIES})
target_link_libraries(rgb_camera_configuration ${catkin_LIBRARIES} ${l3cam_LIBRARIES})
target_link_libraries(thermal_camera_configuration ${catkin_LIBRARIES} ${l3cam_LIBRARIES})
target_link_libraries(allied_camera_configuration ${catkin_LIBRARIES} ${l3cam_LIBRARIES})
target_link_libraries(pointcloud_stream ${catkin_LIBRARIES} ${l3cam_LIBRARIES})
target_link_libraries(polarimetric_stream ${catkin_LIBRARIES} ${l3cam_LIBRARIES})
target_link_libraries(rgb_stream ${catkin_LIBRARIES} ${l3cam_LIBRARIES})
target_link_libraries(thermal_stream ${catkin_LIBRARIES} ${l3cam_LIBRARIES})
