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
  image_transport
  point_cloud_transport
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
  ChangeAlliedCameraAutoExposureTimeRange.srv
  ChangeAlliedCameraAutoGainRange.srv
  ChangeAlliedCameraBalanceRatioSelector.srv
  ChangeAlliedCameraBalanceRatio.srv
  ChangeAlliedCameraBalanceWhiteAutoRate.srv
  ChangeAlliedCameraBalanceWhiteAutoTolerance.srv
  ChangeAlliedCameraExposureTime.srv
  ChangeAlliedCameraGain.srv
  ChangeAlliedCameraGamma.srv
  ChangeAlliedCameraHue.srv
  ChangeAlliedCameraIntensityAutoPrecedence.srv
  ChangeAlliedCameraIntensityControllerRegion.srv
  ChangeAlliedCameraIntensityControllerTarget.srv
  ChangeAlliedCameraSaturation.srv
  ChangeAutobiasValue.srv
  ChangeBiasValue.srv
  ChangeDistanceRange.srv
  ChangeNetworkConfiguration.srv
  ChangePointcloudColorRange.srv
  ChangePointcloudColor.srv
  ChangePolarimetricCameraAutoExposureTimeRange.srv
  ChangePolarimetricCameraAutoGainRange.srv
  ChangePolarimetricCameraBlackLevel.srv
  ChangePolarimetricCameraBrightness.srv
  ChangePolarimetricCameraExposureTime.srv
  ChangePolarimetricCameraGain.srv
  ChangePolarimetricCameraProcessType.srv
  ChangeRgbCameraBrightness.srv
  ChangeRgbCameraContrast.srv
  ChangeRgbCameraExposureTime.srv
  ChangeRgbCameraGain.srv
  ChangeRgbCameraGamma.srv
  ChangeRgbCameraSaturation.srv
  ChangeRgbCameraSharpness.srv
  ChangeRgbCameraWhiteBalance.srv
  ChangeStreamingProtocol.srv
  ChangeThermalCameraColormap.srv
  ChangeThermalCameraProcessingPipeline.srv
  ChangeThermalCameraTemperatureFilter.srv
  EnableAlliedCameraAutoExposureTime.srv
  EnableAlliedCameraAutoGain.srv
  EnableAlliedCameraAutoWhiteBalance.srv
  EnableAutoBias.srv
  EnablePolarimetricCameraStreamProcessedImage.srv
  EnablePolarimetricCameraAutoExposureTime.srv
  EnablePolarimetricCameraAutoGain.srv
  EnableRgbCameraAutoExposureTime.srv
  EnableRgbCameraAutoWhiteBalance.srv
  EnableThermalCameraTemperatureDataUdp.srv
  EnableThermalCameraTemperatureFilter.srv
  FindDevices.srv
  GetAlliedCameraAutoExposureTimeRange.srv
  GetAlliedCameraAutoExposureTime.srv
  GetAlliedCameraAutoGainRange.srv
  GetAlliedCameraAutoGain.srv
  GetAlliedCameraAutoModeRegion.srv
  GetAlliedCameraAutoWhiteBalance.srv
  GetAlliedCameraBalanceRatioSelector.srv
  GetAlliedCameraBalanceRatio.srv
  GetAlliedCameraBalanceWhiteAutoRate.srv
  GetAlliedCameraBalanceWhiteAutoTolerance.srv
  GetAlliedCameraBlackLevel.srv
  GetAlliedCameraExposureTime.srv
  GetAlliedCameraGain.srv
  GetAlliedCameraGamma.srv
  GetAlliedCameraHue.srv
  GetAlliedCameraIntensityAutoPrecedence.srv
  GetAlliedCameraIntensityControllerRegion.srv
  GetAlliedCameraIntensityControllerTarget.srv
  GetAlliedCameraMaxDriverBuffersCount.srv
  GetAlliedCameraSaturation.srv
  GetAlliedCameraSharpness.srv
  GetAutobiasValue.srv
  GetDeviceInfo.srv
  GetDeviceStatus.srv
  GetDeviceTemperatures.srv
  GetLocalServerAddress.srv
  GetNetworkConfiguration.srv
  GetRtspPipeline.srv
  GetSensorsAvailable.srv
  GetVersion.srv
  Initialize.srv
  LibL3camStatus.srv
  PowerOffDevice.srv
  SensorDisconnected.srv
  SetBiasShortRange.srv
  SetPolarimetricCameraDefaultSettings.srv
  SetRgbCameraDefaultSettings.srv
  StartDevice.srv
  StartStream.srv
  StopDevice.srv
  StopStream.srv
  Terminate.srv
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
  cfg/Network.cfg
  cfg/Lidar.cfg
  cfg/Polarimetric.cfg
  cfg/Rgb.cfg
  cfg/Thermal.cfg
  cfg/AlliedWide.cfg
  cfg/AlliedNarrow.cfg
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

add_executable(network_configuration src/network_configuration.cpp)
add_executable(lidar_configuration src/lidar_configuration.cpp)
add_executable(polarimetric_configuration src/polarimetric_configuration.cpp)
add_executable(rgb_configuration src/rgb_configuration.cpp)
add_executable(thermal_configuration src/thermal_configuration.cpp)
add_executable(allied_wide_configuration src/allied_wide_configuration.cpp)
add_executable(allied_narrow_configuration src/allied_narrow_configuration.cpp)

add_executable(lidar_stream src/lidar_stream.cpp src/sensor_stream.cpp)
add_executable(polarimetric_wide_stream src/polarimetric_wide_stream.cpp src/sensor_stream.cpp)
add_executable(rgb_narrow_stream src/rgb_narrow_stream.cpp src/sensor_stream.cpp)
add_executable(thermal_stream src/thermal_stream.cpp src/sensor_stream.cpp)

add_executable(lidar_stream_comp src/lidar_stream_comp.cpp src/sensor_stream.cpp)
add_executable(polarimetric_wide_stream_comp src/polarimetric_wide_stream_comp.cpp src/sensor_stream.cpp)
add_executable(rgb_narrow_stream_comp src/rgb_narrow_stream_comp.cpp src/sensor_stream.cpp)
add_executable(thermal_stream_comp src/thermal_stream_comp.cpp src/sensor_stream.cpp)


## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)

add_dependencies(network_configuration ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg) 
add_dependencies(lidar_configuration ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg) 
add_dependencies(polarimetric_configuration ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg) 
add_dependencies(rgb_configuration ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg) 
add_dependencies(thermal_configuration ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg) 
add_dependencies(allied_wide_configuration ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg) 
add_dependencies(allied_narrow_configuration ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg) 

add_dependencies(lidar_stream ${PROJECT_NAME}_gencpp) 
add_dependencies(polarimetric_wide_stream ${PROJECT_NAME}_gencpp) 
add_dependencies(rgb_narrow_stream ${PROJECT_NAME}_gencpp) 
add_dependencies(thermal_stream ${PROJECT_NAME}_gencpp) 

add_dependencies(lidar_stream_comp ${PROJECT_NAME}_gencpp) 
add_dependencies(polarimetric_wide_stream_comp ${PROJECT_NAME}_gencpp) 
add_dependencies(rgb_narrow_stream_comp ${PROJECT_NAME}_gencpp) 
add_dependencies(thermal_stream_comp ${PROJECT_NAME}_gencpp) 


## Specify libraries to link a library or executable target against
FIND_LIBRARY(l3cam_LIBRARIES libL3Cam.a /usr/local/lib/libl3cam/)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${l3cam_LIBRARIES})

target_link_libraries(network_configuration ${catkin_LIBRARIES} ${l3cam_LIBRARIES})
target_link_libraries(lidar_configuration ${catkin_LIBRARIES} ${l3cam_LIBRARIES})
target_link_libraries(polarimetric_configuration ${catkin_LIBRARIES} ${l3cam_LIBRARIES})
target_link_libraries(rgb_configuration ${catkin_LIBRARIES} ${l3cam_LIBRARIES})
target_link_libraries(thermal_configuration ${catkin_LIBRARIES} ${l3cam_LIBRARIES})
target_link_libraries(allied_wide_configuration ${catkin_LIBRARIES} ${l3cam_LIBRARIES})
target_link_libraries(allied_narrow_configuration ${catkin_LIBRARIES} ${l3cam_LIBRARIES})

target_link_libraries(lidar_stream ${catkin_LIBRARIES} ${l3cam_LIBRARIES})
target_link_libraries(polarimetric_wide_stream ${catkin_LIBRARIES} ${l3cam_LIBRARIES})
target_link_libraries(rgb_narrow_stream ${catkin_LIBRARIES} ${l3cam_LIBRARIES})
target_link_libraries(thermal_stream ${catkin_LIBRARIES} ${l3cam_LIBRARIES})

target_link_libraries(lidar_stream_comp ${catkin_LIBRARIES} ${l3cam_LIBRARIES})
target_link_libraries(polarimetric_wide_stream_comp ${catkin_LIBRARIES} ${l3cam_LIBRARIES})
target_link_libraries(rgb_narrow_stream_comp ${catkin_LIBRARIES} ${l3cam_LIBRARIES})
target_link_libraries(thermal_stream_comp ${catkin_LIBRARIES} ${l3cam_LIBRARIES})
