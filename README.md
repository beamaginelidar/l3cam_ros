# l3cam_ros

This package is an ROS driver for the L3Cam camera manufactured by [Beamagine](https://beamagine.com/). The driver relies on the libL3Cam library provided by Beamagine.


## Installation

### Dependencies


### ROS Driver
`sudo apt install ros-noetic-l3cam-ros`

## ROS Nodes

### l3cam_ros_node
The l3cam_ros_node is the main driver that connects to the camera and configures it according to ROS parameters/services. See the service files (`srv/`) for documentation regarding the various parameters that can be used to configure the L3Cam.

### pointcloud_stream
The pointcloud_stream is the node that publishes pointcloud frames if the LiDAR sensor is avaliable.

### polarimetric_stream
The polarimetric_stream is the node that publishes polarimetric image frames if the polarimetric sensor is avaliable.

### rgb_stream
The rgb_stream is the node that publishes RGB image frames if the RGB sensor is avaliable.

### thermal_stream
The thermal_stream is the node that publishes thermal image frames if the thermal sensor is avaliable.

### general_configuration
The general_configuration is a node that configures the general parameters by using dynamic reconfigure and the services to communicate with the l3cam_ros_node. See the config file (`cfg/General.cfg`) for documentation regarding the various parameters that can be used to configure the general parameters of the L3Cam.

### pointcloud_configuration
The pointcloud_configuration is a node that configures the pointcloud parameters (if a LiDAR sensor is avaliable) by using dynamic reconfigure and the services to communicate with the l3cam_ros_node. See the config file (`cfg/Pointcloud.cfg`) for documentation regarding the various parameters that can be used to configure the pointcloud parameters of the L3Cam.

### polarimetric_camera_configuration
The polarimetric_camera_configuration is a node that configures the polarimetric camera parameters (if a polarimetric sensor is avaliable) by using dynamic reconfigure and the services to communicate with the l3cam_ros_node. See the config file (`cfg/PolarimetricCamera.cfg`) for documentation regarding the various parameters that can be used to configure the polarimetric camera parameters of the L3Cam.

### rgb_camera_configuration
The rgb_camera_configuration is a node that configures the RGB camera parameters (if a RGB sensor is avaliable) by using dynamic reconfigure and the services to communicate with the l3cam_ros_node. See the config file (`cfg/RgbCamera.cfg`) for documentation regarding the various parameters that can be used to configure the RGB camera parameters of the L3Cam.

### thermal_camera_configuration
The thermal_camera_configuration is a node that configures the thermal camera parameters (if a thermal sensor is avaliable) by using dynamic reconfigure and the services to communicate with the l3cam_ros_node. See the config file (`cfg/ThermalCamera.cfg`) for documentation regarding the various parameters that can be used to configure the thermal camera parameters of the L3Cam.
