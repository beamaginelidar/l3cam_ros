# l3cam_ros

This package is an ROS driver for the L3Cam device manufactured by [Beamagine](https://beamagine.com/). The driver relies on the library provided by Beamagine as part of the [L3Cam SDK](https://github.com/beamaginelidar/libl3cam.git). For more info on the L3Cam check the [L3Cam User Manual](https://github.com/beamaginelidar/libl3cam/blob/main/L3CAM%20User%20Manual.pdf).

This package is supported only on Linux systems and has only been tested with ROS noetic on an Ubuntu 20.04 system.

## Installation

### Dependencies

First, you will need to install the L3Cam SDK.

Download the required package from Beamagine's [L3Cam SDK](https://github.com/beamaginelidar/libl3cam.git) repository depending on your hardware architecture and install:

```
sudo dpkg -i <PACKAGE>
```

### ROS Driver

Once the SDK is successfully installed, you can continue with the ROS driver install:

```
sudo apt install ros-<ROS_DISTRO>-l3cam-ros
```

Or get the l3cam_ros package, put it in your catkin workspace and build.

## Operational Advice

### MTU Size

You will need to increase the MTU (Maximum Transmission Unit) on the network interface attached to the camera.

You can check what your current MTU setting is by running the following command:

```
ip a | grep mtu
```

You should increase the MTU to `9000` to allow jumbo frames. If you use Network Manager, this can be done by opening the network interface settings and editing the "MTU" box under the "Identity" tab.

See the "Linux host configuration" section of the [L3Cam User Manual](https://github.com/beamaginelidar/libl3cam/blob/main/L3CAM%20User%20Manual.pdf) for full details.

### Receive Buffer Size

It is also recommended to increase your network default and maximum receive buffer size.

You can check what your current buffer size is:

```
sudo sysctl 'net.core.rmem_max'
sudo sysctl 'net.core.rmem_default'
```

Update the buffer size with the following commands:

```
sudo sh -c "echo 'net.core.rmem_default=268435456' >> /etc/sysctl.conf"
sudo sh -c "echo 'net.core.rmem_max=268435456' >> /etc/sysctl.conf"
sudo sysctl -p
```

### Port usage

`libL3Cam` uses the following ports for streaming and communications:

| PROTOCOL | PORT                             | CONFIGURATION  |
| -------- | -------------------------------- | -------------- |
| TCP      | 6000                             | Fixed          |
| UDP      | 6050 (LIDAR)                     | Fixed          |
| UDP      | 6060 (Allied Wide, Polarimetric) | Fixed          |
| UDP      | 6020 (Allied Narrow, RGB)        | Fixed          |
| UDP      | 6030 (LWIR)                      | Fixed          |
| RTSP     | 5040 (LIDAR)                     | Reconfigurable |
| RTSP     | 5030 (Allied Wide, Polarimetric) | Reconfigurable |
| RTSP     | 5010 (Allied Narrow, RGB)        | Reconfigurable |
| RTSP     | 5020 (LWIR)                      | Reconfigurable |

TCP is used internally by `lib3Cam` and is transparent to the user. However, the system host must have the required port available.

## Launch

### l3cam

To run the l3cam_ros driver, launch the `l3cam.launch` file specifying (if wanted) if the [stream launch file](#stream_l3cam), the [configure launch file](#configure_l3cam), `rviz` (for visualization GUI) and `rqt_reconfigure` (for dynamic reconfigure GUI) have to be launched too. By default, the values are as follows:

```
roslaunch l3cam_ros l3cam.launch stream:=true configure:=true rviz:=false rqt_reconfigure:=false
```

This will launch the [l3cam_ros_node](#l3cam_ros_node), which is the main node that connects to and controls the L3Cam, and the `stream_l3cam.launch` and `configure_l3cam.launch` files.

More parameters can be set if wanted for the default network and sensors parameters, this is seen in the [parameters section](#parameters). The default parameters specified will be passed to the main node ([l3cam_ros_node](#l3cam_ros_node)) and the [configure launch file](#configure_l3cam).

### stream_l3cam

This launch file launches all the [stream nodes](#pointcloud_stream) for all the sensors and `rviz` if specified. Once the main node connects to the L3Cam it will only keep open the stream nodes of the sensors the L3Cam has available, the other ones will shut down automatically.

The stream nodes stream automatically their sensor data to each sensor topic when data is available.

### configure_l3cam

This launch file launches all the [configure nodes](#network_configuration) for all the sensors and `rqt_reconfigure` if specified. Once the main node connects to the L3Cam it will only keep open the configure nodes of the sensors the L3Cam has available, the other ones will shut down automatically.

The configure nodes function like a dynamic reconfigure interface to change parameters, which is more user friendly. In reality, the parameters are changed with the main node through services, the configure nodes call services when a dynamic reconfigure parameter is changed.

If using `rqt_reconfiugre`, the window might show up before the driver is connected to the L3Cam, so you might need to click refresh for it to show the available parameters.

## ROS Nodes

### l3cam_ros_node

The l3cam_ros_node is the main node that connects to the L3Cam and configures it according to ROS [parameters](#parameters)/[services](#services). See the service files (`srv/`) for documentation regarding the various parameters that can be used to configure the L3Cam.

### pointcloud_stream

The pointcloud_stream is the node that publishes pointcloud frames if the LiDAR sensor is available.

### polarimetric_stream

The polarimetric_stream is the node that publishes polarimetric or Allied Wide image frames if the polarimetric or the Allied Wide sensor is available.

### rgb_stream

The rgb_stream is the node that publishes RGB or Allied Narrow image frames if the RGB or the Allied Narrow sensor is available.

### thermal_stream

The thermal_stream is the node that publishes thermal image frames if the thermal sensor is available.

### network_configuration

The network_configuration is a node that configures the network parameters by using dynamic reconfigure and the services to communicate with the l3cam_ros_node. See the config file `cfg/Network.cfg` for documentation regarding the various parameters that can be used to configure the network parameters of the L3Cam.

### pointcloud_configuration

The pointcloud_configuration is a node that configures the pointcloud parameters (if a LiDAR sensor is available) by using dynamic reconfigure and the services to communicate with the l3cam_ros_node. See the config file `cfg/Pointcloud.cfg` for documentation regarding the various parameters that can be used to configure the pointcloud parameters of the L3Cam.

### polarimetric_camera_configuration

The polarimetric_camera_configuration is a node that configures the polarimetric camera parameters (if a polarimetric sensor is available) by using dynamic reconfigure and the services to communicate with the l3cam_ros_node. See the config file `cfg/PolarimetricCamera.cfg` for documentation regarding the various parameters that can be used to configure the polarimetric camera parameters of the L3Cam.

### rgb_camera_configuration

The rgb_camera_configuration is a node that configures the RGB camera parameters (if an RGB sensor is available) by using dynamic reconfigure and the services to communicate with the l3cam_ros_node. See the config file `cfg/RgbCamera.cfg` for documentation regarding the various parameters that can be used to configure the RGB camera parameters of the L3Cam.

### thermal_camera_configuration

The thermal_camera_configuration is a node that configures the thermal camera parameters (if a thermal sensor is available) by using dynamic reconfigure and the services to communicate with the l3cam_ros_node. See the config file `cfg/ThermalCamera.cfg` for documentation regarding the various parameters that can be used to configure the thermal camera parameters of the L3Cam.

### allied_wide_camera_configuration

The allied_wide_camera_configuration is a node that configures the Allied Wide camera parameters (if aan Allied Wide sensor is available) by using dynamic reconfigure and the services to communicate with the l3cam_ros_node. See the config file `cfg/AlliedWideCamera.cfg` for documentation regarding the various parameters that can be used to configure the Allied Wide camera parameters of the L3Cam.

### allied_narrow_camera_configuration

The allied_narrow_camera_configuration is a node that configures the Allied Narrow camera parameters (if aan Allied Narrow sensor is available) by using dynamic reconfigure and the services to communicate with the l3cam_ros_node. See the config file `cfg/AlliedNarrowCamera.cfg` for documentation regarding the various parameters that can be used to configure the Allied Narrow camera parameters of the L3Cam.

## Parameters

Default parameters for the L3Cam can be set by editing the `l3cam.launch` file or specifying them when calling it:

```
roslaunch l3cam_ros l3cam.launch <PARAMETER>:=<VALUE>
```

Some parameters are enumerate's declared on the `libL3Cam`, check the [L3Cam User Manual](https://github.com/beamaginelidar/libl3cam/blob/main/L3CAM%20User%20Manual.pdf) for more info.

### Network parameters

| Parameter    | Type   | Default       |
| ------------ | ------ | ------------- |
| `ip_address` | string | 192.168.1.250 |
| `netmask`    | string | 255.255.255.0 |
| `gateway`    | string | 0.0.0.0       |
| `dhcp`       | bool   | false         |

### Pointcloud parameters

| Parameter                        | Type | Default | Range                 |
| -------------------------------- | ---- | ------- | --------------------- |
| `pointcloud_color`               | enum | 0       | see `pointCloudColor` |
| `pointcloud_color_range_minimum` | int  | 0       | 0 - 400000            |
| `pointcloud_color_range_maximum` | int  | 400000  | 0 - 400000            |
| `distance_range_minimum`         | int  | 0       | 0 - 400000            |
| `distance_range_maximum`         | int  | 400000  | 0 - 400000            |

### Polarimetric parameters

| Parameter                                              | Type   | Default   | Range            |
| ------------------------------------------------------ | ------ | --------- | ---------------- |
| `polarimetric_camera_brightness`                       | int    | 127       | 0 - 255          |
| `polarimetric_camera_black_level`                      | double | 6.0       | 0 - 12.5         |
| `polarimetric_camera_auto_gain`                        | bool   | true      |                  |
| `polarimetric_camera_auto_gain_range_minimum`          | double | 0.0       | 0 - 48           |
| `polarimetric_camera_auto_gain_range_maximum`          | double | 48.0      | 0 - 48           |
| `polarimetric_camera_gain`                             | double | 24.0      | 0 - 48           |
| `polarimetric_camera_auto_exposure_time`               | bool   | true      |                  |
| `polarimetric_camera_auto_exposure_time_range_minimum` | double | 33.456    | 33.456 - 1000000 |
| `polarimetric_camera_auto_exposure_time_range_maximum` | double | 1000000.0 | 33.456 - 1000000 |
| `polarimetric_camera_exposure_time`                    | double | 500000.0  | 33.456 - 1000000 |

### RGB parameters

| Parameter                       | Type | Default | Range                 |
| ------------------------------- | ---- | ------- | --------------------- |
| `rgb_camera_brightness`         | int  | 0       | -15 - 15              |
| `rgb_camera_contrast`           | int  | 10      | 0 - 30                |
| `rgb_camera_saturation`         | int  | 16      | 0 - 60                |
| `rgb_camera_sharpness`          | int  | 16      | 0 - 127               |
| `rgb_camera_gamma`              | int  | 220     | 40 - 500              |
| `rgb_camera_gain`               | int  | 0       | 0 - 63                |
| `rgb_camera_auto_white_balance` | bool | true    |                       |
| `rgb_camera_white_balance`      | int  | 5000    | 1000 - 10000          |
| `rgb_camera_auto_exposure_time` | bool | true    |                       |
| `rgb_camera_exposure_time`      | int  | 156     | 1 - 10000             |
| `rgb_camera_resolution`         | enum | 3       | see `econResolutions` |
| `rgb_camera_framerate`          | int  | 10      | 1 - 16                |

### Thermal parameters

| Parameter                               | Type | Default | Range              |
| --------------------------------------- | ---- | ------- | ------------------ |
| `thermal_camera_colormap`               | enum | 1       | see `thermalTypes` |
| `thermal_camera_temperature_filter`     | bool | false   |                    |
| `thermal_camera_temperature_filter_min` | int  | 0       | -40 - 200          |
| `thermal_camera_temperature_filter_max` | int  | 50      | -40 - 200          |

### Allied Wide parameters

| Parameter                                         | Type   | Default | Range                               |
| ------------------------------------------------- | ------ | ------- | ----------------------------------- |
| `allied_wide_camera_black_level`                  | double | 0       | 0 - 4095                            |
| `allied_wide_camera_exposure_time`                | double | 4992.32 | 63 - 10000000                       |
| `allied_wide_camera_auto_exposure_time`           | bool   | false   |                                     |
| `allied_wide_camera_auto_exposure_time_range_min` | double | 87.596  | 63.03 - 8999990                     |
| `allied_wide_camera_auto_exposure_time_range_max` | double | 87.596  | 87.596 - 10000000                   |
| `allied_wide_camera_gain`                         | double | 0       | 0 - 48                              |
| `allied_wide_camera_auto_gain`                    | bool   | false   |                                     |
| `allied_wide_camera_auto_gain_range_min`          | double | 0       | 0 - 48                              |
| `allied_wide_camera_auto_gain_range_max`          | double | 48      | 0 - 48                              |
| `allied_wide_camera_gamma`                        | double | 1       | 0.4 - 2.4                           |
| `allied_wide_camera_saturation`                   | double | 1       | 0 - 2                               |
| `allied_wide_camera_sharpness`                    | double | 0       | -12 - 12                            |
| `allied_wide_camera_hue`                          | double | 0       | -40 - 40                            |
| `allied_wide_camera_intensity_auto_precedence`    | enum   | 0       | 0(MinimizeNoise) or 1(MinimizeBlur) |
| `allied_wide_camera_auto_white_balance`           | bool   | false   |                                     |
| `allied_wide_camera_balance_ratio_selector`       | enum   | 0       | 0(Red), 1(Blue)                     |
| `allied_wide_camera_balance_ratio`                | double | 2.35498 | 0 - 8                               |
| `allied_wide_camera_balance_white_auto_rate`      | double | 100     | 0 - 100                             |
| `allied_wide_camera_balance_white_auto_tolerance` | double | 5       | 0 - 50                              |
| `allied_wide_camera_auto_mode_region_height`      | int    | 1028    | 0 - 1028                            |
| `allied_wide_camera_auto_mode_region_width`       | int    | 1232    | 0 - 1232                            |
| `allied_wide_camera_intensity_controller_region`  | enum   | 0       | 0(AutoMode), 4(FullImage)           |
| `allied_wide_camera_intensity_controller_target`  | double | 50      | 10 - 90                             |
| `allied_wide_camera_max_driver_buffers_count`     | int    | 64      | 1 - 4096                            |

### Allied Narrow parameters

| Parameter                                           | Type   | Default | Range                               |
| --------------------------------------------------- | ------ | ------- | ----------------------------------- |
| `allied_narrow_camera_black_level`                  | double | 0       | 0 - 4095                            |
| `allied_narrow_camera_exposure_time`                | double | 4992.32 | 63 - 10000000                       |
| `allied_narrow_camera_auto_exposure_time`           | bool   | false   |                                     |
| `allied_narrow_camera_auto_exposure_time_range_min` | double | 87.596  | 63.03 - 8999990                     |
| `allied_narrow_camera_auto_exposure_time_range_max` | double | 87.596  | 87.596 - 10000000                   |
| `allied_narrow_camera_gain`                         | double | 0       | 0 - 48                              |
| `allied_narrow_camera_auto_gain`                    | bool   | false   |                                     |
| `allied_narrow_camera_auto_gain_range_min`          | double | 0       | 0 - 48                              |
| `allied_narrow_camera_auto_gain_range_max`          | double | 48      | 0 - 48                              |
| `allied_narrow_camera_gamma`                        | double | 1       | 0.4 - 2.4                           |
| `allied_narrow_camera_saturation`                   | double | 1       | 0 - 2                               |
| `allied_narrow_camera_sharpness`                    | double | 0       | -12 - 12                            |
| `allied_narrow_camera_hue`                          | double | 0       | -40 - 40                            |
| `allied_narrow_camera_intensity_auto_precedence`    | enum   | 0       | 0(MinimizeNoise) or 1(MinimizeBlur) |
| `allied_narrow_camera_auto_white_balance`           | bool   | false   |                                     |
| `allied_narrow_camera_balance_ratio_selector`       | enum   | 0       | 0(Red), 1(Blue)                     |
| `allied_narrow_camera_balance_ratio`                | double | 2.35498 | 0 - 8                               |
| `allied_narrow_camera_balance_white_auto_rate`      | double | 100     | 0 - 100                             |
| `allied_narrow_camera_balance_white_auto_tolerance` | double | 5       | 0 - 50                              |
| `allied_narrow_camera_auto_mode_region_height`      | int    | 1028    | 0 - 1028                            |
| `allied_narrow_camera_auto_mode_region_width`       | int    | 1232    | 0 - 1232                            |
| `allied_narrow_camera_intensity_controller_region`  | enum   | 0       | 0(AutoMode), 4(FullImage)           |
| `allied_narrow_camera_intensity_controller_target`  | double | 50      | 10 - 90                             |
| `allied_narrow_camera_max_driver_buffers_count`     | int    | 64      | 1 - 4096                            |

## Services

Once the nodes are launched, the parameters can be changed through services. While streaming, some parameters cannot be changed, and the driver starts streaming when it connects to the L3Cam.

Only the changeable parameters while streaming will appear on the dynamic reconfigure of the configure nodes. Even though all the parameters are available, if a non changeable parameter is attempted to be changed, the service will return error.

To list the available services:

```
rosservice list
```

For info on a service:

```
rosservice info <SERVICE>
```

To call a service:

```
rosservice call <SERVICE> <ARGS>
```
