#include <iostream>
#include <ros/ros.h>

#include "libL3Cam.h"
#include "beamagine.h"
#include "beamErrors.h"

#include "l3cam_ros/Sensor.h"

#include "l3cam_ros/GetVersion.h"
#include "l3cam_ros/Initialize.h"
#include "l3cam_ros/Terminate.h"
#include "l3cam_ros/FindDevices.h"
#include "l3cam_ros/GetLocalServerAddress.h"
#include "l3cam_ros/GetDeviceStatus.h"
#include "l3cam_ros/GetSensorsAvaliable.h"
#include "l3cam_ros/GetNetworkConfiguration.h"
#include "l3cam_ros/ChangeNetworkConfiguration.h"
#include "l3cam_ros/PowerOffDevice.h"
#include "l3cam_ros/StartDevice.h"
#include "l3cam_ros/StopDevice.h"
#include "l3cam_ros/StartStream.h"
#include "l3cam_ros/StopStream.h"

#include "l3cam_ros/ChangePointcloudColor.h"
#include "l3cam_ros/ChangePointcloudColorRange.h"
#include "l3cam_ros/ChangeDistanceRange.h"

#include "l3cam_ros/SetPolarimetricCameraDefaultSettings.h"
#include "l3cam_ros/ChangePolarimetricCameraBrightness.h"
#include "l3cam_ros/ChangePolarimetricCameraBlackLevel.h"
#include "l3cam_ros/EnablePolarimetricCameraAutoGain.h"
#include "l3cam_ros/ChangePolarimetricCameraAutoGainRange.h"
#include "l3cam_ros/ChangePolarimetricCameraGain.h"
#include "l3cam_ros/EnablePolarimetricCameraAutoExposureTime.h"
#include "l3cam_ros/ChangePolarimetricCameraAutoExposureTimeRange.h"
#include "l3cam_ros/ChangePolarimetricCameraExposureTime.h"

#include "l3cam_ros/SetRgbCameraDefaultSettings.h"
#include "l3cam_ros/ChangeRgbCameraBrightness.h"
#include "l3cam_ros/ChangeRgbCameraContrast.h"
#include "l3cam_ros/ChangeRgbCameraSaturation.h"
#include "l3cam_ros/ChangeRgbCameraSharpness.h"
#include "l3cam_ros/ChangeRgbCameraGamma.h"
#include "l3cam_ros/ChangeRgbCameraGain.h"
#include "l3cam_ros/ChangeRgbCameraWhiteBalance.h"
#include "l3cam_ros/EnableRgbCameraAutoExposureTime.h"
#include "l3cam_ros/ChangeRgbCameraExposureTime.h"
#include "l3cam_ros/EnableRgbCameraAutoWhiteBalance.h"

#include "l3cam_ros/ChangeThermalCameraColormap.h"
#include "l3cam_ros/ChangeThermalCameraTemperatureFilter.h"
#include "l3cam_ros/EnableThermalCameraTemperatureFilter.h"

l3cam devices[1];
sensor av_sensors[6];

bool getVersion(l3cam_ros::GetVersion::Request &req, l3cam_ros::GetVersion::Response &res)
{
    res.version = GET_VERSION();
    return true;
}

bool initialize(l3cam_ros::Initialize::Request &req, l3cam_ros::Initialize::Response &res)
{
    res.error = INITIALIZE();
    return true;
}

bool terminate(l3cam_ros::Terminate::Request &req, l3cam_ros::Terminate::Response &res)
{
    res.error = TERMINATE(devices[0]);
    return true;
}

bool findDevices(l3cam_ros::FindDevices::Request &req, l3cam_ros::FindDevices::Response &res)
{
    res.error = FIND_DEVICES(&devices[0], &res.num_devices);
    return true;
}

bool getLocalServerAddress(l3cam_ros::GetLocalServerAddress::Request &req, l3cam_ros::GetLocalServerAddress::Response &res)
{
    res.local_ip_address = GET_LOCAL_SERVER_ADDRESS(devices[0]);
    return true;
}

bool getDeviceStatus(l3cam_ros::GetDeviceStatus::Request &req, l3cam_ros::GetDeviceStatus::Response &res)
{
    res.error = GET_DEVICE_STATUS(devices[0], &res.system_status);
    return true;
}

bool getSensorsAvaliable(l3cam_ros::GetSensorsAvaliable::Request &req, l3cam_ros::GetSensorsAvaliable::Response &res)
{
    res.error = GET_SENSORS_AVAILABLE(devices[0], av_sensors, &res.num_sensors);
    res.sensors.resize(res.num_sensors);
    for (int i = 0; i < res.num_sensors; ++i)
    {
        res.sensors[i].protocol = av_sensors[i].protocol;
        res.sensors[i].sensor_type = av_sensors[i].sensor_type;
        res.sensors[i].sensor_status = av_sensors[i].sensor_status;
        res.sensors[i].image_type = av_sensors[i].image_type;
        res.sensors[i].perception_enabled = av_sensors[i].perception_enabled;
    }
    return true;
}

bool getNetworkConfiguration(l3cam_ros::GetNetworkConfiguration::Request &req, l3cam_ros::GetNetworkConfiguration::Response &res)
{
    char *ip_address = NULL;
    char *netmask = NULL;
    char *gateway = NULL;
    res.error = GET_NETWORK_CONFIGURATION(devices[0], &ip_address, &netmask, &gateway);
    res.ip_address = ip_address;
    res.netmask = netmask;
    res.gateway = gateway;
    return true;
}

bool changeNetworkConfiguration(l3cam_ros::ChangeNetworkConfiguration::Request &req, l3cam_ros::ChangeNetworkConfiguration::Response &res)
{
    if (req.enable_dhcp)
        res.error = CHANGE_NETWORK_CONFIGURATION(devices[0], NULL, NULL, NULL, true);
    else
    {
        std::string ip_address = req.ip_address;
        std::string netmask = req.netmask;
        std::string gateway = req.gateway;
        res.error = CHANGE_NETWORK_CONFIGURATION(devices[0], (char *)ip_address.data(), (char *)netmask.data(), (char *)gateway.data(), false);
    }
    return true;
}

bool powerOffDevice(l3cam_ros::PowerOffDevice::Request &req, l3cam_ros::PowerOffDevice::Response &res)
{
    POWER_OFF_DEVICE(devices[0]);
    res.error = 0;
    return true;
}

bool startDevice(l3cam_ros::StartDevice::Request &req, l3cam_ros::StartDevice::Response &res)
{
    res.error = START_DEVICE(devices[0]);
    return true;
}

bool stopDevice(l3cam_ros::StopDevice::Request &req, l3cam_ros::StopDevice::Response &res)
{
    res.error = STOP_DEVICE(devices[0]);
    return true;
}

bool startStream(l3cam_ros::StartStream::Request &req, l3cam_ros::StartStream::Response &res)
{
    res.error = START_STREAM(devices[0]);
    return true;
}

bool stopStream(l3cam_ros::StopStream::Request &req, l3cam_ros::StopStream::Response &res)
{
    res.error = STOP_STREAM(devices[0]);
    return true;
}

// PointCloud
bool changePointcloudColor(l3cam_ros::ChangePointcloudColor::Request &req, l3cam_ros::ChangePointcloudColor::Response &res)
{
    res.error = CHANGE_POINT_CLOUD_COLOR(devices[0], req.visualization_color);
    return true;
}

bool changePointcloudColorRange(l3cam_ros::ChangePointcloudColorRange::Request &req, l3cam_ros::ChangePointcloudColorRange::Response &res)
{
    res.error = CHANGE_POINT_CLOUD_COLOR_RANGE(devices[0], req.min_value, req.max_value);
    return true;
}

bool changeDistanceRange(l3cam_ros::ChangeDistanceRange::Request &req, l3cam_ros::ChangeDistanceRange::Response &res)
{
    res.error = CHANGE_DISTANCE_RANGE(devices[0], req.min_value, req.max_value);
    return true;
}

// Polarimetric
bool setPolarimetricCameraDefaultSettings(l3cam_ros::SetPolarimetricCameraDefaultSettings::Request &req, l3cam_ros::SetPolarimetricCameraDefaultSettings::Response &res)
{
    res.error = SET_POLARIMETRIC_CAMERA_DEFAULT_SETTINGS(devices[0]);
    return true;
}

bool changePolarimetricCameraBrightness(l3cam_ros::ChangePolarimetricCameraBrightness::Request &req, l3cam_ros::ChangePolarimetricCameraBrightness::Response &res)
{
    res.error = CHANGE_POLARIMETRIC_CAMERA_BRIGHTNESS(devices[0], req.brightness);
    return true;
}

bool changePolarimetricCameraBlackLevel(l3cam_ros::ChangePolarimetricCameraBlackLevel::Request &req, l3cam_ros::ChangePolarimetricCameraBlackLevel::Response &res)
{
    res.error = CHANGE_POLARIMETRIC_CAMERA_BLACK_LEVEL(devices[0], req.black_level);
    return true;
}

bool enablePolarimetricCameraAutoGain(l3cam_ros::EnablePolarimetricCameraAutoGain::Request &req, l3cam_ros::EnablePolarimetricCameraAutoGain::Response &res)
{
    res.error = ENABLE_POLARIMETRIC_CAMERA_AUTO_GAIN(devices[0], req.enabled);
    return true;
}

bool changePolarimetricCameraAutoGainRange(l3cam_ros::ChangePolarimetricCameraAutoGainRange::Request &req, l3cam_ros::ChangePolarimetricCameraAutoGainRange::Response &res)
{
    res.error = CHANGE_POLARIMETRIC_CAMERA_AUTO_GAIN_RANGE(devices[0], req.min_gain, req.max_gain);
    return true;
}

bool changePolarimetricCameraGain(l3cam_ros::ChangePolarimetricCameraGain::Request &req, l3cam_ros::ChangePolarimetricCameraGain::Response &res)
{
    res.error = CHANGE_POLARIMETRIC_CAMERA_GAIN(devices[0], req.gain);
    return true;
}

bool enablePolarimetricCameraAutoExposureTime(l3cam_ros::EnablePolarimetricCameraAutoExposureTime::Request &req, l3cam_ros::EnablePolarimetricCameraAutoExposureTime::Response &res)
{
    res.error = ENABLE_POLARIMETRIC_CAMERA_AUTO_EXPOSURE_TIME(devices[0], req.enabled);
    return true;
}

bool changePolarimetricCameraAutoExposureTimeRange(l3cam_ros::ChangePolarimetricCameraAutoExposureTimeRange::Request &req, l3cam_ros::ChangePolarimetricCameraAutoExposureTimeRange::Response &res)
{
    res.error = CHANGE_POLARIMETRIC_CAMERA_AUTO_EXPOSURE_TIME_RANGE(devices[0], req.min_exposure, req.max_exposure);
    return true;
}

bool changePolarimetricCameraExposureTime(l3cam_ros::ChangePolarimetricCameraExposureTime::Request &req, l3cam_ros::ChangePolarimetricCameraExposureTime::Response &res)
{
    res.error = CHANGE_POLARIMETRIC_CAMERA_EXPOSURE_TIME(devices[0], req.exposure_time);
    return true;
}

// RGB
bool setRgbCameraDefaultSettings(l3cam_ros::SetRgbCameraDefaultSettings::Request &req, l3cam_ros::SetRgbCameraDefaultSettings::Response &res)
{
    res.error = SET_RGB_CAMERA_DEFAULT_SETTINGS(devices[0]);
    return true;
}

bool changeRgbCameraBrightness(l3cam_ros::ChangeRgbCameraBrightness::Request &req, l3cam_ros::ChangeRgbCameraBrightness::Response &res)
{
    res.error = CHANGE_RGB_CAMERA_BRIGHTNESS(devices[0], req.brightness);
    return true;
}

bool changeRgbCameraContrast(l3cam_ros::ChangeRgbCameraContrast::Request &req, l3cam_ros::ChangeRgbCameraContrast::Response &res)
{
    res.error = CHANGE_RGB_CAMERA_CONTRAST(devices[0], req.contrast);
    return true;
}

bool changeRgbCameraSaturation(l3cam_ros::ChangeRgbCameraSaturation::Request &req, l3cam_ros::ChangeRgbCameraSaturation::Response &res)
{
    res.error = CHANGE_RGB_CAMERA_SATURATION(devices[0], req.saturation);
    return true;
}

bool changeRgbCameraSharpness(l3cam_ros::ChangeRgbCameraSharpness::Request &req, l3cam_ros::ChangeRgbCameraSharpness::Response &res)
{
    res.error = CHANGE_RGB_CAMERA_SHARPNESS(devices[0], req.sharpness);
    return true;
}

bool changeRgbCameraGamma(l3cam_ros::ChangeRgbCameraGamma::Request &req, l3cam_ros::ChangeRgbCameraGamma::Response &res)
{
    res.error = CHANGE_RGB_CAMERA_GAMMA(devices[0], req.gamma);
    return true;
}

bool changeRgbCameraGain(l3cam_ros::ChangeRgbCameraGain::Request &req, l3cam_ros::ChangeRgbCameraGain::Response &res)
{
    res.error = CHANGE_RGB_CAMERA_GAIN(devices[0], req.gain);
    return true;
}

bool enableRgbCameraAutoWhiteBalance(l3cam_ros::EnableRgbCameraAutoWhiteBalance::Request &req, l3cam_ros::EnableRgbCameraAutoWhiteBalance::Response &res)
{
    res.error = ENABLE_RGB_CAMERA_AUTO_WHITE_BALANCE(devices[0], req.enabled);
    return true;
}

bool changeRgbCameraWhiteBalance(l3cam_ros::ChangeRgbCameraWhiteBalance::Request &req, l3cam_ros::ChangeRgbCameraWhiteBalance::Response &res)
{
    res.error = CHANGE_RGB_CAMERA_WHITE_BALANCE(devices[0], req.white_balance);
    return true;
}

bool enableRgbCameraAutoExposureTime(l3cam_ros::EnableRgbCameraAutoExposureTime::Request &req, l3cam_ros::EnableRgbCameraAutoExposureTime::Response &res)
{
    res.error = ENABLE_RGB_CAMERA_AUTO_EXPOSURE_TIME(devices[0], req.enabled);
    return true;
}

bool changeRgbCameraExposureTime(l3cam_ros::ChangeRgbCameraExposureTime::Request &req, l3cam_ros::ChangeRgbCameraExposureTime::Response &res)
{
    res.error = CHANGE_RGB_CAMERA_EXPOSURE_TIME(devices[0], req.exposure_time);
    return true;
}

// Thermal
bool changeThermalCameraColormap(l3cam_ros::ChangeThermalCameraColormap::Request &req, l3cam_ros::ChangeThermalCameraColormap::Response &res)
{
    res.error = CHANGE_THERMAL_CAMERA_COLORMAP(devices[0], (thermalTypes)req.colormap);
    return true;
}

bool changeThermalCameraTemperatureFilter(l3cam_ros::ChangeThermalCameraTemperatureFilter::Request &req, l3cam_ros::ChangeThermalCameraTemperatureFilter::Response &res)
{
    res.error = CHANGE_THERMAL_CAMERA_TEMPERATURE_FILTER(devices[0], req.min_temperature, req.max_temperature);
    return true;
}

bool enableThermalCameraTemperatureFilter(l3cam_ros::EnableThermalCameraTemperatureFilter::Request &req, l3cam_ros::EnableThermalCameraTemperatureFilter::Response &res)
{
    res.error = ENABLE_THERMAL_CAMERA_TEMPERATURE_FILTER(devices[0], req.enabled);
    return true;
}

int main(int argc, char **argv)
{
    std::cout << "L3Cam version " << GET_VERSION() << std::endl;

    ros::init(argc, argv, "l3cam_ros_node");
    ros::NodeHandle nh;

    // Initialize services
    ros::ServiceServer srvGetVersion = nh.advertiseService("get_version", getVersion);
    ros::ServiceServer srvInitialize = nh.advertiseService("initialize", initialize);
    ros::ServiceServer srvTerminate = nh.advertiseService("terminate", terminate);
    ros::ServiceServer srvFindDevices = nh.advertiseService("find_devices", findDevices);
    ros::ServiceServer srvGetLocalServerAddress = nh.advertiseService("get_local_server_address", getLocalServerAddress);
    ros::ServiceServer srvGetDeviceStatus = nh.advertiseService("get_device_status", getDeviceStatus);
    ros::ServiceServer srvGetSensorsAvaliable = nh.advertiseService("get_sensors_avaliable", getSensorsAvaliable);
    ros::ServiceServer srvGetNetworkConfiguration = nh.advertiseService("get_network_configuration", getNetworkConfiguration);
    ros::ServiceServer srvChangeNetworkConfiguration = nh.advertiseService("change_network_configuration", changeNetworkConfiguration);
    ros::ServiceServer srvPowerOffDevice = nh.advertiseService("power_off_device", powerOffDevice);
    ros::ServiceServer srvStartDevice = nh.advertiseService("start_device", startDevice);
    ros::ServiceServer srvStopDevice = nh.advertiseService("stop_device", stopDevice);
    ros::ServiceServer srvStartStream = nh.advertiseService("start_stream", startStream);
    ros::ServiceServer srvStopStream = nh.advertiseService("stop_stream", stopStream);

    ros::ServiceServer srvChangePointcloudColor = nh.advertiseService("change_pointcloud_color", changePointcloudColor);
    ros::ServiceServer srvChangePointcloudColorRange = nh.advertiseService("change_pointcloud_color_range", changePointcloudColorRange);
    ros::ServiceServer srvChangeDistanceRange = nh.advertiseService("change_distance_range", changeDistanceRange);

    ros::ServiceServer srvSetPolarimetricCameraDefaultSettings = nh.advertiseService("set_polarimetric_camera_default_settings", setPolarimetricCameraDefaultSettings);
    ros::ServiceServer srvChangePolarimetricCameraBrightness = nh.advertiseService("change_polarimetric_camera_brightness", changePolarimetricCameraBrightness);
    ros::ServiceServer srvChangePolarimetricCameraBlackLevel = nh.advertiseService("change_polarimetric_camera_black_level", changePolarimetricCameraBlackLevel);
    ros::ServiceServer srvEnablePolarimetricCameraAutoGain = nh.advertiseService("enable_polarimetric_camera_auto_gain", enablePolarimetricCameraAutoGain);
    ros::ServiceServer srvChangePolarimetricCameraAutoGainRange = nh.advertiseService("change_polarimetric_camera_auto_gain_range", changePolarimetricCameraAutoGainRange);
    ros::ServiceServer srvChangePolarimetricCameraGain = nh.advertiseService("change_polarimetric_camera_gain", changePolarimetricCameraGain);
    ros::ServiceServer srvEnablePolarimetricCameraAutoExposureTime = nh.advertiseService("enable_polarimetric_camera_auto_exposure_time", enablePolarimetricCameraAutoExposureTime);
    ros::ServiceServer srvChangePolarimetricCameraAutoExposureTimeRange = nh.advertiseService("change_polarimetric_camera_auto_exposure_time_range", changePolarimetricCameraAutoExposureTimeRange);
    ros::ServiceServer srvChangePolarimetricCameraExposureTime = nh.advertiseService("change_polarimetric_camera_exposure_time", changePolarimetricCameraExposureTime);

    ros::ServiceServer srvSetRgbCameraDefaultSettings = nh.advertiseService("set_rgb_camera_default_settings", setRgbCameraDefaultSettings);
    ros::ServiceServer srvChangeRgbCameraBrightness = nh.advertiseService("change_rgb_camera_brightness", changeRgbCameraBrightness);
    ros::ServiceServer srvChangeRgbCameraContrast = nh.advertiseService("change_rgb_camera_contrast", changeRgbCameraContrast);
    ros::ServiceServer srvChangeRgbCameraSaturation = nh.advertiseService("change_rgb_camera_saturation", changeRgbCameraSaturation);
    ros::ServiceServer srvChangeRgbCameraSharpness = nh.advertiseService("change_rgb_camera_sharpness", changeRgbCameraSharpness);
    ros::ServiceServer srvChangeRgbCameraGamma = nh.advertiseService("change_rgb_camera_gamma", changeRgbCameraGamma);
    ros::ServiceServer srvChangeRgbCameraGain = nh.advertiseService("change_rgb_camera_gain", changeRgbCameraGain);
    ros::ServiceServer srvEnableRgbCameraAutoWhiteBalance = nh.advertiseService("enable_rgb_camera_auto_white_balance", enableRgbCameraAutoWhiteBalance);
    ros::ServiceServer srvChangeRgbCameraWhiteBalance = nh.advertiseService("change_rgb_camera_white_balance", changeRgbCameraWhiteBalance);
    ros::ServiceServer srvEnableRgbCameraAutoExposureTime = nh.advertiseService("enable_rgb_camera_auto_exposure_time", enableRgbCameraAutoExposureTime);
    ros::ServiceServer srvChangeRgbCameraExposureTime = nh.advertiseService("change_rgb_camera_exposure_time", changeRgbCameraExposureTime);
    
    ros::ServiceServer srvChangeThermalCameraColormap = nh.advertiseService("change_thermal_camera_colormap", changeThermalCameraColormap);
    ros::ServiceServer srvChangeThermalCameraTemperatureFilter = nh.advertiseService("change_thermal_camera_temperature_filter", changeThermalCameraTemperatureFilter);
    ros::ServiceServer srvEnableThermalCameraTemperatureFilter = nh.advertiseService("enable_thermal_camera_temperature_filter", enableThermalCameraTemperatureFilter);

    // Initialize L3Cam
    int error = L3CAM_OK;

    error = INITIALIZE();
    if (error)
    {
        ROS_ERROR_STREAM("(INTIALIZE error " << error << ") " << getBeamErrorDescription(error));
        TERMINATE(devices[0]);
        return 1;
    }

    int num_devices = 0;
    ros::Rate loop_rate(100);
    while (num_devices != 1 && ros::ok())
    {
        error = FIND_DEVICES(devices, &num_devices);
        loop_rate.sleep();
    }
    if (error)
    {
        ROS_ERROR_STREAM("(FIND_DEVICES error " << error << ") " << getBeamErrorDescription(error));
        TERMINATE(devices[0]);
        return 1;
    }
    ROS_INFO_STREAM("Device found " << devices[0].ip_address << " model " << devices[0].model << " serial number " << devices[0].serial_number);

    int status = 0;
    error = GET_DEVICE_STATUS(devices[0], &status);
    if (error)
    {
        ROS_ERROR_STREAM("(GET_DEVICE_STATUS error " << error << ") " << getBeamErrorDescription(error));
        TERMINATE(devices[0]);
        return 1;
    }
    ROS_INFO_STREAM("Device status " << status);

    int num_sensors = 0;
    error = GET_SENSORS_AVAILABLE(devices[0], av_sensors, &num_sensors);
    if (error)
    {
        ROS_ERROR_STREAM("(GET_SENSORS_AVALIABLE error " << error << ") " << getBeamErrorDescription(error));
        TERMINATE(devices[0]);
        return 1;
    }
    ROS_INFO_STREAM(num_sensors << " sensors avaliable");

    error = START_DEVICE(devices[0]);
    if (error)
    {
        ROS_ERROR_STREAM("(START_DEVICE error " << error << ") " << getBeamErrorDescription(error));
        TERMINATE(devices[0]);
        return 1;
    }
    ROS_INFO("Device started");

    error = START_STREAM(devices[0]);
    if (error)
    {
        ROS_ERROR_STREAM("(START_STREAM error " << error << ") " << getBeamErrorDescription(error));
        STOP_DEVICE(devices[0]);
        TERMINATE(devices[0]);
        return 1;
    }
    //ROS_INFO("Streaming...");

    while (ros::ok() && num_devices != 0)
    {
        error = FIND_DEVICES(devices, &num_devices);
        if (error)
        {
            ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
            break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    STOP_STREAM(devices[0]);
    STOP_DEVICE(devices[0]);
    TERMINATE(devices[0]);

    return 0;
}