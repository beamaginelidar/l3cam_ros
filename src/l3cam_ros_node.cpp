/*  Copyright (c) 2023, Beamagine

    All rights reserved.

    Redistribution and use in source and binary forms, with or without modification,
    are permitted provided that the following conditions are met:

        - Redistributions of source code must retain the above copyright notice,
          this list of conditions and the following disclaimer.
        - Redistributions in binary form must reproduce the above copyright notice,
          this list of conditions and the following disclaimer in the documentation and/or
          other materials provided with the distribution.
        - Neither the name of copyright holders nor the names of its contributors may be
          used to endorse or promote products derived from this software without specific
          prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
    MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
    EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
    HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
    TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
    EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <signal.h>

#include <iostream>
#include <ros/ros.h>

#include <libL3Cam.h>
#include <beamagine.h>
#include <beamErrors.h>

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

#include "l3cam_ros/ChangeAlliedCameraBlackLevel.h"
#include "l3cam_ros/ChangeAlliedCameraExposureTime.h"
#include "l3cam_ros/EnableAlliedCameraAutoExposureTime.h"
#include "l3cam_ros/ChangeAlliedCameraAutoExposureTimeRange.h"
#include "l3cam_ros/ChangeAlliedCameraGain.h"
#include "l3cam_ros/EnableAlliedCameraAutoGain.h"
#include "l3cam_ros/ChangeAlliedCameraAutoGainRange.h"
#include "l3cam_ros/ChangeAlliedCameraGamma.h"
#include "l3cam_ros/ChangeAlliedCameraSaturation.h"
#include "l3cam_ros/ChangeAlliedCameraSharpness.h"
#include "l3cam_ros/ChangeAlliedCameraHue.h"
#include "l3cam_ros/ChangeAlliedCameraIntensityAutoPrecedence.h"
#include "l3cam_ros/EnableAlliedCameraAutoWhiteBalance.h"
#include "l3cam_ros/ChangeAlliedCameraBalanceRatioSelector.h"
#include "l3cam_ros/ChangeAlliedCameraBalanceRatio.h"
#include "l3cam_ros/ChangeAlliedCameraBalanceWhiteAutoRate.h"
#include "l3cam_ros/ChangeAlliedCameraBalanceWhiteAutoTolerance.h"
#include "l3cam_ros/ChangeAlliedCameraAutoModeRegion.h"
#include "l3cam_ros/ChangeAlliedCameraIntensityControllerRegion.h"
#include "l3cam_ros/ChangeAlliedCameraIntensityControllerTarget.h"
#include "l3cam_ros/ChangeAlliedCameraMaxDriverBuffersCount.h"

l3cam devices[1];
sensor av_sensors[6];

sensor *m_lidar_sensor = NULL;
sensor *m_rgb_sensor = NULL;
sensor *m_thermal_sensor = NULL;
sensor *m_polarimetric_sensor = NULL;
sensor *m_allied_wide_sensor = NULL;
sensor *m_allied_narrow_sensor = NULL;

//! SIGNALS
std::string getSignalDescription(int signal)
{
    std::string description = "";
    switch (signal)
    {
    case SIGQUIT:
        description = "SIGQUIT";
        break;
    case SIGINT:
        description = "SIGINT";
        break;
    case SIGTERM:
        description = "SIGTERM";
        break;
    case SIGSEGV:
        description = "SIGSEGV";
        break;
    case SIGHUP:
        description = "SIGHUP";
        break;
    case SIGILL:
        description = "SIGILL";
        break;
    case SIGABRT:
        description = "SIGABRT";
        break;
    case SIGFPE:
        description = "SIGFPE";
        break;
    }
    return description;
}

void handleSignalCaptured(int signal)
{
    // std::cout<<"handleSignalCaputred exiting for captured signal "<<signal<<"--"<<getSignalDescription(signal)<<std::endl;
    //! TODO(ebernal) gestionar aqui el cierre controlado
    if (signal == SIGINT)
    {
        ROS_INFO("Terminating...");
        STOP_STREAM(devices[0]);
        STOP_DEVICE(devices[0]);
        TERMINATE(devices[0]);
    }
    exit(0);
}

void setCapturedSignal(int signal)
{
    struct sigaction sig_action;
    sig_action.sa_flags = 0;
    sigemptyset(&sig_action.sa_mask);
    sig_action.sa_handler = handleSignalCaptured;

    if (sigaction(signal, &sig_action, NULL) == 1)
    {
        std::cout << "setCaputredSignal Error setting signal\n";
    }
}

void registerSystemSignals()
{
    setCapturedSignal(SIGQUIT);
    setCapturedSignal(SIGINT);
    setCapturedSignal(SIGTERM);
    setCapturedSignal(SIGSEGV);
    setCapturedSignal(SIGHUP);
    setCapturedSignal(SIGILL);
    setCapturedSignal(SIGABRT);
    setCapturedSignal(SIGFPE);
}

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

// Allied
bool changeAlliedCameraBlackLevel(l3cam_ros::ChangeAlliedCameraBlackLevel::Request &req, l3cam_ros::ChangeAlliedCameraBlackLevel::Response &res)
{
    switch (req.allied_type)
    {
    case 1:
        res.error = CHANGE_ALLIED_CAMERA_BLACK_LEVEL(devices[0], *m_allied_wide_sensor, req.black_level);
        break;
    case 2:
        res.error = CHANGE_ALLIED_CAMERA_BLACK_LEVEL(devices[0], *m_allied_narrow_sensor, req.black_level);
        break;
    default:
        res.error = 1000;
    }
    return true;
}

bool changeAlliedCameraExposureTime(l3cam_ros::ChangeAlliedCameraExposureTime::Request &req, l3cam_ros::ChangeAlliedCameraExposureTime::Response &res)
{
    switch (req.allied_type)
    {
    case 1:
        res.error = CHANGE_ALLIED_CAMERA_EXPOSURE_TIME_US(devices[0], *m_allied_wide_sensor, req.exposure_time);
        break;
    case 2:
        res.error = CHANGE_ALLIED_CAMERA_EXPOSURE_TIME_US(devices[0], *m_allied_narrow_sensor, req.exposure_time);
        break;
    default:
        res.error = 1000;
    }
    return true;
}

bool enableAlliedCameraAutoExposureTime(l3cam_ros::EnableAlliedCameraAutoExposureTime::Request &req, l3cam_ros::EnableAlliedCameraAutoExposureTime::Response &res)
{
    switch (req.allied_type)
    {
    case 1:
        res.error = ENABLE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME(devices[0], *m_allied_wide_sensor, req.enabled);
        break;
    case 2:
        res.error = ENABLE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME(devices[0], *m_allied_narrow_sensor, req.enabled);
        break;
    default:
        res.error = 1000;
    }
    return true;
}

bool changeAlliedCameraAutoExposureTimeRange(l3cam_ros::ChangeAlliedCameraAutoExposureTimeRange::Request &req, l3cam_ros::ChangeAlliedCameraAutoExposureTimeRange::Response &res)
{
    switch (req.allied_type)
    {
    case 1:
        res.error = CHANGE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME_RANGE(devices[0], *m_allied_wide_sensor, req.auto_exposure_time_range_min, req.auto_exposure_time_range_max);
        break;
    case 2:
        res.error = CHANGE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME_RANGE(devices[0], *m_allied_narrow_sensor, req.auto_exposure_time_range_min, req.auto_exposure_time_range_max);
        break;
    default:
        res.error = 1000; //! Can't put max and min
    }
    return true;
}

bool changeAlliedCameraGain(l3cam_ros::ChangeAlliedCameraGain::Request &req, l3cam_ros::ChangeAlliedCameraGain::Response &res)
{
    switch (req.allied_type)
    {
    case 1:
        res.error = CHANGE_ALLIED_CAMERA_GAIN(devices[0], *m_allied_wide_sensor, req.gain);
        break;
    case 2:
        res.error = CHANGE_ALLIED_CAMERA_GAIN(devices[0], *m_allied_narrow_sensor, req.gain);
        break;
    default:
        res.error = 1000;
    }
    return true;
}

bool enableAlliedCameraAutoGain(l3cam_ros::EnableAlliedCameraAutoGain::Request &req, l3cam_ros::EnableAlliedCameraAutoGain::Response &res)
{
    switch (req.allied_type)
    {
    case 1:
        res.error = ENABLE_ALLIED_CAMERA_AUTO_GAIN(devices[0], *m_allied_wide_sensor, req.enabled);
        break;
    case 2:
        res.error = ENABLE_ALLIED_CAMERA_AUTO_GAIN(devices[0], *m_allied_narrow_sensor, req.enabled);
        break;
    default:
        res.error = 1000;
    }
    return true;
}

bool changeAlliedCameraAutoGainRange(l3cam_ros::ChangeAlliedCameraAutoGainRange::Request &req, l3cam_ros::ChangeAlliedCameraAutoGainRange::Response &res)
{
    switch (req.allied_type)
    {
    case 1:
        res.error = CHANGE_ALLIED_CAMERA_AUTO_GAIN_RANGE(devices[0], *m_allied_wide_sensor, req.auto_gain_range_min, req.auto_gain_range_max);
        break;
    case 2:
        res.error = CHANGE_ALLIED_CAMERA_AUTO_GAIN_RANGE(devices[0], *m_allied_narrow_sensor, req.auto_gain_range_min, req.auto_gain_range_max);
        break;
    default:
        res.error = 1000; //! Can't put max and min
    }
    return true;
}

bool changeAlliedCameraGamma(l3cam_ros::ChangeAlliedCameraGamma::Request &req, l3cam_ros::ChangeAlliedCameraGamma::Response &res)
{
    switch (req.allied_type)
    {
    case 1:
        res.error = CHANGE_ALLIED_CAMERA_GAMMA(devices[0], *m_allied_wide_sensor, req.gamma);
        break;
    case 2:
        res.error = CHANGE_ALLIED_CAMERA_GAMMA(devices[0], *m_allied_narrow_sensor, req.gamma);
        break;
    default:
        res.error = 1000;
    }
    return true;
}

bool changeAlliedCameraSaturation(l3cam_ros::ChangeAlliedCameraSaturation::Request &req, l3cam_ros::ChangeAlliedCameraSaturation::Response &res)
{
    switch (req.allied_type)
    {
    case 1:
        res.error = CHANGE_ALLIED_CAMERA_SATURATION(devices[0], *m_allied_wide_sensor, req.saturation);
        break;
    case 2:
        res.error = CHANGE_ALLIED_CAMERA_SATURATION(devices[0], *m_allied_narrow_sensor, req.saturation);
        break;
    default:
        res.error = 1000;
    }
    return true;
}

bool changeAlliedCameraSharpness(l3cam_ros::ChangeAlliedCameraSharpness::Request &req, l3cam_ros::ChangeAlliedCameraSharpness::Response &res)
{
    switch (req.allied_type)
    {
    case 1:
        res.error = CHANGE_ALLIED_CAMERA_SHARPNESS(devices[0], *m_allied_wide_sensor, req.sharpness);
        break;
    case 2:
        res.error = CHANGE_ALLIED_CAMERA_SHARPNESS(devices[0], *m_allied_narrow_sensor, req.sharpness);
        break;
    default:
        res.error = 1000;
    }
    return true;
}

bool changeAlliedCameraHue(l3cam_ros::ChangeAlliedCameraHue::Request &req, l3cam_ros::ChangeAlliedCameraHue::Response &res)
{
    switch (req.allied_type)
    {
    case 1:
        res.error = CHANGE_ALLIED_CAMERA_HUE(devices[0], *m_allied_wide_sensor, req.hue);
        break;
    case 2:
        res.error = CHANGE_ALLIED_CAMERA_HUE(devices[0], *m_allied_narrow_sensor, req.hue);
        break;
    default:
        res.error = 1000;
    }
    return true;
}

bool changeAlliedCameraIntensityAutoPrecedence(l3cam_ros::ChangeAlliedCameraIntensityAutoPrecedence::Request &req, l3cam_ros::ChangeAlliedCameraIntensityAutoPrecedence::Response &res)
{
    switch (req.allied_type)
    {
    case 1:
        switch (req.intensity_auto_precedence)
        {
        case 1:
            res.error = CHANGE_ALLIED_CAMERA_INTENSITY_AUTO_PRECEDENCE(devices[0], *m_allied_wide_sensor, allied_auto_precedence_minimize_noise);
            break;
        case 2:
            res.error = CHANGE_ALLIED_CAMERA_INTENSITY_AUTO_PRECEDENCE(devices[0], *m_allied_wide_sensor, allied_auto_precedence_minimize_blur);
            break;
        }
        break;
    case 2:
        switch (req.intensity_auto_precedence)
        {
        case 1:
            res.error = CHANGE_ALLIED_CAMERA_INTENSITY_AUTO_PRECEDENCE(devices[0], *m_allied_narrow_sensor, allied_auto_precedence_minimize_noise);
            break;
        case 2:
            res.error = CHANGE_ALLIED_CAMERA_INTENSITY_AUTO_PRECEDENCE(devices[0], *m_allied_narrow_sensor, allied_auto_precedence_minimize_blur);
            break;
        }
        break;
    default:
        res.error = 1000;
    }
    return true;
}

bool enableAlliedCameraAutoWhiteBalance(l3cam_ros::EnableAlliedCameraAutoWhiteBalance::Request &req, l3cam_ros::EnableAlliedCameraAutoWhiteBalance::Response &res)
{
    switch (req.allied_type)
    {
    case 1:
        res.error = ENABLE_ALLIED_CAMERA_AUTO_WHITE_BALANCE(devices[0], *m_allied_wide_sensor, req.enabled);
        break;
    case 2:
        res.error = ENABLE_ALLIED_CAMERA_AUTO_WHITE_BALANCE(devices[0], *m_allied_narrow_sensor, req.enabled);
        break;
    default:
        res.error = 1000;
    }
    return true;
}

bool changeAlliedCameraBalanceRatioSelector(l3cam_ros::ChangeAlliedCameraBalanceRatioSelector::Request &req, l3cam_ros::ChangeAlliedCameraBalanceRatioSelector::Response &res)
{
    switch (req.allied_type)
    {
    case 1:
        switch(req.white_balance_ratio_selector){
        case 1:
            res.error = CHANGE_ALLIED_CAMERA_BALANCE_RATIO_SELECTOR(devices[0], *m_allied_wide_sensor, allied_balance_ratio_selector_red);
            break;
        case 2:
            res.error = CHANGE_ALLIED_CAMERA_BALANCE_RATIO_SELECTOR(devices[0], *m_allied_wide_sensor, allied_balance_ratio_selector_blue);
            break;
        }
        break;
    case 2:
        switch(req.white_balance_ratio_selector){
        case 1:
            res.error = CHANGE_ALLIED_CAMERA_BALANCE_RATIO_SELECTOR(devices[0], *m_allied_narrow_sensor, allied_balance_ratio_selector_red);
            break;
        case 2:
            res.error = CHANGE_ALLIED_CAMERA_BALANCE_RATIO_SELECTOR(devices[0], *m_allied_narrow_sensor, allied_balance_ratio_selector_blue);
            break;
        }
        break;
    default:
        res.error = 1000;
    }
    return true;
}

bool changeAlliedCameraBalanceRatio(l3cam_ros::ChangeAlliedCameraBalanceRatio::Request &req, l3cam_ros::ChangeAlliedCameraBalanceRatio::Response &res)
{
    switch (req.allied_type)
    {
    case 1:
        res.error = CHANGE_ALLIED_CAMERA_BALANCE_RATIO(devices[0], *m_allied_wide_sensor, req.balance_ratio);
        break;
    case 2:
        res.error = CHANGE_ALLIED_CAMERA_BALANCE_RATIO(devices[0], *m_allied_narrow_sensor, req.balance_ratio);
        break;
    default:
        res.error = 1000;
    }
    return true;
}

bool changeAlliedCameraBalanceWhiteAutoRate(l3cam_ros::ChangeAlliedCameraBalanceWhiteAutoRate::Request &req, l3cam_ros::ChangeAlliedCameraBalanceWhiteAutoRate::Response &res)
{
    switch (req.allied_type)
    {
    case 1:
        res.error = CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_RATE(devices[0], *m_allied_wide_sensor, req.white_balance_auto_rate);
        break;
    case 2:
        res.error = CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_RATE(devices[0], *m_allied_narrow_sensor, req.white_balance_auto_rate);
        break;
    default:
        res.error = 1000;
    }
    return true;
}

bool changeAlliedCameraBalanceWhiteAutoTolerance(l3cam_ros::ChangeAlliedCameraBalanceWhiteAutoTolerance::Request &req, l3cam_ros::ChangeAlliedCameraBalanceWhiteAutoTolerance::Response &res)
{
    switch (req.allied_type)
    {
    case 1:
        res.error = CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_TOLERANCE(devices[0], *m_allied_wide_sensor, req.white_balance_auto_tolerance);
        break;
    case 2:
        res.error = CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_TOLERANCE(devices[0], *m_allied_narrow_sensor, req.white_balance_auto_tolerance);
        break;
    default:
        res.error = 1000;
    }
    return true;
}

bool changeAlliedCameraAutoModeRegion(l3cam_ros::ChangeAlliedCameraAutoModeRegion::Request &req, l3cam_ros::ChangeAlliedCameraAutoModeRegion::Response &res)
{
    switch (req.allied_type)
    {
    case 1:
        res.error = CHANGE_ALLIED_CAMERA_AUTO_MODE_REGION(devices[0], *m_allied_wide_sensor, req.auto_mode_region_height, req.auto_mode_region_width);
        break;
    case 2:
        res.error = CHANGE_ALLIED_CAMERA_AUTO_MODE_REGION(devices[0], *m_allied_narrow_sensor, req.auto_mode_region_height, req.auto_mode_region_width);
        break;
    default:
        res.error = 1000; //! Can't put height and width
    }
    return true;
}

bool changeAlliedCameraIntensityControllerRegion(l3cam_ros::ChangeAlliedCameraIntensityControllerRegion::Request &req, l3cam_ros::ChangeAlliedCameraIntensityControllerRegion::Response &res)
{
    switch (req.allied_type)
    {
    case 1:
        switch(req.intensity_controller_region)
        {
        case 1:
            res.error = CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_REGION(devices[0], *m_allied_wide_sensor, allied_controller_region_auto_mode_region);
            break;
        case 2:
            res.error = CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_REGION(devices[0], *m_allied_wide_sensor, allied_controller_region_full_image);
            break;
        }
        break;
    case 2:
        switch(req.intensity_controller_region)
        {
        case 1:
            res.error = CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_REGION(devices[0], *m_allied_narrow_sensor, allied_controller_region_auto_mode_region);
            break;
        case 2:
            res.error = CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_REGION(devices[0], *m_allied_narrow_sensor, allied_controller_region_full_image);
            break;
        }
        break;
    default:
        res.error = 1000;
    }
    return true;
}

bool changeAlliedCameraIntensityControllerTarget(l3cam_ros::ChangeAlliedCameraIntensityControllerTarget::Request &req, l3cam_ros::ChangeAlliedCameraIntensityControllerTarget::Response &res)
{
    switch (req.allied_type)
    {
    case 1:
        res.error = CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_TARGET(devices[0], *m_allied_wide_sensor, req.intensity_controller_target);
        break;
    case 2:
        res.error = CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_TARGET(devices[0], *m_allied_narrow_sensor, req.intensity_controller_target);
        break;
    default:
        res.error = 1000;
    }
    return true;
}

bool changeAlliedCameraMaxDriverBuffersCount(l3cam_ros::ChangeAlliedCameraMaxDriverBuffersCount::Request &req, l3cam_ros::ChangeAlliedCameraMaxDriverBuffersCount::Response &res)
{
    switch (req.allied_type)
    {
    case 1:
        res.error = CHANGE_ALLIED_CAMERA_MAX_DRIVER_BUFFERS_COUNT(devices[0], *m_allied_wide_sensor, req.max_driver_buffers_count);
        break;
    case 2:
        res.error = CHANGE_ALLIED_CAMERA_MAX_DRIVER_BUFFERS_COUNT(devices[0], *m_allied_narrow_sensor, req.max_driver_buffers_count);
        break;
    default:
        res.error = 1000;
    }
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

    ros::ServiceServer srvChangeAlliedCameraBlackLevel = nh.advertiseService("change_allied_camera_black_level", changeAlliedCameraBlackLevel);
    ros::ServiceServer srvChangeAlliedCameraExposureTime = nh.advertiseService("change_allied_camera_exposure_time", changeAlliedCameraExposureTime);
    ros::ServiceServer srvEnableAlliedCameraAutoExposureTime = nh.advertiseService("enable_allied_camera_auto_exposure_time", enableAlliedCameraAutoExposureTime);
    ros::ServiceServer srvChangeAlliedCameraAutoExposureTimeRange = nh.advertiseService("change_allied_camera_auto_exposure_time_range", changeAlliedCameraAutoExposureTimeRange);
    ros::ServiceServer srvChangeAlliedCameraGain = nh.advertiseService("change_allied_camera_gain", changeAlliedCameraGain);
    ros::ServiceServer srvEnableAlliedCameraAutoGain = nh.advertiseService("enable_allied_camera_auto_gain", enableAlliedCameraAutoGain);
    ros::ServiceServer srvChangeAlliedCameraAutoGainRange = nh.advertiseService("change_allied_camera_auto_gain_range", changeAlliedCameraAutoGainRange);
    ros::ServiceServer srvChangeAlliedCameraGamma = nh.advertiseService("change_allied_camera_gamma", changeAlliedCameraGamma);
    ros::ServiceServer srvChangeAlliedCameraSaturation = nh.advertiseService("change_allied_camera_saturation", changeAlliedCameraSaturation);
    ros::ServiceServer srvChangeAlliedCameraSharpness = nh.advertiseService("change_allied_camera_sharpness", changeAlliedCameraSharpness);
    ros::ServiceServer srvChangeAlliedCameraHue = nh.advertiseService("change_allied_camera_hue", changeAlliedCameraHue);
    ros::ServiceServer srvChangeAlliedCameraIntensityAutoPrecedence = nh.advertiseService("change_allied_camera_intensity_auto_precedence", changeAlliedCameraIntensityAutoPrecedence);
    ros::ServiceServer srvEnableAlliedCameraAutoWhiteBalance = nh.advertiseService("enable_allied_camera_auto_white_balance", enableAlliedCameraAutoWhiteBalance);
    ros::ServiceServer srvChangeAlliedCameraBalanceRatioSelector = nh.advertiseService("change_allied_camera_balance_ratio_selector", changeAlliedCameraBalanceRatioSelector);
    ros::ServiceServer srvChangeAlliedCameraBalanceRatio = nh.advertiseService("change_allied_camera_balance_ratio", changeAlliedCameraBalanceRatio);
    ros::ServiceServer srvChangeAlliedCameraBalanceWhiteAutoRate = nh.advertiseService("change_allied_camera_balance_white_auto_rate", changeAlliedCameraBalanceWhiteAutoRate);
    ros::ServiceServer srvChangeAlliedCameraBalanceWhiteAutoTolerance = nh.advertiseService("change_allied_camera_balance_white_auto_tolerance", changeAlliedCameraBalanceWhiteAutoTolerance);
    ros::ServiceServer srvChangeAlliedCameraAutoModeRegion = nh.advertiseService("change_allied_camera_auto_mode_region", changeAlliedCameraAutoModeRegion);
    ros::ServiceServer srvChangeAlliedCameraIntensityControllerRegion = nh.advertiseService("change_allied_camera_intensity_controller_region", changeAlliedCameraIntensityControllerRegion);
    ros::ServiceServer srvChangeAlliedCameraIntensityControllerTarget = nh.advertiseService("change_allied_camera_intensity_controller_target", changeAlliedCameraIntensityControllerTarget);
    ros::ServiceServer srvChangeAlliedCameraMaxDriverBuffersCount = nh.advertiseService("change_allied_camera_max_driver_buffers_count", changeAlliedCameraMaxDriverBuffersCount);

    // Initialize L3Cam
    int error = L3CAM_OK;

    registerSystemSignals();

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
    for (int i = 0; i < num_sensors; ++i)
    {
        switch (av_sensors[i].sensor_type)
        {
        case sensor_lidar:
            m_lidar_sensor = &av_sensors[i];
            break;
        case sensor_econ_rgb:
            m_rgb_sensor = &av_sensors[i];
            break;
        case sensor_thermal:
            m_thermal_sensor = &av_sensors[i];
            break;
        case sensor_pol:
            m_polarimetric_sensor = &av_sensors[i];
            break;
        case sensor_allied_wide:
            m_allied_wide_sensor = &av_sensors[i];
            break;
        case sensor_allied_narrow:
            m_allied_narrow_sensor = &av_sensors[i];
            break;
        }
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

    return 0;
}