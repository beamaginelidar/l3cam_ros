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
#include "l3cam_ros/GetSensorsAvailable.h"
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

#include "l3cam_ros/ChangeAlliedCameraExposureTime.h"
#include "l3cam_ros/EnableAlliedCameraAutoExposureTime.h"
#include "l3cam_ros/ChangeAlliedCameraAutoExposureTimeRange.h"
#include "l3cam_ros/ChangeAlliedCameraGain.h"
#include "l3cam_ros/EnableAlliedCameraAutoGain.h"
#include "l3cam_ros/ChangeAlliedCameraAutoGainRange.h"
#include "l3cam_ros/ChangeAlliedCameraGamma.h"
#include "l3cam_ros/ChangeAlliedCameraSaturation.h"
#include "l3cam_ros/ChangeAlliedCameraHue.h"
#include "l3cam_ros/ChangeAlliedCameraIntensityAutoPrecedence.h"
#include "l3cam_ros/EnableAlliedCameraAutoWhiteBalance.h"
#include "l3cam_ros/ChangeAlliedCameraBalanceRatioSelector.h"
#include "l3cam_ros/ChangeAlliedCameraBalanceRatio.h"
#include "l3cam_ros/ChangeAlliedCameraBalanceWhiteAutoRate.h"
#include "l3cam_ros/ChangeAlliedCameraBalanceWhiteAutoTolerance.h"
#include "l3cam_ros/ChangeAlliedCameraIntensityControllerRegion.h"
#include "l3cam_ros/ChangeAlliedCameraIntensityControllerTarget.h"

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

// Services
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

bool getSensorsAvailable(l3cam_ros::GetSensorsAvailable::Request &req, l3cam_ros::GetSensorsAvailable::Response &res)
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
    /*if (req.enable_dhcp)
        res.error = CHANGE_NETWORK_CONFIGURATION(devices[0], NULL, NULL, NULL, true);
    else
    {
        std::string ip_address = req.ip_address;
        std::string netmask = req.netmask;
        std::string gateway = req.gateway;
        res.error = CHANGE_NETWORK_CONFIGURATION(devices[0], (char *)ip_address.data(), (char *)netmask.data(), (char *)gateway.data(), false);
    }

    // TODO: Terminate, Initialize, etc.
*/
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
        res.error = L3CAM_VALUE_OUT_OF_RANGE;
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
        res.error = L3CAM_VALUE_OUT_OF_RANGE;
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
        res.error = L3CAM_VALUE_OUT_OF_RANGE; //! Can't put max and min
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
        res.error = L3CAM_VALUE_OUT_OF_RANGE;
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
        res.error = L3CAM_VALUE_OUT_OF_RANGE;
    }
    return true;
}

bool changeAlliedCameraAutoGainRange(l3cam_ros::ChangeAlliedCameraAutoGainRange::Request &req, l3cam_ros::ChangeAlliedCameraAutoGainRange::Response &res)
{
    switch (req.allied_type)
    {
    case 1:
        res.error = CHANGE_ALLIED_CAMERA_AUTO_GAIN_RANGE(devices[0], *m_allied_wide_sensor, (float)req.auto_gain_range_min, (float)req.auto_gain_range_max);
        break;
    case 2:
        res.error = CHANGE_ALLIED_CAMERA_AUTO_GAIN_RANGE(devices[0], *m_allied_narrow_sensor, (float)req.auto_gain_range_min, (float)req.auto_gain_range_max);
        break;
    default:
        res.error = L3CAM_VALUE_OUT_OF_RANGE;
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
        res.error = L3CAM_VALUE_OUT_OF_RANGE;
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
        res.error = L3CAM_VALUE_OUT_OF_RANGE;
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
        res.error = L3CAM_VALUE_OUT_OF_RANGE;
    }
    return true;
}

bool changeAlliedCameraIntensityAutoPrecedence(l3cam_ros::ChangeAlliedCameraIntensityAutoPrecedence::Request &req, l3cam_ros::ChangeAlliedCameraIntensityAutoPrecedence::Response &res)
{
    switch (req.allied_type)
    {
    case 1:
        res.error = CHANGE_ALLIED_CAMERA_INTENSITY_AUTO_PRECEDENCE(devices[0], *m_allied_wide_sensor, req.intensity_auto_precedence);
        break;
    case 2:
        res.error = CHANGE_ALLIED_CAMERA_INTENSITY_AUTO_PRECEDENCE(devices[0], *m_allied_narrow_sensor, req.intensity_auto_precedence);
        break;
    default:
        res.error = L3CAM_VALUE_OUT_OF_RANGE;
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
        res.error = L3CAM_VALUE_OUT_OF_RANGE;
    }
    return true;
}

bool changeAlliedCameraBalanceRatioSelector(l3cam_ros::ChangeAlliedCameraBalanceRatioSelector::Request &req, l3cam_ros::ChangeAlliedCameraBalanceRatioSelector::Response &res)
{
    switch (req.allied_type)
    {
    case 1:
        res.error = CHANGE_ALLIED_CAMERA_BALANCE_RATIO_SELECTOR(devices[0], *m_allied_wide_sensor, req.white_balance_ratio_selector);
        break;
    case 2:
        res.error = CHANGE_ALLIED_CAMERA_BALANCE_RATIO_SELECTOR(devices[0], *m_allied_narrow_sensor, req.white_balance_ratio_selector);
        break;
    default:
        res.error = L3CAM_VALUE_OUT_OF_RANGE;
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
        res.error = L3CAM_VALUE_OUT_OF_RANGE;
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
        res.error = L3CAM_VALUE_OUT_OF_RANGE;
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
        res.error = L3CAM_VALUE_OUT_OF_RANGE;
    }
    return true;
}

bool changeAlliedCameraIntensityControllerRegion(l3cam_ros::ChangeAlliedCameraIntensityControllerRegion::Request &req, l3cam_ros::ChangeAlliedCameraIntensityControllerRegion::Response &res)
{
    switch (req.allied_type)
    {
    case 1:
        res.error = CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_REGION(devices[0], *m_allied_wide_sensor, req.intensity_controller_region);
        break;
    case 2:
        res.error = CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_REGION(devices[0], *m_allied_narrow_sensor, req.intensity_controller_region);
        break;
    default:
        res.error = L3CAM_VALUE_OUT_OF_RANGE;
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
        res.error = L3CAM_VALUE_OUT_OF_RANGE;
    }
    return true;
}

void printDefaultError(int error)
{
    if (error != L3CAM_OK)
    {
        ROS_ERROR_STREAM("ERROR SETTING DEFAULT PARAMETER (" << error << ") " << getBeamErrorDescription(error));
    }
}

void loadDefaultParams(ros::NodeHandle nh)
{
    if (m_lidar_sensor != NULL)
    {
        int pointcloud_color;
        nh.param("/l3cam_ros_node/pointcloud_color", pointcloud_color, 0);
        printDefaultError(CHANGE_POINT_CLOUD_COLOR(devices[0], pointcloud_color));
        int pointcloud_color_range_minimum;
        nh.param("/l3cam_ros_node/pointcloud_color_range_minimum", pointcloud_color_range_minimum, 0);
        int pointcloud_color_range_maximum;
        nh.param("/l3cam_ros_node/pointcloud_color_range_maximum", pointcloud_color_range_maximum, 400000);
        printDefaultError(CHANGE_POINT_CLOUD_COLOR_RANGE(devices[0], pointcloud_color_range_minimum, pointcloud_color_range_maximum));
        int distance_range_minimum;
        nh.param("/l3cam_ros_node/distance_range_minimum", distance_range_minimum, 0);
        int distance_range_maximum;
        nh.param("/l3cam_ros_node/distance_range_maximum", distance_range_maximum, 400000);
        printDefaultError(CHANGE_DISTANCE_RANGE(devices[0], distance_range_minimum, distance_range_maximum));
    }
    if (m_polarimetric_sensor != NULL)
    {
        int polarimetric_camera_brightness;
        nh.param("/l3cam_ros_node/polarimetric_camera_brightness", polarimetric_camera_brightness, 127);
        printDefaultError(CHANGE_POLARIMETRIC_CAMERA_BRIGHTNESS(devices[0], polarimetric_camera_brightness));
        double polarimetric_camera_black_level;
        nh.param("/l3cam_ros_node/polarimetric_camera_black_level", polarimetric_camera_black_level, 6.0);
        printDefaultError(CHANGE_POLARIMETRIC_CAMERA_BLACK_LEVEL(devices[0], polarimetric_camera_black_level));
        bool polarimetric_camera_auto_gain;
        nh.param("/l3cam_ros_node/polarimetric_camera_auto_gain", polarimetric_camera_auto_gain, true);
        printDefaultError(ENABLE_POLARIMETRIC_CAMERA_AUTO_GAIN(devices[0], polarimetric_camera_auto_gain));
        if(polarimetric_camera_auto_gain)
        { //! Values could not coincide when enabling polarimetric_camera_auto_gain
            double polarimetric_camera_auto_gain_range_minimum;
            nh.param("/l3cam_ros_node/polarimetric_camera_auto_gain_range_minimum", polarimetric_camera_auto_gain_range_minimum, 0.0);
            double polarimetric_camera_auto_gain_range_maximum;
            nh.param("/l3cam_ros_node/polarimetric_camera_auto_gain_range_maximum", polarimetric_camera_auto_gain_range_maximum, 48.0);
            printDefaultError(CHANGE_POLARIMETRIC_CAMERA_AUTO_GAIN_RANGE(devices[0], polarimetric_camera_auto_gain_range_minimum, polarimetric_camera_auto_gain_range_maximum));
        }
        else
        { //! Values could not coincide when disabling polarimetric_camera_auto_gain
            double polarimetric_camera_gain;
            nh.param("/l3cam_ros_node/polarimetric_camera_gain", polarimetric_camera_gain, 24.0);
            printDefaultError(CHANGE_POLARIMETRIC_CAMERA_GAIN(devices[0], polarimetric_camera_gain));
        }
        bool polarimetric_camera_auto_exposure_time;
        nh.param("/l3cam_ros_node/polarimetric_camera_auto_exposure_time", polarimetric_camera_auto_exposure_time, true);
        printDefaultError(ENABLE_POLARIMETRIC_CAMERA_AUTO_EXPOSURE_TIME(devices[0], polarimetric_camera_auto_exposure_time));
        if(polarimetric_camera_auto_exposure_time)
        { //! Values could not coincide when enabling polarimetric_camera_auto_exposure_time
            double polarimetric_camera_auto_exposure_time_range_minimum;
            nh.param("/l3cam_ros_node/polarimetric_camera_auto_exposure_time_range_minimum", polarimetric_camera_auto_exposure_time_range_minimum, 33.456);
            double polarimetric_camera_auto_exposure_time_range_maximum;
            nh.param("/l3cam_ros_node/polarimetric_camera_auto_exposure_time_range_maximum", polarimetric_camera_auto_exposure_time_range_maximum, 66470.6);
            printDefaultError(CHANGE_POLARIMETRIC_CAMERA_AUTO_EXPOSURE_TIME_RANGE(devices[0], polarimetric_camera_auto_exposure_time_range_minimum, polarimetric_camera_auto_exposure_time_range_maximum));
        }
        else
        { //! Values could not coincide when disabling polarimetric_camera_auto_exposure_time
            double polarimetric_camera_exposure_time;
            nh.param("/l3cam_ros_node/polarimetric_camera_exposure_time", polarimetric_camera_exposure_time, 500000.0);
            printDefaultError(CHANGE_POLARIMETRIC_CAMERA_EXPOSURE_TIME(devices[0], polarimetric_camera_exposure_time));
        }
    }

    if (m_rgb_sensor)
    {
        int rgb_camera_brightness;
        nh.param("/l3cam_ros_node/rgb_camera_brightness", rgb_camera_brightness, 0);
        printDefaultError(CHANGE_RGB_CAMERA_BRIGHTNESS(devices[0], rgb_camera_brightness));
        int rgb_camera_contrast;
        nh.param("/l3cam_ros_node/rgb_camera_contrast", rgb_camera_contrast, 10);
        printDefaultError(CHANGE_RGB_CAMERA_CONTRAST(devices[0], rgb_camera_contrast));
        int rgb_camera_saturation;
        nh.param("/l3cam_ros_node/rgb_camera_saturation", rgb_camera_saturation, 16);
        printDefaultError(CHANGE_RGB_CAMERA_SATURATION(devices[0], rgb_camera_saturation));
        int rgb_camera_sharpness;
        nh.param("/l3cam_ros_node/rgb_camera_sharpness", rgb_camera_sharpness, 16);
        printDefaultError(CHANGE_RGB_CAMERA_SHARPNESS(devices[0], rgb_camera_sharpness));
        int rgb_camera_gamma;
        nh.param("/l3cam_ros_node/rgb_camera_gamma", rgb_camera_gamma, 220);
        printDefaultError(CHANGE_RGB_CAMERA_GAMMA(devices[0], rgb_camera_gamma));
        int rgb_camera_gain;
        nh.param("/l3cam_ros_node/rgb_camera_gain", rgb_camera_gain, 0);
        printDefaultError(CHANGE_RGB_CAMERA_GAIN(devices[0], rgb_camera_gain));
        bool rgb_camera_auto_white_balance;
        nh.param("/l3cam_ros_node/rgb_camera_auto_white_balance", rgb_camera_auto_white_balance, true);
        printDefaultError(ENABLE_RGB_CAMERA_AUTO_WHITE_BALANCE(devices[0], rgb_camera_auto_white_balance));
        if(!rgb_camera_auto_white_balance)
        { //! Values could not coincide when disabling rgb_camera_auto_white_balance
            int rgb_camera_white_balance;
            nh.param("/l3cam_ros_node/rgb_camera_white_balance", rgb_camera_white_balance, 5000);
            printDefaultError(CHANGE_RGB_CAMERA_WHITE_BALANCE(devices[0], rgb_camera_white_balance));
        }
        bool rgb_camera_auto_exposure_time;
        nh.param("/l3cam_ros_node/rgb_camera_auto_exposure_time", rgb_camera_auto_exposure_time, true);
        printDefaultError(ENABLE_RGB_CAMERA_AUTO_EXPOSURE_TIME(devices[0], rgb_camera_auto_exposure_time));
        if(!rgb_camera_auto_exposure_time)
        { //! Values could not coincide when disabling rgb_camera_auto_exposure_time
            int rgb_camera_exposure_time;
            nh.param("/l3cam_ros_node/rgb_camera_exposure_time", rgb_camera_exposure_time, 156);
            printDefaultError(CHANGE_RGB_CAMERA_EXPOSURE_TIME(devices[0], rgb_camera_exposure_time));
        }
        int rgb_camera_resolution;
        nh.param("l3cam_ros_node/rgb_camera_resolution", rgb_camera_resolution, 3);
        printDefaultError(CHANGE_RGB_CAMERA_RESOLUTION(devices[0], (econResolutions)rgb_camera_resolution));
        int rgb_camera_framerate;
        nh.param("l3cam_ros_node/rgb_camera_framerate", rgb_camera_framerate, 10);
        printDefaultError(CHANGE_RGB_CAMERA_FRAMERATE(devices[0], rgb_camera_framerate));
    }

    if (m_thermal_sensor != NULL)
    {
        int thermal_camera_colormap;
        nh.param("/l3cam_ros_node/thermal_camera_colormap", thermal_camera_colormap, 1);
        printDefaultError(CHANGE_THERMAL_CAMERA_COLORMAP(devices[0], (thermalTypes)thermal_camera_colormap));
        bool thermal_camera_temperature_filter;
        nh.param("/l3cam_ros_node/thermal_camera_temperature_filter", thermal_camera_temperature_filter, false);
        printDefaultError(ENABLE_THERMAL_CAMERA_TEMPERATURE_FILTER(devices[0], thermal_camera_temperature_filter));
        int thermal_camera_temperature_filter_min;
        nh.param("/l3cam_ros_node/thermal_camera_temperature_filter_min", thermal_camera_temperature_filter_min, 0);
        int thermal_camera_temperature_filter_max;
        nh.param("/l3cam_ros_node/thermal_camera_temperature_filter_max", thermal_camera_temperature_filter_max, 50);
        printDefaultError(CHANGE_THERMAL_CAMERA_TEMPERATURE_FILTER(devices[0], thermal_camera_temperature_filter_min, thermal_camera_temperature_filter_max));
    }


    if (m_allied_wide_sensor != NULL)
    {
        double allied_wide_camera_black_level;
        nh.param("/l3cam_ros_node/allied_wide_camera_black_level", allied_wide_camera_black_level, 0.);
        printDefaultError(CHANGE_ALLIED_CAMERA_BLACK_LEVEL(devices[0], *m_allied_wide_sensor, allied_wide_camera_black_level));
        bool allied_wide_camera_auto_exposure_time;
        nh.param("/l3cam_ros_node/allied_wide_camera_auto_exposure_time", allied_wide_camera_auto_exposure_time, false);
        printDefaultError(ENABLE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME(devices[0], *m_allied_wide_sensor, allied_wide_camera_auto_exposure_time));
        if(allied_wide_camera_auto_exposure_time)
        { //! Values could not coincide when enabling allied_wide_camera_auto_exposure_time
            double allied_wide_camera_auto_exposure_time_range_min;
            nh.param("/l3cam_ros_node/allied_wide_camera_auto_exposure_time_range_min", allied_wide_camera_auto_exposure_time_range_min, 87.596);
            double allied_wide_camera_auto_exposure_time_range_max;
            nh.param("/l3cam_ros_node/allied_wide_camera_auto_exposure_time_range_max", allied_wide_camera_auto_exposure_time_range_max, 87.596);
            printDefaultError(CHANGE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME_RANGE(devices[0], *m_allied_wide_sensor, allied_wide_camera_auto_exposure_time_range_min, allied_wide_camera_auto_exposure_time_range_max));
        }
        else
        { //! Values could not coincide when disabling allied_wide_camera_auto_exposure_time
            double allied_wide_camera_exposure_time;
            nh.param("/l3cam_ros_node/allied_wide_camera_exposure_time", allied_wide_camera_exposure_time, 4992.32);
            printDefaultError(CHANGE_ALLIED_CAMERA_EXPOSURE_TIME_US(devices[0], *m_allied_wide_sensor, allied_wide_camera_exposure_time));
        }
        bool allied_wide_camera_auto_gain;
        nh.param("/l3cam_ros_node/allied_wide_camera_auto_gain", allied_wide_camera_auto_gain, false);
        printDefaultError(ENABLE_ALLIED_CAMERA_AUTO_GAIN(devices[0], *m_allied_wide_sensor, allied_wide_camera_auto_gain));
        if(allied_wide_camera_auto_gain)
        { //! Values could not coincide when enabling allied_wide_camera_auto_gain
            double allied_wide_camera_auto_gain_range_min;
            nh.param("/l3cam_ros_node/allied_wide_camera_auto_gain_range_min", allied_wide_camera_auto_gain_range_min, 0.);
            double allied_wide_camera_auto_gain_range_max;
            nh.param("/l3cam_ros_node/allied_wide_camera_auto_gain_range_max", allied_wide_camera_auto_gain_range_max, 48.);
            printDefaultError(CHANGE_ALLIED_CAMERA_AUTO_GAIN_RANGE(devices[0], *m_allied_wide_sensor, (float)allied_wide_camera_auto_gain_range_min, (float)allied_wide_camera_auto_gain_range_max));
        }
        else
        { //! Values could not coincide when disabling allied_wide_camera_auto_gain
            double allied_wide_camera_gain;
            nh.param("/l3cam_ros_node/allied_wide_camera_gain", allied_wide_camera_gain, 0.);
            printDefaultError(CHANGE_ALLIED_CAMERA_GAIN(devices[0], *m_allied_wide_sensor, allied_wide_camera_gain));
        }
        double allied_wide_camera_gamma;
        nh.param("/l3cam_ros_node/allied_wide_camera_gamma", allied_wide_camera_gamma, 1.);
        printDefaultError(CHANGE_ALLIED_CAMERA_GAMMA(devices[0], *m_allied_wide_sensor, allied_wide_camera_gamma));
        double allied_wide_camera_saturation;
        nh.param("/l3cam_ros_node/allied_wide_camera_saturation", allied_wide_camera_saturation, 1.);
        printDefaultError(CHANGE_ALLIED_CAMERA_SATURATION(devices[0], *m_allied_wide_sensor, allied_wide_camera_saturation));
        double allied_wide_camera_sharpness;
        nh.param("/l3cam_ros_node/allied_wide_camera_sharpness", allied_wide_camera_sharpness, 0.);
        printDefaultError(CHANGE_ALLIED_CAMERA_SHARPNESS(devices[0], *m_allied_wide_sensor, allied_wide_camera_sharpness));
        double allied_wide_camera_hue;
        nh.param("/l3cam_ros_node/allied_wide_camera_hue", allied_wide_camera_hue, 0.);
        printDefaultError(CHANGE_ALLIED_CAMERA_HUE(devices[0], *m_allied_wide_sensor, allied_wide_camera_hue));
        int allied_wide_camera_intensity_auto_precedence;
        nh.param("/l3cam_ros_node/allied_wide_camera_intensity_auto_precedence", allied_wide_camera_intensity_auto_precedence, 0);
        printDefaultError(CHANGE_ALLIED_CAMERA_INTENSITY_AUTO_PRECEDENCE(devices[0], *m_allied_wide_sensor, allied_wide_camera_intensity_auto_precedence));
        bool allied_wide_camera_auto_white_balance;
        nh.param("/l3cam_ros_node/allied_wide_camera_auto_white_balance", allied_wide_camera_auto_white_balance, false);
        printDefaultError(ENABLE_ALLIED_CAMERA_AUTO_WHITE_BALANCE(devices[0], *m_allied_wide_sensor, allied_wide_camera_auto_white_balance));
        int allied_wide_camera_balance_ratio_selector;
        nh.param("/l3cam_ros_node/allied_wide_camera_balance_ratio_selector", allied_wide_camera_balance_ratio_selector, 0);
        printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_RATIO_SELECTOR(devices[0], *m_allied_wide_sensor, allied_wide_camera_balance_ratio_selector));
        double allied_wide_camera_balance_ratio;
        nh.param("/l3cam_ros_node/allied_wide_camera_balance_ratio", allied_wide_camera_balance_ratio, 2.35498);
        printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_RATIO(devices[0], *m_allied_wide_sensor, allied_wide_camera_balance_ratio));
        double allied_wide_camera_balance_white_auto_rate;
        nh.param("/l3cam_ros_node/allied_wide_camera_balance_white_auto_rate", allied_wide_camera_balance_white_auto_rate, 100.);
        printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_RATE(devices[0], *m_allied_wide_sensor, allied_wide_camera_balance_white_auto_rate));
        double allied_wide_camera_balance_white_auto_tolerance;
        nh.param("/l3cam_ros_node/allied_wide_camera_balance_white_auto_tolerance", allied_wide_camera_balance_white_auto_tolerance, 5.);
        printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_TOLERANCE(devices[0], *m_allied_wide_sensor, allied_wide_camera_balance_white_auto_tolerance));
        int allied_wide_camera_auto_mode_region_height;
        nh.param("/l3cam_ros_node/allied_wide_camera_auto_mode_region_height", allied_wide_camera_auto_mode_region_height, 2056);
        int allied_wide_camera_auto_mode_region_width;
        nh.param("/l3cam_ros_node/allied_wide_camera_auto_mode_region_width", allied_wide_camera_auto_mode_region_width, 2464);
        printDefaultError(CHANGE_ALLIED_CAMERA_AUTO_MODE_REGION(devices[0], *m_allied_wide_sensor, allied_wide_camera_auto_mode_region_height, allied_wide_camera_auto_mode_region_width));
        int allied_wide_camera_intensity_controller_region;
        nh.param("/l3cam_ros_node/allied_wide_camera_intensity_controller_region", allied_wide_camera_intensity_controller_region, 0);
        printDefaultError(CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_REGION(devices[0], *m_allied_wide_sensor, allied_wide_camera_intensity_controller_region));
        double allied_wide_camera_intensity_controller_target;
        nh.param("/l3cam_ros_node/allied_wide_camera_intensity_controller_target", allied_wide_camera_intensity_controller_target, 50.);
        printDefaultError(CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_TARGET(devices[0], *m_allied_wide_sensor, allied_wide_camera_intensity_controller_target));
        int allied_wide_camera_max_driver_buffers_count;
        nh.param("/l3cam_ros_node/allied_wide_camera_max_driver_buffers_count", allied_wide_camera_max_driver_buffers_count, 64);
        printDefaultError(CHANGE_ALLIED_CAMERA_MAX_DRIVER_BUFFERS_COUNT(devices[0], *m_allied_wide_sensor, allied_wide_camera_max_driver_buffers_count));
    }

    if (m_allied_narrow_sensor != NULL)
    {
        double allied_narrow_camera_black_level;
        nh.param("/l3cam_ros_node/allied_narrow_camera_black_level", allied_narrow_camera_black_level, 0.);
        printDefaultError(CHANGE_ALLIED_CAMERA_BLACK_LEVEL(devices[0], *m_allied_narrow_sensor, allied_narrow_camera_black_level));
        bool allied_narrow_camera_auto_exposure_time;
        nh.param("/l3cam_ros_node/allied_narrow_camera_auto_exposure_time", allied_narrow_camera_auto_exposure_time, false);
        printDefaultError(ENABLE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME(devices[0], *m_allied_narrow_sensor, allied_narrow_camera_auto_exposure_time));
        if(allied_narrow_camera_auto_exposure_time)
        { //! Values could not coincide when enabling allied_narrow_camera_auto_exposure_time
            double allied_narrow_camera_auto_exposure_time_range_min;
            nh.param("/l3cam_ros_node/allied_narrow_camera_auto_exposure_time_range_min", allied_narrow_camera_auto_exposure_time_range_min, 87.596);
            double allied_narrow_camera_auto_exposure_time_range_max;
            nh.param("/l3cam_ros_node/allied_narrow_camera_auto_exposure_time_range_max", allied_narrow_camera_auto_exposure_time_range_max, 87.596);
            printDefaultError(CHANGE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME_RANGE(devices[0], *m_allied_narrow_sensor, allied_narrow_camera_auto_exposure_time_range_min, allied_narrow_camera_auto_exposure_time_range_max));
        }
        else
        { //! Values could not coincide when disabling allied_narrow_camera_auto_exposure_time
            double allied_narrow_camera_exposure_time;
            nh.param("/l3cam_ros_node/allied_narrow_camera_exposure_time", allied_narrow_camera_exposure_time, 4992.32);
            printDefaultError(CHANGE_ALLIED_CAMERA_EXPOSURE_TIME_US(devices[0], *m_allied_narrow_sensor, allied_narrow_camera_exposure_time));
        }
        bool allied_narrow_camera_auto_gain;
        nh.param("/l3cam_ros_node/allied_narrow_camera_auto_gain", allied_narrow_camera_auto_gain, false);
        printDefaultError(ENABLE_ALLIED_CAMERA_AUTO_GAIN(devices[0], *m_allied_narrow_sensor, allied_narrow_camera_auto_gain));
        if(allied_narrow_camera_auto_gain)
        { //! Values could not coincide when enabling allied_narrow_camera_auto_gain
            double allied_narrow_camera_auto_gain_range_min;
            nh.param("/l3cam_ros_node/allied_narrow_camera_auto_gain_range_min", allied_narrow_camera_auto_gain_range_min, 0.);
            double allied_narrow_camera_auto_gain_range_max;
            nh.param("/l3cam_ros_node/allied_narrow_camera_auto_gain_range_max", allied_narrow_camera_auto_gain_range_max, 48.);
            printDefaultError(CHANGE_ALLIED_CAMERA_AUTO_GAIN_RANGE(devices[0], *m_allied_narrow_sensor, (float)allied_narrow_camera_auto_gain_range_min, (float)allied_narrow_camera_auto_gain_range_max));
        }
        else
        { //! Values could not coincide when disabling allied_narrow_camera_auto_gain
            double allied_narrow_camera_gain;
            nh.param("/l3cam_ros_node/allied_narrow_camera_gain", allied_narrow_camera_gain, 0.);
            printDefaultError(CHANGE_ALLIED_CAMERA_GAIN(devices[0], *m_allied_narrow_sensor, allied_narrow_camera_gain));
        }
        double allied_narrow_camera_gamma;
        nh.param("/l3cam_ros_node/allied_narrow_camera_gamma", allied_narrow_camera_gamma, 1.);
        printDefaultError(CHANGE_ALLIED_CAMERA_GAMMA(devices[0], *m_allied_narrow_sensor, allied_narrow_camera_gamma));
        double allied_narrow_camera_saturation;
        nh.param("/l3cam_ros_node/allied_narrow_camera_saturation", allied_narrow_camera_saturation, 1.);
        printDefaultError(CHANGE_ALLIED_CAMERA_SATURATION(devices[0], *m_allied_narrow_sensor, allied_narrow_camera_saturation));
        double allied_narrow_camera_sharpness;
        nh.param("/l3cam_ros_node/allied_narrow_camera_sharpness", allied_narrow_camera_sharpness, 0.);
        printDefaultError(CHANGE_ALLIED_CAMERA_SHARPNESS(devices[0], *m_allied_narrow_sensor, allied_narrow_camera_sharpness));
        double allied_narrow_camera_hue;
        nh.param("/l3cam_ros_node/allied_narrow_camera_hue", allied_narrow_camera_hue, 0.);
        printDefaultError(CHANGE_ALLIED_CAMERA_HUE(devices[0], *m_allied_narrow_sensor, allied_narrow_camera_hue));
        int allied_narrow_camera_intensity_auto_precedence;
        nh.param("/l3cam_ros_node/allied_narrow_camera_intensity_auto_precedence", allied_narrow_camera_intensity_auto_precedence, 0);
        printDefaultError(CHANGE_ALLIED_CAMERA_INTENSITY_AUTO_PRECEDENCE(devices[0], *m_allied_narrow_sensor, allied_narrow_camera_intensity_auto_precedence));
        bool allied_narrow_camera_auto_white_balance;
        nh.param("/l3cam_ros_node/allied_narrow_camera_auto_white_balance", allied_narrow_camera_auto_white_balance, false);
        printDefaultError(ENABLE_ALLIED_CAMERA_AUTO_WHITE_BALANCE(devices[0], *m_allied_narrow_sensor, allied_narrow_camera_auto_white_balance));
        int allied_narrow_camera_balance_ratio_selector;
        nh.param("/l3cam_ros_node/allied_narrow_camera_balance_ratio_selector", allied_narrow_camera_balance_ratio_selector, 0);
        printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_RATIO_SELECTOR(devices[0], *m_allied_narrow_sensor, allied_narrow_camera_balance_ratio_selector));
        double allied_narrow_camera_balance_ratio;
        nh.param("/l3cam_ros_node/allied_narrow_camera_balance_ratio", allied_narrow_camera_balance_ratio, 2.35498);
        printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_RATIO(devices[0], *m_allied_narrow_sensor, allied_narrow_camera_balance_ratio));
        double allied_narrow_camera_balance_white_auto_rate;
        nh.param("/l3cam_ros_node/allied_narrow_camera_balance_white_auto_rate", allied_narrow_camera_balance_white_auto_rate, 100.);
        printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_RATE(devices[0], *m_allied_narrow_sensor, allied_narrow_camera_balance_white_auto_rate));
        double allied_narrow_camera_balance_white_auto_tolerance;
        nh.param("/l3cam_ros_node/allied_narrow_camera_balance_white_auto_tolerance", allied_narrow_camera_balance_white_auto_tolerance, 5.);
        printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_TOLERANCE(devices[0], *m_allied_narrow_sensor, allied_narrow_camera_balance_white_auto_tolerance));
        int allied_narrow_camera_auto_mode_region_height;
        nh.param("/l3cam_ros_node/allied_narrow_camera_auto_mode_region_height", allied_narrow_camera_auto_mode_region_height, 2056);
        int allied_narrow_camera_auto_mode_region_width;
        nh.param("/l3cam_ros_node/allied_narrow_camera_auto_mode_region_width", allied_narrow_camera_auto_mode_region_width, 2464);
        printDefaultError(CHANGE_ALLIED_CAMERA_AUTO_MODE_REGION(devices[0], *m_allied_narrow_sensor, allied_narrow_camera_auto_mode_region_height, allied_narrow_camera_auto_mode_region_width));
        int allied_narrow_camera_intensity_controller_region;
        nh.param("/l3cam_ros_node/allied_narrow_camera_intensity_controller_region", allied_narrow_camera_intensity_controller_region, 0);
        printDefaultError(CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_REGION(devices[0], *m_allied_narrow_sensor, allied_narrow_camera_intensity_controller_region));
        double allied_narrow_camera_intensity_controller_target;
        nh.param("/l3cam_ros_node/allied_narrow_camera_intensity_controller_target", allied_narrow_camera_intensity_controller_target, 50.);
        printDefaultError(CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_TARGET(devices[0], *m_allied_narrow_sensor, allied_narrow_camera_intensity_controller_target));
        int allied_narrow_camera_max_driver_buffers_count;
        nh.param("/l3cam_ros_node/allied_narrow_camera_max_driver_buffers_count", allied_narrow_camera_max_driver_buffers_count, 64);
        printDefaultError(CHANGE_ALLIED_CAMERA_MAX_DRIVER_BUFFERS_COUNT(devices[0], *m_allied_narrow_sensor, allied_narrow_camera_max_driver_buffers_count));
    }
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
    ros::ServiceServer srvGetSensorsAvailable = nh.advertiseService("get_sensors_available", getSensorsAvailable);
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

    ros::ServiceServer srvChangeAlliedCameraExposureTime = nh.advertiseService("change_allied_camera_exposure_time", changeAlliedCameraExposureTime);
    ros::ServiceServer srvEnableAlliedCameraAutoExposureTime = nh.advertiseService("enable_allied_camera_auto_exposure_time", enableAlliedCameraAutoExposureTime);
    ros::ServiceServer srvChangeAlliedCameraAutoExposureTimeRange = nh.advertiseService("change_allied_camera_auto_exposure_time_range", changeAlliedCameraAutoExposureTimeRange);
    ros::ServiceServer srvChangeAlliedCameraGain = nh.advertiseService("change_allied_camera_gain", changeAlliedCameraGain);
    ros::ServiceServer srvEnableAlliedCameraAutoGain = nh.advertiseService("enable_allied_camera_auto_gain", enableAlliedCameraAutoGain);
    ros::ServiceServer srvChangeAlliedCameraAutoGainRange = nh.advertiseService("change_allied_camera_auto_gain_range", changeAlliedCameraAutoGainRange);
    ros::ServiceServer srvChangeAlliedCameraGamma = nh.advertiseService("change_allied_camera_gamma", changeAlliedCameraGamma);
    ros::ServiceServer srvChangeAlliedCameraSaturation = nh.advertiseService("change_allied_camera_saturation", changeAlliedCameraSaturation);
    ros::ServiceServer srvChangeAlliedCameraHue = nh.advertiseService("change_allied_camera_hue", changeAlliedCameraHue);
    ros::ServiceServer srvChangeAlliedCameraIntensityAutoPrecedence = nh.advertiseService("change_allied_camera_intensity_auto_precedence", changeAlliedCameraIntensityAutoPrecedence);
    ros::ServiceServer srvEnableAlliedCameraAutoWhiteBalance = nh.advertiseService("enable_allied_camera_auto_white_balance", enableAlliedCameraAutoWhiteBalance);
    ros::ServiceServer srvChangeAlliedCameraBalanceRatioSelector = nh.advertiseService("change_allied_camera_balance_ratio_selector", changeAlliedCameraBalanceRatioSelector);
    ros::ServiceServer srvChangeAlliedCameraBalanceRatio = nh.advertiseService("change_allied_camera_balance_ratio", changeAlliedCameraBalanceRatio);
    ros::ServiceServer srvChangeAlliedCameraBalanceWhiteAutoRate = nh.advertiseService("change_allied_camera_balance_white_auto_rate", changeAlliedCameraBalanceWhiteAutoRate);
    ros::ServiceServer srvChangeAlliedCameraBalanceWhiteAutoTolerance = nh.advertiseService("change_allied_camera_balance_white_auto_tolerance", changeAlliedCameraBalanceWhiteAutoTolerance);
    ros::ServiceServer srvChangeAlliedCameraIntensityControllerRegion = nh.advertiseService("change_allied_camera_intensity_controller_region", changeAlliedCameraIntensityControllerRegion);
    ros::ServiceServer srvChangeAlliedCameraIntensityControllerTarget = nh.advertiseService("change_allied_camera_intensity_controller_target", changeAlliedCameraIntensityControllerTarget);

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
        ROS_ERROR_STREAM("(GET_SENSORS_AVAILABLE error " << error << ") " << getBeamErrorDescription(error));
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
    ROS_INFO_STREAM(num_sensors << " sensors available");

    error = START_DEVICE(devices[0]);
    if (error)
    {
        ROS_ERROR_STREAM("(START_DEVICE error " << error << ") " << getBeamErrorDescription(error));
        TERMINATE(devices[0]);
        return 1;
    }
    ROS_INFO("Device started");

    loadDefaultParams(nh);
    ROS_INFO("Default parameters loaded");

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