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

#include <iostream>
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include "l3cam_ros/PolarimetricCameraConfig.h"

#include <libL3Cam.h>
#include <beamagine.h>
#include <beamErrors.h>

#include "l3cam_ros/GetSensorsAvailable.h"
#include "l3cam_ros/ChangePolarimetricCameraBrightness.h"
#include "l3cam_ros/ChangePolarimetricCameraBlackLevel.h"
#include "l3cam_ros/EnablePolarimetricCameraAutoGain.h"
#include "l3cam_ros/ChangePolarimetricCameraAutoGainRange.h"
#include "l3cam_ros/ChangePolarimetricCameraGain.h"
#include "l3cam_ros/EnablePolarimetricCameraAutoExposureTime.h"
#include "l3cam_ros/ChangePolarimetricCameraAutoExposureTimeRange.h"
#include "l3cam_ros/ChangePolarimetricCameraExposureTime.h"

ros::ServiceClient clientGetSensors;
l3cam_ros::GetSensorsAvailable srvGetSensors;
ros::ServiceClient clientBrightness;
l3cam_ros::ChangePolarimetricCameraBrightness srvBrightness;
ros::ServiceClient clientBlackLevel;
l3cam_ros::ChangePolarimetricCameraBlackLevel srvBlackLevel;
ros::ServiceClient clientEnableAutoGain;
l3cam_ros::EnablePolarimetricCameraAutoGain srvEnableAutoGain;
ros::ServiceClient clientAutoGainRange;
l3cam_ros::ChangePolarimetricCameraAutoGainRange srvAutoGainRange;
ros::ServiceClient clientGain;
l3cam_ros::ChangePolarimetricCameraGain srvGain;
ros::ServiceClient clientEnableAutoExposureTime;
l3cam_ros::EnablePolarimetricCameraAutoExposureTime srvEnableAutoExposureTime;
ros::ServiceClient clientAutoExposureTimeRange;
l3cam_ros::ChangePolarimetricCameraAutoExposureTimeRange srvAutoExposureTimeRange;
ros::ServiceClient clientExposureTime;
l3cam_ros::ChangePolarimetricCameraExposureTime srvExposureTime;

int change_polarimetric_camera_brightness;
double change_polarimetric_camera_black_level;
bool enable_polarimetric_camera_auto_gain;
double change_polarimetric_camera_auto_gain_range_minimum;
double change_polarimetric_camera_auto_gain_range_maximum;
double change_polarimetric_camera_gain;
bool enable_polarimetric_camera_auto_exposure_time;
double change_polarimetric_camera_auto_exposure_time_range_minimum;
double change_polarimetric_camera_auto_exposure_time_range_maximum;
double change_polarimetric_camera_exposure_time;

bool default_configured = false;

void configureDefault(l3cam_ros::PolarimetricCameraConfig &config)
{
    ros::param::param("/polarimetric_camera_configuration/polarimetric_camera_brightness", change_polarimetric_camera_brightness, 127);
    ros::param::param("/polarimetric_camera_configuration/polarimetric_camera_black_level", change_polarimetric_camera_black_level, 6.0);
    ros::param::param("/polarimetric_camera_configuration/polarimetric_camera_auto_gain", enable_polarimetric_camera_auto_gain, true);
    ros::param::param("/polarimetric_camera_configuration/polarimetric_camera_auto_gain_range_minimum", change_polarimetric_camera_auto_gain_range_minimum, 0.0);
    ros::param::param("/polarimetric_camera_configuration/polarimetric_camera_auto_gain_range_maximum", change_polarimetric_camera_auto_gain_range_maximum, 48.0);
    ros::param::param("/polarimetric_camera_configuration/polarimetric_camera_gain", change_polarimetric_camera_gain, 24.0);
    ros::param::param("/polarimetric_camera_configuration/polarimetric_camera_auto_exposure_time", enable_polarimetric_camera_auto_exposure_time, true);
    ros::param::param("/polarimetric_camera_configuration/polarimetric_camera_auto_exposure_time_range_minimum", change_polarimetric_camera_auto_exposure_time_range_minimum, 33.456);
    ros::param::param("/polarimetric_camera_configuration/polarimetric_camera_auto_exposure_time_range_maximum", change_polarimetric_camera_auto_exposure_time_range_maximum, 66470.6);
    ros::param::param("/polarimetric_camera_configuration/polarimetric_camera_exposure_time", change_polarimetric_camera_exposure_time, 500000.0);

    if (change_polarimetric_camera_brightness >= 0 && change_polarimetric_camera_brightness <= 255)
        config.change_polarimetric_camera_brightness = change_polarimetric_camera_brightness;
    else
        change_polarimetric_camera_brightness = config.change_polarimetric_camera_brightness;
    if (change_polarimetric_camera_black_level >= 0 && change_polarimetric_camera_black_level <= 12.5)
        config.change_polarimetric_camera_black_level = change_polarimetric_camera_black_level;
    else
        change_polarimetric_camera_black_level = config.change_polarimetric_camera_black_level;
    config.enable_polarimetric_camera_auto_gain = enable_polarimetric_camera_auto_gain;
    if (change_polarimetric_camera_auto_gain_range_minimum >= 0 && change_polarimetric_camera_auto_gain_range_minimum <= 48)
        config.change_polarimetric_camera_auto_gain_range_minimum = change_polarimetric_camera_auto_gain_range_minimum;
    else
        change_polarimetric_camera_auto_gain_range_minimum = config.change_polarimetric_camera_auto_gain_range_minimum;
    if (change_polarimetric_camera_auto_gain_range_maximum >= 0 && change_polarimetric_camera_auto_gain_range_maximum <= 48)
        config.change_polarimetric_camera_auto_gain_range_maximum = change_polarimetric_camera_auto_gain_range_maximum;
    else
        change_polarimetric_camera_auto_gain_range_maximum = config.change_polarimetric_camera_auto_gain_range_maximum;
    if (change_polarimetric_camera_gain >= 0 && change_polarimetric_camera_gain <= 48)
        config.change_polarimetric_camera_gain = change_polarimetric_camera_gain;
    else
        change_polarimetric_camera_gain = config.change_polarimetric_camera_gain;
    config.enable_polarimetric_camera_auto_exposure_time = enable_polarimetric_camera_auto_exposure_time;
    if (change_polarimetric_camera_auto_exposure_time_range_minimum >= 33.456 && change_polarimetric_camera_auto_exposure_time_range_minimum <= 66470.6)
        config.change_polarimetric_camera_auto_exposure_time_range_minimum = change_polarimetric_camera_auto_exposure_time_range_minimum;
    else
        change_polarimetric_camera_auto_exposure_time_range_minimum = config.change_polarimetric_camera_auto_exposure_time_range_minimum;
    if (change_polarimetric_camera_auto_exposure_time_range_maximum >= 33.456 && change_polarimetric_camera_auto_exposure_time_range_maximum <= 66470.6)
        config.change_polarimetric_camera_auto_exposure_time_range_maximum = change_polarimetric_camera_auto_exposure_time_range_maximum;
    else
        change_polarimetric_camera_auto_exposure_time_range_maximum = config.change_polarimetric_camera_auto_exposure_time_range_maximum;
    if (change_polarimetric_camera_exposure_time >= 33.456 && change_polarimetric_camera_exposure_time <= 66470.6)
        config.change_polarimetric_camera_exposure_time = change_polarimetric_camera_exposure_time;
    else
        change_polarimetric_camera_exposure_time = config.change_polarimetric_camera_exposure_time;

    default_configured = true;
}

void callback(l3cam_ros::PolarimetricCameraConfig &config, uint32_t level)
{
    int error = L3CAM_OK;

    if (!default_configured)
        configureDefault(config);
    else
    {
        switch (level)
        {
        case 0:
            srvBrightness.request.brightness = config.change_polarimetric_camera_brightness;
            if (clientBrightness.call(srvBrightness))
            {
                error = srvBrightness.response.error;
                if (!error)
                    change_polarimetric_camera_brightness = config.change_polarimetric_camera_brightness;
                else
                    config.change_polarimetric_camera_brightness = change_polarimetric_camera_brightness;
            }
            else
            {
                ROS_ERROR("Failed to call service change_polarimetric_camera_brightness");
                config.change_polarimetric_camera_brightness = change_polarimetric_camera_brightness;
            }
            break;
        case 1:
            srvBlackLevel.request.black_level = config.change_polarimetric_camera_black_level;
            if (clientBlackLevel.call(srvBlackLevel))
            {
                error = srvBlackLevel.response.error;
                if (!error)
                    change_polarimetric_camera_black_level = config.change_polarimetric_camera_black_level;
                else
                    config.change_polarimetric_camera_black_level = change_polarimetric_camera_black_level;
            }
            else
            {
                ROS_ERROR("Failed to call service change_polarimetric_camera_black_level");
                config.change_polarimetric_camera_black_level = change_polarimetric_camera_black_level;
            }
            break;
        case 2:
            srvEnableAutoGain.request.enabled = config.enable_polarimetric_camera_auto_gain;
            if (clientEnableAutoGain.call(srvEnableAutoGain))
            {
                error = srvEnableAutoGain.response.error;
                if (!error)
                    enable_polarimetric_camera_auto_gain = config.enable_polarimetric_camera_auto_gain;
                else
                    config.enable_polarimetric_camera_auto_gain = enable_polarimetric_camera_auto_gain;
            }
            else
            {
                ROS_ERROR("Failed to call service enable_polarimetric_camera_auto_gain");
                config.enable_polarimetric_camera_auto_gain = enable_polarimetric_camera_auto_gain;
            }
            break;
        case 3:
            if (enable_polarimetric_camera_auto_gain)
            {
                srvAutoGainRange.request.min_gain = config.change_polarimetric_camera_auto_gain_range_minimum;
                srvAutoGainRange.request.max_gain = change_polarimetric_camera_auto_gain_range_maximum;
                if (clientAutoGainRange.call(srvAutoGainRange))
                {
                    error = srvAutoGainRange.response.error;
                    if (!error)
                        change_polarimetric_camera_auto_gain_range_minimum = config.change_polarimetric_camera_auto_gain_range_minimum;
                    else
                        config.change_polarimetric_camera_auto_gain_range_minimum = change_polarimetric_camera_auto_gain_range_minimum;
                }
                else
                {
                    ROS_ERROR("Failed to call service change_polarimetric_camera_auto_gain_range_minimum");
                    config.change_polarimetric_camera_auto_gain_range_minimum = change_polarimetric_camera_auto_gain_range_minimum;
                }
            }
            else
            {
                ROS_INFO("Polarimetric camera auto gain must be enabled to change auto gain range");
                config.change_polarimetric_camera_auto_gain_range_minimum = change_polarimetric_camera_auto_gain_range_minimum;
            }
            break;
        case 4:
            if (enable_polarimetric_camera_auto_gain)
            {
                srvAutoGainRange.request.max_gain = config.change_polarimetric_camera_auto_gain_range_maximum;
                srvAutoGainRange.request.min_gain = change_polarimetric_camera_auto_gain_range_minimum;
                if (clientAutoGainRange.call(srvAutoGainRange))
                {
                    error = srvAutoGainRange.response.error;
                    if (!error)
                        change_polarimetric_camera_auto_gain_range_maximum = config.change_polarimetric_camera_auto_gain_range_maximum;
                    else
                        config.change_polarimetric_camera_auto_gain_range_maximum = change_polarimetric_camera_auto_gain_range_maximum;
                }
                else
                {
                    ROS_ERROR("Failed to call service change_polarimetric_camera_auto_gain_range_maximum");
                    config.change_polarimetric_camera_auto_gain_range_maximum = change_polarimetric_camera_auto_gain_range_maximum;
                }
            }
            else
            {
                ROS_INFO("Polarimetric camera auto gain must be enabled to change auto gain range");
                config.change_polarimetric_camera_auto_gain_range_maximum = change_polarimetric_camera_auto_gain_range_maximum;
            }
            break;
        case 5:
            if (!enable_polarimetric_camera_auto_gain)
            {
                srvGain.request.gain = config.change_polarimetric_camera_gain;
                if (clientGain.call(srvGain))
                {
                    error = srvGain.response.error;
                    if (!error)
                        change_polarimetric_camera_gain = config.change_polarimetric_camera_gain;
                    else
                        config.change_polarimetric_camera_gain = change_polarimetric_camera_gain;
                }
                else
                {
                    ROS_ERROR("Failed to call service change_polarimetric_camera_gain");
                    config.change_polarimetric_camera_gain = change_polarimetric_camera_gain;
                }
            }
            else
            {
                ROS_INFO("Polarimetric camera auto gain must be disabled to change gain");
                config.change_polarimetric_camera_gain = change_polarimetric_camera_gain;
            }
            break;
        case 6:
            srvEnableAutoExposureTime.request.enabled = config.enable_polarimetric_camera_auto_exposure_time;
            if (clientEnableAutoExposureTime.call(srvEnableAutoExposureTime))
            {
                error = srvEnableAutoExposureTime.response.error;
                if (!error)
                    enable_polarimetric_camera_auto_exposure_time = config.enable_polarimetric_camera_auto_exposure_time;
                else
                    config.enable_polarimetric_camera_auto_exposure_time = enable_polarimetric_camera_auto_exposure_time;
            }
            else
            {
                ROS_ERROR("Failed to call service enable_polarimetric_camera_auto_exposure_time");
                config.enable_polarimetric_camera_auto_exposure_time = enable_polarimetric_camera_auto_exposure_time;
            }
            break;
        case 7:
            if (enable_polarimetric_camera_auto_exposure_time)
            {
                srvAutoExposureTimeRange.request.min_exposure = config.change_polarimetric_camera_auto_exposure_time_range_minimum;
                srvAutoExposureTimeRange.request.max_exposure = change_polarimetric_camera_auto_exposure_time_range_maximum;
                if (clientAutoExposureTimeRange.call(srvAutoExposureTimeRange))
                {
                    error = srvAutoExposureTimeRange.response.error;
                    if (!error)
                        change_polarimetric_camera_auto_exposure_time_range_minimum = config.change_polarimetric_camera_auto_exposure_time_range_minimum;
                    else
                        config.change_polarimetric_camera_auto_exposure_time_range_minimum = change_polarimetric_camera_auto_exposure_time_range_minimum;
                }
                else
                {
                    ROS_ERROR("Failed to call service change_polarimetric_camera_auto_exposure_time_range");
                    config.change_polarimetric_camera_auto_exposure_time_range_minimum = change_polarimetric_camera_auto_exposure_time_range_minimum;
                }
            }
            else
            {
                ROS_INFO("Polarimetric camera auto exposure time must be enabled to change auto exposure time range");
                config.change_polarimetric_camera_auto_exposure_time_range_minimum = change_polarimetric_camera_auto_exposure_time_range_minimum;
            }
            break;
        case 8:
            if (enable_polarimetric_camera_auto_exposure_time)
            {
                srvAutoExposureTimeRange.request.max_exposure = config.change_polarimetric_camera_auto_exposure_time_range_maximum;
                srvAutoExposureTimeRange.request.min_exposure = change_polarimetric_camera_auto_exposure_time_range_minimum;
                if (clientAutoExposureTimeRange.call(srvAutoExposureTimeRange))
                {
                    error = srvAutoExposureTimeRange.response.error;
                    if (!error)
                        change_polarimetric_camera_auto_exposure_time_range_maximum = config.change_polarimetric_camera_auto_exposure_time_range_maximum;
                    else
                        config.change_polarimetric_camera_auto_exposure_time_range_maximum = change_polarimetric_camera_auto_exposure_time_range_maximum;
                }
                else
                {
                    ROS_ERROR("Failed to call service change_polarimetric_camera_auto_exposure_time_range");
                    config.change_polarimetric_camera_auto_exposure_time_range_maximum = change_polarimetric_camera_auto_exposure_time_range_maximum;
                }
            }
            else
            {
                ROS_INFO("Polarimetric camera auto exposure time must be enabled to change auto exposure time range");
                config.change_polarimetric_camera_auto_exposure_time_range_maximum = change_polarimetric_camera_auto_exposure_time_range_maximum;
            }
            break;
        case 9:
            if (!enable_polarimetric_camera_auto_exposure_time)
            {
                srvExposureTime.request.exposure_time = config.change_polarimetric_camera_exposure_time;
                if (clientExposureTime.call(srvExposureTime))
                {
                    error = srvExposureTime.response.error;
                    if (!error)
                        change_polarimetric_camera_exposure_time = config.change_polarimetric_camera_exposure_time;
                    else
                        config.change_polarimetric_camera_exposure_time = change_polarimetric_camera_exposure_time;
                }
                else
                {
                    ROS_ERROR("Failed to call service change_polarimetric_camera_exposure_time");
                    config.change_polarimetric_camera_exposure_time = change_polarimetric_camera_exposure_time;
                }
            }
            else
            {
                ROS_INFO("Polarimetric camera auto exposure time must be disabled to change exposure time");
                config.change_polarimetric_camera_exposure_time = change_polarimetric_camera_exposure_time;
            }
            break;
        }
    }

    if (error)
        ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "polarimetric_camera_configuration");
    ros::NodeHandle nh;

    clientGetSensors = nh.serviceClient<l3cam_ros::GetSensorsAvailable>("get_sensors_available");
    int error = L3CAM_OK;

    bool sensor_is_available = false;
    if (clientGetSensors.call(srvGetSensors))
    {
        error = srvGetSensors.response.error;

        if (!error)
            for (int i = 0; i < srvGetSensors.response.num_sensors; ++i)
            {
                if (srvGetSensors.response.sensors[i].sensor_type == sensor_pol)
                    sensor_is_available = true;
            }
        else
        {
            ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
            return 1;
        }
    }
    else
    {
        ROS_ERROR("Failed to call service get_sensors_available");
        return 1;
    }

    if (sensor_is_available)
        ROS_INFO("Polarimetric camera configuration is available");
    else
        return 0;

    dynamic_reconfigure::Server<l3cam_ros::PolarimetricCameraConfig> server;
    dynamic_reconfigure::Server<l3cam_ros::PolarimetricCameraConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    clientBrightness = nh.serviceClient<l3cam_ros::ChangePolarimetricCameraBrightness>("change_polarimetric_camera_brightness");
    clientBlackLevel = nh.serviceClient<l3cam_ros::ChangePolarimetricCameraBlackLevel>("change_polarimetric_camera_black_level");
    clientEnableAutoGain = nh.serviceClient<l3cam_ros::EnablePolarimetricCameraAutoGain>("enable_polarimetric_camera_auto_gain");
    clientAutoGainRange = nh.serviceClient<l3cam_ros::ChangePolarimetricCameraAutoGainRange>("change_polarimetric_camera_auto_gain_range");
    clientGain = nh.serviceClient<l3cam_ros::ChangePolarimetricCameraGain>("change_polarimetric_camera_gain");
    clientEnableAutoExposureTime = nh.serviceClient<l3cam_ros::EnablePolarimetricCameraAutoExposureTime>("enable_polarimetric_camera_auto_exposure_time");
    clientAutoExposureTimeRange = nh.serviceClient<l3cam_ros::ChangePolarimetricCameraAutoExposureTimeRange>("change_polarimetric_camera_auto_exposure_time_range");
    clientExposureTime = nh.serviceClient<l3cam_ros::ChangePolarimetricCameraExposureTime>("change_polarimetric_camera_exposure_time");

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
