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
#include "l3cam_ros/AlliedNarrowCameraConfig.h"

#include <libL3Cam.h>
#include <beamagine.h>
#include <beamErrors.h>

#include "l3cam_ros/GetSensorsAvailable.h"
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

ros::ServiceClient clientGetSensors;
l3cam_ros::GetSensorsAvailable srvGetSensors;
ros::ServiceClient clientExposureTime;
l3cam_ros::ChangeAlliedCameraExposureTime srvExposureTime;
ros::ServiceClient clientEnableAutoExposureTime;
l3cam_ros::EnableAlliedCameraAutoExposureTime srvEnableAutoExposureTime;
ros::ServiceClient clientAutoExposureTimeRange;
l3cam_ros::ChangeAlliedCameraAutoExposureTimeRange srvAutoExposureTimeRange;
ros::ServiceClient clientGain;
l3cam_ros::ChangeAlliedCameraGain srvGain;
ros::ServiceClient clientEnableAutoGain;
l3cam_ros::EnableAlliedCameraAutoGain srvEnableAutoGain;
ros::ServiceClient clientAutoGainRange;
l3cam_ros::ChangeAlliedCameraAutoGainRange srvAutoGainRange;
ros::ServiceClient clientGamma;
l3cam_ros::ChangeAlliedCameraGamma srvGamma;
ros::ServiceClient clientSaturation;
l3cam_ros::ChangeAlliedCameraSaturation srvSaturation;
ros::ServiceClient clientHue;
l3cam_ros::ChangeAlliedCameraHue srvHue;
ros::ServiceClient clientIntensityAutoPrecedence;
l3cam_ros::ChangeAlliedCameraIntensityAutoPrecedence srvIntensityAutoPrecedence;
ros::ServiceClient clientEnableAutoWhiteBalance;
l3cam_ros::EnableAlliedCameraAutoWhiteBalance srvEnableAutoWhiteBalance;
ros::ServiceClient clientBalanceRatioSelector;
l3cam_ros::ChangeAlliedCameraBalanceRatioSelector srvBalanceRatioSelector;
ros::ServiceClient clientBalanceRatio;
l3cam_ros::ChangeAlliedCameraBalanceRatio srvBalanceRatio;
ros::ServiceClient clientBalanceWhiteAutoRate;
l3cam_ros::ChangeAlliedCameraBalanceWhiteAutoRate srvBalanceWhiteAutoRate;
ros::ServiceClient clientBalanceWhiteAutoTolerance;
l3cam_ros::ChangeAlliedCameraBalanceWhiteAutoTolerance srvBalanceWhiteAutoTolerance;
ros::ServiceClient clientIntensityControllerRegion;
l3cam_ros::ChangeAlliedCameraIntensityControllerRegion srvIntensityControllerRegion;
ros::ServiceClient clientIntensityControllerTarget;
l3cam_ros::ChangeAlliedCameraIntensityControllerTarget srvIntensityControllerTarget;

double change_allied_narrow_camera_exposure_time;
bool enable_allied_narrow_camera_auto_exposure_time;
double change_allied_narrow_camera_auto_exposure_time_range_min;
double change_allied_narrow_camera_auto_exposure_time_range_max;
double change_allied_narrow_camera_gain;
bool enable_allied_narrow_camera_auto_gain;
double change_allied_narrow_camera_auto_gain_range_min;
double change_allied_narrow_camera_auto_gain_range_max;
double change_allied_narrow_camera_gamma;
double change_allied_narrow_camera_saturation;
double change_allied_narrow_camera_hue;
int change_allied_narrow_camera_intensity_auto_precedence;
bool enable_allied_narrow_camera_auto_white_balance;
int change_allied_narrow_camera_balance_ratio_selector;
double change_allied_narrow_camera_balance_ratio;
double change_allied_narrow_camera_balance_white_auto_rate;
double change_allied_narrow_camera_balance_white_auto_tolerance;
int change_allied_narrow_camera_intensity_controller_region;
double change_allied_narrow_camera_intensity_controller_target;

bool default_configured = false;

void configureDefault(l3cam_ros::AlliedNarrowCameraConfig &config)
{
    ros::param::param("/allied_narrow_camera_configuration/allied_narrow_camera_exposure_time", change_allied_narrow_camera_exposure_time, 4992.32);
    ros::param::param("/allied_narrow_camera_configuration/allied_narrow_camera_auto_exposure_time", enable_allied_narrow_camera_auto_exposure_time, false);
    ros::param::param("/allied_narrow_camera_configuration/allied_narrow_camera_auto_exposure_time_range_min", change_allied_narrow_camera_auto_exposure_time_range_min, 87.596);
    ros::param::param("/allied_narrow_camera_configuration/allied_narrow_camera_auto_exposure_time_range_max", change_allied_narrow_camera_auto_exposure_time_range_max, 87.596);
    ros::param::param("/allied_narrow_camera_configuration/allied_narrow_camera_gain", change_allied_narrow_camera_gain, 0.);
    ros::param::param("/allied_narrow_camera_configuration/allied_narrow_camera_auto_gain", enable_allied_narrow_camera_auto_gain, false);
    ros::param::param("/allied_narrow_camera_configuration/allied_narrow_camera_auto_gain_range_min", change_allied_narrow_camera_auto_gain_range_min, 0.);
    ros::param::param("/allied_narrow_camera_configuration/allied_narrow_camera_auto_gain_range_max", change_allied_narrow_camera_auto_gain_range_max, 48.);
    ros::param::param("/allied_narrow_camera_configuration/allied_narrow_camera_gamma", change_allied_narrow_camera_gamma, 1.);
    ros::param::param("/allied_narrow_camera_configuration/allied_narrow_camera_saturation", change_allied_narrow_camera_saturation, 1.);
    ros::param::param("/allied_narrow_camera_configuration/allied_narrow_camera_hue", change_allied_narrow_camera_hue, 0.);
    ros::param::param("/allied_narrow_camera_configuration/allied_narrow_camera_intensity_auto_precedence", change_allied_narrow_camera_intensity_auto_precedence, 1);
    ros::param::param("/allied_narrow_camera_configuration/allied_narrow_camera_auto_white_balance", enable_allied_narrow_camera_auto_white_balance, false);
    ros::param::param("/allied_narrow_camera_configuration/allied_narrow_camera_balance_ratio_selector", change_allied_narrow_camera_balance_ratio_selector, 1);
    ros::param::param("/allied_narrow_camera_configuration/allied_narrow_camera_balance_ratio", change_allied_narrow_camera_balance_ratio, 2.35498);
    ros::param::param("/allied_narrow_camera_configuration/allied_narrow_camera_balance_white_auto_rate", change_allied_narrow_camera_balance_white_auto_rate, 100.);
    ros::param::param("/allied_narrow_camera_configuration/allied_narrow_camera_balance_white_auto_tolerance", change_allied_narrow_camera_balance_white_auto_tolerance, 5.);
    ros::param::param("/allied_narrow_camera_configuration/allied_narrow_camera_intensity_controller_region", change_allied_narrow_camera_intensity_controller_region, 1);
    ros::param::param("/allied_narrow_camera_configuration/allied_narrow_camera_intensity_controller_target", change_allied_narrow_camera_intensity_controller_target, 50.);

    if (change_allied_narrow_camera_exposure_time >= 0 && change_allied_narrow_camera_exposure_time <= 4095)
        config.change_allied_narrow_camera_exposure_time = change_allied_narrow_camera_exposure_time;
    else
        change_allied_narrow_camera_exposure_time = config.change_allied_narrow_camera_exposure_time;
    if (enable_allied_narrow_camera_auto_exposure_time >= 63 && enable_allied_narrow_camera_auto_exposure_time <= 10000000)
        config.enable_allied_narrow_camera_auto_exposure_time = enable_allied_narrow_camera_auto_exposure_time;
    else
        enable_allied_narrow_camera_auto_exposure_time = config.enable_allied_narrow_camera_auto_exposure_time;
    if (change_allied_narrow_camera_auto_exposure_time_range_min >= 63.03 && change_allied_narrow_camera_auto_exposure_time_range_min <= 8999990)
        config.change_allied_narrow_camera_auto_exposure_time_range_min = change_allied_narrow_camera_auto_exposure_time_range_min;
    else
        change_allied_narrow_camera_auto_exposure_time_range_min = config.change_allied_narrow_camera_auto_exposure_time_range_min;
    if (change_allied_narrow_camera_auto_exposure_time_range_max >= 87.596 && change_allied_narrow_camera_auto_exposure_time_range_max <= 10000000)
        config.change_allied_narrow_camera_auto_exposure_time_range_max = change_allied_narrow_camera_auto_exposure_time_range_max;
    else
        change_allied_narrow_camera_auto_exposure_time_range_max = config.change_allied_narrow_camera_auto_exposure_time_range_max;
    if (change_allied_narrow_camera_gain >= 0 && change_allied_narrow_camera_gain <= 48)
        config.change_allied_narrow_camera_gain = change_allied_narrow_camera_gain;
    else
        change_allied_narrow_camera_gain = config.change_allied_narrow_camera_gain;
    config.enable_allied_narrow_camera_auto_gain = enable_allied_narrow_camera_auto_gain;
    if (change_allied_narrow_camera_auto_gain_range_min >= 0 && change_allied_narrow_camera_auto_gain_range_min <= 48)
        config.change_allied_narrow_camera_auto_gain_range_min = change_allied_narrow_camera_auto_gain_range_min;
    else
        change_allied_narrow_camera_auto_gain_range_min = config.change_allied_narrow_camera_auto_gain_range_min;
    if (change_allied_narrow_camera_auto_gain_range_max >= 0 && change_allied_narrow_camera_auto_gain_range_max <= 48)
        config.change_allied_narrow_camera_auto_gain_range_max = change_allied_narrow_camera_auto_gain_range_max;
    else
        change_allied_narrow_camera_auto_gain_range_max = config.change_allied_narrow_camera_auto_gain_range_max;
    if (change_allied_narrow_camera_gamma >= 0.4 && change_allied_narrow_camera_gamma <= 2.4)
        config.change_allied_narrow_camera_gamma = change_allied_narrow_camera_gamma;
    else
        change_allied_narrow_camera_gamma = config.change_allied_narrow_camera_gamma;
    if (change_allied_narrow_camera_saturation >= 0 && change_allied_narrow_camera_saturation <= 2)
        config.change_allied_narrow_camera_saturation = change_allied_narrow_camera_saturation;
    else
        change_allied_narrow_camera_saturation = config.change_allied_narrow_camera_saturation;
    if (change_allied_narrow_camera_hue >= -40 && change_allied_narrow_camera_hue <= 40)
        config.change_allied_narrow_camera_hue = change_allied_narrow_camera_hue;
    else
        change_allied_narrow_camera_hue = config.change_allied_narrow_camera_hue;
    if (change_allied_narrow_camera_intensity_auto_precedence == 0 || change_allied_narrow_camera_intensity_auto_precedence == 1)
        config.change_allied_narrow_camera_intensity_auto_precedence = change_allied_narrow_camera_intensity_auto_precedence;
    else
        change_allied_narrow_camera_intensity_auto_precedence = config.change_allied_narrow_camera_intensity_auto_precedence;
    config.enable_allied_narrow_camera_auto_white_balance = enable_allied_narrow_camera_auto_white_balance;
    if (change_allied_narrow_camera_balance_ratio_selector == 0 || change_allied_narrow_camera_balance_ratio_selector == 1)
        config.change_allied_narrow_camera_white_balance_ratio_selector = change_allied_narrow_camera_balance_ratio_selector;
    else
        change_allied_narrow_camera_balance_ratio_selector = config.change_allied_narrow_camera_white_balance_ratio_selector;
    if (change_allied_narrow_camera_balance_ratio >= 0 && change_allied_narrow_camera_balance_ratio <= 8)
        config.change_allied_narrow_camera_balance_ratio = change_allied_narrow_camera_balance_ratio;
    else
        change_allied_narrow_camera_balance_ratio = config.change_allied_narrow_camera_balance_ratio;
    if (change_allied_narrow_camera_balance_white_auto_rate >= 0 && change_allied_narrow_camera_balance_white_auto_rate <= 100)
        config.change_allied_narrow_camera_white_balance_auto_rate = change_allied_narrow_camera_balance_white_auto_rate;
    else
        change_allied_narrow_camera_balance_white_auto_rate = config.change_allied_narrow_camera_white_balance_auto_rate;
    if (change_allied_narrow_camera_balance_white_auto_tolerance >= 0 && change_allied_narrow_camera_balance_white_auto_tolerance <= 50)
        config.change_allied_narrow_camera_white_balance_auto_tolerance = change_allied_narrow_camera_balance_white_auto_tolerance;
    else
        change_allied_narrow_camera_balance_white_auto_tolerance = config.change_allied_narrow_camera_white_balance_auto_tolerance;
    if (change_allied_narrow_camera_intensity_controller_region == 0 || change_allied_narrow_camera_intensity_controller_region == 4)
        config.change_allied_narrow_camera_intensity_controller_region = change_allied_narrow_camera_intensity_controller_region;
    else
        change_allied_narrow_camera_intensity_controller_region = config.change_allied_narrow_camera_intensity_controller_region;
    if (change_allied_narrow_camera_intensity_controller_target >= 10 && change_allied_narrow_camera_intensity_controller_target <= 90)
        config.change_allied_narrow_camera_intensity_controller_target = change_allied_narrow_camera_intensity_controller_target;
    else
        change_allied_narrow_camera_intensity_controller_target = config.change_allied_narrow_camera_intensity_controller_target;

    default_configured = true;
}

void callback(l3cam_ros::AlliedNarrowCameraConfig &config, uint32_t level)
{
    int error = L3CAM_OK;

    if (!default_configured)
        configureDefault(config);
    else
    {
        switch (level)
        {
        case 0:
            if (!config.enable_allied_narrow_camera_auto_exposure_time)
            {
                srvExposureTime.request.exposure_time = config.change_allied_narrow_camera_exposure_time;
                srvExposureTime.request.allied_type = 2;
                if (clientExposureTime.call(srvExposureTime))
                {
                    error = srvExposureTime.response.error;
                    if (!error)
                        change_allied_narrow_camera_exposure_time = config.change_allied_narrow_camera_exposure_time;
                    else
                    {
                        config.change_allied_narrow_camera_exposure_time = change_allied_narrow_camera_exposure_time;
                    }
                }
                else
                {
                    ROS_ERROR("Failed to call service change_allied_narrow_camera_exposure_time");
                    config.change_allied_narrow_camera_exposure_time = change_allied_narrow_camera_exposure_time;
                }
            }
            else
            {
                ROS_INFO("Allied Narrow camera auto exposure time must be disabled to change exposure time");
                config.change_allied_narrow_camera_exposure_time = change_allied_narrow_camera_exposure_time;
            }
            break;
        case 1:
            srvEnableAutoExposureTime.request.enabled = config.enable_allied_narrow_camera_auto_exposure_time;
            srvEnableAutoExposureTime.request.allied_type = 2;
            if (clientEnableAutoExposureTime.call(srvEnableAutoExposureTime))
            {
                error = srvEnableAutoExposureTime.response.error;
                if (!error)
                    enable_allied_narrow_camera_auto_exposure_time = config.enable_allied_narrow_camera_auto_exposure_time;
                else
                {
                    config.enable_allied_narrow_camera_auto_exposure_time = enable_allied_narrow_camera_auto_exposure_time;
                }
            }
            else
            {
                ROS_ERROR("Failed to call service enable_allied_narrow_camera_auto_exposure_time");
                config.enable_allied_narrow_camera_auto_exposure_time = enable_allied_narrow_camera_auto_exposure_time;
            }
            break;
        case 2:
            if (config.enable_allied_narrow_camera_auto_exposure_time)
            {
                srvAutoExposureTimeRange.request.auto_exposure_time_range_min = config.change_allied_narrow_camera_auto_exposure_time_range_min;
                srvAutoExposureTimeRange.request.allied_type = 2;
                srvAutoExposureTimeRange.request.auto_exposure_time_range_max = config.change_allied_narrow_camera_auto_exposure_time_range_max;
                if (clientAutoExposureTimeRange.call(srvAutoExposureTimeRange))
                {
                    error = srvAutoExposureTimeRange.response.error;
                    if (!error)
                        change_allied_narrow_camera_auto_exposure_time_range_min = config.change_allied_narrow_camera_auto_exposure_time_range_min;
                    else
                    {
                        config.change_allied_narrow_camera_auto_exposure_time_range_min = change_allied_narrow_camera_auto_exposure_time_range_min;
                    }
                }
                else
                {
                    ROS_ERROR("Failed to call service change_allied_narrow_camera_auto_exposure_time_range_min");
                    config.change_allied_narrow_camera_auto_exposure_time_range_min = change_allied_narrow_camera_auto_exposure_time_range_min;
                }
            }
            else
            {
                ROS_INFO("Allied Narrow camera auto exposure time must be enabled to change auto exposure time range");
                config.change_allied_narrow_camera_auto_exposure_time_range_min = change_allied_narrow_camera_auto_exposure_time_range_min;
            }
            break;
        case 3:
            if (config.enable_allied_narrow_camera_auto_exposure_time)
            {
                srvAutoExposureTimeRange.request.auto_exposure_time_range_min = config.change_allied_narrow_camera_auto_exposure_time_range_min;
                srvAutoExposureTimeRange.request.allied_type = 2;
                srvAutoExposureTimeRange.request.auto_exposure_time_range_max = config.change_allied_narrow_camera_auto_exposure_time_range_max;
                if (clientAutoExposureTimeRange.call(srvAutoExposureTimeRange))
                {
                    error = srvAutoExposureTimeRange.response.error;
                    if (!error)
                        change_allied_narrow_camera_auto_exposure_time_range_max = config.change_allied_narrow_camera_auto_exposure_time_range_max;
                    else
                    {
                        config.change_allied_narrow_camera_auto_exposure_time_range_max = change_allied_narrow_camera_auto_exposure_time_range_max;
                    }
                }
                else
                {
                    ROS_ERROR("Failed to call service change_allied_narrow_camera_auto_exposure_time_range_max");
                    config.change_allied_narrow_camera_auto_exposure_time_range_max = change_allied_narrow_camera_auto_exposure_time_range_max;
                }
            }
            else
            {
                ROS_INFO("Allied Narrow camera auto exposure time must be enabled to change auto exposure time range");
                config.change_allied_narrow_camera_auto_exposure_time_range_max = change_allied_narrow_camera_auto_exposure_time_range_max;
            }
            break;
        case 4:
            if (!config.enable_allied_narrow_camera_auto_gain)
            {
                srvGain.request.gain = config.change_allied_narrow_camera_gain;
                srvGain.request.allied_type = 2;
                if (clientGain.call(srvGain))
                {
                    error = srvGain.response.error;
                    if (!error)
                    {
                        change_allied_narrow_camera_gain = config.change_allied_narrow_camera_gain;
                    }
                    else
                    {
                        config.change_allied_narrow_camera_gain = change_allied_narrow_camera_gain;
                    }
                }
                else
                {
                    ROS_ERROR("Failed to call service change_allied_narrow_camera_gain");
                    config.change_allied_narrow_camera_gain = change_allied_narrow_camera_gain;
                }
            }
            else
            {
                ROS_INFO("Allied Narrow camera auto gain must be disabled to change gain");
                config.change_allied_narrow_camera_gain = change_allied_narrow_camera_gain;
            }
            break;
        case 5:
            srvEnableAutoGain.request.enabled = config.enable_allied_narrow_camera_auto_gain;
            srvEnableAutoGain.request.allied_type = 2;
            if (clientEnableAutoGain.call(srvEnableAutoGain))
            {
                error = srvEnableAutoGain.response.error;
                if (!error)
                    enable_allied_narrow_camera_auto_gain = config.enable_allied_narrow_camera_auto_gain;
                else
                {
                    config.enable_allied_narrow_camera_auto_gain = enable_allied_narrow_camera_auto_gain;
                }
            }
            else
            {
                ROS_ERROR("Failed to call service enable_allied_narrow_camera_auto_gain");
                config.enable_allied_narrow_camera_auto_gain = enable_allied_narrow_camera_auto_gain;
            }
            break;
        case 6:
            if (config.enable_allied_narrow_camera_auto_gain)
            {
                srvAutoGainRange.request.auto_gain_range_min = config.change_allied_narrow_camera_auto_gain_range_min;
                srvAutoGainRange.request.allied_type = 2;
                srvAutoGainRange.request.auto_gain_range_max = config.change_allied_narrow_camera_auto_gain_range_max;
                if (clientAutoGainRange.call(srvAutoGainRange))
                {
                    error = srvAutoGainRange.response.error;
                    if (!error)
                        change_allied_narrow_camera_auto_gain_range_min = config.change_allied_narrow_camera_auto_gain_range_min;
                    else
                    {
                        config.change_allied_narrow_camera_auto_gain_range_min = change_allied_narrow_camera_auto_gain_range_min;
                    }
                }
                else
                {
                    ROS_ERROR("Failed to call service change_allied_narrow_camera_auto_gain_range_min");
                    config.change_allied_narrow_camera_auto_gain_range_min = change_allied_narrow_camera_auto_gain_range_min;
                }
            }
            else
            {
                ROS_INFO("Allied Narrow camera auto gain must be enabled to change auto gain range");
                config.change_allied_narrow_camera_auto_gain_range_min = change_allied_narrow_camera_auto_gain_range_min;
            }
            break;
        case 7:
            if (config.enable_allied_narrow_camera_auto_gain)
            {
                srvAutoGainRange.request.auto_gain_range_min = config.change_allied_narrow_camera_auto_gain_range_min;
                srvAutoGainRange.request.allied_type = 2;
                srvAutoGainRange.request.auto_gain_range_max = config.change_allied_narrow_camera_auto_gain_range_max;
                if (clientAutoGainRange.call(srvAutoGainRange))
                {
                    error = srvAutoGainRange.response.error;
                    if (!error)
                        change_allied_narrow_camera_auto_gain_range_max = config.change_allied_narrow_camera_auto_gain_range_max;
                    else
                    {
                        config.change_allied_narrow_camera_auto_gain_range_max = change_allied_narrow_camera_auto_gain_range_max;
                    }
                }
                else
                {
                    ROS_ERROR("Failed to call service change_allied_narrow_camera_auto_gain_range_max");
                    config.change_allied_narrow_camera_auto_gain_range_max = change_allied_narrow_camera_auto_gain_range_max;
                }
            }
            else
            {
                ROS_INFO("Allied Narrow camera auto gain must be enabled to change auto gain range");
                config.change_allied_narrow_camera_auto_gain_range_max = change_allied_narrow_camera_auto_gain_range_max;
            }
            break;
        case 8:
            srvGamma.request.gamma = config.change_allied_narrow_camera_gamma;
            srvGamma.request.allied_type = 2;
            if (clientGamma.call(srvGamma))
            {
                error = srvGamma.response.error;
                if (!error)
                    change_allied_narrow_camera_gamma = config.change_allied_narrow_camera_gamma;
                else
                {
                    config.change_allied_narrow_camera_gamma = change_allied_narrow_camera_gamma;
                }
            }
            else
            {
                ROS_ERROR("Failed to call service change_allied_narrow_camera_gamma");
                config.change_allied_narrow_camera_gamma = change_allied_narrow_camera_gamma;
            }
            break;
        case 9:
            srvSaturation.request.saturation = config.change_allied_narrow_camera_saturation;
            srvSaturation.request.allied_type = 2;
            if (clientSaturation.call(srvSaturation))
            {
                error = srvSaturation.response.error;
                if (!error)
                    change_allied_narrow_camera_saturation = config.change_allied_narrow_camera_saturation;
                else
                {
                    config.change_allied_narrow_camera_saturation = change_allied_narrow_camera_saturation;
                }
            }
            else
            {
                ROS_ERROR("Failed to call service change_allied_narrow_camera_saturation");
                config.change_allied_narrow_camera_saturation = change_allied_narrow_camera_saturation;
            }
            break;
        case 10:
            srvHue.request.hue = config.change_allied_narrow_camera_hue;
            srvHue.request.allied_type = 2;
            if (clientHue.call(srvHue))
            {
                error = srvHue.response.error;
                if (!error)
                    change_allied_narrow_camera_hue = config.change_allied_narrow_camera_hue;
                else
                {
                    config.change_allied_narrow_camera_hue = change_allied_narrow_camera_hue;
                }
            }
            else
            {
                ROS_ERROR("Failed to call service change_allied_narrow_camera_hue");
                config.change_allied_narrow_camera_hue = change_allied_narrow_camera_hue;
            }
            break;
        case 11:
            srvIntensityAutoPrecedence.request.intensity_auto_precedence = config.change_allied_narrow_camera_intensity_auto_precedence;
            srvIntensityAutoPrecedence.request.allied_type = 2;
            if (clientIntensityAutoPrecedence.call(srvIntensityAutoPrecedence))
            {
                error = srvIntensityAutoPrecedence.response.error;
                if (!error)
                    change_allied_narrow_camera_intensity_auto_precedence = config.change_allied_narrow_camera_intensity_auto_precedence;
                else
                {
                    config.change_allied_narrow_camera_intensity_auto_precedence = change_allied_narrow_camera_intensity_auto_precedence;
                }
            }
            else
            {
                ROS_ERROR("Failed to call service change_allied_narrow_camera_intensity_auto_precedence");
                config.change_allied_narrow_camera_intensity_auto_precedence = change_allied_narrow_camera_intensity_auto_precedence;
            }
            break;
        case 12:
            srvEnableAutoWhiteBalance.request.enabled = config.change_allied_narrow_camera_intensity_auto_precedence;
            srvEnableAutoWhiteBalance.request.allied_type = 2;
            if (clientEnableAutoWhiteBalance.call(srvEnableAutoWhiteBalance))
            {
                error = srvEnableAutoWhiteBalance.response.error;
                if (!error)
                    change_allied_narrow_camera_intensity_auto_precedence = config.change_allied_narrow_camera_intensity_auto_precedence;
                else
                {
                    config.change_allied_narrow_camera_intensity_auto_precedence = change_allied_narrow_camera_intensity_auto_precedence;
                }
            }
            else
            {
                ROS_ERROR("Failed to call service change_allied_narrow_camera_intensity_auto_precedence");
                config.change_allied_narrow_camera_intensity_auto_precedence = change_allied_narrow_camera_intensity_auto_precedence;
            }
            break;
        case 13:
            srvBalanceRatioSelector.request.white_balance_ratio_selector = config.change_allied_narrow_camera_white_balance_ratio_selector;
            srvBalanceRatioSelector.request.allied_type = 2;
            if (clientBalanceRatioSelector.call(srvBalanceRatioSelector))
            {
                error = srvBalanceRatioSelector.response.error;
                if (!error)
                    change_allied_narrow_camera_balance_ratio_selector = config.change_allied_narrow_camera_white_balance_ratio_selector;
                else
                {
                    config.change_allied_narrow_camera_white_balance_ratio_selector = change_allied_narrow_camera_balance_ratio_selector;
                }
            }
            else
            {
                ROS_ERROR("Failed to call service change_allied_narrow_camera_balance_ratio_selector");
                config.change_allied_narrow_camera_white_balance_ratio_selector = change_allied_narrow_camera_balance_ratio_selector;
            }
            break;
        case 14:
            srvBalanceRatio.request.balance_ratio = config.change_allied_narrow_camera_balance_ratio;
            srvBalanceRatio.request.allied_type = 2;
            if (clientBalanceRatio.call(srvBalanceRatio))
            {
                error = srvBalanceRatio.response.error;
                if (!error)
                    change_allied_narrow_camera_balance_ratio = config.change_allied_narrow_camera_balance_ratio;
                else
                {
                    config.change_allied_narrow_camera_balance_ratio = change_allied_narrow_camera_balance_ratio;
                }
            }
            else
            {
                ROS_ERROR("Failed to call service change_allied_narrow_camera_balance_ratio");
                config.change_allied_narrow_camera_balance_ratio = change_allied_narrow_camera_balance_ratio;
            }
            break;
        case 15:
            srvBalanceWhiteAutoRate.request.white_balance_auto_rate = config.change_allied_narrow_camera_white_balance_auto_rate;
            srvBalanceWhiteAutoRate.request.allied_type = 2;
            if (clientBalanceWhiteAutoRate.call(srvBalanceWhiteAutoRate))
            {
                error = srvBalanceWhiteAutoRate.response.error;
                if (!error)
                    change_allied_narrow_camera_balance_white_auto_rate = config.change_allied_narrow_camera_white_balance_auto_rate;
                else
                {
                    config.change_allied_narrow_camera_white_balance_auto_rate = change_allied_narrow_camera_balance_white_auto_rate;
                }
            }
            else
            {
                ROS_ERROR("Failed to call service change_allied_narrow_camera_balance_white_auto_rate");
                config.change_allied_narrow_camera_white_balance_auto_rate = change_allied_narrow_camera_balance_white_auto_rate;
            }
            break;
        case 16:
            srvBalanceWhiteAutoTolerance.request.white_balance_auto_tolerance = config.change_allied_narrow_camera_white_balance_auto_tolerance;
            srvBalanceWhiteAutoTolerance.request.allied_type = 2;
            if (clientBalanceWhiteAutoTolerance.call(srvBalanceWhiteAutoTolerance))
            {
                error = srvBalanceWhiteAutoTolerance.response.error;
                if (!error)
                    change_allied_narrow_camera_balance_white_auto_tolerance = config.change_allied_narrow_camera_white_balance_auto_tolerance;
                else
                {
                    config.change_allied_narrow_camera_white_balance_auto_tolerance = change_allied_narrow_camera_balance_white_auto_tolerance;
                }
            }
            else
            {
                ROS_ERROR("Failed to call service change_allied_narrow_camera_balance_white_auto_tolerance");
                config.change_allied_narrow_camera_white_balance_auto_tolerance = change_allied_narrow_camera_balance_white_auto_tolerance;
            }
            break;
        case 17:
            srvIntensityControllerRegion.request.intensity_controller_region = config.change_allied_narrow_camera_intensity_controller_region;
            srvIntensityControllerRegion.request.allied_type = 2;
            if (clientIntensityControllerRegion.call(srvIntensityControllerRegion))
            {
                error = srvIntensityControllerRegion.response.error;
                if (!error)
                    change_allied_narrow_camera_intensity_controller_region = config.change_allied_narrow_camera_intensity_controller_region;
                else
                {
                    config.change_allied_narrow_camera_intensity_controller_region = change_allied_narrow_camera_intensity_controller_region;
                }
            }
            else
            {
                ROS_ERROR("Failed to call service change_allied_narrow_camera_intensity_controller_region");
                config.change_allied_narrow_camera_intensity_controller_region = change_allied_narrow_camera_intensity_controller_region;
            }
            break;
        case 18:
            srvIntensityControllerTarget.request.intensity_controller_target = config.change_allied_narrow_camera_intensity_controller_target;
            srvIntensityControllerTarget.request.allied_type = 2;
            if (clientIntensityControllerTarget.call(srvIntensityControllerTarget))
            {
                error = srvIntensityControllerTarget.response.error;
                if (!error)
                    change_allied_narrow_camera_intensity_controller_target = config.change_allied_narrow_camera_intensity_controller_target;
                else
                {
                    config.change_allied_narrow_camera_intensity_controller_target = change_allied_narrow_camera_intensity_controller_target;
                }
            }
            else
            {
                ROS_ERROR("Failed to call service change_allied_narrow_camera_intensity_controller_target");
                config.change_allied_narrow_camera_intensity_controller_target = change_allied_narrow_camera_intensity_controller_target;
            }
            break;
        }
    }

    if (error)
        ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "allied_narrow_camera_configuration");
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
                if (srvGetSensors.response.sensors[i].sensor_type == sensor_allied_narrow)
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
        ROS_INFO("Allied Narrow camera configuration is available");
    else
        return 0;

    dynamic_reconfigure::Server<l3cam_ros::AlliedNarrowCameraConfig> server;
    dynamic_reconfigure::Server<l3cam_ros::AlliedNarrowCameraConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    clientExposureTime = nh.serviceClient<l3cam_ros::ChangeAlliedCameraExposureTime>("change_allied_camera_exposure_time");
    clientEnableAutoExposureTime = nh.serviceClient<l3cam_ros::EnableAlliedCameraAutoExposureTime>("enable_allied_camera_auto_exposure_time");
    clientAutoExposureTimeRange = nh.serviceClient<l3cam_ros::ChangeAlliedCameraAutoExposureTimeRange>("change_allied_camera_auto_exposure_time_range");
    clientGain = nh.serviceClient<l3cam_ros::ChangeAlliedCameraGain>("change_allied_camera_gain");
    clientEnableAutoGain = nh.serviceClient<l3cam_ros::EnableAlliedCameraAutoGain>("enable_allied_camera_auto_gain");
    clientAutoGainRange = nh.serviceClient<l3cam_ros::ChangeAlliedCameraAutoGainRange>("change_allied_camera_auto_gain_range");
    clientGamma = nh.serviceClient<l3cam_ros::ChangeAlliedCameraGamma>("change_allied_camera_gamma");
    clientSaturation = nh.serviceClient<l3cam_ros::ChangeAlliedCameraSaturation>("change_allied_camera_saturation");
    clientHue = nh.serviceClient<l3cam_ros::ChangeAlliedCameraHue>("change_allied_camera_hue");
    clientIntensityAutoPrecedence = nh.serviceClient<l3cam_ros::ChangeAlliedCameraIntensityAutoPrecedence>("change_allied_camera_intensity_auto_precedence");
    clientEnableAutoWhiteBalance = nh.serviceClient<l3cam_ros::EnableAlliedCameraAutoWhiteBalance>("enable_allied_camera_auto_white_balance");
    clientBalanceRatioSelector = nh.serviceClient<l3cam_ros::ChangeAlliedCameraBalanceRatioSelector>("change_allied_camera_balance_ratio_selector");
    clientBalanceRatio = nh.serviceClient<l3cam_ros::ChangeAlliedCameraBalanceRatio>("change_allied_camera_balance_ratio");
    clientBalanceWhiteAutoRate = nh.serviceClient<l3cam_ros::ChangeAlliedCameraBalanceWhiteAutoRate>("change_allied_camera_balance_white_auto_rate");
    clientBalanceWhiteAutoTolerance = nh.serviceClient<l3cam_ros::ChangeAlliedCameraBalanceWhiteAutoTolerance>("change_allied_camera_balance_white_auto_tolerance");
    clientIntensityControllerRegion = nh.serviceClient<l3cam_ros::ChangeAlliedCameraIntensityControllerRegion>("change_allied_camera_intensity_controller_region");
    clientIntensityControllerTarget = nh.serviceClient<l3cam_ros::ChangeAlliedCameraIntensityControllerTarget>("change_allied_camera_intensity_controller_target");

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}