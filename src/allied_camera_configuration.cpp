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
#include "l3cam_ros/AlliedCameraConfig.h"

#include <libL3Cam.h>
#include <beamagine.h>
#include <beamErrors.h>

#include "l3cam_ros/GetSensorsAvaliable.h"
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

ros::ServiceClient clientGetSensors;
l3cam_ros::GetSensorsAvaliable srvGetSensors;
ros::ServiceClient clientBlackLevel;
l3cam_ros::ChangeAlliedCameraBlackLevel srvBlackLevel;
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
ros::ServiceClient clientSharpness;
l3cam_ros::ChangeAlliedCameraSharpness srvSharpness;
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
ros::ServiceClient clientAutoModeRegion;
l3cam_ros::ChangeAlliedCameraAutoModeRegion srvAutoModeRegion;
ros::ServiceClient clientIntensityControllerRegion;
l3cam_ros::ChangeAlliedCameraIntensityControllerRegion srvIntensityControllerRegion;
ros::ServiceClient clientIntensityControllerTarget;
l3cam_ros::ChangeAlliedCameraIntensityControllerTarget srvIntensityControllerTarget;
ros::ServiceClient clientMaxDriverBuffersCount;
l3cam_ros::ChangeAlliedCameraMaxDriverBuffersCount srvMaxDriverBuffersCount;

bool sensor_wide_is_avaliable = false;
bool sensor_narrow_is_avaliable = false;

int allied_camera_type = 1;
float change_allied_camera_black_level = 0;
float change_allied_camera_exposure_time = 4992.32;
bool enable_allied_camera_auto_exposure_time = false;
float change_allied_camera_auto_exposure_time_range_min = 87.596;
float change_allied_camera_auto_exposure_time_range_max = 87.596;
float change_allied_camera_gain = 0;
bool enable_allied_camera_auto_gain = false;
float change_allied_camera_auto_gain_range_min = 0;
float change_allied_camera_auto_gain_range_max = 48;
float change_allied_camera_gamma = 1;
float change_allied_camera_saturation = 1;
float change_allied_camera_sharpness = 0;
float change_allied_camera_hue = 0;
int change_allied_camera_intensity_auto_precedence = 1;
bool enable_allied_camera_auto_white_balance = false;
int change_allied_camera_balance_ratio_selector = 1;
float change_allied_camera_balance_ratio = 2.35498;
float change_allied_camera_balance_white_auto_rate = 100;
float change_allied_camera_balance_white_auto_tolerance = 5;
float change_allied_camera_auto_mode_region_height = 2056;
float change_allied_camera_auto_mode_region_width = 2464;
int change_allied_camera_intensity_controller_region = 1;
float change_allied_camera_intensity_controller_target = 50;
int change_allied_camera_max_driver_buffers_count = 64;

void callback(l3cam_ros::AlliedCameraConfig &config, uint32_t level)
{
    int error = L3CAM_OK;

    switch (level)
    {
    case 0:
        if (config.allied_camera_type == 1 && !sensor_wide_is_avaliable)
            config.allied_camera_type = allied_camera_type;
        if (config.allied_camera_type == 2 && !sensor_narrow_is_avaliable)
            config.allied_camera_type = allied_camera_type;
        break;
    case 1:
        srvBlackLevel.request.black_level = config.change_allied_camera_black_level; //! Can only be changed while not streaming
        srvBlackLevel.request.allied_type = config.allied_camera_type;
        if (clientBlackLevel.call(srvBlackLevel))
        {
            error = srvBlackLevel.response.error;
            if (!error)
                change_allied_camera_black_level = config.change_allied_camera_black_level;
            else
            {
                config.change_allied_camera_black_level = change_allied_camera_black_level;
                ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
            }
        }
        else
        {
            ROS_ERROR("Failed to call service change_allied_camera_black_level");
            config.change_allied_camera_black_level = change_allied_camera_black_level;
        }
        break;
    case 2:
        srvExposureTime.request.exposure_time = config.change_allied_camera_exposure_time;
        srvExposureTime.request.allied_type = config.allied_camera_type;
        if (clientExposureTime.call(srvExposureTime))
        {
            error = srvExposureTime.response.error;
            if (!error)
                change_allied_camera_exposure_time = config.change_allied_camera_exposure_time;
            else
            {
                config.change_allied_camera_exposure_time = change_allied_camera_exposure_time;
                ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
            }
        }
        else
        {
            ROS_ERROR("Failed to call service change_allied_camera_exposure_time");
            config.change_allied_camera_exposure_time = change_allied_camera_exposure_time;
        }
        break;
    case 3:
        srvEnableAutoExposureTime.request.enabled = config.enable_allied_camera_auto_exposure_time;
        srvEnableAutoExposureTime.request.allied_type = config.allied_camera_type;
        if (clientEnableAutoExposureTime.call(srvEnableAutoExposureTime))
        {
            error = srvEnableAutoExposureTime.response.error;
            if (!error)
                enable_allied_camera_auto_exposure_time = config.enable_allied_camera_auto_exposure_time;
            else
            {
                config.enable_allied_camera_auto_exposure_time = enable_allied_camera_auto_exposure_time;
                ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
            }
        }
        else
        {
            ROS_ERROR("Failed to call service enable_allied_camera_auto_exposure_time");
            config.enable_allied_camera_auto_exposure_time = enable_allied_camera_auto_exposure_time;
        }
        break;
    case 4:
        srvAutoExposureTimeRange.request.auto_exposure_time_range_min = config.change_allied_camera_auto_exposure_time_range_min;
        srvAutoExposureTimeRange.request.allied_type = config.allied_camera_type;
        srvAutoExposureTimeRange.request.auto_exposure_time_range_max = config.change_allied_camera_auto_exposure_time_range_max;
        if (clientAutoExposureTimeRange.call(srvAutoExposureTimeRange))
        {
            error = srvAutoExposureTimeRange.response.error;
            if (!error)
                change_allied_camera_auto_exposure_time_range_min = config.change_allied_camera_auto_exposure_time_range_min;
            else
            {
                config.change_allied_camera_auto_exposure_time_range_min = change_allied_camera_auto_exposure_time_range_min;
                ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
            }
        }
        else
        {
            ROS_ERROR("Failed to call service change_allied_camera_auto_exposure_time_range_min");
            config.change_allied_camera_auto_exposure_time_range_min = change_allied_camera_auto_exposure_time_range_min;
        }
        break;
    case 5:
        srvAutoExposureTimeRange.request.auto_exposure_time_range_min = config.change_allied_camera_auto_exposure_time_range_min;
        srvAutoExposureTimeRange.request.allied_type = config.allied_camera_type;
        srvAutoExposureTimeRange.request.auto_exposure_time_range_max = config.change_allied_camera_auto_exposure_time_range_max;
        if (clientAutoExposureTimeRange.call(srvAutoExposureTimeRange))
        {
            error = srvAutoExposureTimeRange.response.error;
            if (!error)
                change_allied_camera_auto_exposure_time_range_max = config.change_allied_camera_auto_exposure_time_range_max;
            else
            {
                config.change_allied_camera_auto_exposure_time_range_max = change_allied_camera_auto_exposure_time_range_max;
                ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
            }
        }
        else
        {
            ROS_ERROR("Failed to call service change_allied_camera_auto_exposure_time_range_max");
            config.change_allied_camera_auto_exposure_time_range_max = change_allied_camera_auto_exposure_time_range_max;
        }
        break;
    case 6:
        srvGain.request.gain = config.change_allied_camera_gain;
        srvGain.request.allied_type = config.allied_camera_type;
        if (clientGain.call(srvGain))
        {
            error = srvGain.response.error;
            if (!error)
            {
                change_allied_camera_gain = config.change_allied_camera_gain;
            }
            else
            {
                config.change_allied_camera_gain = change_allied_camera_gain;
                ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
            }
        }
        else
        {
            ROS_ERROR("Failed to call service change_allied_camera_gain");
            config.change_allied_camera_gain = change_allied_camera_gain;
        }
        break;
    case 7:
        srvEnableAutoGain.request.enabled = config.enable_allied_camera_auto_gain;
        srvEnableAutoGain.request.allied_type = config.allied_camera_type;
        if (clientEnableAutoGain.call(srvEnableAutoGain))
        {
            error = srvEnableAutoGain.response.error;
            if (!error)
                enable_allied_camera_auto_gain = config.enable_allied_camera_auto_gain;
            else
            {
                config.enable_allied_camera_auto_gain = enable_allied_camera_auto_gain;
                ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
            }
        }
        else
        {
            ROS_ERROR("Failed to call service enable_allied_camera_auto_gain");
            config.enable_allied_camera_auto_gain = enable_allied_camera_auto_gain;
        }
        break;
    case 8:
        srvAutoGainRange.request.auto_gain_range_min = config.change_allied_camera_auto_gain_range_min;
        srvAutoGainRange.request.allied_type = config.allied_camera_type;
        srvAutoGainRange.request.auto_gain_range_max = config.change_allied_camera_auto_gain_range_max;
        if (clientAutoGainRange.call(srvAutoGainRange))
        {
            error = srvAutoGainRange.response.error;
            if (!error)
                change_allied_camera_auto_gain_range_min = config.change_allied_camera_auto_gain_range_min;
            else
            {
                config.change_allied_camera_auto_gain_range_min = change_allied_camera_auto_gain_range_min;
                ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
            }
        }
        else
        {
            ROS_ERROR("Failed to call service change_allied_camera_auto_gain_range_min");
            config.change_allied_camera_auto_gain_range_min = change_allied_camera_auto_gain_range_min;
        }
        break;
    case 9:
        srvAutoGainRange.request.auto_gain_range_min = config.change_allied_camera_auto_gain_range_min;
        srvAutoGainRange.request.allied_type = config.allied_camera_type;
        srvAutoGainRange.request.auto_gain_range_max = config.change_allied_camera_auto_gain_range_max;
        if (clientAutoGainRange.call(srvAutoGainRange))
        {
            error = srvAutoGainRange.response.error;
            if (!error)
                change_allied_camera_auto_gain_range_max = config.change_allied_camera_auto_gain_range_max;
            else
            {
                config.change_allied_camera_auto_gain_range_max = change_allied_camera_auto_gain_range_max;
                ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
            }
        }
        else
        {
            ROS_ERROR("Failed to call service change_allied_camera_auto_gain_range_max");
            config.change_allied_camera_auto_gain_range_max = change_allied_camera_auto_gain_range_max;
        }
        break;
    case 10:
        srvGamma.request.gamma = config.change_allied_camera_gamma;
        srvGamma.request.allied_type = config.allied_camera_type;
        if (clientGamma.call(srvGamma))
        {
            error = srvGamma.response.error;
            if (!error)
                change_allied_camera_gamma = config.change_allied_camera_gamma;
            else
            {
                config.change_allied_camera_gamma = change_allied_camera_gamma;
                ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
            }
        }
        else
        {
            ROS_ERROR("Failed to call service change_allied_camera_gamma");
            config.change_allied_camera_gamma = change_allied_camera_gamma;
        }
        break;
    case 11:
        srvSaturation.request.saturation = config.change_allied_camera_saturation;
        srvSaturation.request.allied_type = config.allied_camera_type;
        if (clientSaturation.call(srvSaturation))
        {
            error = srvSaturation.response.error;
            if (!error)
                change_allied_camera_saturation = config.change_allied_camera_saturation;
            else
            {
                config.change_allied_camera_saturation = change_allied_camera_saturation;
                ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
            }
        }
        else
        {
            ROS_ERROR("Failed to call service change_allied_camera_saturation");
            config.change_allied_camera_saturation = change_allied_camera_saturation;
        }
        break;
    case 12:
        srvSharpness.request.sharpness = config.change_allied_camera_sharpness;
        srvSharpness.request.allied_type = config.allied_camera_type;
        if (clientSharpness.call(srvSharpness))
        {
            error = srvSharpness.response.error;
            if (!error)
                change_allied_camera_sharpness = config.change_allied_camera_sharpness;
            else
            {
                config.change_allied_camera_sharpness = change_allied_camera_sharpness;
                ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
            }
        }
        else
        {
            ROS_ERROR("Failed to call service change_allied_camera_sharpness");
            config.change_allied_camera_sharpness = change_allied_camera_sharpness;
        }
        break;
    case 13:
        srvHue.request.hue = config.change_allied_camera_hue;
        srvHue.request.allied_type = config.allied_camera_type;
        if (clientHue.call(srvHue))
        {
            error = srvHue.response.error;
            if (!error)
                change_allied_camera_hue = config.change_allied_camera_hue;
            else
            {
                config.change_allied_camera_hue = change_allied_camera_hue;
                ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
            }
        }
        else
        {
            ROS_ERROR("Failed to call service change_allied_camera_hue");
            config.change_allied_camera_hue = change_allied_camera_hue;
        }
        break;
    case 14:
        srvIntensityAutoPrecedence.request.intensity_auto_precedence = config.change_allied_camera_intensity_auto_precedence;
        srvIntensityAutoPrecedence.request.allied_type = config.allied_camera_type;
        if (clientIntensityAutoPrecedence.call(srvIntensityAutoPrecedence))
        {
            error = srvIntensityAutoPrecedence.response.error;
            if (!error)
                change_allied_camera_intensity_auto_precedence = config.change_allied_camera_intensity_auto_precedence;
            else
            {
                config.change_allied_camera_intensity_auto_precedence = change_allied_camera_intensity_auto_precedence;
                ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
            }
        }
        else
        {
            ROS_ERROR("Failed to call service change_allied_camera_intensity_auto_precedence");
            config.change_allied_camera_intensity_auto_precedence = change_allied_camera_intensity_auto_precedence;
        }
        break;
    case 15:
        srvEnableAutoWhiteBalance.request.enabled = config.change_allied_camera_intensity_auto_precedence;
        srvEnableAutoWhiteBalance.request.allied_type = config.allied_camera_type;
        if (clientEnableAutoWhiteBalance.call(srvEnableAutoWhiteBalance))
        {
            error = srvEnableAutoWhiteBalance.response.error;
            if (!error)
                change_allied_camera_intensity_auto_precedence = config.change_allied_camera_intensity_auto_precedence;
            else
            {
                config.change_allied_camera_intensity_auto_precedence = change_allied_camera_intensity_auto_precedence;
                ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
            }
        }
        else
        {
            ROS_ERROR("Failed to call service change_allied_camera_intensity_auto_precedence");
            config.change_allied_camera_intensity_auto_precedence = change_allied_camera_intensity_auto_precedence;
        }
        break;
    case 16:
        srvBalanceRatioSelector.request.white_balance_ratio_selector = config.change_allied_camera_white_balance_ratio_selector;
        srvBalanceRatioSelector.request.allied_type = config.allied_camera_type;
        if (clientBalanceRatioSelector.call(srvBalanceRatioSelector))
        {
            error = srvBalanceRatioSelector.response.error;
            if (!error)
                change_allied_camera_balance_ratio_selector = config.change_allied_camera_white_balance_ratio_selector;
            else
            {
                config.change_allied_camera_white_balance_ratio_selector = change_allied_camera_balance_ratio_selector;
                ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
            }
        }
        else
        {
            ROS_ERROR("Failed to call service change_allied_camera_balance_ratio_selector");
            config.change_allied_camera_white_balance_ratio_selector = change_allied_camera_balance_ratio_selector;
        }
        break;
    case 17:
        srvBalanceRatio.request.balance_ratio = config.change_allied_camera_balance_ratio;
        srvBalanceRatio.request.allied_type = config.allied_camera_type;
        if (clientBalanceRatio.call(srvBalanceRatio))
        {
            error = srvBalanceRatio.response.error;
            if (!error)
                change_allied_camera_balance_ratio = config.change_allied_camera_balance_ratio;
            else
            {
                config.change_allied_camera_balance_ratio = change_allied_camera_balance_ratio;
                ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
            }
        }
        else
        {
            ROS_ERROR("Failed to call service change_allied_camera_balance_ratio");
            config.change_allied_camera_balance_ratio = change_allied_camera_balance_ratio;
        }
        break;
    case 18:
        srvBalanceWhiteAutoRate.request.white_balance_auto_rate = config.change_allied_camera_white_balance_auto_rate;
        srvBalanceWhiteAutoRate.request.allied_type = config.allied_camera_type;
        if (clientBalanceWhiteAutoRate.call(srvBalanceWhiteAutoRate))
        {
            error = srvBalanceWhiteAutoRate.response.error;
            if (!error)
                change_allied_camera_balance_white_auto_rate = config.change_allied_camera_white_balance_auto_rate;
            else
            {
                config.change_allied_camera_white_balance_auto_rate = change_allied_camera_balance_white_auto_rate;
                ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
            }
        }
        else
        {
            ROS_ERROR("Failed to call service change_allied_camera_balance_white_auto_rate");
            config.change_allied_camera_white_balance_auto_rate = change_allied_camera_balance_white_auto_rate;
        }
        break;
    case 19:
        srvBalanceWhiteAutoTolerance.request.white_balance_auto_tolerance = config.change_allied_camera_white_balance_auto_tolerance;
        srvBalanceWhiteAutoTolerance.request.allied_type = config.allied_camera_type;
        if (clientBalanceWhiteAutoTolerance.call(srvBalanceWhiteAutoTolerance))
        {
            error = srvBalanceWhiteAutoTolerance.response.error;
            if (!error)
                change_allied_camera_balance_white_auto_tolerance = config.change_allied_camera_white_balance_auto_tolerance;
            else
            {
                config.change_allied_camera_white_balance_auto_tolerance = change_allied_camera_balance_white_auto_tolerance;
                ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
            }
        }
        else
        {
            ROS_ERROR("Failed to call service change_allied_camera_balance_white_auto_tolerance");
            config.change_allied_camera_white_balance_auto_tolerance = change_allied_camera_balance_white_auto_tolerance;
        }
        break;
    case 20:
        srvAutoModeRegion.request.auto_mode_region_height = config.change_allied_camera_auto_mode_region_height;
        srvAutoModeRegion.request.allied_type = config.allied_camera_type;
        srvAutoModeRegion.request.auto_mode_region_width = config.change_allied_camera_auto_mode_region_width;
        if (clientAutoModeRegion.call(srvAutoModeRegion))
        {
            error = srvAutoModeRegion.response.error;
            if (!error)
                change_allied_camera_auto_mode_region_height = config.change_allied_camera_auto_mode_region_height;
            else
            {
                config.change_allied_camera_auto_mode_region_height = change_allied_camera_auto_mode_region_height;
                ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
            }
        }
        else
        {
            ROS_ERROR("Failed to call service change_allied_camera_auto_mode_region_height");
            config.change_allied_camera_auto_mode_region_height = change_allied_camera_auto_mode_region_height;
        }
        break;
    case 21:
        srvAutoModeRegion.request.auto_mode_region_height = config.change_allied_camera_auto_mode_region_height;
        srvAutoModeRegion.request.allied_type = config.allied_camera_type;
        srvAutoModeRegion.request.auto_mode_region_width = config.change_allied_camera_auto_mode_region_width;
        if (clientAutoModeRegion.call(srvAutoModeRegion))
        {
            error = srvAutoModeRegion.response.error;
            if (!error)
                change_allied_camera_auto_mode_region_width = config.change_allied_camera_auto_mode_region_width;
            else
            {
                config.change_allied_camera_auto_mode_region_width = change_allied_camera_auto_mode_region_width;
                ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
            }
        }
        else
        {
            ROS_ERROR("Failed to call service change_allied_camera_auto_mode_region_width");
            config.change_allied_camera_auto_mode_region_width = change_allied_camera_auto_mode_region_width;
        }
        break;
    case 22:
        srvIntensityControllerRegion.request.intensity_controller_region = config.change_allied_camera_intensity_controller_region;
        srvIntensityControllerRegion.request.allied_type = config.allied_camera_type;
        if (clientIntensityControllerRegion.call(srvIntensityControllerRegion))
        {
            error = srvIntensityControllerRegion.response.error;
            if (!error)
                change_allied_camera_intensity_controller_region = config.change_allied_camera_intensity_controller_region;
            else
            {
                config.change_allied_camera_intensity_controller_region = change_allied_camera_intensity_controller_region;
                ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
            }
        }
        else
        {
            ROS_ERROR("Failed to call service change_allied_camera_intensity_controller_region");
            config.change_allied_camera_intensity_controller_region = change_allied_camera_intensity_controller_region;
        }
        break;
    case 23:
        srvIntensityControllerTarget.request.intensity_controller_target = config.change_allied_camera_intensity_controller_target;
        srvIntensityControllerTarget.request.allied_type = config.allied_camera_type;
        if (clientIntensityControllerTarget.call(srvIntensityControllerTarget))
        {
            error = srvIntensityControllerTarget.response.error;
            if (!error)
                change_allied_camera_intensity_controller_target = config.change_allied_camera_intensity_controller_target;
            else
            {
                config.change_allied_camera_intensity_controller_target = change_allied_camera_intensity_controller_target;
                ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
            }
        }
        else
        {
            ROS_ERROR("Failed to call service change_allied_camera_intensity_controller_target");
            config.change_allied_camera_intensity_controller_target = change_allied_camera_intensity_controller_target;
        }
        break;
    case 24:
        srvMaxDriverBuffersCount.request.max_driver_buffers_count = config.change_allied_camera_max_driver_buffers_count;
        srvMaxDriverBuffersCount.request.allied_type = config.allied_camera_type;
        if (clientMaxDriverBuffersCount.call(srvMaxDriverBuffersCount))
        {
            error = srvMaxDriverBuffersCount.response.error;
            if (!error)
                change_allied_camera_max_driver_buffers_count = config.change_allied_camera_max_driver_buffers_count;
            else
            {
                config.change_allied_camera_max_driver_buffers_count = change_allied_camera_max_driver_buffers_count;
                ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
            }
        }
        else
        {
            ROS_ERROR("Failed to call service change_allied_camera_max_driver_buffers_count");
            config.change_allied_camera_max_driver_buffers_count = change_allied_camera_max_driver_buffers_count;
        }
        break;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "allied_camera_configuration");
    ros::NodeHandle nh;

    clientGetSensors = nh.serviceClient<l3cam_ros::GetSensorsAvaliable>("get_sensors_avaliable");
    int error = L3CAM_OK;

    if (clientGetSensors.call(srvGetSensors))
    {
        error = srvGetSensors.response.error;

        if (!error)
            for (int i = 0; i < srvGetSensors.response.num_sensors; ++i)
            {
                if (srvGetSensors.response.sensors[i].sensor_type == sensor_allied_narrow)
                    sensor_narrow_is_avaliable = true;
                if (srvGetSensors.response.sensors[i].sensor_type == sensor_allied_wide)
                    sensor_wide_is_avaliable = true;
            }
        else
        {
            ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
            return 1;
        }
    }
    else
    {
        ROS_ERROR("Failed to call service get_sensors_avaliable");
        return 1;
    }

    if (!sensor_wide_is_avaliable && !sensor_narrow_is_avaliable)
        return 0;

    if (sensor_wide_is_avaliable)
        ROS_INFO("Allied wide camera is avaliable");
    else
        allied_camera_type = 2;

    if (sensor_narrow_is_avaliable)
        ROS_INFO("Allied narrow camera is avaliable");

    dynamic_reconfigure::Server<l3cam_ros::AlliedCameraConfig> server;
    dynamic_reconfigure::Server<l3cam_ros::AlliedCameraConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    clientBlackLevel = nh.serviceClient<l3cam_ros::ChangeAlliedCameraBlackLevel>("change_allied_camera_black_level");
    clientExposureTime = nh.serviceClient<l3cam_ros::ChangeAlliedCameraExposureTime>("change_allied_camera_exposure_time");
    clientEnableAutoExposureTime = nh.serviceClient<l3cam_ros::EnableAlliedCameraAutoExposureTime>("enable_allied_camera_auto_exposure_time");
    clientAutoExposureTimeRange = nh.serviceClient<l3cam_ros::ChangeAlliedCameraAutoExposureTimeRange>("change_allied_camera_auto_exposure_time_range");
    clientGain = nh.serviceClient<l3cam_ros::ChangeAlliedCameraGain>("change_allied_camera_gain");
    clientEnableAutoGain = nh.serviceClient<l3cam_ros::EnableAlliedCameraAutoGain>("enable_allied_camera_auto_gain");
    clientAutoGainRange = nh.serviceClient<l3cam_ros::ChangeAlliedCameraAutoGainRange>("change_allied_camera_auto_gain_range");
    clientGamma = nh.serviceClient<l3cam_ros::ChangeAlliedCameraGamma>("change_allied_camera_gamma");
    clientSaturation = nh.serviceClient<l3cam_ros::ChangeAlliedCameraSaturation>("change_allied_camera_saturation");
    clientSharpness = nh.serviceClient<l3cam_ros::ChangeAlliedCameraSharpness>("change_allied_camera_sharpness");
    clientHue = nh.serviceClient<l3cam_ros::ChangeAlliedCameraHue>("change_allied_camera_hue");
    clientIntensityAutoPrecedence = nh.serviceClient<l3cam_ros::ChangeAlliedCameraIntensityAutoPrecedence>("change_allied_camera_intensity_auto_precedence");
    clientEnableAutoWhiteBalance = nh.serviceClient<l3cam_ros::EnableAlliedCameraAutoWhiteBalance>("enable_allied_camera_auto_white_balance");
    clientBalanceRatioSelector = nh.serviceClient<l3cam_ros::ChangeAlliedCameraBalanceRatioSelector>("change_allied_camera_balance_ratio_selector");
    clientBalanceRatio = nh.serviceClient<l3cam_ros::ChangeAlliedCameraBalanceRatio>("change_allied_camera_balance_ratio");
    clientBalanceWhiteAutoRate = nh.serviceClient<l3cam_ros::ChangeAlliedCameraBalanceWhiteAutoRate>("change_allied_camera_balance_white_auto_rate");
    clientBalanceWhiteAutoTolerance = nh.serviceClient<l3cam_ros::ChangeAlliedCameraBalanceWhiteAutoTolerance>("change_allied_camera_balance_white_auto_tolerance");
    clientAutoModeRegion = nh.serviceClient<l3cam_ros::ChangeAlliedCameraAutoModeRegion>("change_allied_camera_auto_mode_region");
    clientIntensityControllerRegion = nh.serviceClient<l3cam_ros::ChangeAlliedCameraIntensityControllerRegion>("change_allied_camera_intensity_controller_region");
    clientIntensityControllerTarget = nh.serviceClient<l3cam_ros::ChangeAlliedCameraIntensityControllerTarget>("change_allied_camera_intensity_controller_target");
    clientMaxDriverBuffersCount = nh.serviceClient<l3cam_ros::ChangeAlliedCameraMaxDriverBuffersCount>("change_allied_camera_max_driver_buffers_count");

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}