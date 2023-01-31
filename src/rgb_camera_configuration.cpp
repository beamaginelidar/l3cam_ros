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
#include "l3cam_ros/RgbCameraConfig.h"

#include "libL3Cam.h"
#include "beamagine.h"
#include "beamErrors.h"

#include "l3cam_ros/GetSensorsAvaliable.h"
#include "l3cam_ros/ChangeRgbCameraBrightness.h"
#include "l3cam_ros/ChangeRgbCameraContrast.h"
#include "l3cam_ros/ChangeRgbCameraSaturation.h"
#include "l3cam_ros/ChangeRgbCameraSharpness.h"
#include "l3cam_ros/ChangeRgbCameraGamma.h"
#include "l3cam_ros/ChangeRgbCameraGain.h"
#include "l3cam_ros/EnableRgbCameraAutoWhiteBalance.h"
#include "l3cam_ros/ChangeRgbCameraWhiteBalance.h"
#include "l3cam_ros/EnableRgbCameraAutoExposureTime.h"
#include "l3cam_ros/ChangeRgbCameraExposureTime.h"

ros::ServiceClient clientGetSensors;
l3cam_ros::GetSensorsAvaliable srvGetSensors;
ros::ServiceClient clientBrightness;
l3cam_ros::ChangeRgbCameraBrightness srvBrightness;
ros::ServiceClient clientContrast;
l3cam_ros::ChangeRgbCameraContrast srvContrast;
ros::ServiceClient clientSaturation;
l3cam_ros::ChangeRgbCameraSaturation srvSaturation;
ros::ServiceClient clientSharpness;
l3cam_ros::ChangeRgbCameraSharpness srvSharpness;
ros::ServiceClient clientGamma;
l3cam_ros::ChangeRgbCameraGamma srvGamma;
ros::ServiceClient clientGain;
l3cam_ros::ChangeRgbCameraGain srvGain;
ros::ServiceClient clientEnableAutoWhiteBalance;
l3cam_ros::EnableRgbCameraAutoWhiteBalance srvEnableAutoWhiteBalance;
ros::ServiceClient clientWhiteBalance;
l3cam_ros::ChangeRgbCameraWhiteBalance srvWhiteBalance;
ros::ServiceClient clientEnableAutoExposureTime;
l3cam_ros::EnableRgbCameraAutoExposureTime srvEnableAutoExposureTime;
ros::ServiceClient clientExposureTime;
l3cam_ros::ChangeRgbCameraExposureTime srvExposureTime;

int change_rgb_camera_brightness;
int change_rgb_camera_contrast;
int change_rgb_camera_saturation;
int change_rgb_camera_sharpness;
int change_rgb_camera_gamma;
int change_rgb_camera_gain;
bool enable_rgb_camera_auto_white_balance;
int change_rgb_camera_white_balance;
bool enable_rgb_camera_auto_exposure_time;
int change_rgb_camera_exposure_time;

void callback(l3cam_ros::RgbCameraConfig &config, uint32_t level)
{
    int error = L3CAM_OK;

    if (config.change_rgb_camera_brightness != change_rgb_camera_brightness)
    {
        srvBrightness.request.brightness = config.change_rgb_camera_brightness;
        if (clientBrightness.call(srvBrightness))
        {
            error = srvBrightness.response.error;
            if (!error)
                change_rgb_camera_brightness = config.change_rgb_camera_brightness;
            else
                config.change_rgb_camera_brightness = change_rgb_camera_brightness;
        }
        else
        {
            ROS_ERROR("Failed to call service change_rgb_camera_brightness");
            config.change_rgb_camera_brightness = change_rgb_camera_brightness;
        }
    }
    if (config.change_rgb_camera_contrast != change_rgb_camera_contrast)
    {
        srvContrast.request.contrast = config.change_rgb_camera_contrast;
        if (clientContrast.call(srvContrast))
        {
            error = srvContrast.response.error;
            if (!error)
                change_rgb_camera_contrast = config.change_rgb_camera_contrast;
            else
                config.change_rgb_camera_contrast = change_rgb_camera_contrast;
        }
        else
        {
            ROS_ERROR("Failed to call service change_rgb_camera_contrast");
            config.change_rgb_camera_contrast = change_rgb_camera_contrast;
        }
    }
    if (config.change_rgb_camera_saturation != change_rgb_camera_saturation)
    {
        srvSaturation.request.saturation = config.change_rgb_camera_saturation;
        if (clientSaturation.call(srvSaturation))
        {
            error = srvSaturation.response.error;
            if (!error)
                change_rgb_camera_saturation = config.change_rgb_camera_saturation;
            else
                config.change_rgb_camera_saturation = change_rgb_camera_saturation;
        }
        else
        {
            ROS_ERROR("Failed to call service change_rgb_camera_saturation");
            config.change_rgb_camera_saturation = change_rgb_camera_saturation;
        }
    }
    if (config.change_rgb_camera_sharpness != change_rgb_camera_sharpness)
    {
        srvSharpness.request.sharpness = config.change_rgb_camera_sharpness;
        if (clientSharpness.call(srvSharpness))
        {
            error = srvSharpness.response.error;
            if (!error)
                change_rgb_camera_sharpness = config.change_rgb_camera_sharpness;
            else
                config.change_rgb_camera_sharpness = change_rgb_camera_sharpness;
        }
        else
        {
            ROS_ERROR("Failed to call service change_rgb_camera_sharpness");
            config.change_rgb_camera_sharpness = change_rgb_camera_sharpness;
        }
    }
    if (config.change_rgb_camera_gamma != change_rgb_camera_gamma)
    {
        srvGamma.request.gamma = config.change_rgb_camera_gamma;
        if (clientGamma.call(srvGamma))
        {
            error = srvGamma.response.error;
            if (!error)
                change_rgb_camera_gamma = config.change_rgb_camera_gamma;
            else
                config.change_rgb_camera_gamma = change_rgb_camera_gamma;
        }
        else
        {
            ROS_ERROR("Failed to call service change_rgb_camera_gamma");
            config.change_rgb_camera_gamma = change_rgb_camera_gamma;
        }
    }
    if (config.change_rgb_camera_gain != change_rgb_camera_gain)
    {
        srvGain.request.gain = config.change_rgb_camera_gain;
        if (clientGain.call(srvGain))
        {
            error = srvGain.response.error;
            if (!error)
                change_rgb_camera_gain = config.change_rgb_camera_gain;
            else
                config.change_rgb_camera_gain = change_rgb_camera_gain;
        }
        else
        {
            ROS_ERROR("Failed to call service change_rgb_camera_gain");
            config.change_rgb_camera_gain = change_rgb_camera_gain;
        }
    }
    if (config.enable_rgb_camera_auto_white_balance != enable_rgb_camera_auto_white_balance)
    {
        srvEnableAutoWhiteBalance.request.enabled = config.enable_rgb_camera_auto_white_balance;
        if (clientEnableAutoWhiteBalance.call(srvEnableAutoWhiteBalance))
        {
            error = srvEnableAutoWhiteBalance.response.error;
            if (!error)
                enable_rgb_camera_auto_white_balance = config.enable_rgb_camera_auto_white_balance;
            else
                config.enable_rgb_camera_auto_white_balance = enable_rgb_camera_auto_white_balance;
        }
        else
        {
            ROS_ERROR("Failed to call service enable_rgb_camera_auto_white_balance");
            config.enable_rgb_camera_auto_white_balance = enable_rgb_camera_auto_white_balance;
        }
    }
    if (config.change_rgb_camera_white_balance != change_rgb_camera_white_balance)
    {
        if (!enable_rgb_camera_auto_white_balance)
        {
            srvWhiteBalance.request.white_balance = config.change_rgb_camera_white_balance;
            if (clientWhiteBalance.call(srvWhiteBalance))
            {
                error = srvWhiteBalance.response.error;
                if (!error)
                    change_rgb_camera_white_balance = config.change_rgb_camera_white_balance;
                else
                    config.change_rgb_camera_white_balance = change_rgb_camera_white_balance;
            }
            else
            {
                ROS_ERROR("Failed to call service change_rgb_camera_white_balance");
                config.change_rgb_camera_white_balance = change_rgb_camera_white_balance;
            }
        }
        else
        {
            ROS_INFO("RGB camera auto white balance must be disabled to change white balance");
            config.change_rgb_camera_white_balance = change_rgb_camera_white_balance;
        }
    }
    if (config.enable_rgb_camera_auto_exposure_time != enable_rgb_camera_auto_exposure_time)
    {
        srvEnableAutoExposureTime.request.enabled = config.enable_rgb_camera_auto_exposure_time;
        if (clientEnableAutoExposureTime.call(srvEnableAutoExposureTime))
        {
            error = srvEnableAutoExposureTime.response.error;
            if (!error)
                enable_rgb_camera_auto_exposure_time = config.enable_rgb_camera_auto_exposure_time;
            else
                config.enable_rgb_camera_auto_exposure_time = enable_rgb_camera_auto_exposure_time;
        }
        else
        {
            ROS_ERROR("Failed to call service enable_rgb_camera_auto_exposure_time");
            config.enable_rgb_camera_auto_exposure_time = enable_rgb_camera_auto_exposure_time;
        }
    }
    if (config.change_rgb_camera_exposure_time != change_rgb_camera_exposure_time)
    {
        if (!enable_rgb_camera_auto_exposure_time)
        {
            srvExposureTime.request.exposure_time = config.change_rgb_camera_exposure_time;
            if (clientExposureTime.call(srvExposureTime))
            {
                error = srvExposureTime.response.error;
                if (!error)
                    change_rgb_camera_exposure_time = config.change_rgb_camera_exposure_time;
                else
                    config.change_rgb_camera_exposure_time = change_rgb_camera_exposure_time;
            }
            else
            {
                ROS_ERROR("Failed to call service change_rgb_camera_exposure_time");
                config.change_rgb_camera_exposure_time = change_rgb_camera_exposure_time;
            }
        }
        else
        {
            ROS_INFO("RGB camera auto exposure time must be disabled to change exposure time");
            config.change_rgb_camera_exposure_time = change_rgb_camera_exposure_time;
        }
    }

    if (error)
        ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgb_camera_configuration");
    ros::NodeHandle nh;

    // Params
    nh.param<int>("rgb_camera_brightness", change_rgb_camera_brightness, 0);
    nh.param<int>("rgb_camera_contrast", change_rgb_camera_contrast, 10);
    nh.param<int>("rgb_camera_saturation", change_rgb_camera_saturation, 16);
    nh.param<int>("rgb_camera_sharpness", change_rgb_camera_sharpness, 16);
    nh.param<int>("rgb_camera_gamma", change_rgb_camera_gamma, 220);
    nh.param<int>("rgb_camera_gain", change_rgb_camera_gain, 0);
    nh.param<bool>("rgb_camera_auto_white_balance", enable_rgb_camera_auto_white_balance, true);
    nh.param<int>("rgb_camera_white_balance", change_rgb_camera_white_balance, 5000);
    nh.param<bool>("rgb_camera_auto_exposure_time", enable_rgb_camera_auto_exposure_time, true);
    nh.param<int>("rgb_camera_exposure_time", change_rgb_camera_exposure_time, 156);

    clientGetSensors = nh.serviceClient<l3cam_ros::GetSensorsAvaliable>("get_sensors_avaliable");
    int error = L3CAM_OK;

    bool sensor_is_avaliable = false;
    if (clientGetSensors.call(srvGetSensors))
    {
        error = srvGetSensors.response.error;

        if (!error)
            for (int i = 0; i < srvGetSensors.response.num_sensors; ++i)
            {
                if (srvGetSensors.response.sensors[i].sensor_type == sensor_econ_rgb)
                    sensor_is_avaliable = true;
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

    if (sensor_is_avaliable)
        ROS_INFO("RGB camera is avaliable");
    else
        return 0;

    dynamic_reconfigure::Server<l3cam_ros::RgbCameraConfig> server;
    dynamic_reconfigure::Server<l3cam_ros::RgbCameraConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    clientBrightness = nh.serviceClient<l3cam_ros::ChangeRgbCameraBrightness>("change_rgb_camera_brightness");
    clientContrast = nh.serviceClient<l3cam_ros::ChangeRgbCameraContrast>("change_rgb_camera_contrast");
    clientSaturation = nh.serviceClient<l3cam_ros::ChangeRgbCameraSaturation>("change_rgb_camera_saturation");
    clientSharpness = nh.serviceClient<l3cam_ros::ChangeRgbCameraSharpness>("change_rgb_camera_sharpness");
    clientGamma = nh.serviceClient<l3cam_ros::ChangeRgbCameraGamma>("change_rgb_camera_gamma");
    clientGain = nh.serviceClient<l3cam_ros::ChangeRgbCameraGain>("change_rgb_camera_gain");
    clientEnableAutoWhiteBalance = nh.serviceClient<l3cam_ros::EnableRgbCameraAutoWhiteBalance>("enable_rgb_camera_auto_white_balance");
    clientWhiteBalance = nh.serviceClient<l3cam_ros::ChangeRgbCameraWhiteBalance>("change_rgb_camera_white_balance");
    clientEnableAutoExposureTime = nh.serviceClient<l3cam_ros::EnableRgbCameraAutoExposureTime>("enable_rgb_camera_auto_exposure_time");
    clientExposureTime = nh.serviceClient<l3cam_ros::ChangeRgbCameraExposureTime>("change_rgb_camera_exposure_time");

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
