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
#include "l3cam_ros/ThermalCameraConfig.h"

#include <libL3Cam.h>
#include <beamagine.h>
#include <beamErrors.h>

#include "l3cam_ros/GetSensorsAvailable.h"
#include "l3cam_ros/ChangeThermalCameraColormap.h"
#include "l3cam_ros/EnableThermalCameraTemperatureFilter.h"
#include "l3cam_ros/ChangeThermalCameraTemperatureFilter.h"

ros::ServiceClient clientGetSensors;
l3cam_ros::GetSensorsAvailable srvGetSensors;
ros::ServiceClient clientColormap;
l3cam_ros::ChangeThermalCameraColormap srvColormap;
ros::ServiceClient clientEnableTemperatureFilter;
l3cam_ros::EnableThermalCameraTemperatureFilter srvEnableTemperatureFilter;
ros::ServiceClient clientTemperatureFilter;
l3cam_ros::ChangeThermalCameraTemperatureFilter srvTemperatureFilter;

int change_thermal_camera_colormap;
bool enable_thermal_camera_temperature_filter;
int change_thermal_camera_temperature_filter_min;
int change_thermal_camera_temperature_filter_max;

bool default_configured = false;

void configureDefault(l3cam_ros::ThermalCameraConfig &config)
{
    ros::param::param("/thermal_camera_configuration/thermal_camera_colormap", change_thermal_camera_colormap, 1);
    ros::param::param("/thermal_camera_configuration/thermal_camera_temperature_filter", enable_thermal_camera_temperature_filter, false);
    ros::param::param("/thermal_camera_configuration/thermal_camera_temperature_filter_min", change_thermal_camera_temperature_filter_min, 0);
    ros::param::param("/thermal_camera_configuration/thermal_camera_temperature_filter_max", change_thermal_camera_temperature_filter_max, 50);

    if (change_thermal_camera_colormap == 1 || change_thermal_camera_colormap == 2 ||
        change_thermal_camera_colormap >= 8 && change_thermal_camera_colormap <= 10 ||
        change_thermal_camera_colormap == 16 || change_thermal_camera_colormap == 17 || change_thermal_camera_colormap == 20 ||
        change_thermal_camera_colormap >= 100 && change_thermal_camera_colormap <= 108)
        config.change_thermal_camera_colormap = change_thermal_camera_colormap;
    else
        change_thermal_camera_colormap = config.change_thermal_camera_colormap;
    config.enable_thermal_camera_temperature_filter = enable_thermal_camera_temperature_filter;
    if (change_thermal_camera_temperature_filter_min >= -40 && change_thermal_camera_temperature_filter_min <= 200)
        config.change_thermal_camera_temperature_filter_min = change_thermal_camera_temperature_filter_min;
    else
        change_thermal_camera_temperature_filter_min = config.change_thermal_camera_temperature_filter_min;
    if (change_thermal_camera_temperature_filter_max >= -40 && change_thermal_camera_temperature_filter_max <= 200)
        config.change_thermal_camera_temperature_filter_max = change_thermal_camera_temperature_filter_max;
    else
        change_thermal_camera_temperature_filter_max = config.change_thermal_camera_temperature_filter_max;

    default_configured = true;
}

void callback(l3cam_ros::ThermalCameraConfig &config, uint32_t level)
{
    int error = L3CAM_OK;

    if (!default_configured)
        configureDefault(config);
    else
    {
        switch (level)
        {
        case 0:
            srvColormap.request.colormap = config.change_thermal_camera_colormap;
            if (clientColormap.call(srvColormap))
            {
                error = srvColormap.response.error;
                if (!error)
                    change_thermal_camera_colormap = config.change_thermal_camera_colormap;
                else
                    config.change_thermal_camera_colormap = change_thermal_camera_colormap;
            }
            else
            {
                ROS_ERROR("Failed to call service change_thermal_camera_colormap");
                config.change_thermal_camera_colormap = change_thermal_camera_colormap;
            }
            break;
        case 1:
            srvEnableTemperatureFilter.request.enabled = config.enable_thermal_camera_temperature_filter;
            if (clientEnableTemperatureFilter.call(srvEnableTemperatureFilter))
            {
                error = srvEnableTemperatureFilter.response.error;
                if (!error)
                    enable_thermal_camera_temperature_filter = config.enable_thermal_camera_temperature_filter;
                else
                    config.enable_thermal_camera_temperature_filter = enable_thermal_camera_temperature_filter;
            }
            else
            {
                ROS_ERROR("Failed to call service enable_thermal_camera_temperature_filter");
                config.enable_thermal_camera_temperature_filter = enable_thermal_camera_temperature_filter;
            }
            break;
        case 2:
            srvTemperatureFilter.request.min_temperature = config.change_thermal_camera_temperature_filter_min;
            srvTemperatureFilter.request.max_temperature = change_thermal_camera_temperature_filter_max;
            if (clientTemperatureFilter.call(srvTemperatureFilter))
            {
                error = srvTemperatureFilter.response.error;
                if (!error)
                    change_thermal_camera_temperature_filter_min = config.change_thermal_camera_temperature_filter_min;
                else
                    config.change_thermal_camera_temperature_filter_min = change_thermal_camera_temperature_filter_min;
            }
            else
            {
                ROS_ERROR("Failed to call service change_thermal_camera_temperature_filter_min");
                config.change_thermal_camera_temperature_filter_min = change_thermal_camera_temperature_filter_min;
            }
            break;
        case 3:
            srvTemperatureFilter.request.max_temperature = config.change_thermal_camera_temperature_filter_max;
            srvTemperatureFilter.request.min_temperature = change_thermal_camera_temperature_filter_min;
            if (clientTemperatureFilter.call(srvTemperatureFilter))
            {
                error = srvTemperatureFilter.response.error;
                if (!error)
                    change_thermal_camera_temperature_filter_max = config.change_thermal_camera_temperature_filter_max;
                else
                    config.change_thermal_camera_temperature_filter_max = change_thermal_camera_temperature_filter_max;
            }
            else
            {
                ROS_ERROR("Failed to call service change_thermal_camera_temperature_filter_max");
                config.change_thermal_camera_temperature_filter_max = change_thermal_camera_temperature_filter_max;
            }
            break;
        }
    }

    if (error)
        ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgb_camera_configuration");
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
                if (srvGetSensors.response.sensors[i].sensor_type == sensor_thermal)
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
        ROS_INFO("Thermal camera configuration is available");
    else
        return 0;

    dynamic_reconfigure::Server<l3cam_ros::ThermalCameraConfig> server;
    dynamic_reconfigure::Server<l3cam_ros::ThermalCameraConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    clientColormap = nh.serviceClient<l3cam_ros::ChangeThermalCameraColormap>("change_thermal_camera_colormap");
    clientEnableTemperatureFilter = nh.serviceClient<l3cam_ros::EnableThermalCameraTemperatureFilter>("enable_thermal_camera_temperature_filter");
    clientTemperatureFilter = nh.serviceClient<l3cam_ros::ChangeThermalCameraTemperatureFilter>("change_thermal_camera_temperature_filter");

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
