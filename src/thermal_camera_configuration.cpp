#include <iostream>
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include "l3cam_ros/ThermalCameraConfig.h"

#include "libL3Cam.h"
#include "beamagine.h"
#include "beamErrors.h"

#include "l3cam_ros/GetSensorsAvaliable.h"
#include "l3cam_ros/ChangeThermalCameraColormap.h"
#include "l3cam_ros/EnableThermalCameraTemperatureFilter.h"
#include "l3cam_ros/ChangeThermalCameraTemperatureFilter.h"

ros::ServiceClient clientGetSensors;
l3cam_ros::GetSensorsAvaliable srvGetSensors;
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

void callback(l3cam_ros::ThermalCameraConfig &config, uint32_t level)
{
    int error = L3CAM_OK;

    if (config.change_thermal_camera_colormap != change_thermal_camera_colormap)
    {
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
    }
    if (config.enable_thermal_camera_temperature_filter != enable_thermal_camera_temperature_filter)
    {
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
    }
    if (config.change_thermal_camera_temperature_filter_min != change_thermal_camera_temperature_filter_min)
    {
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
    }
    if (config.change_thermal_camera_temperature_filter_max != change_thermal_camera_temperature_filter_max)
    {
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
    }

    if (error)
        ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgb_camera_configuration");
    ros::NodeHandle nh;

    nh.param("thermal_camera_colormap", change_thermal_camera_colormap, 1);
    nh.param("thermal_camera_temperature_filter", enable_thermal_camera_temperature_filter, false);
    nh.param("thermal_camera_temperature_filter_min", change_thermal_camera_temperature_filter_min, 0);
    nh.param("thermal_camera_temperature_filter_max", change_thermal_camera_temperature_filter_max, 50);

    clientGetSensors = nh.serviceClient<l3cam_ros::GetSensorsAvaliable>("get_sensors_avaliable");
    int error = L3CAM_OK;

    bool sensor_is_avaliable = false;
    if (clientGetSensors.call(srvGetSensors))
    {
        error = srvGetSensors.response.error;

        if (!error)
            for (int i = 0; i < srvGetSensors.response.num_sensors; ++i)
            {
                if (srvGetSensors.response.sensors[i].sensor_type == sensor_thermal)
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
        ROS_INFO("Thermal camera is avaliable");
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
