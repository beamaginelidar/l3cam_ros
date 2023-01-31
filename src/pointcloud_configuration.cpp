#include <iostream>
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include "l3cam_ros/PointcloudConfig.h"

#include "libL3Cam.h"
#include "beamagine.h"
#include "beamErrors.h"

#include "l3cam_ros/GetSensorsAvaliable.h"
#include "l3cam_ros/ChangePointcloudColor.h"
#include "l3cam_ros/ChangePointcloudColorRange.h"
#include "l3cam_ros/ChangeDistanceRange.h"

ros::ServiceClient clientGetSensors;
l3cam_ros::GetSensorsAvaliable srvGetSensors;
ros::ServiceClient clientColor;
l3cam_ros::ChangePointcloudColor srvColor;
ros::ServiceClient clientColorRange;
l3cam_ros::ChangePointcloudColorRange srvColorRange;
ros::ServiceClient clientDistanceRange;
l3cam_ros::ChangeDistanceRange srvDistanceRange;

int change_pointcloud_color;
int change_pointcloud_color_range_minimum;
int change_pointcloud_color_range_maximum;
int change_distance_range_minimum;
int change_distance_range_maximum;

void callback(l3cam_ros::PointcloudConfig &config, uint32_t level)
{
    int error = L3CAM_OK;

    if (config.change_pointcloud_color != change_pointcloud_color)
    {
        srvColor.request.visualization_color = config.change_pointcloud_color;
        if (clientColor.call(srvColor))
        {
            error = srvColor.response.error;
            if (!error)
                change_pointcloud_color = config.change_pointcloud_color;
            else
                config.change_pointcloud_color = change_pointcloud_color;
        }
        else
        {
            ROS_ERROR("Failed to call service change_pointcloud_color");
            config.change_pointcloud_color = change_pointcloud_color;
        }
    }
    else if (config.change_pointcloud_color_range_minimum != change_pointcloud_color_range_minimum)
    {
        srvColorRange.request.min_value = config.change_pointcloud_color_range_minimum;
        srvColorRange.request.max_value = change_pointcloud_color_range_maximum;
        if (clientColorRange.call(srvColorRange))
        {
            error = srvColorRange.response.error;
            if (!error)
                change_pointcloud_color_range_minimum = config.change_pointcloud_color_range_minimum;
            else
                config.change_pointcloud_color_range_minimum = change_pointcloud_color_range_minimum;
        }
        else
        {
            ROS_ERROR("Failed to call service change_pointcloud_color_range");
            config.change_pointcloud_color_range_minimum = change_pointcloud_color_range_minimum;
        }
    }
    else if (config.change_pointcloud_color_range_maximum != change_pointcloud_color_range_maximum)
    {
        srvColorRange.request.max_value = config.change_pointcloud_color_range_maximum;
        srvColorRange.request.min_value = change_pointcloud_color_range_minimum;
        if (clientColorRange.call(srvColorRange))
        {
            error = srvColorRange.response.error;
            if (!error)
                change_pointcloud_color_range_maximum = config.change_pointcloud_color_range_maximum;
            else
                config.change_pointcloud_color_range_maximum = change_pointcloud_color_range_maximum;
        }
        else
        {
            ROS_ERROR("Failed to call service change_pointcloud_color_range");
            config.change_pointcloud_color_range_maximum = change_pointcloud_color_range_maximum;
        }
    }
    else if (config.change_distance_range_minimum != change_distance_range_minimum)
    {
        srvDistanceRange.request.min_value = config.change_distance_range_minimum;
        srvDistanceRange.request.max_value = change_distance_range_maximum;
        if (clientDistanceRange.call(srvDistanceRange))
        {
            error = srvDistanceRange.response.error;
            if (!error)
                change_distance_range_minimum = config.change_distance_range_minimum;
            else
                config.change_distance_range_minimum = change_distance_range_minimum;
        }
        else
        {
            ROS_ERROR("Failed to call service change_distance_range");
            config.change_distance_range_minimum = change_distance_range_minimum;
        }
    }
    else if (config.change_distance_range_maximum != change_distance_range_maximum)
    {
        srvDistanceRange.request.max_value = config.change_distance_range_maximum;
        srvDistanceRange.request.min_value = change_distance_range_minimum;
        if (clientDistanceRange.call(srvDistanceRange))
        {
            error = srvDistanceRange.response.error;
            if (!error)
                change_distance_range_maximum = config.change_distance_range_maximum;
            else
                config.change_distance_range_maximum = change_distance_range_maximum;
        }
        else
        {
            ROS_ERROR("Failed to call service change_distance_range");
            config.change_distance_range_maximum = change_distance_range_maximum;
        }
    }

    if (error)
        ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_configuration");
    ros::NodeHandle nh;

    // Params
    nh.param("pointcloud_color", change_pointcloud_color, 0);
    nh.param("pointcloud_color_range_minimum", change_pointcloud_color_range_minimum, 0);
    nh.param("pointcloud_color_range_maximum", change_pointcloud_color_range_maximum, 400000);
    nh.param("distance_range_minimum", change_distance_range_minimum, 0);
    nh.param("distance_range_maximum", change_distance_range_maximum, 400000);

    clientGetSensors = nh.serviceClient<l3cam_ros::GetSensorsAvaliable>("get_sensors_avaliable");
    int error = L3CAM_OK;

    bool sensor_is_avaliable = false;
    if (clientGetSensors.call(srvGetSensors))
    {
        error = srvGetSensors.response.error;

        if (!error)
            for (int i = 0; i < srvGetSensors.response.num_sensors; ++i)
            {
                if (srvGetSensors.response.sensors[i].sensor_type == sensor_lidar)
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

    if(sensor_is_avaliable)
        ROS_INFO("LiDAR is avaliable");
    else
        return 0;

    dynamic_reconfigure::Server<l3cam_ros::PointcloudConfig> server;
    dynamic_reconfigure::Server<l3cam_ros::PointcloudConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    clientColor = nh.serviceClient<l3cam_ros::ChangePointcloudColor>("change_pointcloud_color");
    clientColorRange = nh.serviceClient<l3cam_ros::ChangePointcloudColorRange>("change_pointcloud_color_range");
    clientDistanceRange = nh.serviceClient<l3cam_ros::ChangeDistanceRange>("change_distance_range");

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
