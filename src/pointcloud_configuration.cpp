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
#include "l3cam_ros/PointcloudConfig.h"

#include <libL3Cam.h>
#include <beamagine.h>
#include <beamErrors.h>

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

bool default_configured = false;

void callback(l3cam_ros::PointcloudConfig &config, uint32_t level)
{
    int error = L3CAM_OK;

    if (!default_configured)
    {
        ros::param::param("/pointcloud_configuration/pointcloud_color", config.change_pointcloud_color, 0);
        ros::param::param("/pointcloud_configuration/pointcloud_color_range_minimum", config.change_pointcloud_color_range_minimum, 0);
        ros::param::param("/pointcloud_configuration/pointcloud_color_range_maximum", config.change_pointcloud_color_range_maximum, 400000);
        ros::param::param("/pointcloud_configuration/distance_range_minimum", config.change_distance_range_minimum, 0);
        ros::param::param("/pointcloud_configuration/distance_range_maximum", config.change_distance_range_maximum, 400000);

        ros::param::param("/pointcloud_configuration/pointcloud_color", change_pointcloud_color, 0);
        ros::param::param("/pointcloud_configuration/pointcloud_color_range_minimum", change_pointcloud_color_range_minimum, 0);
        ros::param::param("/pointcloud_configuration/pointcloud_color_range_maximum", change_pointcloud_color_range_maximum, 400000);
        ros::param::param("/pointcloud_configuration/distance_range_minimum", change_distance_range_minimum, 0);
        ros::param::param("/pointcloud_configuration/distance_range_maximum", change_distance_range_maximum, 400000);

        default_configured = true;
    }
    else
    {
        switch (level)
        {
        case 0:
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
            break;
        case 1:
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
            break;
        case 2:
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
            break;
        case 3:
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
            break;
        case 4:
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
            break;
        }
    }

    if (error)
        ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_configuration");
    ros::NodeHandle nh;

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

    if (sensor_is_avaliable)
        ROS_INFO("LiDAR configuration is avaliable");
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
