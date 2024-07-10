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
#include "l3cam_ros/LidarConfig.h"

#include <libL3Cam.h>
#include <beamagine.h>
#include <beamErrors.h>

#include "l3cam_ros/GetSensorsAvailable.h"
#include "l3cam_ros/ChangePointcloudColor.h"
#include "l3cam_ros/ChangePointcloudColorRange.h"
#include "l3cam_ros/ChangeDistanceRange.h"
#include "l3cam_ros/SetBiasShortRange.h"
#include "l3cam_ros/EnableAutoBias.h"
#include "l3cam_ros/ChangeBiasValue.h"
#include "l3cam_ros/ChangeAutobiasValue.h"
#include "l3cam_ros/ChangeStreamingProtocol.h"
#include "l3cam_ros/GetRtspPipeline.h"

#include "l3cam_ros/SensorDisconnected.h"

#include "l3cam_ros_utils.hpp"

namespace l3cam_ros
{
    class LidarConfiguration : public ros::NodeHandle
    {
    public:
        explicit LidarConfiguration() : ros::NodeHandle("~")
        {
            // Create service clients
            client_get_sensors_ = serviceClient<l3cam_ros::GetSensorsAvailable>("/L3Cam/l3cam_ros_node/get_sensors_available");
            client_color_ = serviceClient<l3cam_ros::ChangePointcloudColor>("/L3Cam/l3cam_ros_node/change_pointcloud_color");
            client_color_range_ = serviceClient<l3cam_ros::ChangePointcloudColorRange>("/L3Cam/l3cam_ros_node/change_pointcloud_color_range");
            client_distance_range_ = serviceClient<l3cam_ros::ChangeDistanceRange>("/L3Cam/l3cam_ros_node/change_distance_range");
            client_bias_short_range_ = serviceClient<l3cam_ros::SetBiasShortRange>("/L3Cam/l3cam_ros_node/set_bias_short_range");
            client_auto_bias_ = serviceClient<l3cam_ros::EnableAutoBias>("/L3Cam/l3cam_ros_node/enable_auto_bias");
            client_bias_value_ = serviceClient<l3cam_ros::ChangeBiasValue>("/L3Cam/l3cam_ros_node/change_bias_value");
            client_autobias_value_ = serviceClient<l3cam_ros::ChangeAutobiasValue>("/L3Cam/l3cam_ros_node/change_autobias_value");
            client_change_streaming_protocol_ = serviceClient<l3cam_ros::ChangeStreamingProtocol>("/L3Cam/l3cam_ros_node/change_streaming_protocol");
            client_get_rtsp_pipeline_ = serviceClient<l3cam_ros::GetRtspPipeline>("/L3Cam/l3cam_ros_node/get_rtsp_pipeline");

            loadDefaultParams();

            // Create service server
            srv_sensor_disconnected_ = advertiseService("lidar_configuration_disconnected", &LidarConfiguration::sensorDisconnectedCallback, this);

            m_default_configured = false;
            m_shutdown_requested = false;
        }

        void setDynamicReconfigure()
        {
            // Dynamic reconfigure callback
            // server_ is a pointer so we only declare it if sensor is available and reconfigure should be available
            server_ = new dynamic_reconfigure::Server<l3cam_ros::LidarConfig>;
            server_->setCallback(std::bind(&LidarConfiguration::parametersCallback, this, std::placeholders::_1, std::placeholders::_2));
        }

        void spin()
        {
            while (ros::ok() && !m_shutdown_requested)
            {
                ros::spinOnce();
                ros::Duration(0.1).sleep();
            }

            ros::shutdown();
        }

        ros::ServiceClient client_get_sensors_;
        l3cam_ros::GetSensorsAvailable srv_get_sensors_;

        int timeout_secs_;

    private:
        template <typename T>
        void loadParam(const std::string &param_name, T &param_var, const T &default_val)
        {
            std::string full_param_name;

            if (searchParam(param_name, full_param_name))
            {
                if (!getParam(full_param_name, param_var))
                {
                    ROS_ERROR_STREAM(this->getNamespace() << " error: Could not retreive '" << full_param_name << "' param value");
                }
            }
            else
            {
                ROS_WARN_STREAM(this->getNamespace() << " Parameter '" << param_name << "' not defined");
                param_var = default_val;
            }
        }

        void loadDefaultParams()
        {
            // Get and save parameters
            loadParam("timeout_secs", timeout_secs_, 60);
            loadParam("pointcloud_color", pointcloud_color_, 0);
            loadParam("pointcloud_color_range_minimum", pointcloud_color_range_minimum_, 0);
            loadParam("pointcloud_color_range_maximum", pointcloud_color_range_maximum_, 300000);
            loadParam("distance_range_minimum", distance_range_minimum_, 0);
            loadParam("distance_range_maximum", distance_range_maximum_, 300000);
            loadParam("bias_short_range", bias_short_range_, false);
            loadParam("auto_bias", auto_bias_, true);
            loadParam("bias_value_right", bias_value_right_, 1580);
            loadParam("bias_value_left", bias_value_left_, 1380);
            loadParam("autobias_value_right", autobias_value_right_, 50);
            loadParam("autobias_value_left", autobias_value_left_, 50);
            loadParam("lidar_streaming_protocol", lidar_streaming_protocol_, 0);
        }

        void configureDefault(l3cam_ros::LidarConfig &config)
        {
            // Configure default params to dynamix reconfigure if inside range
            if (pointcloud_color_ >= 0 && pointcloud_color_ <= 7 || pointcloud_color_ >= 12 && pointcloud_color_ <= 15)
            {
                config.pointcloud_color = pointcloud_color_;
            }
            else
            {
                pointcloud_color_ = config.pointcloud_color;
            }

            if (pointcloud_color_range_minimum_ >= 0 && pointcloud_color_range_minimum_ <= 300000)
            {
                config.pointcloud_color_range_minimum = pointcloud_color_range_minimum_;
            }
            else
            {
                pointcloud_color_range_minimum_ = config.pointcloud_color_range_minimum;
            }

            if (pointcloud_color_range_maximum_ >= 0 && pointcloud_color_range_maximum_ <= 300000)
            {
                config.pointcloud_color_range_maximum = pointcloud_color_range_maximum_;
            }
            else
            {
                pointcloud_color_range_maximum_ = config.pointcloud_color_range_maximum;
            }

            if (distance_range_minimum_ >= 0 && distance_range_minimum_ <= 300000)
            {
                config.distance_range_minimum = distance_range_minimum_;
            }
            else
            {
                distance_range_minimum_ = config.distance_range_minimum;
            }

            if (distance_range_maximum_ >= 0 && distance_range_maximum_ <= 300000)
            {
                config.distance_range_maximum = distance_range_maximum_;
            }
            else
            {
                distance_range_maximum_ = config.distance_range_maximum;
            }

            config.bias_short_range = bias_short_range_;

            config.auto_bias = auto_bias_;
            if (bias_value_right_ >= 700 && bias_value_right_ <= 3500)
            {
                config.bias_value_right = bias_value_right_;
            }
            else
            {
                bias_value_right_ = config.bias_value_right;
            }

            if (bias_value_left_ >= 700 && bias_value_left_ <= 3500)
            {
                config.bias_value_left = bias_value_left_;
            }
            else
            {
                bias_value_left_ = config.bias_value_left;
            }

            if (autobias_value_right_ >= 0 && autobias_value_right_ <= 100)
            {
                config.autobias_value_right = autobias_value_right_;
            }
            else
            {
                autobias_value_right_ = config.autobias_value_right;
            }

            if (autobias_value_left_ >= 0 && autobias_value_left_ <= 100)
            {
                config.autobias_value_left = autobias_value_left_;
            }
            else
            {
                autobias_value_left_ = config.autobias_value_left;
            }

            if (lidar_streaming_protocol_ == 0 || lidar_streaming_protocol_ == 1)
            {
                config.lidar_streaming_protocol = lidar_streaming_protocol_;
            }
            else
            {
                lidar_streaming_protocol_ = config.lidar_streaming_protocol;
            }

            // Get pipeline
            int error = L3CAM_OK;
            ros::Duration timeout_duration(timeout_secs_);
            if (!client_get_rtsp_pipeline_.waitForExistence(timeout_duration))
            {
                ROS_ERROR_STREAM(this->getNamespace() << " error: " << getErrorDescription(L3CAM_ROS_SERVICE_AVAILABILITY_TIMEOUT_ERROR) << ". Waited " << timeout_duration << " seconds");
            }
            else
            {
                srv_get_rtsp_pipeline_.request.sensor_type = (int)sensorTypes::sensor_lidar;
                if (client_get_rtsp_pipeline_.call(srv_get_rtsp_pipeline_))
                {
                    error = srv_get_rtsp_pipeline_.response.error;
                    if (!error)
                    {
                        lidar_rtsp_pipeline_ = srv_get_rtsp_pipeline_.response.pipeline;
                        config.lidar_rtsp_pipeline = srv_get_rtsp_pipeline_.response.pipeline;
                    }
                    else
                    {
                        ROS_ERROR_STREAM(this->getNamespace() << " error " << error << " while getting RTSP pipeline: " << getErrorDescription(error));
                        config.lidar_rtsp_pipeline = "";
                        lidar_rtsp_pipeline_ = "";
                    }
                }
                else
                {
                    ROS_ERROR_STREAM(this->getNamespace() << " error: Failed to call service get_rtsp_pipeline");
                    config.lidar_rtsp_pipeline = "";
                    lidar_rtsp_pipeline_ = "";
                }
            }

            m_default_configured = true;
        }

        void parametersCallback(l3cam_ros::LidarConfig &config, uint32_t level)
        {
            int error = L3CAM_OK;

            if (!m_default_configured)
            {
                configureDefault(config);
            }
            else
            {
                // Filter by parameter and call service
                switch (level)
                {
                case 0: // pointcloud_color
                    error = callColor(config);
                    break;
                case 1: // pointcloud_color_range_minimum
                    error = callColorRange(config);
                    break;
                case 2: // pointcloud_color_range_maximum
                    error = callColorRange(config);
                    break;
                case 3: // distance_range_minimum
                    error = callDistanceRange(config);
                    break;
                case 4: // distance_range_maximum
                    error = callDistanceRange(config);
                    break;
                case 5: // bias_short_range
                    error = callBiasShortRange(config);
                    break;
                case 6: // auto_bias
                    error = callAutoBias(config);
                    break;
                case 7: // bias_value_right
                    error = callBiasValueRight(config);
                    break;
                case 8: // bias_value_left
                    error = callBiasValueLeft(config);
                    break;
                case 9: // autobias_value_right
                    error = callAutobiasValueRight(config);
                    break;
                case 10: // autobias_value_left
                    error = callAutobiasValueLeft(config);
                    break;
                case 11: // lidar_streaming_protocol
                    error = callStreamingProtocol(config);
                    break;
                case 12: // lidar_rtsp_pipeline
                    error = callRtspPipeline(config);
                    break;
                }
            }

            if (error)
            {
                ROS_ERROR_STREAM(this->getNamespace() << " error " << error << " while changing parameter: " << getErrorDescription(error));
            }
        }

        // Service calls
        int callColor(l3cam_ros::LidarConfig &config)
        {
            int error = L3CAM_OK;

            srv_color_.request.visualization_color = config.pointcloud_color;
            if (client_color_.call(srv_color_))
            {
                error = srv_color_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    pointcloud_color_ = config.pointcloud_color;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.pointcloud_color = pointcloud_color_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.pointcloud_color = pointcloud_color_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callColorRange(l3cam_ros::LidarConfig &config)
        {
            int error = L3CAM_OK;

            srv_color_range_.request.min_value = config.pointcloud_color_range_minimum;
            srv_color_range_.request.max_value = config.pointcloud_color_range_maximum;
            if (client_color_range_.call(srv_color_range_))
            {
                error = srv_color_range_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    pointcloud_color_range_minimum_ = config.pointcloud_color_range_minimum;
                    pointcloud_color_range_maximum_ = config.pointcloud_color_range_maximum;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.pointcloud_color_range_minimum = pointcloud_color_range_minimum_;
                    config.pointcloud_color_range_maximum = pointcloud_color_range_maximum_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.pointcloud_color_range_minimum = pointcloud_color_range_minimum_;
                config.pointcloud_color_range_maximum = pointcloud_color_range_maximum_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callDistanceRange(l3cam_ros::LidarConfig &config)
        {
            int error = L3CAM_OK;

            srv_distance_range_.request.min_value = config.distance_range_minimum;
            srv_distance_range_.request.max_value = config.distance_range_maximum;
            if (client_distance_range_.call(srv_distance_range_))
            {
                error = srv_distance_range_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    distance_range_minimum_ = config.distance_range_minimum;
                    distance_range_maximum_ = config.distance_range_maximum;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.distance_range_minimum = distance_range_minimum_;
                    config.distance_range_maximum = distance_range_maximum_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.distance_range_minimum = distance_range_minimum_;
                config.distance_range_maximum = distance_range_maximum_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callBiasShortRange(l3cam_ros::LidarConfig &config)
        {
            int error = L3CAM_OK;

            srv_bias_short_range_.request.enabled = config.bias_short_range;
            if (client_bias_short_range_.call(srv_bias_short_range_))
            {
                error = srv_bias_short_range_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    bias_short_range_ = config.bias_short_range;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.bias_short_range = bias_short_range_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.bias_short_range = bias_short_range_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callAutoBias(l3cam_ros::LidarConfig &config)
        {
            int error = L3CAM_OK;

            srv_auto_bias_.request.enabled = config.auto_bias;
            if (client_auto_bias_.call(srv_auto_bias_))
            {
                error = srv_bias_short_range_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    auto_bias_ = config.auto_bias;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.auto_bias = auto_bias_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.auto_bias = auto_bias_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callBiasValueRight(l3cam_ros::LidarConfig &config)
        {
            int error = L3CAM_OK;

            srv_bias_value_.request.index = 1;
            srv_bias_value_.request.bias = config.bias_value_right;
            if (client_bias_value_.call(srv_bias_value_))
            {
                error = srv_bias_value_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    bias_value_right_ = config.bias_value_right;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.bias_value_right = bias_value_right_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.bias_value_right = bias_value_right_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callBiasValueLeft(l3cam_ros::LidarConfig &config)
        {
            int error = L3CAM_OK;

            srv_bias_value_.request.index = 2;
            srv_bias_value_.request.bias = config.bias_value_left;
            if (client_bias_value_.call(srv_bias_value_))
            {
                error = srv_bias_value_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    bias_value_left_ = config.bias_value_left;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.bias_value_left = bias_value_left_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.bias_value_left = bias_value_left_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callAutobiasValueRight(l3cam_ros::LidarConfig &config)
        {
            int error = L3CAM_OK;

            srv_autobias_value_.request.index = 1;
            srv_autobias_value_.request.autobias = config.autobias_value_right;
            if (client_autobias_value_.call(srv_autobias_value_))
            {
                error = srv_autobias_value_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    autobias_value_right_ = config.autobias_value_right;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.autobias_value_right = autobias_value_right_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.autobias_value_right = autobias_value_right_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callAutobiasValueLeft(l3cam_ros::LidarConfig &config)
        {
            int error = L3CAM_OK;

            srv_autobias_value_.request.index = 2;
            srv_autobias_value_.request.autobias = config.autobias_value_left;
            if (client_autobias_value_.call(srv_autobias_value_))
            {
                error = srv_autobias_value_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    autobias_value_left_ = config.autobias_value_left;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.autobias_value_left = autobias_value_left_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.autobias_value_left = autobias_value_left_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callStreamingProtocol(l3cam_ros::LidarConfig &config)
        {
            int error = L3CAM_OK;

            srv_change_streaming_protocol_.request.protocol = config.lidar_streaming_protocol;
            srv_change_streaming_protocol_.request.sensor_type = (int)sensorTypes::sensor_lidar;
            if (client_change_streaming_protocol_.call(srv_change_streaming_protocol_))
            {
                error = srv_change_streaming_protocol_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    lidar_streaming_protocol_ = config.lidar_streaming_protocol;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.lidar_streaming_protocol = lidar_streaming_protocol_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.lidar_streaming_protocol = lidar_streaming_protocol_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callRtspPipeline(l3cam_ros::LidarConfig &config)
        {
            // Read-only
            ROS_WARN_STREAM(this->getNamespace() << " The RTSP Pipeline parameter is read-only, only changeable at launch");
            config.lidar_rtsp_pipeline = lidar_rtsp_pipeline_;

            return L3CAM_OK;
        }

        bool sensorDisconnectedCallback(l3cam_ros::SensorDisconnected::Request &req, l3cam_ros::SensorDisconnected::Response &res)
        {
            ROS_BMG_UNUSED(res);
            if (req.code == 0)
            {
                ROS_INFO_STREAM(this->getNamespace() << " Exiting " << this->getNamespace() << " cleanly.");
            }
            else
            {
                ROS_WARN_STREAM(this->getNamespace() << " Exiting " << this->getNamespace() << ". Sensor got disconnected with error " << req.code << ": " << getErrorDescription(req.code));
            }

            m_shutdown_requested = true;
            return true;
        }

        dynamic_reconfigure::Server<l3cam_ros::LidarConfig> *server_;

        ros::ServiceClient client_color_;
        l3cam_ros::ChangePointcloudColor srv_color_;
        ros::ServiceClient client_color_range_;
        l3cam_ros::ChangePointcloudColorRange srv_color_range_;
        ros::ServiceClient client_distance_range_;
        l3cam_ros::ChangeDistanceRange srv_distance_range_;
        ros::ServiceClient client_bias_short_range_;
        l3cam_ros::SetBiasShortRange srv_bias_short_range_;
        ros::ServiceClient client_auto_bias_;
        l3cam_ros::EnableAutoBias srv_auto_bias_;
        ros::ServiceClient client_bias_value_;
        l3cam_ros::ChangeBiasValue srv_bias_value_;
        ros::ServiceClient client_autobias_value_;
        l3cam_ros::ChangeAutobiasValue srv_autobias_value_;
        ros::ServiceClient client_change_streaming_protocol_;
        l3cam_ros::ChangeStreamingProtocol srv_change_streaming_protocol_;
        ros::ServiceClient client_get_rtsp_pipeline_;
        l3cam_ros::GetRtspPipeline srv_get_rtsp_pipeline_;

        ros::ServiceServer srv_sensor_disconnected_;

        int pointcloud_color_;
        int pointcloud_color_range_minimum_;
        int pointcloud_color_range_maximum_;
        int distance_range_minimum_;
        int distance_range_maximum_;
        bool bias_short_range_;
        bool auto_bias_;
        int bias_value_right_;
        int bias_value_left_;
        int autobias_value_right_;
        int autobias_value_left_;
        int lidar_streaming_protocol_;
        std::string lidar_rtsp_pipeline_;

        bool m_default_configured;
        bool m_shutdown_requested;

    }; // class LidarConfiguration

} // namespace l3cam_ros

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_configuration");

    l3cam_ros::LidarConfiguration *node = new l3cam_ros::LidarConfiguration();

    // Check if service is available
    ros::Duration timeout_duration(node->timeout_secs_);
    if (!node->client_get_sensors_.waitForExistence(timeout_duration))
    {
        ROS_ERROR_STREAM(node->getNamespace() << " error: " << getErrorDescription(L3CAM_ROS_SERVICE_AVAILABILITY_TIMEOUT_ERROR) << ". Waited " << timeout_duration << " seconds");
        return L3CAM_ROS_SERVICE_AVAILABILITY_TIMEOUT_ERROR;
    }

    int error = L3CAM_OK;
    bool sensor_is_available = false;
    // Shutdown if sensor is not available or if error returned
    if (node->client_get_sensors_.call(node->srv_get_sensors_))
    {
        error = node->srv_get_sensors_.response.error;

        if (!error)
        {
            for (int i = 0; i < node->srv_get_sensors_.response.num_sensors; ++i)
            {
                if (node->srv_get_sensors_.response.sensors[i].sensor_type == sensor_lidar && node->srv_get_sensors_.response.sensors[i].sensor_available)
                {
                    sensor_is_available = true;
                }
            }
        }
        else
        {
            ROS_ERROR_STREAM(node->getNamespace() << " error " << error << " while checking sensor availability: " << getErrorDescription(error));
            return error;
        }
    }
    else
    {
        ROS_ERROR_STREAM(node->getNamespace() << " error: Failed to call service get_sensors_available");
        return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
    }

    if (sensor_is_available)
    {
        ROS_INFO("LiDAR configuration is available");
    }
    else
    {
        return 0;
    }

    node->setDynamicReconfigure();

    node->spin();

    ros::shutdown();
    return 0;
}
