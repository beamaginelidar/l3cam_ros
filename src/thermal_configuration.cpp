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
#include "l3cam_ros/ThermalConfig.h"

#include <libL3Cam.h>
#include <beamagine.h>
#include <beamErrors.h>

#include "l3cam_ros/GetSensorsAvailable.h"
#include "l3cam_ros/ChangeThermalCameraColormap.h"
#include "l3cam_ros/EnableThermalCameraTemperatureFilter.h"
#include "l3cam_ros/ChangeThermalCameraTemperatureFilter.h"
#include "l3cam_ros/ChangeThermalCameraProcessingPipeline.h"
#include "l3cam_ros/EnableThermalCameraTemperatureDataUdp.h"
#include "l3cam_ros/ChangeStreamingProtocol.h"
#include "l3cam_ros/GetRtspPipeline.h"

#include "l3cam_ros/SensorDisconnected.h"

#include "l3cam_ros_utils.hpp"

namespace l3cam_ros
{
    class ThermalConfiguration : public ros::NodeHandle
    {
    public:
        explicit ThermalConfiguration() : ros::NodeHandle("~")
        {
            client_get_sensors_ = serviceClient<l3cam_ros::GetSensorsAvailable>("/L3Cam/l3cam_ros_node/get_sensors_available");
            client_colormap_ = serviceClient<l3cam_ros::ChangeThermalCameraColormap>("/L3Cam/l3cam_ros_node/change_thermal_colormap");
            client_enable_temperature_filter_ = serviceClient<l3cam_ros::EnableThermalCameraTemperatureFilter>("/L3Cam/l3cam_ros_node/enable_thermal_temperature_filter");
            client_temperature_filter_ = serviceClient<l3cam_ros::ChangeThermalCameraTemperatureFilter>("/L3Cam/l3cam_ros_node/change_thermal_temperature_filter");
            client_processing_pipeline_ = serviceClient<l3cam_ros::ChangeThermalCameraProcessingPipeline>("/L3Cam/l3cam_ros_node/change_thermal_processing_pipeline");
            client_temperature_data_udp_ = serviceClient<l3cam_ros::EnableThermalCameraTemperatureDataUdp>("/L3Cam/l3cam_ros_node/enable_thermal_temperature_data_udp");
            client_change_streaming_protocol_ = serviceClient<l3cam_ros::ChangeStreamingProtocol>("/L3Cam/l3cam_ros_node/change_streaming_protocol");
            client_get_rtsp_pipeline_ = serviceClient<l3cam_ros::GetRtspPipeline>("/L3Cam/l3cam_ros_node/get_rtsp_pipeline");

            loadDefaultParams();

            // Create service server
            srv_sensor_disconnected_ = advertiseService("thermal_configuration_disconnected", &ThermalConfiguration::sensorDisconnectedCallback, this);

            m_default_configured = false;
            m_shutdown_requested = false;
        }

        void setDynamicReconfigure()
        {
            // Dynamic reconfigure callback
            // server_ is a pointer so we only declare it if sensor is available and reconfigure should be available
            server_ = new dynamic_reconfigure::Server<l3cam_ros::ThermalConfig>;
            server_->setCallback(std::bind(&ThermalConfiguration::parametersCallback, this, std::placeholders::_1, std::placeholders::_2));
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
            loadParam("thermal_colormap", thermal_colormap_, 1);
            loadParam("thermal_temperature_filter", thermal_temperature_filter_, false);
            loadParam("thermal_temperature_filter_min", thermal_temperature_filter_min_, 0);
            loadParam("thermal_temperature_filter_max", thermal_temperature_filter_max_, 50);
            loadParam("thermal_processing_pipeline", thermal_processing_pipeline_, 1);
            loadParam("thermal_temperature_data_udp", thermal_temperature_data_udp_, false);
            loadParam("thermal_streaming_protocol", thermal_streaming_protocol_, 0);
        }

        void configureDefault(l3cam_ros::ThermalConfig &config)
        {
            // Configure default params to dynamix reconfigure if inside range
            if (thermal_colormap_ >= 0 || thermal_colormap_ <= 8)
            {
                config.thermal_colormap = thermal_colormap_;
            }
            else
            {
                thermal_colormap_ = config.thermal_colormap;
            }

            config.thermal_temperature_filter = thermal_temperature_filter_;

            if (thermal_temperature_filter_min_ >= -40 && thermal_temperature_filter_min_ <= 200)
            {
                config.thermal_temperature_filter_min = thermal_temperature_filter_min_;
            }
            else
            {
                thermal_temperature_filter_min_ = config.thermal_temperature_filter_min;
            }
            if (thermal_temperature_filter_max_ >= -40 && thermal_temperature_filter_max_ <= 200)
            {
                config.thermal_temperature_filter_max = thermal_temperature_filter_max_;
            }
            else
            {
                thermal_temperature_filter_max_ = config.thermal_temperature_filter_max;
            }

            if (thermal_processing_pipeline_ >= 0 && thermal_processing_pipeline_ <= 2)
            {
                config.thermal_processing_pipeline = thermal_processing_pipeline_;
            }
            else
            {
                thermal_processing_pipeline_ = config.thermal_processing_pipeline;
            }

            if (thermal_streaming_protocol_ == 0 || thermal_streaming_protocol_ == 1)
            {
                config.thermal_streaming_protocol = thermal_streaming_protocol_;
            }
            else
            {
                thermal_streaming_protocol_ = config.thermal_streaming_protocol;
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
                srv_get_rtsp_pipeline_.request.sensor_type = (int)sensorTypes::sensor_thermal;
                if (client_get_rtsp_pipeline_.call(srv_get_rtsp_pipeline_))
                {
                    error = srv_get_rtsp_pipeline_.response.error;
                    if (!error)
                    {
                        thermal_rtsp_pipeline_ = srv_get_rtsp_pipeline_.response.pipeline;
                        config.thermal_rtsp_pipeline = srv_get_rtsp_pipeline_.response.pipeline;
                    }
                    else
                    {
                        ROS_ERROR_STREAM(this->getNamespace() << " error " << error << " while getting RTSP pipeline: " << getErrorDescription(error));
                        config.thermal_rtsp_pipeline = "";
                        thermal_rtsp_pipeline_ = "";
                    }
                }
                else
                {
                    ROS_ERROR_STREAM(this->getNamespace() << " error: Failed to call service get_rtsp_pipeline");
                    config.thermal_rtsp_pipeline = "";
                    thermal_rtsp_pipeline_ = "";
                }
            }

            m_default_configured = true;
        }

        void parametersCallback(l3cam_ros::ThermalConfig &config, uint32_t level)
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
                case 0: // thermal_colormap
                    error = callThermalColormap(config);
                    break;
                case 1: // thermal_temperature_filter
                    error = callThermalTemperatureFilter(config);
                    break;
                case 2: // thermal_temperature_filter_min
                    error = callThermalTemperatureFilterRange(config);
                    break;
                case 3: // thermal_temperature_filter_max
                    error = callThermalTemperatureFilterRange(config);
                    break;
                case 4: // thermal_processing_pipeline
                    error = callThermalProcessingPipeline(config);
                    break;
                case 5: // thermal_temperature_data_udp
                    error = callThermalTemperatureDataUdp(config);
                    break;
                case 6: // thermal_streaming_protocol
                    error = callThermalStreamingProtocol(config);
                    break;
                case 7: // thermal_rtsp_pipeline
                    error = callThermalRtspPipeline(config);
                    break;
                }
            }

            if (error)
            {
                ROS_ERROR_STREAM(this->getNamespace() << " error " << error << " while changing parameter: " << getErrorDescription(error));
            }
        }

        // Service calls
        int callThermalColormap(l3cam_ros::ThermalConfig &config)
        {
            int error = L3CAM_OK;

            srv_colormap_.request.colormap = config.thermal_colormap;
            if (client_colormap_.call(srv_colormap_))
            {
                error = srv_colormap_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    thermal_colormap_ = config.thermal_colormap;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.thermal_colormap = thermal_colormap_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.thermal_colormap = thermal_colormap_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callThermalTemperatureFilter(l3cam_ros::ThermalConfig &config)
        {
            int error = L3CAM_OK;

            srv_enable_temperature_filter_.request.enabled = config.thermal_temperature_filter;
            if (client_enable_temperature_filter_.call(srv_enable_temperature_filter_))
            {
                error = srv_enable_temperature_filter_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    thermal_temperature_filter_ = config.thermal_temperature_filter;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.thermal_temperature_filter = thermal_temperature_filter_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.thermal_temperature_filter = thermal_temperature_filter_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callThermalTemperatureFilterRange(l3cam_ros::ThermalConfig &config)
        {
            int error = L3CAM_OK;

            srv_temperature_filter_.request.min_temperature = config.thermal_temperature_filter_min;
            srv_temperature_filter_.request.max_temperature = config.thermal_temperature_filter_max;
            if (client_temperature_filter_.call(srv_temperature_filter_))
            {
                error = srv_temperature_filter_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    thermal_temperature_filter_min_ = config.thermal_temperature_filter_min;
                    thermal_temperature_filter_max_ = config.thermal_temperature_filter_max;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.thermal_temperature_filter_min = thermal_temperature_filter_min_;
                    config.thermal_temperature_filter_max = thermal_temperature_filter_max_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.thermal_temperature_filter_min = thermal_temperature_filter_min_;
                config.thermal_temperature_filter_max = thermal_temperature_filter_max_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callThermalProcessingPipeline(l3cam_ros::ThermalConfig &config)
        {
            int error = L3CAM_OK;

            srv_processing_pipeline_.request.pipeline = config.thermal_processing_pipeline;
            if (client_processing_pipeline_.call(srv_processing_pipeline_))
            {
                error = srv_processing_pipeline_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    thermal_processing_pipeline_ = config.thermal_processing_pipeline;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.thermal_processing_pipeline = thermal_processing_pipeline_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.thermal_processing_pipeline = thermal_processing_pipeline_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callThermalTemperatureDataUdp(l3cam_ros::ThermalConfig &config)
        {
            int error = L3CAM_OK;

            srv_temperature_data_udp_.request.enabled = config.thermal_temperature_data_udp;
            if (client_temperature_data_udp_.call(srv_temperature_data_udp_))
            {
                error = srv_temperature_data_udp_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    thermal_temperature_data_udp_ = config.thermal_temperature_data_udp;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.thermal_temperature_data_udp = thermal_temperature_data_udp_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.thermal_temperature_data_udp = thermal_temperature_data_udp_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }


        int callThermalStreamingProtocol(l3cam_ros::ThermalConfig &config)
        {
            int error = L3CAM_OK;

            srv_change_streaming_protocol_.request.protocol = config.thermal_streaming_protocol;
            srv_change_streaming_protocol_.request.sensor_type = (int)sensorTypes::sensor_thermal;
            if (client_change_streaming_protocol_.call(srv_change_streaming_protocol_))
            {
                error = srv_change_streaming_protocol_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    thermal_streaming_protocol_ = config.thermal_streaming_protocol;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.thermal_streaming_protocol = thermal_streaming_protocol_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.thermal_streaming_protocol = thermal_streaming_protocol_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callThermalRtspPipeline(l3cam_ros::ThermalConfig &config)
        {
            // Read-only
            ROS_WARN_STREAM(this->getNamespace() << " The RTSP Pipeline parameter is read-only, only changeable at launch");
            config.thermal_rtsp_pipeline = thermal_rtsp_pipeline_;

            return L3CAM_OK;
        }

        bool sensorDisconnectedCallback(l3cam_ros::SensorDisconnected::Request &req, l3cam_ros::SensorDisconnected::Response &res)
        {
            ROS_BMG_UNUSED(res);
            if (req.code == 0)
            {
                ROS_INFO_STREAM("Exiting " << this->getNamespace() << " cleanly.");
            }
            else
            {
                ROS_WARN_STREAM("Exiting " << this->getNamespace() << ". Sensor got disconnected with error " << req.code << ": " << getErrorDescription(req.code));
            }

            m_shutdown_requested = true;
            return true;
        }

        dynamic_reconfigure::Server<l3cam_ros::ThermalConfig> *server_;

        ros::ServiceClient client_colormap_;
        l3cam_ros::ChangeThermalCameraColormap srv_colormap_;
        ros::ServiceClient client_enable_temperature_filter_;
        l3cam_ros::EnableThermalCameraTemperatureFilter srv_enable_temperature_filter_;
        ros::ServiceClient client_temperature_filter_;
        l3cam_ros::ChangeThermalCameraTemperatureFilter srv_temperature_filter_;
        ros::ServiceClient client_processing_pipeline_;
        l3cam_ros::ChangeThermalCameraProcessingPipeline srv_processing_pipeline_;
        ros::ServiceClient client_temperature_data_udp_;
        l3cam_ros::EnableThermalCameraTemperatureDataUdp srv_temperature_data_udp_;
        ros::ServiceClient client_change_streaming_protocol_;
        l3cam_ros::ChangeStreamingProtocol srv_change_streaming_protocol_;
        ros::ServiceClient client_get_rtsp_pipeline_;
        l3cam_ros::GetRtspPipeline srv_get_rtsp_pipeline_;

        ros::ServiceServer srv_sensor_disconnected_;

        int thermal_colormap_;
        bool thermal_temperature_filter_;
        int thermal_temperature_filter_min_;
        int thermal_temperature_filter_max_;
        int thermal_processing_pipeline_;
        bool thermal_temperature_data_udp_;
        int thermal_streaming_protocol_;
        std::string thermal_rtsp_pipeline_;

        bool m_default_configured;
        bool m_shutdown_requested;

    }; // class ThermalConfiguration

} // namespace l3cam_ros

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thermal_configuration");

    l3cam_ros::ThermalConfiguration *node = new l3cam_ros::ThermalConfiguration();

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
                if (node->srv_get_sensors_.response.sensors[i].sensor_type == sensor_thermal && node->srv_get_sensors_.response.sensors[i].sensor_available)
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
        ROS_INFO("Thermal configuration is available");
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
