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
#include "l3cam_ros/NetworkConfig.h"

#include <libL3Cam.h>
#include <beamagine.h>
#include <beamErrors.h>

#include "l3cam_ros/GetNetworkConfiguration.h"
#include "l3cam_ros/ChangeNetworkConfiguration.h"

#include "l3cam_ros/SensorDisconnected.h"

#include "l3cam_ros_utils.hpp"

namespace l3cam_ros
{
    class NetworkConfiguration : public ros::NodeHandle
    {
    public:
        explicit NetworkConfiguration() : ros::NodeHandle("~")
        {
            // Create service clients
            client_get_ = serviceClient<l3cam_ros::GetNetworkConfiguration>("/L3Cam/l3cam_ros_node/get_network_configuration");
            client_change_ = serviceClient<l3cam_ros::ChangeNetworkConfiguration>("/L3Cam/l3cam_ros_node/change_network_configuration");

            // Create service server
            srv_sensor_disconnected_ = advertiseService("network_configuration_disconnected", &NetworkConfiguration::sensorDisconnectedCallback, this);

            loadParam("timeout_secs", timeout_secs_, 60);

            m_default_configured = false;
            m_shutdown_requested = false;
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

        int getNetworkConfiguration()
        {
            // Check if service is available
            ros::Duration timeout_duration(timeout_secs_);
            if (!client_get_.waitForExistence(timeout_duration))
            {
                ROS_ERROR_STREAM(this->getNamespace() << " error: " << getErrorDescription(L3CAM_ROS_SERVICE_AVAILABILITY_TIMEOUT_ERROR) << ". Waited " << timeout_duration << " seconds");
                return L3CAM_ROS_SERVICE_AVAILABILITY_TIMEOUT_ERROR;
            }

            int error = L3CAM_OK;
            if (client_get_.call(srv_get_))
            {
                error = srv_get_.response.error;

                if (!error)
                {
                    ip_address_ = srv_get_.response.ip_address;
                    netmask_ = srv_get_.response.netmask;
                    gateway_ = srv_get_.response.gateway;
                    dhcp_ = false;

                    setDynamicReconfigure();
                }
                else
                {
                    ROS_ERROR_STREAM(this->getNamespace() << " error " << error << " while getting network configuration in " << __func__ << ": " << getErrorDescription(error));
                    return error;
                }
            }
            else
            {
                ROS_ERROR_STREAM(this->getNamespace() << " error: Failed to call service get_network_configuration");
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        ros::ServiceClient client_get_;
        l3cam_ros::GetNetworkConfiguration srv_get_;

    private:
        void setDynamicReconfigure()
        {
            // Dynamic reconfigure callback
            server_.setCallback(std::bind(&NetworkConfiguration::parametersCallback, this, std::placeholders::_1, std::placeholders::_2));
        }

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

        void parametersCallback(l3cam_ros::NetworkConfig &config, uint32_t level)
        {
            int error = L3CAM_OK;

            if (!m_default_configured)
            {
                config.ip_address = ip_address_;
                config.netmask = netmask_;
                config.gateway = gateway_;
                config.dhcp = dhcp_;

                m_default_configured = true;
            }
            else
            {
                error = callNetwork(config);
            }

            if (error)
            {
                ROS_ERROR_STREAM(this->getNamespace() << " error " << error << " while changing parameter: " << getErrorDescription(error));
            }
        }

        // Sensor calls
        int callNetwork(l3cam_ros::NetworkConfig &config)
        {
            int error = L3CAM_OK;

            srv_change_.request.ip_address = config.ip_address;
            srv_change_.request.netmask = config.netmask;
            srv_change_.request.gateway = config.gateway;
            srv_change_.request.enable_dhcp = config.dhcp;
            if (client_change_.call(srv_change_))
            {
                error = srv_change_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    ip_address_ = config.ip_address;
                    netmask_ = config.netmask;
                    gateway_ = config.gateway;
                    dhcp_ = config.dhcp;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.ip_address = ip_address_;
                    config.netmask = netmask_;
                    config.gateway = gateway_;
                    config.dhcp = dhcp_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.ip_address = ip_address_;
                config.netmask = netmask_;
                config.gateway = gateway_;
                config.dhcp = dhcp_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
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

        dynamic_reconfigure::Server<l3cam_ros::NetworkConfig> server_;

        ros::ServiceClient client_change_;
        l3cam_ros::ChangeNetworkConfiguration srv_change_;

        ros::ServiceServer srv_sensor_disconnected_;

        std::string ip_address_;
        std::string netmask_;
        std::string gateway_;
        bool dhcp_;

        int timeout_secs_;

        bool m_default_configured;
        bool m_shutdown_requested;

    }; // class NetworkConfiguration

} // namespace l3cam_ros

int main(int argc, char **argv)
{
    ros::init(argc, argv, "network_configuration");

    l3cam_ros::NetworkConfiguration *node = new l3cam_ros::NetworkConfiguration();

    // Shutdown if error returned
    int error = node->getNetworkConfiguration();
    if (error)
    {
        return error;
    }

    ROS_INFO("Network configuration is available");

    node->spin();

    ros::shutdown();
    return 0;
}
