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
#include "l3cam_ros/GeneralConfig.h"

#include "libL3Cam.h"
#include "beamagine.h"
#include "beamErrors.h"

#include "l3cam_ros/ChangeNetworkConfiguration.h"

ros::ServiceClient client;
l3cam_ros::ChangeNetworkConfiguration srv;

std::string change_network_configuration_ip_address;
std::string change_network_configuration_netmask;
std::string change_network_configuration_gateway;
bool enable_network_configuration_dhcp;

void callback(l3cam_ros::GeneralConfig &config, uint32_t level)
{
    int error = L3CAM_OK;

    if (config.enable_network_configuration_dhcp != enable_network_configuration_dhcp)
    {
        srv.request.ip_address = change_network_configuration_ip_address;
        srv.request.netmask = change_network_configuration_netmask;
        srv.request.gateway = change_network_configuration_gateway;
        srv.request.enable_dhcp = config.enable_network_configuration_dhcp;
        if (client.call(srv))
        {
            error = srv.response.error;
            if (!error)
                enable_network_configuration_dhcp = config.enable_network_configuration_dhcp;
            else
                config.enable_network_configuration_dhcp = enable_network_configuration_dhcp;
        }
        else
        {
            ROS_ERROR("Failed to call service change_network_configuration");
            config.enable_network_configuration_dhcp = enable_network_configuration_dhcp;
        }
    }
    else if (config.change_network_configuration_ip_address != change_network_configuration_ip_address)
    {
        srv.request.ip_address = config.change_network_configuration_ip_address;
        srv.request.netmask = change_network_configuration_netmask;
        srv.request.gateway = change_network_configuration_gateway;
        srv.request.enable_dhcp = enable_network_configuration_dhcp;
        if (client.call(srv))
        {
            error = srv.response.error;
            if (!error)
                change_network_configuration_ip_address = config.change_network_configuration_ip_address;
            else
                config.change_network_configuration_ip_address = change_network_configuration_ip_address;
        }
        else
        {
            ROS_ERROR("Failed to call service change_network_configuration");
            config.change_network_configuration_ip_address = change_network_configuration_ip_address;
        }
    }
    else if (config.change_network_configuration_netmask != change_network_configuration_netmask)
    {
        srv.request.ip_address = change_network_configuration_ip_address;
        srv.request.netmask = config.change_network_configuration_netmask;
        srv.request.gateway = change_network_configuration_gateway;
        srv.request.enable_dhcp = enable_network_configuration_dhcp;
        if (client.call(srv))
        {
            error = srv.response.error;
            if (!error)
                change_network_configuration_netmask = config.change_network_configuration_netmask;
            else
                config.change_network_configuration_netmask = change_network_configuration_netmask;
        }
        else
        {
            ROS_ERROR("Failed to call service change_network_configuration");
            config.change_network_configuration_netmask = change_network_configuration_netmask;
        }
    }
    else if (config.change_network_configuration_gateway != change_network_configuration_gateway)
    {
        srv.request.ip_address = change_network_configuration_ip_address;
        srv.request.netmask = change_network_configuration_netmask;
        srv.request.gateway = config.change_network_configuration_gateway;
        srv.request.enable_dhcp = enable_network_configuration_dhcp;
        if (client.call(srv))
        {
            error = srv.response.error;
            if (!error)
                change_network_configuration_gateway = config.change_network_configuration_gateway;
            else
                config.change_network_configuration_gateway = change_network_configuration_gateway;
        }
        else
        {
            ROS_ERROR("Failed to call service change_network_configuration");
            config.change_network_configuration_gateway = change_network_configuration_gateway;
        }
    }

    if (error)
        ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "general_configuration");
    ros::NodeHandle nh;

    // Params
    nh.param<std::string>("ip_address", change_network_configuration_ip_address, "192.168.5.15");
    nh.param<std::string>("netmask", change_network_configuration_netmask, "255.255.255.0");
    nh.param<std::string>("gateway", change_network_configuration_gateway, "0.0.0.0");
    nh.param<bool>("dhcp", enable_network_configuration_dhcp, false);

    dynamic_reconfigure::Server<l3cam_ros::GeneralConfig> server;
    dynamic_reconfigure::Server<l3cam_ros::GeneralConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    client = nh.serviceClient<l3cam_ros::ChangeNetworkConfiguration>("change_network_configuration");

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
