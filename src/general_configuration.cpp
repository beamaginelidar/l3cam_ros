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
    nh.param("ip_address", change_network_configuration_ip_address, "192.168.5.15");
    nh.param("netmask", change_network_configuration_netmask, "255.255.255.0");
    nh.param("gateway", change_network_configuration_gateway, "0.0.0.0");
    nh.param("dhcp", enable_network_configuration_dhcp, false);

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
