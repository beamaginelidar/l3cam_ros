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

#include "sensor_stream.hpp"

namespace l3cam_ros
{
    SensorStream::SensorStream() : ros::NodeHandle("~")
    {
        client_get_sensors_ = this->serviceClient<l3cam_ros::GetSensorsAvailable>("/L3Cam/l3cam_ros_node/get_sensors_available");

        loadParam("timeout_secs", timeout_secs_, 60);
    }

    void SensorStream::spin()
    {
        while (ros::ok() && !m_shutdown_requested)
        {

            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }

        ros::shutdown();
    }

    void SensorStream::declareServiceServers(const std::string &sensor)
    {
        srv_sensor_disconnected_ = this->advertiseService(sensor + "_stream_disconnected", &SensorStream::sensorDisconnectedCallback, this);
    }

    template <typename T>
    void SensorStream::loadParam(const std::string &param_name, T &param_var, const T &default_val)
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

    bool SensorStream::sensorDisconnectedCallback(l3cam_ros::SensorDisconnected::Request &req, l3cam_ros::SensorDisconnected::Response &res)
    {
        ROS_BMG_UNUSED(res);
        stopListening();

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

} // namespace l3cam_ros