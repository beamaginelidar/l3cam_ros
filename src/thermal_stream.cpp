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

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include <pthread.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <libL3Cam.h>
#include <beamagine.h>
#include <beamErrors.h>

#include "l3cam_ros/GetSensorsAvailable.h"

pthread_t thermal_thread;

ros::Publisher thermal_pub;

bool g_listening = false;

ros::ServiceClient clientGetSensors;
l3cam_ros::GetSensorsAvailable srvGetSensors;

void *ImageThread(void *functionData)
{
    struct sockaddr_in m_socket;
    int m_socket_descriptor;           // Socket descriptor
    std::string m_address = "0.0.0.0"; // Local address of the network interface port connected to the L3CAM
    int m_udp_port = 6030;             // For Thermal it's 6030

    socklen_t socket_len = sizeof(m_socket);
    char *buffer;
    buffer = (char *)malloc(64000);

    uint16_t m_image_height;
    uint16_t m_image_width;
    uint8_t m_image_channels;
    uint32_t m_timestamp;
    int m_image_data_size;
    bool m_is_reading_image;
    bool m_image_ready;
    char *m_image_buffer = NULL;
    int bytes_count = 0;

    if ((m_socket_descriptor = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        perror("Opening socket");
        return 0;
    }
    // else ROS_INFO("Socket Thermal created");
    memset((char *)&m_socket, 0, sizeof(struct sockaddr_in));
    m_socket.sin_addr.s_addr = inet_addr((char *)m_address.c_str());
    m_socket.sin_family = AF_INET;
    m_socket.sin_port = htons(m_udp_port);

    if (inet_aton((char *)m_address.c_str(), &m_socket.sin_addr) == 0)
    {
        perror("inet_aton() failed");
        return 0;
    }

    if (bind(m_socket_descriptor, (struct sockaddr *)&m_socket, sizeof(struct sockaddr_in)) == -1)
    {
        perror("Could not bind name to socket");
        close(m_socket_descriptor);
        return 0;
    }

    int rcvbufsize = 134217728;
    if (0 != setsockopt(m_socket_descriptor, SOL_SOCKET, SO_RCVBUF, (char *)&rcvbufsize, sizeof(rcvbufsize)))
    {
        perror("Error setting size to socket");
        return 0;
    }

    g_listening = true;
    ROS_INFO("Thermal streaming");
    uint8_t *image_pointer = NULL;

    while (g_listening)
    {
        int size_read = recvfrom(m_socket_descriptor, buffer, 64004, 0, (struct sockaddr *)&m_socket, &socket_len);
        if (size_read == 11)
        {
            memcpy(&m_image_height, &buffer[1], 2);
            memcpy(&m_image_width, &buffer[3], 2);
            memcpy(&m_image_channels, &buffer[5], 1);

            if(image_pointer != NULL)
            {
                free(image_pointer);
                image_pointer = NULL;
            }
            if(m_image_buffer != NULL)
            {
                free(m_image_buffer);
                m_image_buffer = NULL;
            }

            m_image_buffer = (char *)malloc(m_image_height * m_image_width * m_image_channels);
            image_pointer = (uint8_t *)malloc(m_image_height * m_image_width * m_image_channels);
            
            memcpy(&m_timestamp, &buffer[6], sizeof(uint32_t));
            m_image_data_size = m_image_height * m_image_width * m_image_channels;
            m_is_reading_image = true;
            m_image_ready = false;
            bytes_count = 0;
        }
        else if (size_read == 1)
        {
            m_is_reading_image = false;
            m_image_ready = true;
            bytes_count = 0;
            memcpy(image_pointer, m_image_buffer, m_image_data_size);

            cv::Mat img_data(m_image_height, m_image_width, CV_8UC3, image_pointer);

            cv_bridge::CvImage img_bridge;
            sensor_msgs::Image img_msg; // message to be sent

            std_msgs::Header header;         // empty header
            header.stamp = ros::Time::now(); // time
            header.frame_id = "lidar";
            img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img_data);
            img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
            thermal_pub.publish(img_msg);
        }
        else if (size_read > 0)
        {
            if (m_is_reading_image)
            {
                memcpy(&m_image_buffer[bytes_count], buffer, size_read);
                bytes_count += size_read;

                // check if under size
                if (bytes_count >= m_image_data_size)
                    m_is_reading_image = false;
            }
        }
    }

    free(buffer);
    free(m_image_buffer);

    shutdown(m_socket_descriptor, SHUT_RDWR);
    close(m_socket_descriptor);

    pthread_exit(0);
}

bool isThermalAvailable()
{
    int error = L3CAM_OK;

    if (clientGetSensors.call(srvGetSensors))
    {
        error = srvGetSensors.response.error;

        if (!error)
            for (int i = 0; i < srvGetSensors.response.num_sensors; ++i)
            {
                if (srvGetSensors.response.sensors[i].sensor_type == sensor_thermal)
                    return true;
            }
        else
        {
            ROS_ERROR_STREAM('(' << error << ") " << getBeamErrorDescription(error));
            return false;
        }
    }
    else
    {
        ROS_ERROR("Failed to call service get_sensors_available");
        return false;
    }

    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thermal_stream");
    ros::NodeHandle nh;

    clientGetSensors = nh.serviceClient<l3cam_ros::GetSensorsAvailable>("get_sensors_available");
    int error = L3CAM_OK;

    if (!isThermalAvailable())
        return 0;

    pthread_create(&thermal_thread, NULL, &ImageThread, NULL);

    thermal_pub = nh.advertise<sensor_msgs::Image>("/img_thermal", 2);

    ros::Rate loop_rate(1);
    while (ros::ok() && isThermalAvailable())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    g_listening = false;
    thermal_pub.shutdown();

    return 0;
}
