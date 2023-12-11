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

#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <pthread.h>

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud_conversion.h"

#include <libL3Cam.h>
#include <beamagine.h>
#include <beamErrors.h>

pthread_t stream_thread;

bool g_listening = false;

struct threadData
{
    ros::Publisher publisher;
};

void *PointCloudThread(void *functionData)
{
    threadData *data = (struct threadData *)functionData;

    struct sockaddr_in m_socket;
    int m_socket_descriptor;           // Socket descriptor
    std::string m_address = "0.0.0.0"; // Local address of the network interface port connected to the L3CAM
    int m_udp_port = 6050;             // For the lidar it's 6050

    socklen_t socket_len = sizeof(m_socket);
    char *buffer;
    buffer = (char *)malloc(64000);

    int32_t m_pointcloud_size;
    int32_t *m_pointcloud_data;
    uint32_t m_timestamp;
    bool m_is_reading_pointcloud;
    int points_received = 1;
    int pointcloud_index = 1;

    if ((m_socket_descriptor = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        perror("Opening socket");
        return 0;
    }
    // else ROS_INFO("Socket Lidar created");

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

    // 1 second timeout for socket
    struct timeval read_timeout;
    read_timeout.tv_sec = 1;
    setsockopt(m_socket_descriptor, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof read_timeout);

    g_listening = true;
    ROS_INFO("LiDAR streaming.");

    while (g_listening)
    {
        int size_read = recvfrom(m_socket_descriptor, buffer, 64004, 0, (struct sockaddr *)&m_socket, &socket_len);

        if (size_read == 17) // Header
        {
            memcpy(&m_pointcloud_size, &buffer[1], 4);
            m_pointcloud_data = (int32_t *)malloc(sizeof(int32_t) * (((m_pointcloud_size) * 5) + 1));
            memcpy(&m_pointcloud_data[0], &m_pointcloud_size, sizeof(int32_t));
            int32_t suma_1, suma_2;
            memcpy(&suma_1, &buffer[5], sizeof(int32_t));
            memcpy(&suma_2, &buffer[9], sizeof(int32_t));
            memcpy(&m_timestamp, &buffer[13], sizeof(uint32_t));
            m_is_reading_pointcloud = true;
            points_received = 0;
            pointcloud_index = 1;
        }
        else if (size_read == 1) // End, send point cloud
        {
            m_is_reading_pointcloud = false;
            int32_t *data_received = (int32_t *)malloc(sizeof(int32_t) * (m_pointcloud_size * 5) + 1);
            memcpy(&data_received[0], &m_pointcloud_data[0], sizeof(int32_t) * ((m_pointcloud_size * 5) + 1));

            int size_pc = data_received[0];

            sensor_msgs::PointCloud cloud_;
            cloud_.points.resize(size_pc);
            cloud_.header.frame_id = "map";

            sensor_msgs::ChannelFloat32 intensity_channel;
            intensity_channel.name = "intensity";
            intensity_channel.values.resize(size_pc);
            sensor_msgs::ChannelFloat32 rgb_channel;
            rgb_channel.name = "rgb";
            rgb_channel.values.resize(size_pc);

            for (int i = 0; i < size_pc; i++)
            {
                cloud_.points[i].y = -(double)data_received[5 * i + 1] / 1000.0;

                cloud_.points[i].z = -(double)data_received[5 * i + 2] / 1000.0;

                cloud_.points[i].x = (double)data_received[5 * i + 3] / 1000.0;

                intensity_channel.values[i] = data_received[5 * i + 4];

                rgb_channel.values[i] = data_received[5 * i + 5];
            }

            cloud_.channels.push_back(intensity_channel);
            cloud_.channels.push_back(rgb_channel);

            sensor_msgs::PointCloud2 PC2_msg;
            sensor_msgs::convertPointCloudToPointCloud2(cloud_, PC2_msg);
            PC2_msg.header.frame_id = "lidar";
            // m_timestamp format: hhmmsszzz
            PC2_msg.header.stamp.sec = (uint32_t)(m_timestamp / 10000000) * 3600 +     // hh
                                       (uint32_t)((m_timestamp / 100000) % 100) * 60 + // mm
                                       (uint32_t)((m_timestamp / 1000) % 100);         // ss
            PC2_msg.header.stamp.nsec = (m_timestamp % 1000) * 10e6;                   // zzz

            data->publisher.publish(PC2_msg);

            free(m_pointcloud_data);
            points_received = 0;
            pointcloud_index = 1;
        }
        else if (size_read > 0) // Data
        {
            if (m_is_reading_pointcloud)
            {
                int32_t points = 0;
                memcpy(&points, &buffer[0], 4);
                memcpy(&m_pointcloud_data[pointcloud_index], &buffer[4], (sizeof(int32_t) * (points * 5)));

                pointcloud_index += (points * 5);

                points_received += points;

                // check if under size
                if (points_received >= m_pointcloud_size)
                    m_is_reading_pointcloud = false;
            }
        }
        // size_read == -1 --> timeout
    }

    data->publisher.shutdown();
    ROS_INFO("Exiting lidar streaming thread");
    free(buffer);
    free(m_pointcloud_data);

    shutdown(m_socket_descriptor, SHUT_RDWR);
    close(m_socket_descriptor);

    pthread_exit(0);
}

namespace l3cam_ros
{
    class LidarStream : public SensorStream
    {
    public:
        explicit LidarStream() : SensorStream()
        {
            declareServiceServers("lidar");
        }

        ros::Publisher publisher_;

    private:
        void stopListening()
        {
            g_listening = false;
        }

    }; // class LidarStream

} // namespace l3cam_ros

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_stream");

    l3cam_ros::LidarStream *node = new l3cam_ros::LidarStream();

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
            ROS_ERROR_STREAM(node->getNamespace() << " error " << error << " while checking sensor availability in " << __func__ << ": " << getErrorDescription(error));
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
        ROS_INFO("LiDAR available for streaming");
    }
    else
    {
        return 0;
    }

    node->publisher_ = node->advertise<sensor_msgs::PointCloud2>("/PC2_lidar", 10);

    threadData *data = (struct threadData *)malloc(sizeof(struct threadData));
    data->publisher = node->publisher_;
    pthread_create(&stream_thread, NULL, &PointCloudThread, (void *)data);

    node->spin();

    node->publisher_.shutdown();
    g_listening = false;
    usleep(2000000);

    ros::shutdown();
    return 0;
}
