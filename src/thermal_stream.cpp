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

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <libL3Cam.h>
#include <beamagine.h>
#include <beamErrors.h>

pthread_t stream_thread;
pthread_t stream_f_thread;

bool g_listening = false;

int utc = 1; // UTC +1

struct threadData
{
    ros::Publisher publisher;
};

bool openSocket(int &m_socket_descriptor, sockaddr_in &m_socket, std::string &m_address, int m_udp_port)
{
    if ((m_socket_descriptor = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        perror("Opening socket");
        return false;
    }
    // else ROS_INFO("Socket created");

    memset((char *)&m_socket, 0, sizeof(struct sockaddr_in));
    m_socket.sin_addr.s_addr = inet_addr((char *)m_address.c_str());
    m_socket.sin_family = AF_INET;
    m_socket.sin_port = htons(m_udp_port);

    if (inet_aton((char *)m_address.c_str(), &m_socket.sin_addr) == 0)
    {
        perror("inet_aton() failed");
        return false;
    }

    if (bind(m_socket_descriptor, (struct sockaddr *)&m_socket, sizeof(struct sockaddr_in)) == -1)
    {
        perror("Could not bind name to socket");
        close(m_socket_descriptor);
        return false;
    }

    int rcvbufsize = 134217728;
    if (0 != setsockopt(m_socket_descriptor, SOL_SOCKET, SO_RCVBUF, (char *)&rcvbufsize, sizeof(rcvbufsize)))
    {
        perror("Error setting size to socket");
        return false;
    }

    // 1 second timeout for socket
    struct timeval read_timeout;
    read_timeout.tv_sec = 1;
    setsockopt(m_socket_descriptor, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof read_timeout);

    return true;
}

void *ImageThread(void *functionData)
{
    threadData *data = (struct threadData *)functionData;

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
    bool m_is_reading_image = false;
    char *m_image_buffer = NULL;
    int bytes_count = 0;

    if(!openSocket(m_socket_descriptor, m_socket, m_address, m_udp_port))
    {
        return 0;
    }

    g_listening = true;
    ROS_INFO("Thermal streaming");

    uint8_t *image_pointer = NULL;

    while (g_listening)
    {
        int size_read = recvfrom(m_socket_descriptor, buffer, 64000, 0, (struct sockaddr *)&m_socket, &socket_len);
        if (size_read == 11) // Header
        {
            memcpy(&m_image_height, &buffer[1], 2);
            memcpy(&m_image_width, &buffer[3], 2);
            memcpy(&m_image_channels, &buffer[5], 1);

            if (image_pointer != NULL)
            {
                free(image_pointer);
                image_pointer = NULL;
            }
            if (m_image_buffer != NULL)
            {
                free(m_image_buffer);
                m_image_buffer = NULL;
            }

            m_image_buffer = (char *)malloc(m_image_height * m_image_width * m_image_channels);
            image_pointer = (uint8_t *)malloc(m_image_height * m_image_width * m_image_channels);

            memcpy(&m_timestamp, &buffer[6], sizeof(uint32_t));
            m_image_data_size = m_image_height * m_image_width * m_image_channels;
            m_is_reading_image = true;
            bytes_count = 0;
        }
        else if (size_read == 1 && bytes_count == m_image_data_size) // End, send image
        {
            m_is_reading_image = false;
            bytes_count = 0;
            memcpy(image_pointer, m_image_buffer, m_image_data_size);

            cv::Mat img_data;
            if (m_image_channels == 1)
            {
                img_data = cv::Mat(m_image_height, m_image_width, CV_8UC1, image_pointer);
            }
            else if (m_image_channels == 3)
            {
                img_data = cv::Mat(m_image_height, m_image_width, CV_8UC3, image_pointer);
            }

            std_msgs::Header header;
            header.frame_id = "thermal";
            // m_timestamp format: hhmmsszzz
            header.stamp.sec = (uint32_t)(m_timestamp / 10000000) * 3600 +     // hh
                               (uint32_t)((m_timestamp / 100000) % 100) * 60 + // mm
                               (uint32_t)((m_timestamp / 1000) % 100);         // ss
            header.stamp.nsec = (m_timestamp % 1000) * 10e6;                   // zzz

            const std::string encoding = m_image_channels == 1 ? sensor_msgs::image_encodings::MONO8 : sensor_msgs::image_encodings::BGR8;
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, encoding, img_data).toImageMsg();

            data->publisher.publish(img_msg);
        }
        else if (size_read > 0 && m_is_reading_image) // Data
        {
            memcpy(&m_image_buffer[bytes_count], buffer, size_read);
            bytes_count += size_read;

            // check if under size
            if (bytes_count >= m_image_data_size)
                m_is_reading_image = false;
        }
        // size_read == -1 --> timeout
    }

    data->publisher.shutdown();
    ROS_INFO_STREAM("Exiting thermal streaming thread");
    free(buffer);

    shutdown(m_socket_descriptor, SHUT_RDWR);
    close(m_socket_descriptor);

    pthread_exit(0);
}

void *FloatImageThread(void *functionData)
{
    threadData *data = (struct threadData *)functionData;

    struct sockaddr_in m_socket;
    int m_socket_descriptor;           // Socket descriptor
    std::string m_address = "0.0.0.0"; // Local address of the network interface port connected to the L3CAM
    int m_udp_port = 6031;             // For float Thermal it's 6031

    socklen_t socket_len = sizeof(m_socket);
    char *buffer;
    buffer = (char *)malloc(64000);

    uint16_t m_image_height;
    uint16_t m_image_width;
    uint32_t m_timestamp;
    int m_image_data_size;
    bool m_is_reading_image = false;
    int bytes_count = 0;

    if(!openSocket(m_socket_descriptor, m_socket, m_address, m_udp_port))
    {
        return 0;
    }

    g_listening = true;
    ROS_INFO("Float Thermal streaming");

    float *thermal_data_pointer = NULL;
    int float_pointer_cnt = 0;

    while (g_listening)
    {
        int size_read = recvfrom(m_socket_descriptor, buffer, 64000, 0, (struct sockaddr *)&m_socket, &socket_len);
        if (size_read == 9) // Header
        {
            memcpy(&m_image_height, &buffer[1], 2);
            memcpy(&m_image_width, &buffer[3], 2);
            memcpy(&m_timestamp, &buffer[5], 4);

            if (thermal_data_pointer != NULL)
            {
                free(thermal_data_pointer);
                thermal_data_pointer = NULL;
            }

            m_image_data_size = m_image_height * m_image_width * sizeof(float);

            thermal_data_pointer = (float *)malloc(m_image_data_size);

            m_is_reading_image = true;
            bytes_count = 0;
            float_pointer_cnt = 0;
        }
        else if (size_read == 1 && bytes_count == m_image_data_size) // End, send image
        {
            m_is_reading_image = false;
            bytes_count = 0;
            float_pointer_cnt = 0;

            cv::Mat float_image = cv::Mat(m_image_height, m_image_width, CV_32FC1, thermal_data_pointer);
            
            // publish float image
            std_msgs::Header header;
            header.frame_id = "f_thermal";
            // m_timestamp format: hhmmsszzz
            time_t raw_time = ros::Time::now().toSec();
            std::tm *time_info = std::localtime(&raw_time);
            time_info->tm_sec = 0;
            time_info->tm_min = 0;
            time_info->tm_hour = utc;
            header.stamp.sec = std::mktime(time_info) +
                               (uint32_t)(m_timestamp / 10000000) * 3600 +     // hh
                               (uint32_t)((m_timestamp / 100000) % 100) * 60 + // mm
                               (uint32_t)((m_timestamp / 1000) % 100);         // ss
            header.stamp.nsec = (m_timestamp % 1000) * 10e6;                   // zzz

            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_32FC1, float_image).toImageMsg();

            data->publisher.publish(img_msg);

        }
        else if (size_read > 0 && m_is_reading_image) // Data
        {
            memcpy(&thermal_data_pointer[float_pointer_cnt], buffer, size_read);
            bytes_count += size_read;
            float_pointer_cnt += size_read / 4;
        }
        // size_read == -1 --> timeout
    }

    data->publisher.shutdown();
    ROS_INFO_STREAM("Exiting float thermal streaming thread");
    free(buffer);

    shutdown(m_socket_descriptor, SHUT_RDWR);
    close(m_socket_descriptor);

    pthread_exit(0);
}

namespace l3cam_ros
{
    class ThermalStream : public SensorStream
    {
    public:
        explicit ThermalStream() : SensorStream()
        {
            declareServiceServers("thermal");
        }

        ros::Publisher publisher_;
        ros::Publisher f_publisher_;

    private:
        void stopListening()
        {
            g_listening = false;
        }

    }; // class ThermalStream

} // namespace l3cam_ros

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thermal_stream");

    l3cam_ros::ThermalStream *node = new l3cam_ros::ThermalStream();

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
        ROS_INFO_STREAM("Thermal camera available for streaming");
    }
    else
    {
        return 0;
    }

    node->publisher_ = node->advertise<sensor_msgs::Image>("/img_thermal", 10);
    node->f_publisher_ = node->advertise<sensor_msgs::Image>("/img_f_thermal", 10);

    threadData *data = (struct threadData *)malloc(sizeof(struct threadData));
    data->publisher = node->publisher_;
    pthread_create(&stream_thread, NULL, &ImageThread, (void *)data);

    threadData *f_data = (struct threadData *)malloc(sizeof(struct threadData));
    f_data->publisher = node->f_publisher_;
    pthread_create(&stream_f_thread, NULL, &FloatImageThread, (void *)f_data);

    node->spin();

    node->publisher_.shutdown();
    node->f_publisher_.shutdown();
    g_listening = false;
    usleep(2000000);

    ros::shutdown();
    return 0;
}
