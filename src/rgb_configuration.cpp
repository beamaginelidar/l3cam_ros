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
#include "l3cam_ros/RgbConfig.h"

#include <libL3Cam.h>
#include <beamagine.h>
#include <beamErrors.h>

#include "l3cam_ros/GetSensorsAvailable.h"
#include "l3cam_ros/ChangeRgbCameraBrightness.h"
#include "l3cam_ros/ChangeRgbCameraContrast.h"
#include "l3cam_ros/ChangeRgbCameraSaturation.h"
#include "l3cam_ros/ChangeRgbCameraSharpness.h"
#include "l3cam_ros/ChangeRgbCameraGamma.h"
#include "l3cam_ros/ChangeRgbCameraGain.h"
#include "l3cam_ros/EnableRgbCameraAutoWhiteBalance.h"
#include "l3cam_ros/ChangeRgbCameraWhiteBalance.h"
#include "l3cam_ros/EnableRgbCameraAutoExposureTime.h"
#include "l3cam_ros/ChangeRgbCameraExposureTime.h"
#include "l3cam_ros/ChangeStreamingProtocol.h"
#include "l3cam_ros/GetRtspPipeline.h"

#include "l3cam_ros/SensorDisconnected.h"

#include "l3cam_ros_utils.hpp"

namespace l3cam_ros
{
    class RgbConfiguration : public ros::NodeHandle
    {
    public:
        explicit RgbConfiguration() : ros::NodeHandle("~")
        {
            client_get_sensors_ = serviceClient<l3cam_ros::GetSensorsAvailable>("/L3Cam/l3cam_ros_node/get_sensors_available");
            client_brightness_ = serviceClient<l3cam_ros::ChangeRgbCameraBrightness>("/L3Cam/l3cam_ros_node/change_rgb_brightness");
            client_contrast_ = serviceClient<l3cam_ros::ChangeRgbCameraContrast>("/L3Cam/l3cam_ros_node/change_rgb_contrast");
            client_saturation_ = serviceClient<l3cam_ros::ChangeRgbCameraSaturation>("/L3Cam/l3cam_ros_node/change_rgb_saturation");
            client_sharpness_ = serviceClient<l3cam_ros::ChangeRgbCameraSharpness>("/L3Cam/l3cam_ros_node/change_rgb_sharpness");
            client_gamma_ = serviceClient<l3cam_ros::ChangeRgbCameraGamma>("/L3Cam/l3cam_ros_node/change_rgb_gamma");
            client_gain_ = serviceClient<l3cam_ros::ChangeRgbCameraGain>("/L3Cam/l3cam_ros_node/change_rgb_gain");
            client_enable_auto_white_balance_ = serviceClient<l3cam_ros::EnableRgbCameraAutoWhiteBalance>("/L3Cam/l3cam_ros_node/enable_rgb_auto_white_balance");
            client_white_balance_ = serviceClient<l3cam_ros::ChangeRgbCameraWhiteBalance>("/L3Cam/l3cam_ros_node/change_rgb_white_balance");
            client_enable_auto_exposure_time_ = serviceClient<l3cam_ros::EnableRgbCameraAutoExposureTime>("/L3Cam/l3cam_ros_node/enable_rgb_auto_exposure_time");
            client_exposure_time_ = serviceClient<l3cam_ros::ChangeRgbCameraExposureTime>("/L3Cam/l3cam_ros_node/change_rgb_exposure_time");
            client_change_streaming_protocol_ = serviceClient<l3cam_ros::ChangeStreamingProtocol>("/L3Cam/l3cam_ros_node/change_streaming_protocol");
            client_get_rtsp_pipeline_ = serviceClient<l3cam_ros::GetRtspPipeline>("/L3Cam/l3cam_ros_node/get_rtsp_pipeline");

            loadDefaultParams();

            // Create service server
            srv_sensor_disconnected_ = advertiseService("rgb_configuration_disconnected", &RgbConfiguration::sensorDisconnectedCallback, this);

            m_default_configured = false;
            m_shutdown_requested = false;
        }

        void setDynamicReconfigure()
        {
            // Dynamic reconfigure callback
            // server_ is a pointer so we only declare it if sensor is available and reconfigure should be available
            server_ = new dynamic_reconfigure::Server<l3cam_ros::RgbConfig>;
            server_->setCallback(std::bind(&RgbConfiguration::parametersCallback, this, std::placeholders::_1, std::placeholders::_2));
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
                ROS_WARN_STREAM("Parameter '" << param_name << "' not defined");
                param_var = default_val;
            }
        }

        void loadDefaultParams()
        {
            // Get and save parameters
            loadParam("timeout_secs", timeout_secs_, 60);
            loadParam("rgb_brightness", rgb_brightness_, 0);
            loadParam("rgb_contrast", rgb_contrast_, 10);
            loadParam("rgb_saturation", rgb_saturation_, 16);
            loadParam("rgb_sharpness", rgb_sharpness_, 16);
            loadParam("rgb_gamma", rgb_gamma_, 220);
            loadParam("rgb_gain", rgb_gain_, 0);
            loadParam("rgb_auto_white_balance", rgb_auto_white_balance_, true);
            loadParam("rgb_white_balance", rgb_white_balance_, 5000);
            loadParam("rgb_auto_exposure_time", rgb_auto_exposure_time_, true);
            loadParam("rgb_exposure_time", rgb_exposure_time_, 156);
            loadParam("rgb_streaming_protocol", rgb_streaming_protocol_, 0);
        }

        void configureDefault(l3cam_ros::RgbConfig &config)
        {
            // Configure default params to dynamix reconfigure if inside range
            if (rgb_brightness_ >= -15 && rgb_brightness_ <= 15)
            {
                config.rgb_brightness = rgb_brightness_;
            }
            else
            {
                rgb_brightness_ = config.rgb_brightness;
            }

            if (rgb_contrast_ >= 0 && rgb_contrast_ <= 30)
            {
                config.rgb_contrast = rgb_contrast_;
            }
            else
            {
                rgb_contrast_ = config.rgb_contrast;
            }

            if (rgb_saturation_ >= 0 && rgb_saturation_ <= 60)
            {
                config.rgb_saturation = rgb_saturation_;
            }
            else
            {
                rgb_saturation_ = config.rgb_saturation;
            }

            if (rgb_sharpness_ >= 0 && rgb_sharpness_ <= 127)
            {
                config.rgb_sharpness = rgb_sharpness_;
            }
            else
            {
                rgb_sharpness_ = config.rgb_sharpness;
            }

            if (rgb_gamma_ >= 40 && rgb_gamma_ <= 500)
            {
                config.rgb_gamma = rgb_gamma_;
            }
            else
            {
                rgb_gamma_ = config.rgb_gamma;
            }

            if (rgb_gain_ >= 0 && rgb_gain_ <= 63)
            {
                config.rgb_gain = rgb_gain_;
            }
            else
            {
                rgb_gain_ = config.rgb_gain;
            }

            config.rgb_auto_white_balance = rgb_auto_white_balance_;

            if (rgb_white_balance_ >= 1000 && rgb_white_balance_ <= 10000)
            {
                config.rgb_white_balance = rgb_white_balance_;
            }
            else
            {
                rgb_white_balance_ = config.rgb_white_balance;
            }

            config.rgb_auto_exposure_time = rgb_auto_exposure_time_;

            if (rgb_exposure_time_ >= 1 && rgb_exposure_time_ <= 10000)
            {
                config.rgb_exposure_time = rgb_exposure_time_;
            }
            else
            {
                rgb_exposure_time_ = config.rgb_exposure_time;
            }

            if (rgb_streaming_protocol_ == 0 || rgb_streaming_protocol_ == 1)
            {
                config.rgb_streaming_protocol = rgb_streaming_protocol_;
            }
            else
            {
                rgb_streaming_protocol_ = config.rgb_streaming_protocol;
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
                srv_get_rtsp_pipeline_.request.sensor_type = (int)sensorTypes::sensor_econ_rgb;
                if (client_get_rtsp_pipeline_.call(srv_get_rtsp_pipeline_))
                {
                    error = srv_get_rtsp_pipeline_.response.error;
                    if (!error)
                    {
                        rgb_rtsp_pipeline_ = srv_get_rtsp_pipeline_.response.pipeline;
                        config.rgb_rtsp_pipeline = srv_get_rtsp_pipeline_.response.pipeline;
                    }
                    else
                    {
                        ROS_ERROR_STREAM(this->getNamespace() << " error " << error << " while getting RTSP pipeline: " << getErrorDescription(error));
                        config.rgb_rtsp_pipeline = "";
                        rgb_rtsp_pipeline_ = "";
                    }
                }
                else
                {
                    ROS_ERROR_STREAM(this->getNamespace() << " error: Failed to call service get_rtsp_pipeline");
                    config.rgb_rtsp_pipeline = "";
                    rgb_rtsp_pipeline_ = "";
                }
            }

            m_default_configured = true;
        }

        void parametersCallback(l3cam_ros::RgbConfig &config, uint32_t level)
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
                case 0: // rgb_brightness
                    error = callRgbBrightness(config);
                    break;
                case 1: // rgb_contrast
                    error = callRgbContrast(config);
                    break;
                case 2: // rgb_saturation
                    error = callRgbSaturation(config);
                    break;
                case 3: // rgb_sharpness
                    error = callRgbSharpness(config);
                    break;
                case 4: // rgb_gamma
                    error = callRgbGamma(config);
                    break;
                case 5: // rgb_gain
                    error = callRgbGain(config);
                    break;
                case 6: // rgb_auto_white_balance
                    error = callRgbAutoWhiteBalance(config);
                    break;
                case 7: // rgb_white_balance
                    error = callRgbWhiteBalance(config);
                    break;
                case 8: // rgb_auto_exposure_time
                    error = callRgbAutoExposureTime(config);
                    break;
                case 9: // rgb_exposure_time
                    error = callRgbExposureTime(config);
                    break;
                case 10: // rgb_streaming_protocol
                    error = callRgbStreamingProtocol(config);
                    break;
                case 11: // rgb_rtsp_pipeline
                    error = callRgbRtspPipeline(config);
                    break;
                }
            }

            if (error)
            {
                ROS_ERROR_STREAM(this->getNamespace() << " error " << error << " while changing parameter: " << getErrorDescription(error));
            }
        }

        // Service calls
        int callRgbBrightness(l3cam_ros::RgbConfig &config)
        {
            int error = L3CAM_OK;

            srv_brightness_.request.brightness = config.rgb_brightness;
            if (client_brightness_.call(srv_brightness_))
            {
                error = srv_brightness_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    rgb_brightness_ = config.rgb_brightness;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.rgb_brightness = rgb_brightness_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.rgb_brightness = rgb_brightness_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callRgbContrast(l3cam_ros::RgbConfig &config)
        {
            int error = L3CAM_OK;

            srv_contrast_.request.contrast = config.rgb_contrast;
            if (client_contrast_.call(srv_contrast_))
            {
                error = srv_contrast_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    rgb_contrast_ = config.rgb_contrast;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.rgb_contrast = rgb_contrast_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.rgb_contrast = rgb_contrast_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callRgbSaturation(l3cam_ros::RgbConfig &config)
        {
            int error = L3CAM_OK;

            srv_saturation_.request.saturation = config.rgb_saturation;
            if (client_saturation_.call(srv_saturation_))
            {
                error = srv_saturation_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    rgb_saturation_ = config.rgb_saturation;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.rgb_saturation = rgb_saturation_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.rgb_saturation = rgb_saturation_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callRgbSharpness(l3cam_ros::RgbConfig &config)
        {
            int error = L3CAM_OK;

            srv_sharpness_.request.sharpness = config.rgb_sharpness;
            if (client_sharpness_.call(srv_sharpness_))
            {
                error = srv_sharpness_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    rgb_sharpness_ = config.rgb_sharpness;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.rgb_sharpness = rgb_sharpness_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.rgb_sharpness = rgb_sharpness_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callRgbGamma(l3cam_ros::RgbConfig &config)
        {
            int error = L3CAM_OK;

            srv_gamma_.request.gamma = config.rgb_gamma;
            if (client_gamma_.call(srv_gamma_))
            {
                error = srv_gamma_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    rgb_gamma_ = config.rgb_gamma;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.rgb_gamma = rgb_gamma_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.rgb_gamma = rgb_gamma_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callRgbGain(l3cam_ros::RgbConfig &config)
        {
            int error = L3CAM_OK;

            srv_gain_.request.gain = config.rgb_gain;
            if (client_gain_.call(srv_gain_))
            {
                error = srv_gain_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    rgb_gain_ = config.rgb_gain;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.rgb_gain = rgb_gain_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.rgb_gain = rgb_gain_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callRgbAutoWhiteBalance(l3cam_ros::RgbConfig &config)
        {
            int error = L3CAM_OK;

            srv_enable_auto_white_balance_.request.enabled = config.rgb_auto_white_balance;
            if (client_enable_auto_white_balance_.call(srv_enable_auto_white_balance_))
            {
                error = srv_enable_auto_white_balance_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    rgb_auto_white_balance_ = config.rgb_auto_white_balance;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.rgb_auto_white_balance = rgb_auto_white_balance_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.rgb_auto_white_balance = rgb_auto_white_balance_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callRgbWhiteBalance(l3cam_ros::RgbConfig &config)
        {
            int error = L3CAM_OK;

            if (!rgb_auto_white_balance_)
            {
                srv_white_balance_.request.white_balance = config.rgb_white_balance;
                if (client_white_balance_.call(srv_white_balance_))
                {
                    error = srv_white_balance_.response.error;
                    if (!error)
                    {
                        // Parameter changed successfully, save value
                        rgb_white_balance_ = config.rgb_white_balance;
                    }
                    else
                    {
                        // Parameter could not be changed, reset parameter to value before change
                        config.rgb_white_balance = rgb_white_balance_;
                    }
                }
                else
                {
                    // Service could not be called, reset parameter to value before change
                    config.rgb_white_balance = rgb_white_balance_;
                    return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
                }
            }
            else
            {
                ROS_WARN("RGB camera auto white balance must be disabled to change white balance");
                config.rgb_white_balance = rgb_white_balance_;
            }

            return error;
        }

        int callRgbAutoExposureTime(l3cam_ros::RgbConfig &config)
        {
            int error = L3CAM_OK;

            srv_enable_auto_exposure_time_.request.enabled = config.rgb_auto_exposure_time;
            if (client_enable_auto_exposure_time_.call(srv_enable_auto_exposure_time_))
            {
                error = srv_enable_auto_exposure_time_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    rgb_auto_exposure_time_ = config.rgb_auto_exposure_time;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.rgb_auto_exposure_time = rgb_auto_exposure_time_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.rgb_auto_exposure_time = rgb_auto_exposure_time_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callRgbExposureTime(l3cam_ros::RgbConfig &config)
        {
            int error = L3CAM_OK;

            if (!rgb_auto_exposure_time_)
            {
                srv_exposure_time_.request.exposure_time = config.rgb_exposure_time;
                if (client_exposure_time_.call(srv_exposure_time_))
                {
                    error = srv_exposure_time_.response.error;
                    if (!error)
                    {
                        // Parameter changed successfully, save value
                        rgb_exposure_time_ = config.rgb_exposure_time;
                    }
                    else
                    {
                        // Parameter could not be changed, reset parameter to value before change
                        config.rgb_exposure_time = rgb_exposure_time_;
                    }
                }
                else
                {
                    // Service could not be called, reset parameter to value before change
                    config.rgb_exposure_time = rgb_exposure_time_;
                    return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
                }
            }
            else
            {
                ROS_WARN("RGB camera auto exposure time must be disabled to change exposure time");
                config.rgb_exposure_time = rgb_exposure_time_;
            }

            return error;
        }

        int callRgbStreamingProtocol(l3cam_ros::RgbConfig &config)
        {
            int error = L3CAM_OK;

            srv_change_streaming_protocol_.request.protocol = config.rgb_streaming_protocol;
            srv_change_streaming_protocol_.request.sensor_type = (int)sensorTypes::sensor_econ_rgb;
            if (client_change_streaming_protocol_.call(srv_change_streaming_protocol_))
            {
                error = srv_change_streaming_protocol_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    rgb_streaming_protocol_ = config.rgb_streaming_protocol;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.rgb_streaming_protocol = rgb_streaming_protocol_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.rgb_streaming_protocol = rgb_streaming_protocol_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callRgbRtspPipeline(l3cam_ros::RgbConfig &config)
        {
            // Read-only
            ROS_WARN("The RTSP Pipeline parameter is read-only, only changeable at launch");
            config.rgb_rtsp_pipeline = rgb_rtsp_pipeline_;

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

        dynamic_reconfigure::Server<l3cam_ros::RgbConfig> *server_;

        ros::ServiceClient client_brightness_;
        l3cam_ros::ChangeRgbCameraBrightness srv_brightness_;
        ros::ServiceClient client_contrast_;
        l3cam_ros::ChangeRgbCameraContrast srv_contrast_;
        ros::ServiceClient client_saturation_;
        l3cam_ros::ChangeRgbCameraSaturation srv_saturation_;
        ros::ServiceClient client_sharpness_;
        l3cam_ros::ChangeRgbCameraSharpness srv_sharpness_;
        ros::ServiceClient client_gamma_;
        l3cam_ros::ChangeRgbCameraGamma srv_gamma_;
        ros::ServiceClient client_gain_;
        l3cam_ros::ChangeRgbCameraGain srv_gain_;
        ros::ServiceClient client_enable_auto_white_balance_;
        l3cam_ros::EnableRgbCameraAutoWhiteBalance srv_enable_auto_white_balance_;
        ros::ServiceClient client_white_balance_;
        l3cam_ros::ChangeRgbCameraWhiteBalance srv_white_balance_;
        ros::ServiceClient client_enable_auto_exposure_time_;
        l3cam_ros::EnableRgbCameraAutoExposureTime srv_enable_auto_exposure_time_;
        ros::ServiceClient client_exposure_time_;
        l3cam_ros::ChangeRgbCameraExposureTime srv_exposure_time_;
        ros::ServiceClient client_change_streaming_protocol_;
        l3cam_ros::ChangeStreamingProtocol srv_change_streaming_protocol_;
        ros::ServiceClient client_get_rtsp_pipeline_;
        l3cam_ros::GetRtspPipeline srv_get_rtsp_pipeline_;

        ros::ServiceServer srv_sensor_disconnected_;

        int rgb_brightness_;
        int rgb_contrast_;
        int rgb_saturation_;
        int rgb_sharpness_;
        int rgb_gamma_;
        int rgb_gain_;
        bool rgb_auto_white_balance_;
        int rgb_white_balance_;
        bool rgb_auto_exposure_time_;
        int rgb_exposure_time_;
        int rgb_streaming_protocol_;
        std::string rgb_rtsp_pipeline_;

        bool m_default_configured;
        bool m_shutdown_requested;

    }; // class RgbConfiguration

} // namespace l3cam_ros

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgb_configuration");

    l3cam_ros::RgbConfiguration *node = new l3cam_ros::RgbConfiguration();

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
                if (node->srv_get_sensors_.response.sensors[i].sensor_type == sensor_econ_rgb && node->srv_get_sensors_.response.sensors[i].sensor_available)
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
        ROS_INFO("RGB configuration is available");
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
