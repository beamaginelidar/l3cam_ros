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
#include "l3cam_ros/PolarimetricConfig.h"

#include <libL3Cam.h>
#include <beamagine.h>
#include <beamErrors.h>

#include "l3cam_ros/GetSensorsAvailable.h"
#include "l3cam_ros/EnablePolarimetricCameraStreamProcessedImage.h"
#include "l3cam_ros/ChangePolarimetricCameraProcessType.h"
#include "l3cam_ros/ChangePolarimetricCameraBrightness.h"
#include "l3cam_ros/ChangePolarimetricCameraBlackLevel.h"
#include "l3cam_ros/EnablePolarimetricCameraAutoGain.h"
#include "l3cam_ros/ChangePolarimetricCameraAutoGainRange.h"
#include "l3cam_ros/ChangePolarimetricCameraGain.h"
#include "l3cam_ros/EnablePolarimetricCameraAutoExposureTime.h"
#include "l3cam_ros/ChangePolarimetricCameraAutoExposureTimeRange.h"
#include "l3cam_ros/ChangePolarimetricCameraExposureTime.h"
#include "l3cam_ros/ChangeStreamingProtocol.h"
#include "l3cam_ros/GetRtspPipeline.h"

#include "l3cam_ros/SensorDisconnected.h"

#include "l3cam_ros_utils.hpp"

namespace l3cam_ros
{
    class PolarimetricConfiguration : public ros::NodeHandle
    {
    public:
        explicit PolarimetricConfiguration() : ros::NodeHandle("~")
        {
            // Create service clients
            client_get_sensors_ = serviceClient<l3cam_ros::GetSensorsAvailable>("/L3Cam/l3cam_ros_node/get_sensors_available");
            client_stream_processed_ = serviceClient<l3cam_ros::EnablePolarimetricCameraStreamProcessedImage>("/L3Cam/polarimetric_wide_stream/enable_polarimetric_stream_processed_image");
            client_process_type_ = serviceClient<l3cam_ros::ChangePolarimetricCameraProcessType>("/L3Cam/polarimetric_wide_stream/change_polarimetric_process_type");
            client_brightness_ = serviceClient<l3cam_ros::ChangePolarimetricCameraBrightness>("/L3Cam/l3cam_ros_node/change_polarimetric_brightness");
            client_black_level_ = serviceClient<l3cam_ros::ChangePolarimetricCameraBlackLevel>("/L3Cam/l3cam_ros_node/change_polarimetric_black_level");
            client_enable_auto_gain_ = serviceClient<l3cam_ros::EnablePolarimetricCameraAutoGain>("/L3Cam/l3cam_ros_node/enable_polarimetric_auto_gain");
            client_auto_gain_range_ = serviceClient<l3cam_ros::ChangePolarimetricCameraAutoGainRange>("/L3Cam/l3cam_ros_node/change_polarimetric_auto_gain_range");
            client_gain_ = serviceClient<l3cam_ros::ChangePolarimetricCameraGain>("/L3Cam/l3cam_ros_node/change_polarimetric_gain");
            client_enable_auto_exposure_time_ = serviceClient<l3cam_ros::EnablePolarimetricCameraAutoExposureTime>("/L3Cam/l3cam_ros_node/enable_polarimetric_auto_exposure_time");
            client_auto_exposure_time_range_ = serviceClient<l3cam_ros::ChangePolarimetricCameraAutoExposureTimeRange>("/L3Cam/l3cam_ros_node/change_polarimetric_auto_exposure_time_range");
            client_exposure_time_ = serviceClient<l3cam_ros::ChangePolarimetricCameraExposureTime>("/L3Cam/l3cam_ros_node/change_polarimetric_exposure_time");
            client_change_streaming_protocol_ = serviceClient<l3cam_ros::ChangeStreamingProtocol>("/L3Cam/l3cam_ros_node/change_streaming_protocol");
            client_get_rtsp_pipeline_ = serviceClient<l3cam_ros::GetRtspPipeline>("/L3Cam/l3cam_ros_node/get_rtsp_pipeline");

            loadDefaultParams();

            // Create service server
            srv_sensor_disconnected_ = advertiseService("polarimetric_configuration_disconnected", &PolarimetricConfiguration::sensorDisconnectedCallback, this);

            m_default_configured = false;
            m_shutdown_requested = false;
        }

        void setDynamicReconfigure()
        {
            // Dynamic reconfigure callback
            // server_ is a pointer so we only declare it if sensor is available and reconfigure should be available
            server_ = new dynamic_reconfigure::Server<l3cam_ros::PolarimetricConfig>;
            server_->setCallback(std::bind(&PolarimetricConfiguration::parametersCallback, this, std::placeholders::_1, std::placeholders::_2));
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
            loadParam("polarimetric_stream_processed_image", polarimetric_stream_processed_, true);
            loadParam("polarimetric_process_type", polarimetric_process_type_, 4);
            loadParam("polarimetric_brightness", polarimetric_brightness_, 127);
            loadParam("polarimetric_black_level", polarimetric_black_level_, 6.0);
            loadParam("polarimetric_auto_gain", polarimetric_auto_gain_, true);
            loadParam("polarimetric_auto_gain_range_minimum", polarimetric_auto_gain_range_minimum_, 0.0);
            loadParam("polarimetric_auto_gain_range_maximum", polarimetric_auto_gain_range_maximum_, 48.0);
            loadParam("polarimetric_gain", polarimetric_gain_, 24.0);
            loadParam("polarimetric_auto_exposure_time", polarimetric_auto_exposure_time_, true);
            loadParam("polarimetric_auto_exposure_time_range_minimum", polarimetric_auto_exposure_time_range_minimum_, 33.456);
            loadParam("polarimetric_auto_exposure_time_range_maximum", polarimetric_auto_exposure_time_range_maximum_, 66470.6);
            loadParam("polarimetric_exposure_time", polarimetric_exposure_time_, 500000.0);
            loadParam("polarimetric_streaming_protocol", polarimetric_streaming_protocol_, 0);
        }

        void configureDefault(l3cam_ros::PolarimetricConfig &config)
        {
            // Configure default params to dynamix reconfigure if inside range
            config.polarimetric_stream_processed_image = polarimetric_stream_processed_;

            if (polarimetric_process_type_ >= 0 && polarimetric_process_type_ <= 4)
            {
                config.polarimetric_process_type = polarimetric_process_type_;
            }
            else
            {
                polarimetric_process_type_ = config.polarimetric_process_type;
            }

            if (polarimetric_brightness_ >= 0 && polarimetric_brightness_ <= 255)
            {
                config.polarimetric_brightness = polarimetric_brightness_;
            }
            else
            {
                polarimetric_brightness_ = config.polarimetric_brightness;
            }

            if (polarimetric_black_level_ >= 0 && polarimetric_black_level_ <= 12.5)
            {
                config.polarimetric_black_level = polarimetric_black_level_;
            }
            else
            {
                polarimetric_black_level_ = config.polarimetric_black_level;
            }

            config.polarimetric_auto_gain = polarimetric_auto_gain_;

            if (polarimetric_auto_gain_range_minimum_ >= 0 && polarimetric_auto_gain_range_minimum_ <= 48)
            {
                config.polarimetric_auto_gain_range_minimum = polarimetric_auto_gain_range_minimum_;
            }
            else
            {
                polarimetric_auto_gain_range_minimum_ = config.polarimetric_auto_gain_range_minimum;
            }

            if (polarimetric_auto_gain_range_maximum_ >= 0 && polarimetric_auto_gain_range_maximum_ <= 48)
            {
                config.polarimetric_auto_gain_range_maximum = polarimetric_auto_gain_range_maximum_;
            }
            else
            {
                polarimetric_auto_gain_range_maximum_ = config.polarimetric_auto_gain_range_maximum;
            }

            if (polarimetric_gain_ >= 0 && polarimetric_gain_ <= 48)
            {
                config.polarimetric_gain = polarimetric_gain_;
            }
            else
            {
                polarimetric_gain_ = config.polarimetric_gain;
            }

            config.polarimetric_auto_exposure_time = polarimetric_auto_exposure_time_;

            if (polarimetric_auto_exposure_time_range_minimum_ >= 33.456 && polarimetric_auto_exposure_time_range_minimum_ <= 66470.6)
            {
                config.polarimetric_auto_exposure_time_range_minimum = polarimetric_auto_exposure_time_range_minimum_;
            }
            else
            {
                polarimetric_auto_exposure_time_range_minimum_ = config.polarimetric_auto_exposure_time_range_minimum;
            }

            if (polarimetric_auto_exposure_time_range_maximum_ >= 33.456 && polarimetric_auto_exposure_time_range_maximum_ <= 66470.6)
            {
                config.polarimetric_auto_exposure_time_range_maximum = polarimetric_auto_exposure_time_range_maximum_;
            }
            else
            {
                polarimetric_auto_exposure_time_range_maximum_ = config.polarimetric_auto_exposure_time_range_maximum;
            }

            if (polarimetric_exposure_time_ >= 33.456 && polarimetric_exposure_time_ <= 66470.6)
            {
                config.polarimetric_exposure_time = polarimetric_exposure_time_;
            }
            else
            {
                polarimetric_exposure_time_ = config.polarimetric_exposure_time;
            }

            if (polarimetric_streaming_protocol_ == 0 || polarimetric_streaming_protocol_ == 1)
            {
                config.polarimetric_streaming_protocol = polarimetric_streaming_protocol_;
            }
            else
            {
                polarimetric_streaming_protocol_ = config.polarimetric_streaming_protocol;
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
                srv_get_rtsp_pipeline_.request.sensor_type = (int)sensorTypes::sensor_pol;
                if (client_get_rtsp_pipeline_.call(srv_get_rtsp_pipeline_))
                {
                    error = srv_get_rtsp_pipeline_.response.error;
                    if (!error)
                    {
                        polarimetric_rtsp_pipeline_ = srv_get_rtsp_pipeline_.response.pipeline;
                        config.polarimetric_rtsp_pipeline = srv_get_rtsp_pipeline_.response.pipeline;
                    }
                    else
                    {
                        ROS_ERROR_STREAM(this->getNamespace() << " error " << error << " while getting RTSP pipeline: " << getErrorDescription(error));
                        config.polarimetric_rtsp_pipeline = "";
                        polarimetric_rtsp_pipeline_ = "";
                    }
                }
                else
                {
                    ROS_ERROR_STREAM(this->getNamespace() << " error: Failed to call service get_rtsp_pipeline");
                    config.polarimetric_rtsp_pipeline = "";
                    polarimetric_rtsp_pipeline_ = "";
                }
            }

            m_default_configured = true;
        }

        void parametersCallback(l3cam_ros::PolarimetricConfig &config, uint32_t level)
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
                case 0: // polarimetric_stream_processed_image
                    error = callPolarimetricStreamProcessedImage(config);
                    break;
                case 1: // polarimetric_process_type
                    error = callPolarimetricProcessType(config);
                    break;
                case 2: // polarimetric_brightness
                    error = callPolarimetricBrightness(config);
                    break;
                case 3: // polarimetric_black_level
                    error = callPolarimetricBlackLevel(config);
                    break;
                case 4: // polarimetric_auto_gain
                    error = callPolarimetricAutoGain(config);
                    break;
                case 5: // polarimetric_auto_gain_range_minimum
                    error = callPolarimetricAutoGainRange(config);
                    break;
                case 6: // polarimetric_auto_gain_range_maximum
                    error = callPolarimetricAutoGainRange(config);
                    break;
                case 7: // polarimetric_gain
                    error = callPolarimetricGain(config);
                    break;
                case 8: // polarimetric_auto_exposure_time
                    error = callPolarimetricAutoExposureTime(config);
                    break;
                case 9: // polarimetric_auto_exposure_time_range_minimum
                    error = callPolarimetricAutoExposureTimeRange(config);
                    break;
                case 10: // polarimetric_auto_exposure_time_range_maximum
                    error = callPolarimetricAutoExposureTimeRange(config);
                    break;
                case 11: // polarimetric_exposure_time
                    error = callPolarimetricExposureTime(config);
                    break;
                case 12: // polarimetric_streaming_protocol
                    error = callPolarimetricStreamingProtocol(config);
                    break;
                case 13: // polarimetric_rtsp_pipeline
                    error = callPolarimetricRtspPipeline(config);
                    break;
                }
            }

            if (error)
            {
                ROS_ERROR_STREAM(this->getNamespace() << " error " << error << " while changing parameter: " << getErrorDescription(error));
            }
        }

        // Service calls
        int callPolarimetricStreamProcessedImage(l3cam_ros::PolarimetricConfig &config)
        {
            int error = L3CAM_OK;

            srv_stream_processed_.request.enabled = config.polarimetric_stream_processed_image;
            if (client_stream_processed_.call(srv_stream_processed_))
            {
                error = srv_stream_processed_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    polarimetric_stream_processed_ = config.polarimetric_stream_processed_image;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.polarimetric_stream_processed_image = polarimetric_stream_processed_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.polarimetric_stream_processed_image = polarimetric_stream_processed_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callPolarimetricProcessType(l3cam_ros::PolarimetricConfig &config)
        {
            int error = L3CAM_OK;

            srv_process_type_.request.type = config.polarimetric_process_type;
            if (client_process_type_.call(srv_process_type_))
            {
                error = srv_process_type_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    polarimetric_process_type_ = config.polarimetric_process_type;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.polarimetric_process_type = polarimetric_process_type_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.polarimetric_process_type = polarimetric_process_type_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callPolarimetricBrightness(l3cam_ros::PolarimetricConfig &config)
        {
            int error = L3CAM_OK;

            srv_brightness_.request.brightness = config.polarimetric_brightness;
            if (client_brightness_.call(srv_brightness_))
            {
                error = srv_brightness_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    polarimetric_brightness_ = config.polarimetric_brightness;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.polarimetric_brightness = polarimetric_brightness_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.polarimetric_brightness = polarimetric_brightness_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callPolarimetricBlackLevel(l3cam_ros::PolarimetricConfig &config)
        {
            int error = L3CAM_OK;

            srv_black_level_.request.black_level = config.polarimetric_black_level;
            if (client_black_level_.call(srv_black_level_))
            {
                error = srv_black_level_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    polarimetric_black_level_ = config.polarimetric_black_level;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.polarimetric_black_level = polarimetric_black_level_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.polarimetric_black_level = polarimetric_black_level_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callPolarimetricAutoGain(l3cam_ros::PolarimetricConfig &config)
        {
            int error = L3CAM_OK;

            srv_enable_auto_gain_.request.enabled = config.polarimetric_auto_gain;
            if (client_enable_auto_gain_.call(srv_enable_auto_gain_))
            {
                error = srv_enable_auto_gain_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    polarimetric_auto_gain_ = config.polarimetric_auto_gain;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.polarimetric_auto_gain = polarimetric_auto_gain_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.polarimetric_auto_gain = polarimetric_auto_gain_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callPolarimetricAutoGainRange(l3cam_ros::PolarimetricConfig &config)
        {
            int error = L3CAM_OK;

            if (polarimetric_auto_gain_)
            {
                srv_auto_gain_range_.request.min_gain = config.polarimetric_auto_gain_range_minimum;
                srv_auto_gain_range_.request.max_gain = config.polarimetric_auto_gain_range_maximum;
                if (client_auto_gain_range_.call(srv_auto_gain_range_))
                {
                    error = srv_auto_gain_range_.response.error;
                    if (!error)
                    {
                        // Parameter changed successfully, save value
                        polarimetric_auto_gain_range_minimum_ = config.polarimetric_auto_gain_range_minimum;
                        polarimetric_auto_gain_range_maximum_ = config.polarimetric_auto_gain_range_maximum;
                    }
                    else
                    {
                        // Parameter could not be changed, reset parameter to value before change
                        config.polarimetric_auto_gain_range_minimum = polarimetric_auto_gain_range_minimum_;
                        config.polarimetric_auto_gain_range_maximum = polarimetric_auto_gain_range_maximum_;
                    }
                }
                else
                {
                    // Service could not be called, reset parameter to value before change
                    config.polarimetric_auto_gain_range_minimum = polarimetric_auto_gain_range_minimum_;
                    config.polarimetric_auto_gain_range_maximum = polarimetric_auto_gain_range_maximum_;
                    return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
                }
            }
            else
            {
                ROS_WARN_STREAM(this->getNamespace() << " Polarimetric camera auto gain must be enabled to change auto gain range");
                config.polarimetric_auto_gain_range_minimum = polarimetric_auto_gain_range_minimum_;
                config.polarimetric_auto_gain_range_maximum = polarimetric_auto_gain_range_maximum_;
            }

            return error;
        }

        int callPolarimetricGain(l3cam_ros::PolarimetricConfig &config)
        {
            int error = L3CAM_OK;

            if (!polarimetric_auto_gain_)
            {
                srv_gain_.request.gain = config.polarimetric_gain;
                if (client_gain_.call(srv_gain_))
                {
                    error = srv_gain_.response.error;
                    if (!error)
                    {
                        // Parameter changed successfully, save value
                        polarimetric_gain_ = config.polarimetric_gain;
                    }
                    else
                    {
                        // Parameter could not be changed, reset parameter to value before change
                        config.polarimetric_gain = polarimetric_gain_;
                    }
                }
                else
                {
                    // Service could not be called, reset parameter to value before change
                    config.polarimetric_gain = polarimetric_gain_;
                    return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
                }
            }
            else
            {
                ROS_WARN_STREAM(this->getNamespace() << " Polarimetric camera auto gain must be disabled to change gain");
                config.polarimetric_gain = polarimetric_gain_;
            }

            return error;
        }

        int callPolarimetricAutoExposureTime(l3cam_ros::PolarimetricConfig &config)
        {
            int error = L3CAM_OK;

            srv_enable_auto_exposure_time_.request.enabled = config.polarimetric_auto_exposure_time;
            if (client_enable_auto_exposure_time_.call(srv_enable_auto_exposure_time_))
            {
                error = srv_enable_auto_exposure_time_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    polarimetric_auto_exposure_time_ = config.polarimetric_auto_exposure_time;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.polarimetric_auto_exposure_time = polarimetric_auto_exposure_time_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.polarimetric_auto_exposure_time = polarimetric_auto_exposure_time_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callPolarimetricAutoExposureTimeRange(l3cam_ros::PolarimetricConfig &config)
        {
            int error = L3CAM_OK;

            if (polarimetric_auto_exposure_time_)
            {
                srv_auto_exposure_time_range_.request.min_exposure = config.polarimetric_auto_exposure_time_range_minimum;
                srv_auto_exposure_time_range_.request.max_exposure = config.polarimetric_auto_exposure_time_range_maximum;
                if (client_auto_exposure_time_range_.call(srv_auto_exposure_time_range_))
                {
                    error = srv_auto_exposure_time_range_.response.error;
                    if (!error)
                    {
                        // Parameter changed successfully, save value
                        polarimetric_auto_exposure_time_range_minimum_ = config.polarimetric_auto_exposure_time_range_minimum;
                        polarimetric_auto_exposure_time_range_maximum_ = config.polarimetric_auto_exposure_time_range_maximum;
                    }
                    else
                    {
                        // Parameter could not be changed, reset parameter to value before change
                        config.polarimetric_auto_exposure_time_range_minimum = polarimetric_auto_exposure_time_range_minimum_;
                        config.polarimetric_auto_exposure_time_range_maximum = polarimetric_auto_exposure_time_range_maximum_;
                    }
                }
                else
                {
                    // Service could not be called, reset parameter to value before change
                    config.polarimetric_auto_exposure_time_range_minimum = polarimetric_auto_exposure_time_range_minimum_;
                    config.polarimetric_auto_exposure_time_range_maximum = polarimetric_auto_exposure_time_range_maximum_;
                    return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
                }
            }
            else
            {
                ROS_WARN_STREAM(this->getNamespace() << " Polarimetric camera auto exposure time must be enabled to change auto exposure time range");
                config.polarimetric_auto_exposure_time_range_minimum = polarimetric_auto_exposure_time_range_minimum_;
                config.polarimetric_auto_exposure_time_range_maximum = polarimetric_auto_exposure_time_range_maximum_;
            }

            return error;
        }

        int callPolarimetricExposureTime(l3cam_ros::PolarimetricConfig &config)
        {
            int error = L3CAM_OK;

            if (!polarimetric_auto_exposure_time_)
            {
                srv_exposure_time_.request.exposure_time = config.polarimetric_exposure_time;
                if (client_exposure_time_.call(srv_exposure_time_))
                {
                    error = srv_exposure_time_.response.error;
                    if (!error)
                    {
                        // Parameter changed successfully, save value
                        polarimetric_exposure_time_ = config.polarimetric_exposure_time;
                    }
                    else
                    {
                        // Parameter could not be changed, reset parameter to value before change
                        config.polarimetric_exposure_time = polarimetric_exposure_time_;
                    }
                }
                else
                {
                    // Service could not be called, reset parameter to value before change
                    config.polarimetric_exposure_time = polarimetric_exposure_time_;
                    return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
                }
            }
            else
            {
                ROS_WARN_STREAM(this->getNamespace() << " Polarimetric camera auto exposure time must be disabled to change exposure time");
                config.polarimetric_exposure_time = polarimetric_exposure_time_;
            }

            return error;
        }

        int callPolarimetricStreamingProtocol(l3cam_ros::PolarimetricConfig &config)
        {
            int error = L3CAM_OK;

            srv_change_streaming_protocol_.request.protocol = config.polarimetric_streaming_protocol;
            srv_change_streaming_protocol_.request.sensor_type = (int)sensorTypes::sensor_pol;
            if (client_change_streaming_protocol_.call(srv_change_streaming_protocol_))
            {
                error = srv_change_streaming_protocol_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    polarimetric_streaming_protocol_ = config.polarimetric_streaming_protocol;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.polarimetric_streaming_protocol = polarimetric_streaming_protocol_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.polarimetric_streaming_protocol = polarimetric_streaming_protocol_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callPolarimetricRtspPipeline(l3cam_ros::PolarimetricConfig &config)
        {
            // Read-only
            ROS_WARN_STREAM(this->getNamespace() << " The RTSP Pipeline parameter is read-only, only changeable at launch");
            config.polarimetric_rtsp_pipeline = polarimetric_rtsp_pipeline_;

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
            ros::shutdown();
            return true;
        }

        dynamic_reconfigure::Server<l3cam_ros::PolarimetricConfig> *server_;

        ros::ServiceClient client_stream_processed_;
        l3cam_ros::EnablePolarimetricCameraStreamProcessedImage srv_stream_processed_;
        ros::ServiceClient client_process_type_;
        l3cam_ros::ChangePolarimetricCameraProcessType srv_process_type_;
        ros::ServiceClient client_brightness_;
        l3cam_ros::ChangePolarimetricCameraBrightness srv_brightness_;
        ros::ServiceClient client_black_level_;
        l3cam_ros::ChangePolarimetricCameraBlackLevel srv_black_level_;
        ros::ServiceClient client_enable_auto_gain_;
        l3cam_ros::EnablePolarimetricCameraAutoGain srv_enable_auto_gain_;
        ros::ServiceClient client_auto_gain_range_;
        l3cam_ros::ChangePolarimetricCameraAutoGainRange srv_auto_gain_range_;
        ros::ServiceClient client_gain_;
        l3cam_ros::ChangePolarimetricCameraGain srv_gain_;
        ros::ServiceClient client_enable_auto_exposure_time_;
        l3cam_ros::EnablePolarimetricCameraAutoExposureTime srv_enable_auto_exposure_time_;
        ros::ServiceClient client_auto_exposure_time_range_;
        l3cam_ros::ChangePolarimetricCameraAutoExposureTimeRange srv_auto_exposure_time_range_;
        ros::ServiceClient client_exposure_time_;
        l3cam_ros::ChangePolarimetricCameraExposureTime srv_exposure_time_;
        ros::ServiceClient client_change_streaming_protocol_;
        l3cam_ros::ChangeStreamingProtocol srv_change_streaming_protocol_;
        ros::ServiceClient client_get_rtsp_pipeline_;
        l3cam_ros::GetRtspPipeline srv_get_rtsp_pipeline_;

        ros::ServiceServer srv_sensor_disconnected_;

        bool polarimetric_stream_processed_;
        int polarimetric_process_type_;
        int polarimetric_brightness_;
        double polarimetric_black_level_;
        bool polarimetric_auto_gain_;
        double polarimetric_auto_gain_range_minimum_;
        double polarimetric_auto_gain_range_maximum_;
        double polarimetric_gain_;
        bool polarimetric_auto_exposure_time_;
        double polarimetric_auto_exposure_time_range_minimum_;
        double polarimetric_auto_exposure_time_range_maximum_;
        double polarimetric_exposure_time_;
        int polarimetric_streaming_protocol_;
        std::string polarimetric_rtsp_pipeline_;

        bool m_default_configured;
        bool m_shutdown_requested;

    }; // class PolarimetricConfiguration

} // namespace l3cam_ros

int main(int argc, char **argv)
{
    ros::init(argc, argv, "polarimetric_configuration");

    l3cam_ros::PolarimetricConfiguration *node = new l3cam_ros::PolarimetricConfiguration();

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
                if (node->srv_get_sensors_.response.sensors[i].sensor_type == sensor_pol && node->srv_get_sensors_.response.sensors[i].sensor_available)
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
        ROS_INFO("Polarimetric configuration is available");
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
