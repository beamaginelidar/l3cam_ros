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
#include "l3cam_ros/AlliedNarrowConfig.h"

#include <libL3Cam.h>
#include <beamagine.h>
#include <beamErrors.h>

#include "l3cam_ros/GetSensorsAvailable.h"
#include "l3cam_ros/ChangeAlliedCameraExposureTime.h"
#include "l3cam_ros/EnableAlliedCameraAutoExposureTime.h"
#include "l3cam_ros/ChangeAlliedCameraAutoExposureTimeRange.h"
#include "l3cam_ros/ChangeAlliedCameraGain.h"
#include "l3cam_ros/EnableAlliedCameraAutoGain.h"
#include "l3cam_ros/ChangeAlliedCameraAutoGainRange.h"
#include "l3cam_ros/ChangeAlliedCameraGamma.h"
#include "l3cam_ros/ChangeAlliedCameraSaturation.h"
#include "l3cam_ros/ChangeAlliedCameraHue.h"
#include "l3cam_ros/ChangeAlliedCameraIntensityAutoPrecedence.h"
#include "l3cam_ros/EnableAlliedCameraAutoWhiteBalance.h"
#include "l3cam_ros/ChangeAlliedCameraBalanceRatioSelector.h"
#include "l3cam_ros/ChangeAlliedCameraBalanceRatio.h"
#include "l3cam_ros/ChangeAlliedCameraBalanceWhiteAutoRate.h"
#include "l3cam_ros/ChangeAlliedCameraBalanceWhiteAutoTolerance.h"
#include "l3cam_ros/ChangeAlliedCameraIntensityControllerRegion.h"
#include "l3cam_ros/ChangeAlliedCameraIntensityControllerTarget.h"
#include "l3cam_ros/ChangeStreamingProtocol.h"
#include "l3cam_ros/GetRtspPipeline.h"
#include "l3cam_ros/GetAlliedCameraExposureTime.h"
#include "l3cam_ros/GetAlliedCameraGain.h"

#include "l3cam_ros/SensorDisconnected.h"

#include "l3cam_ros_utils.hpp"

namespace l3cam_ros
{
    class AlliedNarrowConfiguration : public ros::NodeHandle
    {
    public:
        explicit AlliedNarrowConfiguration() : ros::NodeHandle("~")
        {
            // Create service clients
            client_get_sensors_ = serviceClient<l3cam_ros::GetSensorsAvailable>("/L3Cam/l3cam_ros_node/get_sensors_available");
            client_exposure_time_ = serviceClient<l3cam_ros::ChangeAlliedCameraExposureTime>("/L3Cam/l3cam_ros_node/change_allied_exposure_time");
            client_enable_auto_exposure_time_ = serviceClient<l3cam_ros::EnableAlliedCameraAutoExposureTime>("/L3Cam/l3cam_ros_node/enable_allied_auto_exposure_time");
            client_auto_exposure_time_range_ = serviceClient<l3cam_ros::ChangeAlliedCameraAutoExposureTimeRange>("/L3Cam/l3cam_ros_node/change_allied_auto_exposure_time_range");
            client_gain_ = serviceClient<l3cam_ros::ChangeAlliedCameraGain>("/L3Cam/l3cam_ros_node/change_allied_gain");
            client_enable_auto_gain_ = serviceClient<l3cam_ros::EnableAlliedCameraAutoGain>("/L3Cam/l3cam_ros_node/enable_allied_auto_gain");
            client_auto_gain_range_ = serviceClient<l3cam_ros::ChangeAlliedCameraAutoGainRange>("/L3Cam/l3cam_ros_node/change_allied_auto_gain_range");
            client_gamma_ = serviceClient<l3cam_ros::ChangeAlliedCameraGamma>("/L3Cam/l3cam_ros_node/change_allied_gamma");
            client_saturation_ = serviceClient<l3cam_ros::ChangeAlliedCameraSaturation>("/L3Cam/l3cam_ros_node/change_allied_saturation");
            client_hue_ = serviceClient<l3cam_ros::ChangeAlliedCameraHue>("/L3Cam/l3cam_ros_node/change_allied_hue");
            client_intensity_auto_precedence_ = serviceClient<l3cam_ros::ChangeAlliedCameraIntensityAutoPrecedence>("/L3Cam/l3cam_ros_node/change_allied_intensity_auto_precedence");
            client_enable_auto_white_balance_ = serviceClient<l3cam_ros::EnableAlliedCameraAutoWhiteBalance>("/L3Cam/l3cam_ros_node/enable_allied_auto_white_balance");
            client_balance_ratio_selector_ = serviceClient<l3cam_ros::ChangeAlliedCameraBalanceRatioSelector>("/L3Cam/l3cam_ros_node/change_allied_balance_ratio_selector");
            client_balance_ratio_ = serviceClient<l3cam_ros::ChangeAlliedCameraBalanceRatio>("/L3Cam/l3cam_ros_node/change_allied_balance_ratio");
            client_balance_white_auto_rate_ = serviceClient<l3cam_ros::ChangeAlliedCameraBalanceWhiteAutoRate>("/L3Cam/l3cam_ros_node/change_allied_balance_white_auto_rate");
            client_balance_white_auto_tolerance_ = serviceClient<l3cam_ros::ChangeAlliedCameraBalanceWhiteAutoTolerance>("/L3Cam/l3cam_ros_node/change_allied_balance_white_auto_tolerance");
            client_intensity_controller_region_ = serviceClient<l3cam_ros::ChangeAlliedCameraIntensityControllerRegion>("/L3Cam/l3cam_ros_node/change_allied_intensity_controller_region");
            client_intensity_controller_target_ = serviceClient<l3cam_ros::ChangeAlliedCameraIntensityControllerTarget>("/L3Cam/l3cam_ros_node/change_allied_intensity_controller_target");
            client_change_streaming_protocol_ = serviceClient<l3cam_ros::ChangeStreamingProtocol>("/L3Cam/l3cam_ros_node/change_streaming_protocol");
            client_get_rtsp_pipeline_ = serviceClient<l3cam_ros::GetRtspPipeline>("/L3Cam/l3cam_ros_node/get_rtsp_pipeline");
            client_get_exposure_time_ = serviceClient<l3cam_ros::GetAlliedCameraExposureTime>("/L3Cam/l3cam_ros_node/get_allied_exposure_time");
            client_get_gain_ = serviceClient<l3cam_ros::GetAlliedCameraGain>("/L3Cam/l3cam_ros_node/get_allied_gain");

            loadDefaultParams();

            // Create service server
            srv_sensor_disconnected_ = this->advertiseService("allied_narrow_configuration_disconnected", &AlliedNarrowConfiguration::sensorDisconnectedCallback, this);

            m_default_configured = false;
            m_shutdown_requested = false;
        }

        void setDynamicReconfigure()
        {
            // Dynamic reconfigure callback
            // server_ is a pointer so we only declare it if sensor is available and reconfigure should be available
            server_ = new dynamic_reconfigure::Server<l3cam_ros::AlliedNarrowConfig>;
            server_->setCallback(std::bind(&AlliedNarrowConfiguration::parametersCallback, this, std::placeholders::_1, std::placeholders::_2));
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
                if(!getParam(full_param_name, param_var))
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
            loadParam("allied_narrow_exposure_time", allied_narrow_exposure_time_, 4992.32);
            loadParam("allied_narrow_auto_exposure_time", allied_narrow_auto_exposure_time_, false);
            loadParam("allied_narrow_auto_exposure_time_range_min", allied_narrow_auto_exposure_time_range_min_, 87.596);
            loadParam("allied_narrow_auto_exposure_time_range_max", allied_narrow_auto_exposure_time_range_max_, 87.596);
            loadParam("allied_narrow_gain", allied_narrow_gain_, 0.);
            loadParam("allied_narrow_auto_gain", allied_narrow_auto_gain_, false);
            loadParam("allied_narrow_auto_gain_range_min", allied_narrow_auto_gain_range_min_, 0.);
            loadParam("allied_narrow_auto_gain_range_max", allied_narrow_auto_gain_range_max_, 48.);
            loadParam("allied_narrow_gamma", allied_narrow_gamma_, 1.);
            loadParam("allied_narrow_saturation", allied_narrow_saturation_, 1.);
            loadParam("allied_narrow_hue", allied_narrow_hue_, 0.);
            loadParam("allied_narrow_intensity_auto_precedence", allied_narrow_intensity_auto_precedence_, 1);
            loadParam("allied_narrow_auto_white_balance", allied_narrow_auto_white_balance_, false);
            loadParam("allied_narrow_balance_ratio_selector", allied_narrow_balance_ratio_selector_, 1);
            loadParam("allied_narrow_balance_ratio", allied_narrow_balance_ratio_, 2.35498);
            loadParam("allied_narrow_balance_white_auto_rate", allied_narrow_balance_white_auto_rate_, 100.);
            loadParam("allied_narrow_balance_white_auto_tolerance", allied_narrow_balance_white_auto_tolerance_, 5.);
            loadParam("allied_narrow_intensity_controller_region", allied_narrow_intensity_controller_region_, 1);
            loadParam("allied_narrow_intensity_controller_target", allied_narrow_intensity_controller_target_, 50.);
            loadParam("allied_narrow_streaming_protocol", allied_narrow_streaming_protocol_, 0);
        }

        void configureDefault(l3cam_ros::AlliedNarrowConfig &config)
        {
            // Configure default params to dynamix reconfigure if inside range
            if (allied_narrow_exposure_time_ >= 0 && allied_narrow_exposure_time_ <= 4095)
            {
                config.allied_narrow_exposure_time = allied_narrow_exposure_time_;
            }
            else
            {
                allied_narrow_exposure_time_ = config.allied_narrow_exposure_time;
            }

            if (allied_narrow_auto_exposure_time_ >= 63 && allied_narrow_auto_exposure_time_ <= 10000000)
            {
                config.allied_narrow_auto_exposure_time = allied_narrow_auto_exposure_time_;
            }
            else
            {
                allied_narrow_auto_exposure_time_ = config.allied_narrow_auto_exposure_time;
            }

            if (allied_narrow_auto_exposure_time_range_min_ >= 63.03 && allied_narrow_auto_exposure_time_range_min_ <= 8999990)
            {
                config.allied_narrow_auto_exposure_time_range_min = allied_narrow_auto_exposure_time_range_min_;
            }
            else
            {
                allied_narrow_auto_exposure_time_range_min_ = config.allied_narrow_auto_exposure_time_range_min;
            }

            if (allied_narrow_auto_exposure_time_range_max_ >= 87.596 && allied_narrow_auto_exposure_time_range_max_ <= 10000000)
            {
                config.allied_narrow_auto_exposure_time_range_max = allied_narrow_auto_exposure_time_range_max_;
            }
            else
            {
                allied_narrow_auto_exposure_time_range_max_ = config.allied_narrow_auto_exposure_time_range_max;
            }

            if (allied_narrow_gain_ >= 0 && allied_narrow_gain_ <= 48)
            {
                config.allied_narrow_gain = allied_narrow_gain_;
            }
            else
            {
                allied_narrow_gain_ = config.allied_narrow_gain;
            }

            config.allied_narrow_auto_gain = allied_narrow_auto_gain_;

            if (allied_narrow_auto_gain_range_min_ >= 0 && allied_narrow_auto_gain_range_min_ <= 48)
            {
                config.allied_narrow_auto_gain_range_min = allied_narrow_auto_gain_range_min_;
            }
            else
            {
                allied_narrow_auto_gain_range_min_ = config.allied_narrow_auto_gain_range_min;
            }

            if (allied_narrow_auto_gain_range_max_ >= 0 && allied_narrow_auto_gain_range_max_ <= 48)
            {
                config.allied_narrow_auto_gain_range_max = allied_narrow_auto_gain_range_max_;
            }
            else
            {
                allied_narrow_auto_gain_range_max_ = config.allied_narrow_auto_gain_range_max;
            }

            if (allied_narrow_gamma_ >= 0.4 && allied_narrow_gamma_ <= 2.4)
            {
                config.allied_narrow_gamma = allied_narrow_gamma_;
            }
            else
            {
                allied_narrow_gamma_ = config.allied_narrow_gamma;
            }

            if (allied_narrow_saturation_ >= 0 && allied_narrow_saturation_ <= 2)
            {
                config.allied_narrow_saturation = allied_narrow_saturation_;
            }
            else
            {
                allied_narrow_saturation_ = config.allied_narrow_saturation;
            }

            if (allied_narrow_hue_ >= -40 && allied_narrow_hue_ <= 40)
            {
                config.allied_narrow_hue = allied_narrow_hue_;
            }
            else
            {
                allied_narrow_hue_ = config.allied_narrow_hue;
            }

            if (allied_narrow_intensity_auto_precedence_ == 0 || allied_narrow_intensity_auto_precedence_ == 1)
            {
                config.allied_narrow_intensity_auto_precedence = allied_narrow_intensity_auto_precedence_;
            }
            else
            {
                allied_narrow_intensity_auto_precedence_ = config.allied_narrow_intensity_auto_precedence;
            }

            config.allied_narrow_auto_white_balance = allied_narrow_auto_white_balance_;

            if (allied_narrow_balance_ratio_selector_ == 0 || allied_narrow_balance_ratio_selector_ == 1)
            {
                config.allied_narrow_white_balance_ratio_selector = allied_narrow_balance_ratio_selector_;
            }
            else
            {
                allied_narrow_balance_ratio_selector_ = config.allied_narrow_white_balance_ratio_selector;
            }

            if (allied_narrow_balance_ratio_ >= 0 && allied_narrow_balance_ratio_ <= 8)
            {
                config.allied_narrow_balance_ratio = allied_narrow_balance_ratio_;
            }
            else
            {
                allied_narrow_balance_ratio_ = config.allied_narrow_balance_ratio;
            }

            if (allied_narrow_balance_white_auto_rate_ >= 1 && allied_narrow_balance_white_auto_rate_ <= 100)
            {
                config.allied_narrow_white_balance_auto_rate = allied_narrow_balance_white_auto_rate_;
            }
            else
            {
                allied_narrow_balance_white_auto_rate_ = config.allied_narrow_white_balance_auto_rate;
            }

            if (allied_narrow_balance_white_auto_tolerance_ >= 0 && allied_narrow_balance_white_auto_tolerance_ <= 50)
            {
                config.allied_narrow_white_balance_auto_tolerance = allied_narrow_balance_white_auto_tolerance_;
            }
            else
            {
                allied_narrow_balance_white_auto_tolerance_ = config.allied_narrow_white_balance_auto_tolerance;
            }

            if (allied_narrow_intensity_controller_region_ == 0 || allied_narrow_intensity_controller_region_ == 4)
            {
                config.allied_narrow_intensity_controller_region = allied_narrow_intensity_controller_region_;
            }
            else
            {
                allied_narrow_intensity_controller_region_ = config.allied_narrow_intensity_controller_region;
            }

            if (allied_narrow_intensity_controller_target_ >= 10 && allied_narrow_intensity_controller_target_ <= 90)
            {
                config.allied_narrow_intensity_controller_target = allied_narrow_intensity_controller_target_;
            }
            else
            {
                allied_narrow_intensity_controller_target_ = config.allied_narrow_intensity_controller_target;
            }

            if (allied_narrow_streaming_protocol_ == 0 || allied_narrow_streaming_protocol_ == 1)
            {
                config.allied_narrow_streaming_protocol = allied_narrow_streaming_protocol_;
            }
            else
            {
                allied_narrow_streaming_protocol_ = config.allied_narrow_streaming_protocol;
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
                srv_get_rtsp_pipeline_.request.sensor_type = (int)sensorTypes::sensor_allied_narrow;
                if (client_get_rtsp_pipeline_.call(srv_get_rtsp_pipeline_))
                {
                    error = srv_get_rtsp_pipeline_.response.error;
                    if (!error)
                    {
                        allied_narrow_rtsp_pipeline_ = srv_get_rtsp_pipeline_.response.pipeline;
                        config.allied_narrow_rtsp_pipeline = srv_get_rtsp_pipeline_.response.pipeline;
                    }
                    else
                    {
                        ROS_ERROR_STREAM(this->getNamespace() << " error " << error << " while getting RTSP pipeline: " << getErrorDescription(error));
                        config.allied_narrow_rtsp_pipeline = "";
                        allied_narrow_rtsp_pipeline_ = "";
                    }
                }
                else
                {
                    ROS_ERROR_STREAM(this->getNamespace() << " error: Failed to call service get_rtsp_pipeline");
                    config.allied_narrow_rtsp_pipeline = "";
                    allied_narrow_rtsp_pipeline_ = "";
                }
            }

            m_default_configured = true;
        }

        void parametersCallback(AlliedNarrowConfig &config, uint32_t level)
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
                case 0: // allied_narrow_exposure_time
                    error = callExposureTime(config);
                    break;
                case 1: // allied_narrow_auto_exposure_time
                    error = callAutoExposureTime(config);
                    break;
                case 2: // allied_narrow_auto_exposure_time_range_min
                    error = callAutoExposureTimeRange(config);
                    break;
                case 3: // allied_narrow_auto_exposure_time_range_max
                    error = callAutoExposureTimeRange(config);
                    break;
                case 4: // allied_narrow_gain
                    error = callGain(config);
                    break;
                case 5: // allied_narrow_auto_gain
                    error = callAutoGain(config);
                    break;
                case 6: // allied_narrow_auto_gain_range_min
                    error = callAutoGainRange(config);
                    break;
                case 7: // allied_narrow_auto_gain_range_max
                    error = callAutoGainRange(config);
                    break;
                case 8: // allied_narrow_gamma
                    error = callGamma(config);
                    break;
                case 9: // allied_narrow_saturation
                    error = callSaturation(config);
                    break;
                case 10: // allied_narrow_hue
                    error = callHue(config);
                    break;
                case 11: // allied_narrow_intensity_auto_precedence
                    error = callIntensityAutoPrecedence(config);
                    break;
                case 12: // allied_narrow_auto_white_balance
                    error = callAutoWhiteBalance(config);
                    break;
                case 13: // allied_narrow_white_balance_ratio_selector
                    error = callBalanceRatioSelector(config);
                    break;
                case 14: // allied_narrow_balance_ratio
                    error = callBalanceRatio(config);
                    break;
                case 15: // allied_narrow_white_balance_auto_rate
                    error = callBalanceWhiteAutoRate(config);
                    break;
                case 16: // allied_narrow_white_balance_auto_tolerance
                    error = callBalanceWhiteAutoTolerance(config);
                    break;
                case 17: // allied_narrow_intensity_controller_region
                    error = callIntensityControllerRegion(config);
                    break;
                case 18: // allied_narrow_intensity_controller_target
                    error = callIntensityControllerTarget(config);
                    break;
                case 19: // allied_narrow_streaming_protocol
                    error = callStreamingProtocol(config);
                    break;
                case 20: // allied_narrow_rtsp_pipeline
                    error = callRtspPipeline(config);
                    break;
                }
            }

            if (error)
            {
                ROS_ERROR_STREAM(this->getNamespace() << " error " << error << " while changing parameter: " << getErrorDescription(error));
            }
        }

        // Service calls
        int callExposureTime(l3cam_ros::AlliedNarrowConfig &config)
        {
            int error = L3CAM_OK;

            if (!config.allied_narrow_auto_exposure_time)
            {
                srv_exposure_time_.request.exposure_time = config.allied_narrow_exposure_time;
                srv_exposure_time_.request.allied_type = alliedCamerasIds::narrow_camera;
                if (client_exposure_time_.call(srv_exposure_time_))
                {
                    error = srv_exposure_time_.response.error;
                    if (!error)
                    {
                        // Parameter changed successfully, save value
                        allied_narrow_exposure_time_ = config.allied_narrow_exposure_time;
                    }
                    else
                    {
                        // Parameter could not be changed, reset parameter to value before change
                        config.allied_narrow_exposure_time = allied_narrow_exposure_time_;
                    }
                }
                else
                {
                    // Service could not be called, reset parameter to value before change
                    config.allied_narrow_exposure_time = allied_narrow_exposure_time_;
                    return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
                }
            }
            else
            {
                ROS_WARN("Allied Narrow camera auto exposure time must be disabled to change exposure time");
                config.allied_narrow_exposure_time = allied_narrow_exposure_time_;
            }

            return error;
        }

        int callAutoExposureTime(l3cam_ros::AlliedNarrowConfig &config)
        {
            int error = L3CAM_OK;

            srv_enable_auto_exposure_time_.request.enabled = config.allied_narrow_auto_exposure_time;
            srv_enable_auto_exposure_time_.request.allied_type = alliedCamerasIds::narrow_camera;
            if (client_enable_auto_exposure_time_.call(srv_enable_auto_exposure_time_))
            {
                error = srv_enable_auto_exposure_time_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    allied_narrow_auto_exposure_time_ = config.allied_narrow_auto_exposure_time;

                    // If auto exposure time deactivated we have to get the actual exposure time to know its value
                    srv_get_exposure_time_.request.allied_type = alliedCamerasIds::narrow_camera;
                    if (client_get_exposure_time_.call(srv_get_exposure_time_))
                    {
                        error = srv_get_exposure_time_.response.error;
                        if (!error)
                        {
                            // Got parameter successfully
                            allied_narrow_exposure_time_ = srv_get_exposure_time_.response.exposure_time;
                            config.allied_narrow_exposure_time = srv_get_exposure_time_.response.exposure_time;
                        }
                        else
                        {
                            ROS_ERROR_STREAM(this->getNamespace() << " error " << error << " while getting parameter in " << __func__ << ": " << getErrorDescription(error));
                        }
                    }
                    else
                    {
                        return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
                    }
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.allied_narrow_auto_exposure_time = allied_narrow_auto_exposure_time_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.allied_narrow_auto_exposure_time = allied_narrow_auto_exposure_time_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callAutoExposureTimeRange(l3cam_ros::AlliedNarrowConfig &config)
        {
            int error = L3CAM_OK;

            if (config.allied_narrow_auto_exposure_time)
            {
                srv_auto_exposure_time_range_.request.auto_exposure_time_range_min = config.allied_narrow_auto_exposure_time_range_min;
                srv_auto_exposure_time_range_.request.allied_type = alliedCamerasIds::narrow_camera;
                srv_auto_exposure_time_range_.request.auto_exposure_time_range_max = config.allied_narrow_auto_exposure_time_range_max;
                if (client_auto_exposure_time_range_.call(srv_auto_exposure_time_range_))
                {
                    error = srv_auto_exposure_time_range_.response.error;
                    if (!error)
                    {
                        // Parameter changed successfully, save value
                        allied_narrow_auto_exposure_time_range_min_ = config.allied_narrow_auto_exposure_time_range_min;
                        allied_narrow_auto_exposure_time_range_max_ = config.allied_narrow_auto_exposure_time_range_max;
                    }
                    else
                    {
                        // Parameter could not be changed, reset parameter to value before change
                        config.allied_narrow_auto_exposure_time_range_min = allied_narrow_auto_exposure_time_range_min_;
                        config.allied_narrow_auto_exposure_time_range_max = allied_narrow_auto_exposure_time_range_max_;
                    }
                }
                else
                {
                    // Service could not be called, reset parameter to value before change
                    config.allied_narrow_auto_exposure_time_range_min = allied_narrow_auto_exposure_time_range_min_;
                    config.allied_narrow_auto_exposure_time_range_max = allied_narrow_auto_exposure_time_range_max_;
                    return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
                }
            }
            else
            {
                ROS_WARN("Allied Narrow camera auto exposure time must be enabled to change auto exposure time range");
                config.allied_narrow_auto_exposure_time_range_min = allied_narrow_auto_exposure_time_range_min_;
                config.allied_narrow_auto_exposure_time_range_max = allied_narrow_auto_exposure_time_range_max_;
            }

            return error;
        }

        int callGain(l3cam_ros::AlliedNarrowConfig &config)
        {
            int error = L3CAM_OK;

            if (!config.allied_narrow_auto_gain)
            {
                srv_gain_.request.gain = config.allied_narrow_gain;
                srv_gain_.request.allied_type = alliedCamerasIds::narrow_camera;
                if (client_gain_.call(srv_gain_))
                {
                    error = srv_gain_.response.error;
                    if (!error)
                    {
                        // Parameter changed successfully, save value
                        allied_narrow_gain_ = config.allied_narrow_gain;
                    }
                    else
                    {
                        // Parameter could not be changed, reset parameter to value before change
                        config.allied_narrow_gain = allied_narrow_gain_;
                    }
                }
                else
                {
                    // Service could not be called, reset parameter to value before change
                    config.allied_narrow_gain = allied_narrow_gain_;
                    return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
                }
            }
            else
            {
                ROS_WARN("Allied Narrow camera auto gain must be disabled to change gain");
                config.allied_narrow_gain = allied_narrow_gain_;
            }

            return error;
        }

        int callAutoGain(l3cam_ros::AlliedNarrowConfig &config)
        {
            int error = L3CAM_OK;

            srv_enable_auto_gain_.request.enabled = config.allied_narrow_auto_gain;
            srv_enable_auto_gain_.request.allied_type = alliedCamerasIds::narrow_camera;
            if (client_enable_auto_gain_.call(srv_enable_auto_gain_))
            {
                error = srv_enable_auto_gain_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    allied_narrow_auto_gain_ = config.allied_narrow_auto_gain;

                    // If auto gain deactivated we have to get the actual gain to know its value
                    srv_get_gain_.request.allied_type = alliedCamerasIds::narrow_camera;
                    if (client_get_gain_.call(srv_get_gain_))
                    {
                        error = srv_get_gain_.response.error;
                        if (!error)
                        {
                            // Got parameter successfully
                            allied_narrow_gain_ = srv_get_gain_.response.gain;
                            config.allied_narrow_gain = srv_get_gain_.response.gain;
                        }
                        else
                        {
                            ROS_ERROR_STREAM(this->getNamespace() << " error " << error << " while getting parameter in " << __func__ << ": " << getErrorDescription(error));
                        }
                    }
                    else
                    {
                        return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
                    }
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.allied_narrow_auto_gain = allied_narrow_auto_gain_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.allied_narrow_auto_gain = allied_narrow_auto_gain_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callAutoGainRange(l3cam_ros::AlliedNarrowConfig &config)
        {
            int error = L3CAM_OK;

            if (config.allied_narrow_auto_gain)
            {
                srv_auto_gain_range_.request.auto_gain_range_min = config.allied_narrow_auto_gain_range_min;
                srv_auto_gain_range_.request.allied_type = alliedCamerasIds::narrow_camera;
                srv_auto_gain_range_.request.auto_gain_range_max = config.allied_narrow_auto_gain_range_max;
                if (client_auto_gain_range_.call(srv_auto_gain_range_))
                {
                    error = srv_auto_gain_range_.response.error;
                    if (!error)
                    {
                        // Parameter changed successfully, save value
                        allied_narrow_auto_gain_range_min_ = config.allied_narrow_auto_gain_range_min;
                        allied_narrow_auto_gain_range_max_ = config.allied_narrow_auto_gain_range_max;
                    }
                    else
                    {
                        // Parameter could not be changed, reset parameter to value before change
                        config.allied_narrow_auto_gain_range_min = allied_narrow_auto_gain_range_min_;
                        config.allied_narrow_auto_gain_range_max = allied_narrow_auto_gain_range_max_;
                    }
                }
                else
                {
                    // Service could not be called, reset parameter to value before change
                    config.allied_narrow_auto_gain_range_min = allied_narrow_auto_gain_range_min_;
                    config.allied_narrow_auto_gain_range_max = allied_narrow_auto_gain_range_max_;
                    return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
                }
            }
            else
            {
                ROS_WARN("Allied Narrow camera auto gain must be enabled to change auto gain range");
                config.allied_narrow_auto_gain_range_min = allied_narrow_auto_gain_range_min_;
                config.allied_narrow_auto_gain_range_max = allied_narrow_auto_gain_range_max_;
            }

            return error;
        }

        int callGamma(l3cam_ros::AlliedNarrowConfig &config)
        {
            int error = L3CAM_OK;

            srv_gamma_.request.gamma = config.allied_narrow_gamma;
            srv_gamma_.request.allied_type = alliedCamerasIds::narrow_camera;
            if (client_gamma_.call(srv_gamma_))
            {
                error = srv_gamma_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    allied_narrow_gamma_ = config.allied_narrow_gamma;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.allied_narrow_gamma = allied_narrow_gamma_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.allied_narrow_gamma = allied_narrow_gamma_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callSaturation(l3cam_ros::AlliedNarrowConfig &config)
        {
            int error = L3CAM_OK;

            srv_saturation_.request.saturation = config.allied_narrow_saturation;
            srv_saturation_.request.allied_type = alliedCamerasIds::narrow_camera;
            if (client_saturation_.call(srv_saturation_))
            {
                error = srv_saturation_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    allied_narrow_saturation_ = config.allied_narrow_saturation;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.allied_narrow_saturation = allied_narrow_saturation_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.allied_narrow_saturation = allied_narrow_saturation_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callHue(l3cam_ros::AlliedNarrowConfig &config)
        {
            int error = L3CAM_OK;

            srv_hue_.request.hue = config.allied_narrow_hue;
            srv_hue_.request.allied_type = alliedCamerasIds::narrow_camera;
            if (client_hue_.call(srv_hue_))
            {
                error = srv_hue_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    allied_narrow_hue_ = config.allied_narrow_hue;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.allied_narrow_hue = allied_narrow_hue_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.allied_narrow_hue = allied_narrow_hue_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callIntensityAutoPrecedence(l3cam_ros::AlliedNarrowConfig &config)
        {
            int error = L3CAM_OK;

            srv_intensity_auto_precedence_.request.intensity_auto_precedence = config.allied_narrow_intensity_auto_precedence;
            srv_intensity_auto_precedence_.request.allied_type = alliedCamerasIds::narrow_camera;
            if (client_intensity_auto_precedence_.call(srv_intensity_auto_precedence_))
            {
                error = srv_intensity_auto_precedence_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    allied_narrow_intensity_auto_precedence_ = config.allied_narrow_intensity_auto_precedence;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.allied_narrow_intensity_auto_precedence = allied_narrow_intensity_auto_precedence_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.allied_narrow_intensity_auto_precedence = allied_narrow_intensity_auto_precedence_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callAutoWhiteBalance(l3cam_ros::AlliedNarrowConfig &config)
        {
            int error = L3CAM_OK;

            srv_enable_auto_white_balance_.request.enabled = config.allied_narrow_intensity_auto_precedence;
            srv_enable_auto_white_balance_.request.allied_type = alliedCamerasIds::narrow_camera;
            if (client_enable_auto_white_balance_.call(srv_enable_auto_white_balance_))
            {
                error = srv_enable_auto_white_balance_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    allied_narrow_intensity_auto_precedence_ = config.allied_narrow_intensity_auto_precedence;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.allied_narrow_intensity_auto_precedence = allied_narrow_intensity_auto_precedence_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.allied_narrow_intensity_auto_precedence = allied_narrow_intensity_auto_precedence_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callBalanceRatioSelector(l3cam_ros::AlliedNarrowConfig &config)
        {
            int error = L3CAM_OK;

            srv_balance_ratio_selector_.request.white_balance_ratio_selector = config.allied_narrow_white_balance_ratio_selector;
            srv_balance_ratio_selector_.request.allied_type = alliedCamerasIds::narrow_camera;
            if (client_balance_ratio_selector_.call(srv_balance_ratio_selector_))
            {
                error = srv_balance_ratio_selector_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    allied_narrow_balance_ratio_selector_ = config.allied_narrow_white_balance_ratio_selector;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.allied_narrow_white_balance_ratio_selector = allied_narrow_balance_ratio_selector_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.allied_narrow_white_balance_ratio_selector = allied_narrow_balance_ratio_selector_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callBalanceRatio(l3cam_ros::AlliedNarrowConfig &config)
        {
            int error = L3CAM_OK;

            srv_balance_ratio_.request.balance_ratio = config.allied_narrow_balance_ratio;
            srv_balance_ratio_.request.allied_type = alliedCamerasIds::narrow_camera;
            if (client_balance_ratio_.call(srv_balance_ratio_))
            {
                error = srv_balance_ratio_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    allied_narrow_balance_ratio_ = config.allied_narrow_balance_ratio;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.allied_narrow_balance_ratio = allied_narrow_balance_ratio_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.allied_narrow_balance_ratio = allied_narrow_balance_ratio_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callBalanceWhiteAutoRate(l3cam_ros::AlliedNarrowConfig &config)
        {
            int error = L3CAM_OK;

            srv_balance_white_auto_rate_.request.white_balance_auto_rate = config.allied_narrow_white_balance_auto_rate;
            srv_balance_white_auto_rate_.request.allied_type = alliedCamerasIds::narrow_camera;
            if (client_balance_white_auto_rate_.call(srv_balance_white_auto_rate_))
            {
                error = srv_balance_white_auto_rate_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    allied_narrow_balance_white_auto_rate_ = config.allied_narrow_white_balance_auto_rate;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.allied_narrow_white_balance_auto_rate = allied_narrow_balance_white_auto_rate_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.allied_narrow_white_balance_auto_rate = allied_narrow_balance_white_auto_rate_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callBalanceWhiteAutoTolerance(l3cam_ros::AlliedNarrowConfig &config)
        {
            int error = L3CAM_OK;

            srv_balance_white_auto_tolerance_.request.white_balance_auto_tolerance = config.allied_narrow_white_balance_auto_tolerance;
            srv_balance_white_auto_tolerance_.request.allied_type = alliedCamerasIds::narrow_camera;
            if (client_balance_white_auto_tolerance_.call(srv_balance_white_auto_tolerance_))
            {
                error = srv_balance_white_auto_tolerance_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    allied_narrow_balance_white_auto_tolerance_ = config.allied_narrow_white_balance_auto_tolerance;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.allied_narrow_white_balance_auto_tolerance = allied_narrow_balance_white_auto_tolerance_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.allied_narrow_white_balance_auto_tolerance = allied_narrow_balance_white_auto_tolerance_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callIntensityControllerRegion(l3cam_ros::AlliedNarrowConfig &config)
        {
            int error = L3CAM_OK;

            srv_intensity_controller_region_.request.intensity_controller_region = config.allied_narrow_intensity_controller_region;
            srv_intensity_controller_region_.request.allied_type = alliedCamerasIds::narrow_camera;
            if (client_intensity_controller_region_.call(srv_intensity_controller_region_))
            {
                error = srv_intensity_controller_region_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    allied_narrow_intensity_controller_region_ = config.allied_narrow_intensity_controller_region;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.allied_narrow_intensity_controller_region = allied_narrow_intensity_controller_region_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.allied_narrow_intensity_controller_region = allied_narrow_intensity_controller_region_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callIntensityControllerTarget(l3cam_ros::AlliedNarrowConfig &config)
        {
            int error = L3CAM_OK;

            srv_intensity_controller_target_.request.intensity_controller_target = config.allied_narrow_intensity_controller_target;
            srv_intensity_controller_target_.request.allied_type = alliedCamerasIds::narrow_camera;
            if (client_intensity_controller_target_.call(srv_intensity_controller_target_))
            {
                error = srv_intensity_controller_target_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    allied_narrow_intensity_controller_target_ = config.allied_narrow_intensity_controller_target;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.allied_narrow_intensity_controller_target = allied_narrow_intensity_controller_target_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.allied_narrow_intensity_controller_target = allied_narrow_intensity_controller_target_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callStreamingProtocol(l3cam_ros::AlliedNarrowConfig &config)
        {
            int error = L3CAM_OK;

            srv_change_streaming_protocol_.request.protocol = config.allied_narrow_streaming_protocol;
            srv_change_streaming_protocol_.request.sensor_type = (int)sensorTypes::sensor_allied_narrow;
            if (client_change_streaming_protocol_.call(srv_change_streaming_protocol_))
            {
                error = srv_change_streaming_protocol_.response.error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    allied_narrow_streaming_protocol_ = config.allied_narrow_streaming_protocol;
                }
                else
                {
                    // Parameter could not be changed, reset parameter to value before change
                    config.allied_narrow_streaming_protocol = allied_narrow_streaming_protocol_;
                }
            }
            else
            {
                // Service could not be called, reset parameter to value before change
                config.allied_narrow_streaming_protocol = allied_narrow_streaming_protocol_;
                return L3CAM_ROS_FAILED_TO_CALL_SERVICE;
            }

            return error;
        }

        int callRtspPipeline(l3cam_ros::AlliedNarrowConfig &config)
        {
            // Read-only
            ROS_WARN("The RTSP Pipeline parameter is read-only, only changeable at launch");
            config.allied_narrow_rtsp_pipeline = allied_narrow_rtsp_pipeline_;

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

        dynamic_reconfigure::Server<l3cam_ros::AlliedNarrowConfig> *server_;

        ros::ServiceClient client_exposure_time_;
        l3cam_ros::ChangeAlliedCameraExposureTime srv_exposure_time_;
        ros::ServiceClient client_enable_auto_exposure_time_;
        l3cam_ros::EnableAlliedCameraAutoExposureTime srv_enable_auto_exposure_time_;
        ros::ServiceClient client_auto_exposure_time_range_;
        l3cam_ros::ChangeAlliedCameraAutoExposureTimeRange srv_auto_exposure_time_range_;
        ros::ServiceClient client_gain_;
        l3cam_ros::ChangeAlliedCameraGain srv_gain_;
        ros::ServiceClient client_enable_auto_gain_;
        l3cam_ros::EnableAlliedCameraAutoGain srv_enable_auto_gain_;
        ros::ServiceClient client_auto_gain_range_;
        l3cam_ros::ChangeAlliedCameraAutoGainRange srv_auto_gain_range_;
        ros::ServiceClient client_gamma_;
        l3cam_ros::ChangeAlliedCameraGamma srv_gamma_;
        ros::ServiceClient client_saturation_;
        l3cam_ros::ChangeAlliedCameraSaturation srv_saturation_;
        ros::ServiceClient client_hue_;
        l3cam_ros::ChangeAlliedCameraHue srv_hue_;
        ros::ServiceClient client_intensity_auto_precedence_;
        l3cam_ros::ChangeAlliedCameraIntensityAutoPrecedence srv_intensity_auto_precedence_;
        ros::ServiceClient client_enable_auto_white_balance_;
        l3cam_ros::EnableAlliedCameraAutoWhiteBalance srv_enable_auto_white_balance_;
        ros::ServiceClient client_balance_ratio_selector_;
        l3cam_ros::ChangeAlliedCameraBalanceRatioSelector srv_balance_ratio_selector_;
        ros::ServiceClient client_balance_ratio_;
        l3cam_ros::ChangeAlliedCameraBalanceRatio srv_balance_ratio_;
        ros::ServiceClient client_balance_white_auto_rate_;
        l3cam_ros::ChangeAlliedCameraBalanceWhiteAutoRate srv_balance_white_auto_rate_;
        ros::ServiceClient client_balance_white_auto_tolerance_;
        l3cam_ros::ChangeAlliedCameraBalanceWhiteAutoTolerance srv_balance_white_auto_tolerance_;
        ros::ServiceClient client_intensity_controller_region_;
        l3cam_ros::ChangeAlliedCameraIntensityControllerRegion srv_intensity_controller_region_;
        ros::ServiceClient client_intensity_controller_target_;
        l3cam_ros::ChangeAlliedCameraIntensityControllerTarget srv_intensity_controller_target_;
        ros::ServiceClient client_change_streaming_protocol_;
        l3cam_ros::ChangeStreamingProtocol srv_change_streaming_protocol_;
        ros::ServiceClient client_get_rtsp_pipeline_;
        l3cam_ros::GetRtspPipeline srv_get_rtsp_pipeline_;
        ros::ServiceClient client_get_exposure_time_;
        l3cam_ros::GetAlliedCameraExposureTime srv_get_exposure_time_;
        ros::ServiceClient client_get_gain_;
        l3cam_ros::GetAlliedCameraGain srv_get_gain_;

        ros::ServiceServer srv_sensor_disconnected_;

        double allied_narrow_exposure_time_;
        bool allied_narrow_auto_exposure_time_;
        double allied_narrow_auto_exposure_time_range_min_;
        double allied_narrow_auto_exposure_time_range_max_;
        double allied_narrow_gain_;
        bool allied_narrow_auto_gain_;
        double allied_narrow_auto_gain_range_min_;
        double allied_narrow_auto_gain_range_max_;
        double allied_narrow_gamma_;
        double allied_narrow_saturation_;
        double allied_narrow_hue_;
        int allied_narrow_intensity_auto_precedence_;
        bool allied_narrow_auto_white_balance_;
        int allied_narrow_balance_ratio_selector_;
        double allied_narrow_balance_ratio_;
        double allied_narrow_balance_white_auto_rate_;
        double allied_narrow_balance_white_auto_tolerance_;
        int allied_narrow_intensity_controller_region_;
        double allied_narrow_intensity_controller_target_;
        int allied_narrow_streaming_protocol_;
        std::string allied_narrow_rtsp_pipeline_;

        bool m_default_configured;
        bool m_shutdown_requested;

    }; // class AlliedNarrowConfiguration

} // namespace l3cam_ros

int main(int argc, char **argv)
{
    ros::init(argc, argv, "allied_narrow_configuration");

    l3cam_ros::AlliedNarrowConfiguration *node = new l3cam_ros::AlliedNarrowConfiguration();

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
                if (node->srv_get_sensors_.response.sensors[i].sensor_type == sensor_allied_narrow && node->srv_get_sensors_.response.sensors[i].sensor_available)
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
        ROS_INFO("Allied Narrow camera configuration is available");
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