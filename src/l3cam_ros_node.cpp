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

#include "l3cam_ros_node.hpp"

#include <signal.h>

#include <libL3Cam.h>
#include <libL3Cam_allied.h>
#include <libL3Cam_econ.h>
#include <libL3Cam_polarimetric.h>
#include <libL3Cam_thermal.h>
#include <beamErrors.h>

#include "l3cam_ros/Sensor.h"

l3cam_ros::L3Cam *node;

namespace l3cam_ros
{
    L3Cam::L3Cam() : ros::NodeHandle("~")
    {
        // Register callback for sensor disconnection errors
        registerErrorCallback(errorNotification);

        // L3Cam node status
        m_status = LibL3CamStatus::undefined;
        srv_libl3cam_status_ = advertiseService("libl3cam_status", &L3Cam::libL3camStatus, this);

        loadParam("timeout_secs", timeout_secs_, 60);

        m_num_devices = 0;
        m_shutdown_requested = false;
    }

    void L3Cam::spin()
    {
        while (ros::ok() && !m_shutdown_requested)
        {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }

        // Before exiting stop stream, device and terminate
        std::cout << "Terminating..." << std::endl;
        STOP_STREAM(node->m_devices[0]);
        STOP_DEVICE(node->m_devices[0]);
        TERMINATE(node->m_devices[0]);
        node->m_status = LibL3CamStatus::terminated;
        node = NULL;
        // Needs to wait for a second to Terminate
        usleep(1000000);
        std::cout << "Terminated." << std::endl;

        ros::shutdown();
    }

    int L3Cam::initializeDevice()
    {
        // Initialize L3Cam
        int error = L3CAM_OK;

        std::string local_address, device_address;
        param("/l3cam_ros_node/local_address", local_address);
        param("/l3cam_ros_node/device_address", device_address);

        if (local_address == "" || device_address == "")
        {
            error = INITIALIZE(NULL, NULL);
        }
        else
        {
            error = INITIALIZE(&local_address[0], &device_address[0]);
        }

        if (error)
            return error;

        int i = 0;
        while (m_num_devices == 0 && error == L3CAM_OK)
        {
            error = FIND_DEVICES(m_devices, &m_num_devices);

            if (!ros::ok())
            {
                return L3CAM_ROS_INTERRUPTED;
            }

            if (i >= timeout_secs_ * 2)
                return L3CAM_ROS_FIND_DEVICES_TIMEOUT_ERROR;
            usleep(500000);
            ++i;
        }

        if (error)
            return error;
        m_status = LibL3CamStatus::connected;
        ROS_INFO_STREAM("Device found " << std::string(m_devices[0].ip_address)
                                        << ", model " << (int)m_devices[0].model
                                        << ", serial number " << std::string(m_devices[0].serial_number)
                                        << ", app version " << std::string(m_devices[0].app_version));

        int status = 0;
        error = GET_DEVICE_STATUS(m_devices[0], &status);
        if (error)
            return error;
        ROS_INFO_STREAM("Device status " << status);

        int num_sensors = 0;
        error = GET_SENSORS_AVAILABLE(m_devices[0], m_av_sensors, &num_sensors);
        if (error)
            return error;

        for (int i = 0; i < num_sensors; ++i)
        {
            switch (m_av_sensors[i].sensor_type)
            {
            case sensor_lidar:
                m_lidar_sensor = &m_av_sensors[i];
                break;
            case sensor_econ_rgb:
                m_rgb_sensor = &m_av_sensors[i];
                break;
            case sensor_thermal:
                m_thermal_sensor = &m_av_sensors[i];
                break;
            case sensor_pol:
                m_polarimetric_sensor = &m_av_sensors[i];
                break;
            case sensor_allied_wide:
                m_allied_wide_sensor = &m_av_sensors[i];
                break;
            case sensor_allied_narrow:
                m_allied_narrow_sensor = &m_av_sensors[i];
                break;
            }
        }

        ROS_INFO_STREAM(num_sensors << ((num_sensors == 1) ? " sensor" : " sensors") << " available");

        initializeServices();

        return L3CAM_OK;
    }

    int L3Cam::startDeviceStream()
    {
        int error = L3CAM_OK;

        error = START_DEVICE(m_devices[0]);
        if (error)
        {
            return error;
        }

        m_status = LibL3CamStatus::started;
        ROS_INFO("Device started");

        loadDefaultParams();

        error = START_STREAM(m_devices[0]);
        if (error)
            return error;

        m_status = LibL3CamStatus::streaming;
        ROS_INFO("Device streaming ready\n");

        return L3CAM_OK;
    }

    void L3Cam::disconnectAll(int code)
    {
        networkDisconnected(code);
        if (m_lidar_sensor != NULL && m_lidar_sensor->sensor_available)
        {
            lidarDisconnected(code);
        }
        if (m_polarimetric_sensor != NULL && m_polarimetric_sensor->sensor_available)
        {
            polDisconnected(code);
        }
        if (m_rgb_sensor != NULL && m_rgb_sensor->sensor_available)
        {
            rgbDisconnected(code);
        }
        if (m_allied_wide_sensor != NULL && m_allied_wide_sensor->sensor_available)
        {
            alliedwideDisconnected(code);
        }
        if (m_allied_narrow_sensor != NULL && m_allied_narrow_sensor->sensor_available)
        {
            alliedNarrowDisconnect(code);
        }
        if (m_thermal_sensor != NULL && m_thermal_sensor->sensor_available)
        {
            thermalDisconnected(code);
        }
    }

    // Initialize Services
    void L3Cam::initializeServices()
    {
        srv_get_version_ = advertiseService("get_version", &L3Cam::getVersion, this);
        srv_initialize_ = advertiseService("initialize", &L3Cam::initialize, this);
        srv_terminate_ = advertiseService("terminate", &L3Cam::terminate, this);
        srv_find_devices_ = advertiseService("find_devices", &L3Cam::findDevices, this);
        srv_get_local_server_address_ = advertiseService("get_local_server_address", &L3Cam::getLocalServerAddress, this);
        srv_get_device_info_ = advertiseService("get_device_info", &L3Cam::getDeviceInfo, this);
        srv_get_device_status_ = advertiseService("get_device_status", &L3Cam::getDeviceStatus, this);
        srv_get_sensors_available_ = advertiseService("get_sensors_available", &L3Cam::getSensorsAvailable, this);
        srv_change_streaming_protocol_ = advertiseService("change_streaming_protocol", &L3Cam::changeStreamingProtocol, this);
        srv_get_rtsp_pipeline_ = advertiseService("get_rtsp_pipeline", &L3Cam::getRtspPipeline, this);
        srv_get_network_configuration_ = advertiseService("get_network_configuration", &L3Cam::getNetworkConfiguration, this);
        srv_change_network_configuration_ = advertiseService("change_network_configuration", &L3Cam::changeNetworkConfiguration, this);
        srv_power_off_device_ = advertiseService("power_off_device", &L3Cam::powerOffDevice, this);
        srv_start_device_ = advertiseService("start_device", &L3Cam::startDevice, this);
        srv_stop_device_ = advertiseService("stop_device", &L3Cam::stopDevice, this);
        srv_start_stream_ = advertiseService("start_stream", &L3Cam::startStream, this);
        srv_stop_stream_ = advertiseService("stop_stream", &L3Cam::stopStream, this);
        srv_get_device_temperatures_ = advertiseService("get_device_temperatures", &L3Cam::getDeviceTemperatures, this);

        client_network_disconnected_ = serviceClient<l3cam_ros::SensorDisconnected>("/L3Cam/network_configuration/network_disconnected");

        if (m_lidar_sensor != NULL && m_lidar_sensor->sensor_available) // if lidar is available
        {
            initializeLidarServices();
        }

        if (m_polarimetric_sensor != NULL && m_polarimetric_sensor->sensor_available) // if polarimetric is available
        {
            initializePolarimetricServices();
        }

        if (m_rgb_sensor != NULL && m_rgb_sensor->sensor_available) // if rgb is available
        {
            initializeRgbServices();
        }

        if (m_thermal_sensor != NULL && m_thermal_sensor->sensor_available) // if thermal is available
        {
            initializeThermalServices();
        }

        if (m_allied_wide_sensor != NULL && m_allied_wide_sensor->sensor_available) // if allied wide is available
        {
            initializeAlliedWideServices();
        }

        if (m_allied_narrow_sensor != NULL && m_allied_narrow_sensor->sensor_available) // if allied narrow is available
        {
            initializeAlliedNarrowServices();
        }

        ROS_INFO("Services ready");
    }

    void L3Cam::initializeLidarServices()
    {
        srv_change_pointcloud_color_ = advertiseService("change_pointcloud_color", &L3Cam::changePointcloudColor, this);
        srv_change_pointcloud_color_range_ = advertiseService("change_pointcloud_color_range", &L3Cam::changePointcloudColorRange, this);
        srv_change_distance_range_ = advertiseService("change_distance_range", &L3Cam::changeDistanceRange, this);
        srv_enable_auto_bias_ = advertiseService("enable_auto_bias", &L3Cam::enableAutoBias, this);
        srv_change_bias_value_ = advertiseService("change_bias_value", &L3Cam::changeBiasValue, this);

        client_lidar_stream_disconnected_ = serviceClient<l3cam_ros::SensorDisconnected>("/L3Cam/lidar_stream/lidar_stream_disconnected");
        client_lidar_configuration_disconnected_ = serviceClient<l3cam_ros::SensorDisconnected>("/L3Cam/lidar_configuration/lidar_configuration_disconnected");
    }

    void L3Cam::initializePolarimetricServices()
    {
        srv_set_polarimetric_default_settings_ = advertiseService("set_polarimetric_default_settings", &L3Cam::setPolarimetricCameraDefaultSettings, this);
        srv_change_polarimetric_brightness_ = advertiseService("change_polarimetric_brightness", &L3Cam::changePolarimetricCameraBrightness, this);
        srv_change_polarimetric_black_level_ = advertiseService("change_polarimetric_black_level", &L3Cam::changePolarimetricCameraBlackLevel, this);
        srv_enable_polarimetric_auto_gain_ = advertiseService("enable_polarimetric_auto_gain", &L3Cam::enablePolarimetricCameraAutoGain, this);
        srv_change_polarimetric_auto_gain_range_ = advertiseService("change_polarimetric_auto_gain_range", &L3Cam::changePolarimetricCameraAutoGainRange, this);
        srv_change_polarimetric_gain_ = advertiseService("change_polarimetric_gain", &L3Cam::changePolarimetricCameraGain, this);
        srv_enable_polarimetric_auto_exposure_time_ = advertiseService("enable_polarimetric_auto_exposure_time", &L3Cam::enablePolarimetricCameraAutoExposureTime, this);
        srv_change_polarimetric_auto_exposure_time_range_ = advertiseService("change_polarimetric_auto_exposure_time_range", &L3Cam::changePolarimetricCameraAutoExposureTimeRange, this);
        srv_change_polarimetric_exposure_time_ = advertiseService("change_polarimetric_exposure_time", &L3Cam::changePolarimetricCameraExposureTime, this);

        client_pol_wide_stream_disconnected_ = serviceClient<l3cam_ros::SensorDisconnected>("/L3Cam/polarimetric_wide_stream/polarimetric_wide_stream_disconnected");
        client_pol_configuration_disconnected_ = serviceClient<l3cam_ros::SensorDisconnected>("/L3Cam/polarimetric_configuration/polarimetric_configuration_disconnected");
    }

    void L3Cam::initializeRgbServices()
    {
        srv_set_rgb_default_settings_ = advertiseService("set_rgb_default_settings", &L3Cam::setRgbCameraDefaultSettings, this);
        srv_change_rgb_brightness_ = advertiseService("change_rgb_brightness", &L3Cam::changeRgbCameraBrightness, this);
        srv_change_rgb_contrast_ = advertiseService("change_rgb_contrast", &L3Cam::changeRgbCameraContrast, this);
        srv_change_rgb_saturation_ = advertiseService("change_rgb_saturation", &L3Cam::changeRgbCameraSaturation, this);
        srv_change_rgb_sharpness_ = advertiseService("change_rgb_sharpness", &L3Cam::changeRgbCameraSharpness, this);
        srv_change_rgb_gamma_ = advertiseService("change_rgb_gamma", &L3Cam::changeRgbCameraGamma, this);
        srv_change_rgb_gain_ = advertiseService("change_rgb_gain", &L3Cam::changeRgbCameraGain, this);
        srv_enable_rgb_auto_white_balance_ = advertiseService("enable_rgb_auto_white_balance", &L3Cam::enableRgbCameraAutoWhiteBalance, this);
        srv_change_rgb_white_balance_ = advertiseService("change_rgb_white_balance", &L3Cam::changeRgbCameraWhiteBalance, this);
        srv_enable_rgb_auto_exposure_time_ = advertiseService("enable_rgb_auto_exposure_time", &L3Cam::enableRgbCameraAutoExposureTime, this);
        srv_change_rgb_exposure_time_ = advertiseService("change_rgb_exposure_time", &L3Cam::changeRgbCameraExposureTime, this);

        client_rgb_narrow_stream_disconnected_ = serviceClient<l3cam_ros::SensorDisconnected>("/L3Cam/rgb_narrow_stream/rgb_narrow_stream_disconnected");
        client_rgb_configuration_disconnected_ = serviceClient<l3cam_ros::SensorDisconnected>("/L3Cam/rgb_configuration/rgb_configuration_disconnected");
    }

    void L3Cam::initializeThermalServices()
    {
        srv_change_thermal_colormap_ = advertiseService("change_thermal_colormap", &L3Cam::changeThermalCameraColormap, this);
        srv_enable_thermal_temperature_filter_ = advertiseService("enable_thermal_temperature_filter", &L3Cam::enableThermalCameraTemperatureFilter, this);
        srv_change_thermal_temperature_filter_ = advertiseService("change_thermal_temperature_filter", &L3Cam::changeThermalCameraTemperatureFilter, this);
        srv_change_thermal_processing_pipeline_ = advertiseService("change_thermal_processing_pipeline", &L3Cam::changeThermalCameraProcessingPipeline, this);
        srv_enable_thermal_temperature_data_udp_ = advertiseService("enable_thermal_temperature_data_udp", &L3Cam::enableThermalCameraTemperatureDataUdp, this);

        client_thermal_stream_disconnected_ = serviceClient<l3cam_ros::SensorDisconnected>("/L3Cam/thermal_stream/thermal_stream_disconnected");
        client_thermal_configuration_disconnected_ = serviceClient<l3cam_ros::SensorDisconnected>("/L3Cam/thermal_configuration/thermal_configuration_disconnected");
    }

    void L3Cam::initializeAlliedWideServices()
    {
        srv_change_allied_exposure_time_ = advertiseService("change_allied_exposure_time", &L3Cam::changeAlliedCameraExposureTime, this);
        srv_enable_allied_auto_exposure_time_ = advertiseService("enable_allied_auto_exposure_time", &L3Cam::enableAlliedCameraAutoExposureTime, this);
        srv_change_allied_auto_exposure_time_range_ = advertiseService("change_allied_auto_exposure_time_range", &L3Cam::changeAlliedCameraAutoExposureTimeRange, this);
        srv_change_allied_gain_ = advertiseService("change_allied_gain", &L3Cam::changeAlliedCameraGain, this);
        srv_enable_allied_auto_gain_ = advertiseService("enable_allied_auto_gain", &L3Cam::enableAlliedCameraAutoGain, this);
        srv_change_allied_auto_gain_range_ = advertiseService("change_allied_auto_gain_range", &L3Cam::changeAlliedCameraAutoGainRange, this);
        srv_change_allied_gamma_ = advertiseService("change_allied_gamma", &L3Cam::changeAlliedCameraGamma, this);
        srv_change_allied_saturation_ = advertiseService("change_allied_saturation", &L3Cam::changeAlliedCameraSaturation, this);
        srv_change_allied_hue_ = advertiseService("change_allied_hue", &L3Cam::changeAlliedCameraHue, this);
        srv_change_allied_intensity_auto_precedence_ = advertiseService("change_allied_intensity_auto_precedence", &L3Cam::changeAlliedCameraIntensityAutoPrecedence, this);
        srv_enable_allied_auto_white_balance_ = advertiseService("enable_allied_auto_white_balance", &L3Cam::enableAlliedCameraAutoWhiteBalance, this);
        srv_change_allied_balance_ratio_selector_ = advertiseService("change_allied_balance_ratio_selector", &L3Cam::changeAlliedCameraBalanceRatioSelector, this);
        srv_change_allied_balance_ratio_ = advertiseService("change_allied_balance_ratio", &L3Cam::changeAlliedCameraBalanceRatio, this);
        srv_change_allied_balance_white_auto_rate_ = advertiseService("change_allied_balance_white_auto_rate", &L3Cam::changeAlliedCameraBalanceWhiteAutoRate, this);
        srv_change_allied_balance_white_auto_tolerance_ = advertiseService("change_allied_balance_white_auto_tolerance", &L3Cam::changeAlliedCameraBalanceWhiteAutoTolerance, this);
        srv_change_allied_intensity_controller_region_ = advertiseService("change_allied_intensity_controller_region", &L3Cam::changeAlliedCameraIntensityControllerRegion, this);
        srv_change_allied_intensity_controller_target_ = advertiseService("change_allied_intensity_controller_target", &L3Cam::changeAlliedCameraIntensityControllerTarget, this);

        srv_get_allied_black_level_ = advertiseService("get_allied_black_level", &L3Cam::getAlliedCameraBlackLevel, this);
        srv_get_allied_exposure_time_ = advertiseService("get_allied_exposure_time", &L3Cam::getAlliedCameraExposureTime, this);
        srv_get_allied_auto_exposure_time_ = advertiseService("get_allied_auto_exposure_time", &L3Cam::getAlliedCameraAutoExposureTime, this);
        srv_get_allied_auto_exposure_time_range_ = advertiseService("get_allied_auto_exposure_time_range", &L3Cam::getAlliedCameraAutoExposureTimeRange, this);
        srv_get_allied_gain_ = advertiseService("get_allied_gain", &L3Cam::getAlliedCameraGain, this);
        srv_get_allied_auto_gain_ = advertiseService("get_allied_auto_gain", &L3Cam::getAlliedCameraAutoGain, this);
        srv_get_allied_auto_gain_range_ = advertiseService("get_allied_auto_gain_range", &L3Cam::getAlliedCameraAutoGainRange, this);
        srv_get_allied_gamma_ = advertiseService("get_allied_gamma", &L3Cam::getAlliedCameraGamma, this);
        srv_get_allied_saturation_ = advertiseService("get_allied_saturation", &L3Cam::getAlliedCameraSaturation, this);
        srv_get_allied_sharpness_ = advertiseService("get_allied_sharpness", &L3Cam::getAlliedCameraSharpness, this);
        srv_get_allied_hue_ = advertiseService("get_allied_hue", &L3Cam::getAlliedCameraHue, this);
        srv_get_allied_intensity_auto_precedence_ = advertiseService("get_allied_intensity_auto_precedence", &L3Cam::getAlliedCameraIntensityAutoPrecedence, this);
        srv_get_allied_auto_white_balance_ = advertiseService("get_allied_auto_white_balance", &L3Cam::getAlliedCameraAutoWhiteBalance, this);
        srv_get_allied_balance_ratio_selector_ = advertiseService("get_allied_balance_ratio_selector", &L3Cam::getAlliedCameraBalanceRatioSelector, this);
        srv_get_allied_balance_ratio_ = advertiseService("get_allied_balance_ratio", &L3Cam::getAlliedCameraBalanceRatio, this);
        srv_get_allied_balance_white_auto_rate_ = advertiseService("get_allied_balance_white_auto_rate", &L3Cam::getAlliedCameraBalanceWhiteAutoRate, this);
        srv_get_allied_balance_white_auto_tolerance_ = advertiseService("get_allied_balance_white_auto_tolerance", &L3Cam::getAlliedCameraBalanceWhiteAutoTolerance, this);
        srv_get_allied_auto_mode_region_ = advertiseService("get_allied_auto_mode_region", &L3Cam::getAlliedCameraAutoModeRegion, this);
        srv_get_allied_intensity_controller_region_ = advertiseService("get_allied_intensity_controller_region", &L3Cam::getAlliedCameraIntensityControllerRegion, this);
        srv_get_allied_intensity_controller_target_ = advertiseService("get_allied_intensity_controller_target", &L3Cam::getAlliedCameraIntensityControllerTarget, this);
        srv_get_allied_max_driver_buffers_count_ = advertiseService("get_allied_max_driver_buffers_count", &L3Cam::getAlliedCameraMaxDriverBuffersCount, this);

        client_pol_wide_stream_disconnected_ = serviceClient<l3cam_ros::SensorDisconnected>("/L3Cam/polarimetric_wide_stream/polarimetric_wide_stream_disconnected");
        client_wide_configuration_disconnected_ = serviceClient<l3cam_ros::SensorDisconnected>("/L3Cam/allied_wide_configuration/allied_wide_configuration_disconnected");
    }

    void L3Cam::initializeAlliedNarrowServices()
    {
        srv_change_allied_exposure_time_ = advertiseService("change_allied_exposure_time", &L3Cam::changeAlliedCameraExposureTime, this);
        srv_enable_allied_auto_exposure_time_ = advertiseService("enable_allied_auto_exposure_time", &L3Cam::enableAlliedCameraAutoExposureTime, this);
        srv_change_allied_auto_exposure_time_range_ = advertiseService("change_allied_auto_exposure_time_range", &L3Cam::changeAlliedCameraAutoExposureTimeRange, this);
        srv_change_allied_gain_ = advertiseService("change_allied_gain", &L3Cam::changeAlliedCameraGain, this);
        srv_enable_allied_auto_gain_ = advertiseService("enable_allied_auto_gain", &L3Cam::enableAlliedCameraAutoGain, this);
        srv_change_allied_auto_gain_range_ = advertiseService("change_allied_auto_gain_range", &L3Cam::changeAlliedCameraAutoGainRange, this);
        srv_change_allied_gamma_ = advertiseService("change_allied_gamma", &L3Cam::changeAlliedCameraGamma, this);
        srv_change_allied_saturation_ = advertiseService("change_allied_saturation", &L3Cam::changeAlliedCameraSaturation, this);
        srv_change_allied_hue_ = advertiseService("change_allied_hue", &L3Cam::changeAlliedCameraHue, this);
        srv_change_allied_intensity_auto_precedence_ = advertiseService("change_allied_intensity_auto_precedence", &L3Cam::changeAlliedCameraIntensityAutoPrecedence, this);
        srv_enable_allied_auto_white_balance_ = advertiseService("enable_allied_auto_white_balance", &L3Cam::enableAlliedCameraAutoWhiteBalance, this);
        srv_change_allied_balance_ratio_selector_ = advertiseService("change_allied_balance_ratio_selector", &L3Cam::changeAlliedCameraBalanceRatioSelector, this);
        srv_change_allied_balance_ratio_ = advertiseService("change_allied_balance_ratio", &L3Cam::changeAlliedCameraBalanceRatio, this);
        srv_change_allied_balance_white_auto_rate_ = advertiseService("change_allied_balance_white_auto_rate", &L3Cam::changeAlliedCameraBalanceWhiteAutoRate, this);
        srv_change_allied_balance_white_auto_tolerance_ = advertiseService("change_allied_balance_white_auto_tolerance", &L3Cam::changeAlliedCameraBalanceWhiteAutoTolerance, this);
        srv_change_allied_intensity_controller_region_ = advertiseService("change_allied_intensity_controller_region", &L3Cam::changeAlliedCameraIntensityControllerRegion, this);
        srv_change_allied_intensity_controller_target_ = advertiseService("change_allied_intensity_controller_target", &L3Cam::changeAlliedCameraIntensityControllerTarget, this);

        srv_get_allied_black_level_ = advertiseService("get_allied_black_level", &L3Cam::getAlliedCameraBlackLevel, this);
        srv_get_allied_exposure_time_ = advertiseService("get_allied_exposure_time", &L3Cam::getAlliedCameraExposureTime, this);
        srv_get_allied_auto_exposure_time_ = advertiseService("get_allied_auto_exposure_time", &L3Cam::getAlliedCameraAutoExposureTime, this);
        srv_get_allied_auto_exposure_time_range_ = advertiseService("get_allied_auto_exposure_time_range", &L3Cam::getAlliedCameraAutoExposureTimeRange, this);
        srv_get_allied_gain_ = advertiseService("get_allied_gain", &L3Cam::getAlliedCameraGain, this);
        srv_get_allied_auto_gain_ = advertiseService("get_allied_auto_gain", &L3Cam::getAlliedCameraAutoGain, this);
        srv_get_allied_auto_gain_range_ = advertiseService("get_allied_auto_gain_range", &L3Cam::getAlliedCameraAutoGainRange, this);
        srv_get_allied_gamma_ = advertiseService("get_allied_gamma", &L3Cam::getAlliedCameraGamma, this);
        srv_get_allied_saturation_ = advertiseService("get_allied_saturation", &L3Cam::getAlliedCameraSaturation, this);
        srv_get_allied_sharpness_ = advertiseService("get_allied_sharpness", &L3Cam::getAlliedCameraSharpness, this);
        srv_get_allied_hue_ = advertiseService("get_allied_hue", &L3Cam::getAlliedCameraHue, this);
        srv_get_allied_intensity_auto_precedence_ = advertiseService("get_allied_intensity_auto_precedence", &L3Cam::getAlliedCameraIntensityAutoPrecedence, this);
        srv_get_allied_auto_white_balance_ = advertiseService("get_allied_auto_white_balance", &L3Cam::getAlliedCameraAutoWhiteBalance, this);
        srv_get_allied_balance_ratio_selector_ = advertiseService("get_allied_balance_ratio_selector", &L3Cam::getAlliedCameraBalanceRatioSelector, this);
        srv_get_allied_balance_ratio_ = advertiseService("get_allied_balance_ratio", &L3Cam::getAlliedCameraBalanceRatio, this);
        srv_get_allied_balance_white_auto_rate_ = advertiseService("get_allied_balance_white_auto_rate", &L3Cam::getAlliedCameraBalanceWhiteAutoRate, this);
        srv_get_allied_balance_white_auto_tolerance_ = advertiseService("get_allied_balance_white_auto_tolerance", &L3Cam::getAlliedCameraBalanceWhiteAutoTolerance, this);
        srv_get_allied_auto_mode_region_ = advertiseService("get_allied_auto_mode_region", &L3Cam::getAlliedCameraAutoModeRegion, this);
        srv_get_allied_intensity_controller_region_ = advertiseService("get_allied_intensity_controller_region", &L3Cam::getAlliedCameraIntensityControllerRegion, this);
        srv_get_allied_intensity_controller_target_ = advertiseService("get_allied_intensity_controller_target", &L3Cam::getAlliedCameraIntensityControllerTarget, this);
        srv_get_allied_max_driver_buffers_count_ = advertiseService("get_allied_max_driver_buffers_count", &L3Cam::getAlliedCameraMaxDriverBuffersCount, this);

        client_rgb_narrow_stream_disconnected_ = serviceClient<l3cam_ros::SensorDisconnected>("/L3Cam/rgb_narrow_stream/rgb_narrow_stream_disconnected");
        client_narrow_configuration_disconnected_ = serviceClient<l3cam_ros::SensorDisconnected>("/L3Cam/allied_narrow_configuration/allied_narrow_configuration_disconnected");
    }

    inline void L3Cam::printDefaultError(int error, std::string param)
    {
        if (error != L3CAM_OK)
        {
            ROS_WARN_STREAM(this->getNamespace() << " error " << error << " while setting default parameter " << param << ": "
                                                 << getErrorDescription(error));
        }
    }

    // Load default params
    template <typename T>
    void L3Cam::loadParam(const std::string &param_name, T &param_var, const T &default_val)
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

    void L3Cam::loadDefaultParams()
    {
        loadNetworkDefaultParams();
        if (m_lidar_sensor != NULL) // if lidar should be available in the L3Cam
        {
            if (m_lidar_sensor->sensor_available) // if lidar is available
            {
                loadPointcloudDefaultParams();
            }
            else
            {
                ROS_ERROR_STREAM(this->getNamespace() << " error: LiDAR not available.");
            }
        }
        if (m_polarimetric_sensor != NULL) // if polarimetric should be available in the L3Cam
        {
            if (m_polarimetric_sensor->sensor_available) // if polarimetric is available
            {
                loadPolarimetricDefaultParams();
            }
            else
            {
                ROS_ERROR_STREAM(this->getNamespace() << " error: Polarimetric camera not available.");
            }
        }
        if (m_rgb_sensor != NULL) // if rgb should be available in the L3Cam
        {
            if (m_rgb_sensor->sensor_available) // if rgb is available
            {
                loadRgbDefaultParams();
            }
            else
            {
                ROS_ERROR_STREAM(this->getNamespace() << " error: RGB camera not available.");
            }
        }
        if (m_thermal_sensor != NULL) // if thermal should be available in the L3Cam
        {
            if (m_thermal_sensor->sensor_available) // if thermal is available
            {
                loadThermalDefaultParams();
            }
            else
            {
                ROS_ERROR_STREAM(this->getNamespace() << " error: Thermal camera not available.");
            }
        }
        if (m_allied_wide_sensor != NULL) // if allied wide should be available in the L3Cam
        {
            if (m_allied_wide_sensor->sensor_available) // if allied wide is available
            {
                loadAlliedWideDefaultParams();
            }
            else
            {
                ROS_ERROR_STREAM(this->getNamespace() << " error: Allied Wide camera not available.");
            }
        }
        if (m_allied_narrow_sensor != NULL) // if allied narrow should be available in the L3Cam
        {
            if (m_allied_narrow_sensor->sensor_available) // if allied narrow is available
            {
                loadAlliedNarrowDefaultParams();
            }
            else
            {
                ROS_ERROR_STREAM(this->getNamespace() << " error: Allied Narrow camera not available.");
            }
        }

        ROS_INFO("Default parameters loaded");
    }

    void L3Cam::loadNetworkDefaultParams()
    {
        char *ip_address = NULL;
        char *netmask = NULL;
        char *gateway = NULL;
        int error = GET_NETWORK_CONFIGURATION(m_devices[0], &ip_address, &netmask, &gateway);
        if (!error)
        {
            param("/ip_address", std::string(ip_address));
            param("/netmask", std::string(netmask));
            param("/gateway", std::string(gateway));
        }
    }

    void L3Cam::loadPointcloudDefaultParams()
    {
        int pointcloud_color;
        loadParam("pointcloud_color", pointcloud_color, 0);
        printDefaultError(CHANGE_POINT_CLOUD_COLOR(m_devices[0], pointcloud_color), "pointcloud_color");
        int pointcloud_color_range_minimum;
        loadParam("pointcloud_color_range_minimum", pointcloud_color_range_minimum, 0);
        int pointcloud_color_range_maximum;
        loadParam("pointcloud_color_range_maximum", pointcloud_color_range_maximum, 300000);
        printDefaultError(CHANGE_POINT_CLOUD_COLOR_RANGE(m_devices[0], pointcloud_color_range_minimum, pointcloud_color_range_maximum), "pointcloud_color_range");
        int distance_range_minimum;
        loadParam("distance_range_minimum", distance_range_minimum, 0);
        int distance_range_maximum;
        loadParam("distance_range_maximum", distance_range_maximum, 300000);
        printDefaultError(CHANGE_DISTANCE_RANGE(m_devices[0], distance_range_minimum, distance_range_maximum), "distance_range");
        bool auto_bias;
        loadParam("auto_bias", auto_bias, true);
        ENABLE_AUTO_BIAS(m_devices[0], auto_bias);
        if (!auto_bias)
        {
            int bias_value_right;
            loadParam("bias_value_right", bias_value_right, 1580);
            CHANGE_BIAS_VALUE(m_devices[0], 1, bias_value_right);
            int bias_value_left;
            loadParam("bias_value_left", bias_value_left, 1380);
            CHANGE_BIAS_VALUE(m_devices[0], 2, bias_value_left);
        }
        int lidar_streaming_protocol;
        loadParam("lidar_streaming_protocol", lidar_streaming_protocol, 0);
        if (lidar_streaming_protocol == 1)
        {
            m_lidar_sensor->protocol = protocol_gstreamer;
            printDefaultError(CHANGE_STREAMING_PROTOCOL(m_devices[0], m_lidar_sensor), "lidar_streaming_protocol");
        }
        std::string lidar_rtsp_pipeline;
        loadParam("lidar_rtsp_pipeline", lidar_rtsp_pipeline, std::string(""));
        if (lidar_rtsp_pipeline != "")
        {
            char *pipeline = &lidar_rtsp_pipeline[0];
            printDefaultError(CHANGE_RTSP_PIPELINE(m_devices[0], *m_lidar_sensor, pipeline), "lidar_rtsp_pipeline");
        }
    }

    void L3Cam::loadPolarimetricDefaultParams()
    {
        int polarimetric_brightness;
        loadParam("polarimetric_brightness", polarimetric_brightness, 127);
        printDefaultError(CHANGE_POLARIMETRIC_CAMERA_BRIGHTNESS(m_devices[0], polarimetric_brightness), "polarimetric_brightness");
        double polarimetric_black_level;
        loadParam("polarimetric_black_level", polarimetric_black_level, 6.0);
        printDefaultError(CHANGE_POLARIMETRIC_CAMERA_BLACK_LEVEL(m_devices[0], polarimetric_black_level), "polarimetric_black_level");
        bool polarimetric_auto_gain;
        loadParam("polarimetric_auto_gain", polarimetric_auto_gain, true);
        printDefaultError(ENABLE_POLARIMETRIC_CAMERA_AUTO_GAIN(m_devices[0], polarimetric_auto_gain), "polarimetric_auto_gain");
        if (polarimetric_auto_gain)
        { //! Values might not coincide when enabling polarimetric_auto_gain
            double polarimetric_auto_gain_range_minimum;
            loadParam("polarimetric_auto_gain_range_minimum", polarimetric_auto_gain_range_minimum, 0.0);
            double polarimetric_auto_gain_range_maximum;
            loadParam("polarimetric_auto_gain_range_maximum", polarimetric_auto_gain_range_maximum, 48.0);
            printDefaultError(CHANGE_POLARIMETRIC_CAMERA_AUTO_GAIN_RANGE(m_devices[0], polarimetric_auto_gain_range_minimum, polarimetric_auto_gain_range_maximum), "polarimetric_auto_gain_range");
        }
        else
        { //! Values might not coincide when disabling polarimetric_auto_gain
            double polarimetric_gain;
            loadParam("polarimetric_gain", polarimetric_gain, 24.0);
            printDefaultError(CHANGE_POLARIMETRIC_CAMERA_GAIN(m_devices[0], polarimetric_gain), "polarimetric_gain");
        }
        bool polarimetric_auto_exposure_time;
        loadParam("polarimetric_auto_exposure_time", polarimetric_auto_exposure_time, true);
        printDefaultError(ENABLE_POLARIMETRIC_CAMERA_AUTO_EXPOSURE_TIME(m_devices[0], polarimetric_auto_exposure_time), "polarimetric_auto_exposure_time");
        if (polarimetric_auto_exposure_time)
        { //! Values might not coincide when enabling polarimetric_auto_exposure_time
            double polarimetric_auto_exposure_time_range_minimum;
            loadParam("polarimetric_auto_exposure_time_range_minimum", polarimetric_auto_exposure_time_range_minimum, 33.456);
            double polarimetric_auto_exposure_time_range_maximum;
            loadParam("polarimetric_auto_exposure_time_range_maximum", polarimetric_auto_exposure_time_range_maximum, 66470.6);
            printDefaultError(CHANGE_POLARIMETRIC_CAMERA_AUTO_EXPOSURE_TIME_RANGE(m_devices[0], polarimetric_auto_exposure_time_range_minimum, polarimetric_auto_exposure_time_range_maximum), "polarimetric_auto_exposure_time_range");
        }
        else
        { //! Values might not coincide when disabling polarimetric_auto_exposure_time
            double polarimetric_exposure_time;
            loadParam("polarimetric_exposure_time", polarimetric_exposure_time, 500000.0);
            printDefaultError(CHANGE_POLARIMETRIC_CAMERA_EXPOSURE_TIME(m_devices[0], polarimetric_exposure_time), "polarimetric_exposure_time");
        }
        int polarimetric_streaming_protocol;
        loadParam("polarimetric_streaming_protocol", polarimetric_streaming_protocol, 0);
        if (polarimetric_streaming_protocol == 1)
        {
            m_polarimetric_sensor->protocol = protocol_gstreamer;
            printDefaultError(CHANGE_STREAMING_PROTOCOL(m_devices[0], m_polarimetric_sensor), "polarimetric_streaming_protocol");
        }
        std::string polarimetric_rtsp_pipeline;
        loadParam("polarimetric_rtsp_pipeline", polarimetric_rtsp_pipeline, std::string(""));
        if (polarimetric_rtsp_pipeline != "")
        {
            char *pipeline = &polarimetric_rtsp_pipeline[0];
            printDefaultError(CHANGE_RTSP_PIPELINE(m_devices[0], *m_polarimetric_sensor, pipeline), "polarimetric_rtsp_pipeline");
        }
    }

    void L3Cam::loadRgbDefaultParams()
    {
        int rgb_brightness;
        param("/l3cam_ros_nod/rgb_brightness", rgb_brightness, 0);
        printDefaultError(CHANGE_RGB_CAMERA_BRIGHTNESS(m_devices[0], rgb_brightness), "rgb_brightness");
        int rgb_contrast;
        param("/l3cam_ros_nod/rgb_contrast", rgb_contrast, 10);
        printDefaultError(CHANGE_RGB_CAMERA_CONTRAST(m_devices[0], rgb_contrast), "rgb_contrast");
        int rgb_saturation;
        param("/l3cam_ros_nod/rgb_saturation", rgb_saturation, 16);
        printDefaultError(CHANGE_RGB_CAMERA_SATURATION(m_devices[0], rgb_saturation), "rgb_saturation");
        int rgb_sharpness;
        loadParam("rgb_sharpness", rgb_sharpness, 16);
        printDefaultError(CHANGE_RGB_CAMERA_SHARPNESS(m_devices[0], rgb_sharpness), "rgb_sharpness");
        int rgb_gamma;
        loadParam("rgb_gamma", rgb_gamma, 220);
        printDefaultError(CHANGE_RGB_CAMERA_GAMMA(m_devices[0], rgb_gamma), "rgb_gamma");
        int rgb_gain;
        loadParam("rgb_gain", rgb_gain, 0);
        printDefaultError(CHANGE_RGB_CAMERA_GAIN(m_devices[0], rgb_gain), "rgb_gain");
        bool rgb_auto_white_balance;
        loadParam("rgb_auto_white_balance", rgb_auto_white_balance, true);
        printDefaultError(ENABLE_RGB_CAMERA_AUTO_WHITE_BALANCE(m_devices[0], rgb_auto_white_balance), "rgb_auto_white_balance");
        if (!rgb_auto_white_balance)
        { //! Values might not coincide when disabling rgb_auto_white_balance
            int rgb_white_balance;
            loadParam("rgb_white_balance", rgb_white_balance, 5000);
            printDefaultError(CHANGE_RGB_CAMERA_WHITE_BALANCE(m_devices[0], rgb_white_balance), "rgb_white_balance");
        }
        bool rgb_auto_exposure_time;
        loadParam("rgb_auto_exposure_time", rgb_auto_exposure_time, true);
        printDefaultError(ENABLE_RGB_CAMERA_AUTO_EXPOSURE_TIME(m_devices[0], rgb_auto_exposure_time), "rgb_auto_exposure_time");
        if (!rgb_auto_exposure_time)
        { //! Values might not coincide when disabling rgb_auto_exposure_time
            int rgb_exposure_time;
            loadParam("rgb_exposure_time", rgb_exposure_time, 156);
            printDefaultError(CHANGE_RGB_CAMERA_EXPOSURE_TIME(m_devices[0], rgb_exposure_time), "rgb_exposure_time");
        }
        int rgb_resolution;
        loadParam("rgb_resolution", rgb_resolution, 3);
        printDefaultError(CHANGE_RGB_CAMERA_RESOLUTION(m_devices[0], (econResolutions)rgb_resolution), "rgb_resolution");
        int rgb_framerate;
        loadParam("rgb_framerate", rgb_framerate, 10);
        printDefaultError(CHANGE_RGB_CAMERA_FRAMERATE(m_devices[0], rgb_framerate), "rgb_framerate");
        int rgb_streaming_protocol;
        loadParam("rgb_streaming_protocol", rgb_streaming_protocol, 0);
        if (rgb_streaming_protocol == 1)
        {
            m_rgb_sensor->protocol = protocol_gstreamer;
            printDefaultError(CHANGE_STREAMING_PROTOCOL(m_devices[0], m_rgb_sensor), "rgb_streaming_protocol");
        }
        std::string rgb_rtsp_pipeline;
        loadParam("rgb_rtsp_pipeline", rgb_rtsp_pipeline, std::string(""));
        if (rgb_rtsp_pipeline != "")
        {
            char *pipeline = &rgb_rtsp_pipeline[0];
            printDefaultError(CHANGE_RTSP_PIPELINE(m_devices[0], *m_rgb_sensor, pipeline), "rgb_rtsp_pipeline");
        }
    }

    void L3Cam::loadThermalDefaultParams()
    {
        int thermal_colormap;
        loadParam("thermal_colormap", thermal_colormap, 1);
        printDefaultError(CHANGE_THERMAL_CAMERA_COLORMAP(m_devices[0], thermal_colormap), "thermal_colormap");
        bool thermal_temperature_filter;
        loadParam("thermal_temperature_filter", thermal_temperature_filter, false);
        printDefaultError(ENABLE_THERMAL_CAMERA_TEMPERATURE_FILTER(m_devices[0], thermal_temperature_filter), "thermal_temperature_filter");
        int thermal_temperature_filter_min;
        loadParam("thermal_temperature_filter_min", thermal_temperature_filter_min, 0);
        int thermal_temperature_filter_max;
        loadParam("thermal_temperature_filter_max", thermal_temperature_filter_max, 50);
        printDefaultError(CHANGE_THERMAL_CAMERA_TEMPERATURE_FILTER(m_devices[0], thermal_temperature_filter_min, thermal_temperature_filter_max), "thermal_temperature_filter_max");
        int thermal_processing_pipeline;
        loadParam("thermal_processing_pipeline", thermal_processing_pipeline, 1);
        printDefaultError(CHANGE_THERMAL_CAMERA_PROCESSING_PIPELINE(m_devices[0], thermal_processing_pipeline), "thermal_processing_pipeline");
        bool thermal_temperature_data_udp;
        loadParam("thermal_temperature_data_udp", thermal_temperature_data_udp, false);
        printDefaultError(ENABLE_THERMAL_CAMERA_TEMPERATURE_DATA_UDP(m_devices[0], thermal_temperature_data_udp), "thermal_temperature_data_udp");
        int thermal_streaming_protocol;
        loadParam("thermal_streaming_protocol", thermal_streaming_protocol, 0);
        if (thermal_streaming_protocol == 1)
        {
            m_thermal_sensor->protocol = protocol_gstreamer;
            printDefaultError(CHANGE_STREAMING_PROTOCOL(m_devices[0], m_thermal_sensor), "thermal_streaming_protocol");
        }
        std::string thermal_rtsp_pipeline;
        loadParam("thermal_rtsp_pipeline", thermal_rtsp_pipeline, std::string(""));
        if (thermal_rtsp_pipeline != "")
        {
            char *pipeline = &thermal_rtsp_pipeline[0];
            printDefaultError(CHANGE_RTSP_PIPELINE(m_devices[0], *m_thermal_sensor, pipeline), "thermal_rtsp_pipeline");
        }
    }

    void L3Cam::loadAlliedWideDefaultParams()
    {
        double allied_wide_black_level;
        loadParam("allied_wide_black_level", allied_wide_black_level, 0.);
        printDefaultError(CHANGE_ALLIED_CAMERA_BLACK_LEVEL(m_devices[0], *m_allied_wide_sensor, allied_wide_black_level), "allied_wide_black_level");
        bool allied_wide_auto_exposure_time;
        loadParam("allied_wide_auto_exposure_time", allied_wide_auto_exposure_time, false);
        printDefaultError(ENABLE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME(m_devices[0], *m_allied_wide_sensor, allied_wide_auto_exposure_time), "allied_wide_auto_exposure_time");
        if (allied_wide_auto_exposure_time)
        {
            double allied_wide_auto_exposure_time_range_min;
            loadParam("allied_wide_auto_exposure_time_range_min", allied_wide_auto_exposure_time_range_min, 87.596);
            double allied_wide_auto_exposure_time_range_max;
            loadParam("allied_wide_auto_exposure_time_range_max", allied_wide_auto_exposure_time_range_max, 87.596);
            printDefaultError(CHANGE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME_RANGE(m_devices[0], *m_allied_wide_sensor, allied_wide_auto_exposure_time_range_min, allied_wide_auto_exposure_time_range_max), "allied_wide_auto_exposure_time_range");
        }
        else
        {
            double allied_wide_exposure_time;
            loadParam("allied_wide_exposure_time", allied_wide_exposure_time, 4992.32);
            printDefaultError(CHANGE_ALLIED_CAMERA_EXPOSURE_TIME_US(m_devices[0], *m_allied_wide_sensor, allied_wide_exposure_time), "allied_wide_exposure_time");
        }
        bool allied_wide_auto_gain;
        loadParam("allied_wide_auto_gain", allied_wide_auto_gain, false);
        printDefaultError(ENABLE_ALLIED_CAMERA_AUTO_GAIN(m_devices[0], *m_allied_wide_sensor, allied_wide_auto_gain), "allied_wide_auto_gain");
        if (allied_wide_auto_gain)
        {
            double allied_wide_auto_gain_range_min;
            loadParam("allied_wide_auto_gain_range_min", allied_wide_auto_gain_range_min, 0.);
            double allied_wide_auto_gain_range_max;
            loadParam("allied_wide_auto_gain_range_max", allied_wide_auto_gain_range_max, 48.);
            printDefaultError(CHANGE_ALLIED_CAMERA_AUTO_GAIN_RANGE(m_devices[0], *m_allied_wide_sensor, (float)allied_wide_auto_gain_range_min, (float)allied_wide_auto_gain_range_max), "allied_wide_auto_gain_range");
        }
        else
        {
            double allied_wide_gain;
            loadParam("allied_wide_gain", allied_wide_gain, 0.);
            printDefaultError(CHANGE_ALLIED_CAMERA_GAIN(m_devices[0], *m_allied_wide_sensor, allied_wide_gain), "allied_wide_gain");
        }
        double allied_wide_gamma;
        loadParam("allied_wide_gamma", allied_wide_gamma, 1.);
        printDefaultError(CHANGE_ALLIED_CAMERA_GAMMA(m_devices[0], *m_allied_wide_sensor, allied_wide_gamma), "allied_wide_gamma");
        double allied_wide_saturation;
        loadParam("allied_wide_saturation", allied_wide_saturation, 1.);
        printDefaultError(CHANGE_ALLIED_CAMERA_SATURATION(m_devices[0], *m_allied_wide_sensor, allied_wide_saturation), "allied_wide_saturation");
        double allied_wide_sharpness;
        loadParam("allied_wide_sharpness", allied_wide_sharpness, 0.);
        printDefaultError(CHANGE_ALLIED_CAMERA_SHARPNESS(m_devices[0], *m_allied_wide_sensor, allied_wide_sharpness), "allied_wide_sharpness");
        double allied_wide_hue;
        loadParam("allied_wide_hue", allied_wide_hue, 0.);
        printDefaultError(CHANGE_ALLIED_CAMERA_HUE(m_devices[0], *m_allied_wide_sensor, allied_wide_hue), "allied_wide_hue");
        int allied_wide_intensity_auto_precedence;
        loadParam("allied_wide_intensity_auto_precedence", allied_wide_intensity_auto_precedence, 0);
        printDefaultError(CHANGE_ALLIED_CAMERA_INTENSITY_AUTO_PRECEDENCE(m_devices[0], *m_allied_wide_sensor, allied_wide_intensity_auto_precedence), "allied_wide_intensity_auto_precedence");
        bool allied_wide_auto_white_balance;
        loadParam("allied_wide_auto_white_balance", allied_wide_auto_white_balance, false);
        printDefaultError(ENABLE_ALLIED_CAMERA_AUTO_WHITE_BALANCE(m_devices[0], *m_allied_wide_sensor, allied_wide_auto_white_balance), "allied_wide_auto_white_balance");
        int allied_wide_balance_ratio_selector;
        loadParam("allied_wide_balance_ratio_selector", allied_wide_balance_ratio_selector, 0);
        printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_RATIO_SELECTOR(m_devices[0], *m_allied_wide_sensor, allied_wide_balance_ratio_selector), "allied_wide_balance_ratio_selector");
        double allied_wide_balance_ratio;
        loadParam("allied_wide_balance_ratio", allied_wide_balance_ratio, 2.35498);
        printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_RATIO(m_devices[0], *m_allied_wide_sensor, allied_wide_balance_ratio), "allied_wide_balance_ratio");
        double allied_wide_balance_white_auto_rate;
        loadParam("allied_wide_balance_white_auto_rate", allied_wide_balance_white_auto_rate, 100.);
        printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_RATE(m_devices[0], *m_allied_wide_sensor, allied_wide_balance_white_auto_rate), "allied_wide_balance_white_auto_rate");
        double allied_wide_balance_white_auto_tolerance;
        loadParam("allied_wide_balance_white_auto_tolerance", allied_wide_balance_white_auto_tolerance, 5.);
        printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_TOLERANCE(m_devices[0], *m_allied_wide_sensor, allied_wide_balance_white_auto_tolerance), "allied_wide_balance_white_auto_tolerance");
        int allied_wide_auto_mode_region_height;
        loadParam("allied_wide_auto_mode_region_height", allied_wide_auto_mode_region_height, 2056);
        int allied_wide_auto_mode_region_width;
        loadParam("allied_wide_auto_mode_region_width", allied_wide_auto_mode_region_width, 2464);
        printDefaultError(CHANGE_ALLIED_CAMERA_AUTO_MODE_REGION(m_devices[0], *m_allied_wide_sensor, allied_wide_auto_mode_region_height, allied_wide_auto_mode_region_width), "allied_wide_auto_mode_region_width");
        int allied_wide_intensity_controller_region;
        loadParam("allied_wide_intensity_controller_region", allied_wide_intensity_controller_region, 0);
        printDefaultError(CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_REGION(m_devices[0], *m_allied_wide_sensor, allied_wide_intensity_controller_region), "allied_wide_intensity_controller_region");
        double allied_wide_intensity_controller_target;
        loadParam("allied_wide_intensity_controller_target", allied_wide_intensity_controller_target, 50.);
        printDefaultError(CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_TARGET(m_devices[0], *m_allied_wide_sensor, allied_wide_intensity_controller_target), "allied_wide_intensity_controller_target");
        int allied_wide_max_driver_buffers_count;
        loadParam("allied_wide_max_driver_buffers_count", allied_wide_max_driver_buffers_count, 64);
        printDefaultError(CHANGE_ALLIED_CAMERA_MAX_DRIVER_BUFFERS_COUNT(m_devices[0], *m_allied_wide_sensor, allied_wide_max_driver_buffers_count), "allied_wide_max_driver_buffers_count");
        int allied_wide_streaming_protocol;
        loadParam("allied_wide_streaming_protocol", allied_wide_streaming_protocol, 0);
        if (allied_wide_streaming_protocol == 1)
        {
            m_allied_wide_sensor->protocol = protocol_gstreamer;
            printDefaultError(CHANGE_STREAMING_PROTOCOL(m_devices[0], m_allied_wide_sensor), "allied_wide_streaming_protocol");
        }
        std::string allied_wide_rtsp_pipeline;
        loadParam("allied_wide_rtsp_pipeline", allied_wide_rtsp_pipeline, std::string(""));
        if (allied_wide_rtsp_pipeline != "")
        {
            char *pipeline = &allied_wide_rtsp_pipeline[0];
            printDefaultError(CHANGE_RTSP_PIPELINE(m_devices[0], *m_allied_wide_sensor, pipeline), "allied_wide_rtsp_pipeline");
        }
    }

    void L3Cam::loadAlliedNarrowDefaultParams()
    {
        double allied_narrow_black_level;
        loadParam("allied_narrow_black_level", allied_narrow_black_level, 0.);
        printDefaultError(CHANGE_ALLIED_CAMERA_BLACK_LEVEL(m_devices[0], *m_allied_narrow_sensor, allied_narrow_black_level), "allied_narrow_black_level");
        bool allied_narrow_auto_exposure_time;
        loadParam("allied_narrow_auto_exposure_time", allied_narrow_auto_exposure_time, false);
        printDefaultError(ENABLE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME(m_devices[0], *m_allied_narrow_sensor, allied_narrow_auto_exposure_time), "allied_narrow_auto_exposure_time");
        if (allied_narrow_auto_exposure_time)
        {
            double allied_narrow_auto_exposure_time_range_min;
            loadParam("allied_narrow_auto_exposure_time_range_min", allied_narrow_auto_exposure_time_range_min, 87.596);
            double allied_narrow_auto_exposure_time_range_max;
            loadParam("allied_narrow_auto_exposure_time_range_max", allied_narrow_auto_exposure_time_range_max, 87.596);
            printDefaultError(CHANGE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME_RANGE(m_devices[0], *m_allied_narrow_sensor, allied_narrow_auto_exposure_time_range_min, allied_narrow_auto_exposure_time_range_max), "allied_narrow_auto_exposure_time_range");
        }
        else
        {
            double allied_narrow_exposure_time;
            loadParam("allied_narrow_exposure_time", allied_narrow_exposure_time, 4992.32);
            printDefaultError(CHANGE_ALLIED_CAMERA_EXPOSURE_TIME_US(m_devices[0], *m_allied_narrow_sensor, allied_narrow_exposure_time), "allied_narrow_exposure_time");
        }
        bool allied_narrow_auto_gain;
        loadParam("allied_narrow_auto_gain", allied_narrow_auto_gain, false);
        printDefaultError(ENABLE_ALLIED_CAMERA_AUTO_GAIN(m_devices[0], *m_allied_narrow_sensor, allied_narrow_auto_gain), "allied_narrow_auto_gain");
        if (allied_narrow_auto_gain)
        {
            double allied_narrow_auto_gain_range_min;
            loadParam("allied_narrow_auto_gain_range_min", allied_narrow_auto_gain_range_min, 0.);
            double allied_narrow_auto_gain_range_max;
            loadParam("allied_narrow_auto_gain_range_max", allied_narrow_auto_gain_range_max, 48.);
            printDefaultError(CHANGE_ALLIED_CAMERA_AUTO_GAIN_RANGE(m_devices[0], *m_allied_narrow_sensor, (float)allied_narrow_auto_gain_range_min, (float)allied_narrow_auto_gain_range_max), "allied_narrow_auto_gain_range");
        }
        else
        {
            double allied_narrow_gain;
            loadParam("allied_narrow_gain", allied_narrow_gain, 0.);
            printDefaultError(CHANGE_ALLIED_CAMERA_GAIN(m_devices[0], *m_allied_narrow_sensor, allied_narrow_gain), "allied_narrow_gain");
        }
        double allied_narrow_gamma;
        loadParam("allied_narrow_gamma", allied_narrow_gamma, 1.);
        printDefaultError(CHANGE_ALLIED_CAMERA_GAMMA(m_devices[0], *m_allied_narrow_sensor, allied_narrow_gamma), "allied_narrow_gamma");
        double allied_narrow_saturation;
        loadParam("allied_narrow_saturation", allied_narrow_saturation, 1.);
        printDefaultError(CHANGE_ALLIED_CAMERA_SATURATION(m_devices[0], *m_allied_narrow_sensor, allied_narrow_saturation), "allied_narrow_saturation");
        double allied_narrow_sharpness;
        loadParam("allied_narrow_sharpness", allied_narrow_sharpness, 0.);
        printDefaultError(CHANGE_ALLIED_CAMERA_SHARPNESS(m_devices[0], *m_allied_narrow_sensor, allied_narrow_sharpness), "allied_narrow_sharpness");
        double allied_narrow_hue;
        loadParam("allied_narrow_hue", allied_narrow_hue, 0.);
        printDefaultError(CHANGE_ALLIED_CAMERA_HUE(m_devices[0], *m_allied_narrow_sensor, allied_narrow_hue), "allied_narrow_hue");
        int allied_narrow_intensity_auto_precedence;
        loadParam("allied_narrow_intensity_auto_precedence", allied_narrow_intensity_auto_precedence, 0);
        printDefaultError(CHANGE_ALLIED_CAMERA_INTENSITY_AUTO_PRECEDENCE(m_devices[0], *m_allied_narrow_sensor, allied_narrow_intensity_auto_precedence), "allied_narrow_intensity_auto_precedence");
        bool allied_narrow_auto_white_balance;
        loadParam("allied_narrow_auto_white_balance", allied_narrow_auto_white_balance, false);
        printDefaultError(ENABLE_ALLIED_CAMERA_AUTO_WHITE_BALANCE(m_devices[0], *m_allied_narrow_sensor, allied_narrow_auto_white_balance), "allied_narrow_auto_white_balance");
        int allied_narrow_balance_ratio_selector;
        loadParam("allied_narrow_balance_ratio_selector", allied_narrow_balance_ratio_selector, 0);
        printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_RATIO_SELECTOR(m_devices[0], *m_allied_narrow_sensor, allied_narrow_balance_ratio_selector), "allied_narrow_balance_ratio_selector");
        double allied_narrow_balance_ratio;
        loadParam("allied_narrow_balance_ratio", allied_narrow_balance_ratio, 2.35498);
        printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_RATIO(m_devices[0], *m_allied_narrow_sensor, allied_narrow_balance_ratio), "allied_narrow_balance_ratio");
        double allied_narrow_balance_white_auto_rate;
        loadParam("allied_narrow_balance_white_auto_rate", allied_narrow_balance_white_auto_rate, 100.);
        printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_RATE(m_devices[0], *m_allied_narrow_sensor, allied_narrow_balance_white_auto_rate), "allied_narrow_balance_white_auto_rate");
        double allied_narrow_balance_white_auto_tolerance;
        loadParam("allied_narrow_balance_white_auto_tolerance", allied_narrow_balance_white_auto_tolerance, 5.);
        printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_TOLERANCE(m_devices[0], *m_allied_narrow_sensor, allied_narrow_balance_white_auto_tolerance), "allied_narrow_balance_white_auto_tolerance");
        int allied_narrow_auto_mode_region_height;
        loadParam("allied_narrow_auto_mode_region_height", allied_narrow_auto_mode_region_height, 2056);
        int allied_narrow_auto_mode_region_width;
        loadParam("allied_narrow_auto_mode_region_width", allied_narrow_auto_mode_region_width, 2464);
        printDefaultError(CHANGE_ALLIED_CAMERA_AUTO_MODE_REGION(m_devices[0], *m_allied_narrow_sensor, allied_narrow_auto_mode_region_height, allied_narrow_auto_mode_region_width), "allied_narrow_auto_mode_region_width");
        int allied_narrow_intensity_controller_region;
        loadParam("allied_narrow_intensity_controller_region", allied_narrow_intensity_controller_region, 0);
        printDefaultError(CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_REGION(m_devices[0], *m_allied_narrow_sensor, allied_narrow_intensity_controller_region), "allied_narrow_intensity_controller_region");
        double allied_narrow_intensity_controller_target;
        loadParam("allied_narrow_intensity_controller_target", allied_narrow_intensity_controller_target, 50.);
        printDefaultError(CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_TARGET(m_devices[0], *m_allied_narrow_sensor, allied_narrow_intensity_controller_target), "allied_narrow_intensity_controller_target");
        int allied_narrow_max_driver_buffers_count;
        loadParam("allied_narrow_max_driver_buffers_count", allied_narrow_max_driver_buffers_count, 64);
        printDefaultError(CHANGE_ALLIED_CAMERA_MAX_DRIVER_BUFFERS_COUNT(m_devices[0], *m_allied_narrow_sensor, allied_narrow_max_driver_buffers_count), "allied_narrow_max_driver_buffers_count");
        int allied_narrow_streaming_protocol;
        loadParam("allied_narrow_streaming_protocol", allied_narrow_streaming_protocol, 0);
        if (allied_narrow_streaming_protocol == 1)
        {
            m_allied_narrow_sensor->protocol = protocol_gstreamer;
            printDefaultError(CHANGE_STREAMING_PROTOCOL(m_devices[0], m_allied_narrow_sensor), "allied_narrow_streaming_protocol");
        }
        std::string allied_narrow_rtsp_pipeline;
        loadParam("allied_narrow_rtsp_pipeline", allied_narrow_rtsp_pipeline, std::string(""));
        if (allied_narrow_rtsp_pipeline != "")
        {
            char *pipeline = &allied_narrow_rtsp_pipeline[0];
            printDefaultError(CHANGE_RTSP_PIPELINE(m_devices[0], *m_allied_narrow_sensor, pipeline), "allied_narrow_rtsp_pipeline");
        }
    }

    // Service callbacks
    bool L3Cam::libL3camStatus(l3cam_ros::LibL3camStatus::Request &req, l3cam_ros::LibL3camStatus::Response &res)
    {
        ROS_BMG_UNUSED(req);
        res.status = (int)m_status;
        return true;
    }

    bool L3Cam::getVersion(l3cam_ros::GetVersion::Request &req, l3cam_ros::GetVersion::Response &res)
    {
        ROS_BMG_UNUSED(req);
        res.version = GET_VERSION();
        return true;
    }

    bool L3Cam::initialize(l3cam_ros::Initialize::Request &req, l3cam_ros::Initialize::Response &res)
    {
        ROS_BMG_UNUSED(res);
        res.error = INITIALIZE(&req.local_address[0], &req.device_address[0]);
        return true;
    }

    bool L3Cam::terminate(l3cam_ros::Terminate::Request &req, l3cam_ros::Terminate::Response &res)
    {
        ROS_BMG_UNUSED(req);
        res.error = STOP_STREAM(m_devices[0]);
        if (!res.error)
        {
            m_status = LibL3CamStatus::started;
            res.error = STOP_DEVICE(m_devices[0]);
        }
        if (!res.error)
        {
            m_status = LibL3CamStatus::connected;
            res.error = TERMINATE(m_devices[0]);
            if (!res.error)
            {
                m_status = LibL3CamStatus::terminated;
                disconnectAll(0);
                m_shutdown_requested = true;
            }
        }
        return true;
    }

    bool L3Cam::findDevices(l3cam_ros::FindDevices::Request &req, l3cam_ros::FindDevices::Response &res)
    {
        ROS_BMG_UNUSED(req);
        res.error = FIND_DEVICES(&m_devices[0], &res.num_devices);
        return true;
    }

    bool L3Cam::getLocalServerAddress(l3cam_ros::GetLocalServerAddress::Request &req, l3cam_ros::GetLocalServerAddress::Response &res)
    {
        ROS_BMG_UNUSED(req);
        res.local_ip_address = GET_LOCAL_SERVER_ADDRESS(m_devices[0]);
        return true;
    }

    bool L3Cam::getDeviceInfo(l3cam_ros::GetDeviceInfo::Request &req, l3cam_ros::GetDeviceInfo::Response &res)
    {
        ROS_BMG_UNUSED(req);
        res.ip_address = std::string(m_devices[0].ip_address);
        res.model = m_devices[0].model;
        res.serial_number = std::string(m_devices[0].serial_number);
        res.app_version = std::string(m_devices[0].app_version);
        return true;
    }

    bool L3Cam::getDeviceStatus(l3cam_ros::GetDeviceStatus::Request &req, l3cam_ros::GetDeviceStatus::Response &res)
    {
        ROS_BMG_UNUSED(req);
        res.error = GET_DEVICE_STATUS(m_devices[0], &res.system_status);
        return true;
    }

    bool L3Cam::getSensorsAvailable(l3cam_ros::GetSensorsAvailable::Request &req, l3cam_ros::GetSensorsAvailable::Response &res)
    {
        ROS_BMG_UNUSED(req);
        res.error = GET_SENSORS_AVAILABLE(m_devices[0], m_av_sensors, &res.num_sensors);
        res.sensors.resize(res.num_sensors);
        for (int i = 0; i < res.num_sensors; ++i)
        {
            res.sensors[i].protocol = m_av_sensors[i].protocol;
            res.sensors[i].sensor_type = m_av_sensors[i].sensor_type;
            res.sensors[i].sensor_status = m_av_sensors[i].sensor_status;
            res.sensors[i].image_type = m_av_sensors[i].image_type;
            res.sensors[i].perception_enabled = m_av_sensors[i].perception_enabled;
            res.sensors[i].sensor_available = m_av_sensors[i].sensor_available;
        }
        return true;
    }

    bool L3Cam::changeStreamingProtocol(l3cam_ros::ChangeStreamingProtocol::Request &req, l3cam_ros::ChangeStreamingProtocol::Response &res)
    {
        STOP_STREAM(m_devices[0]);
        m_status = LibL3CamStatus::started;

        streamingProtocols protocol;
        switch (req.protocol)
        {
        case 0:
            protocol = protocol_raw_udp;
            break;
        case 1:
            protocol = protocol_gstreamer;
            break;
        default:
            protocol = protocol_raw_udp;
            break;
        }

        switch (req.sensor_type)
        {
        case ((int)sensorTypes::sensor_lidar):
            m_lidar_sensor->protocol = protocol;
            res.error = CHANGE_STREAMING_PROTOCOL(m_devices[0], m_lidar_sensor);
            break;
        case ((int)sensorTypes::sensor_pol):
            m_polarimetric_sensor->protocol = protocol;
            res.error = CHANGE_STREAMING_PROTOCOL(m_devices[0], m_polarimetric_sensor);
            break;
        case ((int)sensorTypes::sensor_econ_rgb):
            m_rgb_sensor->protocol = protocol;
            res.error = CHANGE_STREAMING_PROTOCOL(m_devices[0], m_rgb_sensor);
            break;
        case ((int)sensorTypes::sensor_thermal):
            m_thermal_sensor->protocol = protocol;
            res.error = CHANGE_STREAMING_PROTOCOL(m_devices[0], m_thermal_sensor);
            break;
        case ((int)sensorTypes::sensor_allied_wide):
            m_allied_wide_sensor->protocol = protocol;
            res.error = CHANGE_STREAMING_PROTOCOL(m_devices[0], m_allied_wide_sensor);
            break;
        case ((int)sensorTypes::sensor_allied_narrow):
            m_allied_narrow_sensor->protocol = protocol;
            res.error = CHANGE_STREAMING_PROTOCOL(m_devices[0], m_allied_narrow_sensor);
            break;
        }

        START_STREAM(m_devices[0]);
        m_status = LibL3CamStatus::streaming;

        return true;
    }

    bool L3Cam::getRtspPipeline(l3cam_ros::GetRtspPipeline::Request &req, l3cam_ros::GetRtspPipeline::Response &res)
    {
        char *pipeline = NULL;
        switch (req.sensor_type)
        {
        case (int)sensorTypes::sensor_lidar:
            res.error = GET_RTSP_PIPELINE(m_devices[0], *m_lidar_sensor, &pipeline);
            res.pipeline = std::string(pipeline);
            break;
        case (int)sensorTypes::sensor_pol:
            res.error = GET_RTSP_PIPELINE(m_devices[0], *m_polarimetric_sensor, &pipeline);
            res.pipeline = std::string(pipeline);
            break;
        case (int)sensorTypes::sensor_econ_rgb:
            res.error = GET_RTSP_PIPELINE(m_devices[0], *m_rgb_sensor, &pipeline);
            res.pipeline = std::string(pipeline);
            break;
        case (int)sensorTypes::sensor_thermal:
            res.error = GET_RTSP_PIPELINE(m_devices[0], *m_thermal_sensor, &pipeline);
            res.pipeline = std::string(pipeline);
            break;
        case (int)sensorTypes::sensor_allied_wide:
            res.error = GET_RTSP_PIPELINE(m_devices[0], *m_allied_wide_sensor, &pipeline);
            res.pipeline = std::string(pipeline);
            break;
        case (int)sensorTypes::sensor_allied_narrow:
            res.error = GET_RTSP_PIPELINE(m_devices[0], *m_allied_narrow_sensor, &pipeline);
            res.pipeline = std::string(pipeline);
            break;
        }

        return true;
    }

    bool L3Cam::getNetworkConfiguration(l3cam_ros::GetNetworkConfiguration::Request &req, l3cam_ros::GetNetworkConfiguration::Response &res)
    {
        ROS_BMG_UNUSED(req);
        char *ip_address = NULL;
        char *netmask = NULL;
        char *gateway = NULL;
        res.error = GET_NETWORK_CONFIGURATION(m_devices[0], &ip_address, &netmask, &gateway);
        res.ip_address = std::string(ip_address);
        res.netmask = std::string(netmask);
        res.gateway = std::string(gateway);
        return true;
    }

    bool L3Cam::changeNetworkConfiguration(l3cam_ros::ChangeNetworkConfiguration::Request &req, l3cam_ros::ChangeNetworkConfiguration::Response &res)
    {
        if (req.enable_dhcp)
            res.error = CHANGE_NETWORK_CONFIGURATION(m_devices[0], NULL, NULL, NULL, true);
        else
        {
            std::string ip_address = req.ip_address;
            std::string netmask = req.netmask;
            std::string gateway = req.gateway;
            res.error = CHANGE_NETWORK_CONFIGURATION(m_devices[0], (char *)ip_address.data(), (char *)netmask.data(), (char *)gateway.data(), false);
        }

        return true;
    }

    bool L3Cam::powerOffDevice(l3cam_ros::PowerOffDevice::Request &req, l3cam_ros::PowerOffDevice::Response &res)
    {
        ROS_BMG_UNUSED(req);
        POWER_OFF_DEVICE(m_devices[0]);
        res.error = 0;
        return true;
    }

    bool L3Cam::startDevice(l3cam_ros::StartDevice::Request &req, l3cam_ros::StartDevice::Response &res)
    {
        ROS_BMG_UNUSED(req);
        res.error = START_DEVICE(m_devices[0]);
        if (!res.error)
        {
            m_status = LibL3CamStatus::started;
        }
        return true;
    }

    bool L3Cam::stopDevice(l3cam_ros::StopDevice::Request &req, l3cam_ros::StopDevice::Response &res)
    {
        ROS_BMG_UNUSED(req);
        res.error = STOP_DEVICE(m_devices[0]);
        if (!res.error)
        {
            m_status = LibL3CamStatus::started;
            res.error = STOP_DEVICE(m_devices[0]);
            if (!res.error)
            {
                m_status = LibL3CamStatus::connected;
            }
        }
        return true;
    }

    bool L3Cam::startStream(l3cam_ros::StartStream::Request &req, l3cam_ros::StartStream::Response &res)
    {
        ROS_BMG_UNUSED(req);
        res.error = START_STREAM(m_devices[0]);
        if (!res.error)
        {
            m_status = LibL3CamStatus::streaming;
        }
        return true;
    }

    bool L3Cam::stopStream(l3cam_ros::StopStream::Request &req, l3cam_ros::StopStream::Response &res)
    {
        ROS_BMG_UNUSED(req);
        res.error = STOP_STREAM(m_devices[0]);
        if (!res.error)
        {
            m_status = LibL3CamStatus::started;
        }
        return true;
    }

    bool L3Cam::getDeviceTemperatures(l3cam_ros::GetDeviceTemperatures::Request &req, l3cam_ros::GetDeviceTemperatures::Response &res)
    {
        ROS_BMG_UNUSED(req);
        int32_t *temperatures = (int32_t *)malloc(sizeof(int32_t) * 11);
        int error = GET_DEVICE_TEMPERATURES(m_devices[0], temperatures);
        res.error = error;
        if (error != L3CAM_OK)
        {
            ROS_ERROR_STREAM(this->getNamespace() << " error " << error << " in temperatures error: " << getErrorDescription(error));
            res.bcpu_temp = 0;
            res.mcpu_temp = 0;
            res.gpu_temp = 0;
            res.pll_temp = 0;
            res.board_temp = 0;
            res.diode_temp = 0;
            res.pmic_temp = 0;
            res.fan_temp = 0;
            res.inter_temp = 0;
            res.allied_wide_temp = 0;
            res.allied_narrow_temp = 0;
        }
        else
        {
            res.bcpu_temp = temperatures[0] / 1000.0;
            res.mcpu_temp = temperatures[1] / 1000.0;
            res.gpu_temp = temperatures[2] / 1000.0;
            res.pll_temp = temperatures[3] / 1000.0;
            res.board_temp = temperatures[4] / 1000.0;
            res.diode_temp = temperatures[5] / 1000.0;
            res.pmic_temp = temperatures[6] / 1000.0;
            res.fan_temp = temperatures[7] / 1000.0;
            res.inter_temp = temperatures[8] / 1000.0;
            res.allied_wide_temp = temperatures[9] / 1000.0;
            res.allied_narrow_temp = temperatures[10] / 1000.0;
        }
        free(temperatures);

        return true;
    }

    // Point Cloud
    bool L3Cam::changePointcloudColor(l3cam_ros::ChangePointcloudColor::Request &req, l3cam_ros::ChangePointcloudColor::Response &res)
    {
        res.error = CHANGE_POINT_CLOUD_COLOR(m_devices[0], req.visualization_color);
        return true;
    }

    bool L3Cam::changePointcloudColorRange(l3cam_ros::ChangePointcloudColorRange::Request &req, l3cam_ros::ChangePointcloudColorRange::Response &res)
    {
        res.error = CHANGE_POINT_CLOUD_COLOR_RANGE(m_devices[0], req.min_value, req.max_value);
        return true;
    }

    bool L3Cam::changeDistanceRange(l3cam_ros::ChangeDistanceRange::Request &req, l3cam_ros::ChangeDistanceRange::Response &res)
    {
        res.error = CHANGE_DISTANCE_RANGE(m_devices[0], req.min_value, req.max_value);
        return true;
    }

    bool L3Cam::enableAutoBias(l3cam_ros::EnableAutoBias::Request &req, l3cam_ros::EnableAutoBias::Response &res)
    {
        ROS_BMG_UNUSED(res);
        ENABLE_AUTO_BIAS(m_devices[0], req.enabled);
        return true;
    }

    bool L3Cam::changeBiasValue(l3cam_ros::ChangeBiasValue::Request &req, l3cam_ros::ChangeBiasValue::Response &res)
    {
        ROS_BMG_UNUSED(res);
        CHANGE_BIAS_VALUE(m_devices[0], req.index, req.bias);
        return true;
    }

    // Polarimetric
    bool L3Cam::setPolarimetricCameraDefaultSettings(l3cam_ros::SetPolarimetricCameraDefaultSettings::Request &req, l3cam_ros::SetPolarimetricCameraDefaultSettings::Response &res)
    {
        ROS_BMG_UNUSED(req);
        res.error = SET_POLARIMETRIC_CAMERA_DEFAULT_SETTINGS(m_devices[0]);
        return true;
    }

    bool L3Cam::changePolarimetricCameraBrightness(l3cam_ros::ChangePolarimetricCameraBrightness::Request &req, l3cam_ros::ChangePolarimetricCameraBrightness::Response &res)
    {
        res.error = CHANGE_POLARIMETRIC_CAMERA_BRIGHTNESS(m_devices[0], req.brightness);
        return true;
    }

    bool L3Cam::changePolarimetricCameraBlackLevel(l3cam_ros::ChangePolarimetricCameraBlackLevel::Request &req, l3cam_ros::ChangePolarimetricCameraBlackLevel::Response &res)
    {
        res.error = CHANGE_POLARIMETRIC_CAMERA_BLACK_LEVEL(m_devices[0], req.black_level);
        return true;
    }

    bool L3Cam::enablePolarimetricCameraAutoGain(l3cam_ros::EnablePolarimetricCameraAutoGain::Request &req, l3cam_ros::EnablePolarimetricCameraAutoGain::Response &res)
    {
        res.error = ENABLE_POLARIMETRIC_CAMERA_AUTO_GAIN(m_devices[0], req.enabled);
        return true;
    }

    bool L3Cam::changePolarimetricCameraAutoGainRange(l3cam_ros::ChangePolarimetricCameraAutoGainRange::Request &req, l3cam_ros::ChangePolarimetricCameraAutoGainRange::Response &res)
    {
        res.error = CHANGE_POLARIMETRIC_CAMERA_AUTO_GAIN_RANGE(m_devices[0], req.min_gain, req.max_gain);
        return true;
    }

    bool L3Cam::changePolarimetricCameraGain(l3cam_ros::ChangePolarimetricCameraGain::Request &req, l3cam_ros::ChangePolarimetricCameraGain::Response &res)
    {
        res.error = CHANGE_POLARIMETRIC_CAMERA_GAIN(m_devices[0], req.gain);
        return true;
    }

    bool L3Cam::enablePolarimetricCameraAutoExposureTime(l3cam_ros::EnablePolarimetricCameraAutoExposureTime::Request &req, l3cam_ros::EnablePolarimetricCameraAutoExposureTime::Response &res)
    {
        res.error = ENABLE_POLARIMETRIC_CAMERA_AUTO_EXPOSURE_TIME(m_devices[0], req.enabled);
        return true;
    }

    bool L3Cam::changePolarimetricCameraAutoExposureTimeRange(l3cam_ros::ChangePolarimetricCameraAutoExposureTimeRange::Request &req, l3cam_ros::ChangePolarimetricCameraAutoExposureTimeRange::Response &res)
    {
        res.error = CHANGE_POLARIMETRIC_CAMERA_AUTO_EXPOSURE_TIME_RANGE(m_devices[0], req.min_exposure, req.max_exposure);
        return true;
    }

    bool L3Cam::changePolarimetricCameraExposureTime(l3cam_ros::ChangePolarimetricCameraExposureTime::Request &req, l3cam_ros::ChangePolarimetricCameraExposureTime::Response &res)
    {
        res.error = CHANGE_POLARIMETRIC_CAMERA_EXPOSURE_TIME(m_devices[0], req.exposure_time);
        return true;
    }

    // RGB
    bool L3Cam::setRgbCameraDefaultSettings(l3cam_ros::SetRgbCameraDefaultSettings::Request &req, l3cam_ros::SetRgbCameraDefaultSettings::Response &res)
    {
        ROS_BMG_UNUSED(req);
        res.error = SET_RGB_CAMERA_DEFAULT_SETTINGS(m_devices[0]);
        return true;
    }

    bool L3Cam::changeRgbCameraBrightness(l3cam_ros::ChangeRgbCameraBrightness::Request &req, l3cam_ros::ChangeRgbCameraBrightness::Response &res)
    {
        res.error = CHANGE_RGB_CAMERA_BRIGHTNESS(m_devices[0], req.brightness);
        return true;
    }

    bool L3Cam::changeRgbCameraContrast(l3cam_ros::ChangeRgbCameraContrast::Request &req, l3cam_ros::ChangeRgbCameraContrast::Response &res)
    {
        res.error = CHANGE_RGB_CAMERA_CONTRAST(m_devices[0], req.contrast);
        return true;
    }

    bool L3Cam::changeRgbCameraSaturation(l3cam_ros::ChangeRgbCameraSaturation::Request &req, l3cam_ros::ChangeRgbCameraSaturation::Response &res)
    {
        res.error = CHANGE_RGB_CAMERA_SATURATION(m_devices[0], req.saturation);
        return true;
    }

    bool L3Cam::changeRgbCameraSharpness(l3cam_ros::ChangeRgbCameraSharpness::Request &req, l3cam_ros::ChangeRgbCameraSharpness::Response &res)
    {
        res.error = CHANGE_RGB_CAMERA_SHARPNESS(m_devices[0], req.sharpness);
        return true;
    }

    bool L3Cam::changeRgbCameraGamma(l3cam_ros::ChangeRgbCameraGamma::Request &req, l3cam_ros::ChangeRgbCameraGamma::Response &res)
    {
        res.error = CHANGE_RGB_CAMERA_GAMMA(m_devices[0], req.gamma);
        return true;
    }

    bool L3Cam::changeRgbCameraGain(l3cam_ros::ChangeRgbCameraGain::Request &req, l3cam_ros::ChangeRgbCameraGain::Response &res)
    {
        res.error = CHANGE_RGB_CAMERA_GAIN(m_devices[0], req.gain);
        return true;
    }

    bool L3Cam::enableRgbCameraAutoWhiteBalance(l3cam_ros::EnableRgbCameraAutoWhiteBalance::Request &req, l3cam_ros::EnableRgbCameraAutoWhiteBalance::Response &res)
    {
        res.error = ENABLE_RGB_CAMERA_AUTO_WHITE_BALANCE(m_devices[0], req.enabled);
        return true;
    }

    bool L3Cam::changeRgbCameraWhiteBalance(l3cam_ros::ChangeRgbCameraWhiteBalance::Request &req, l3cam_ros::ChangeRgbCameraWhiteBalance::Response &res)
    {
        res.error = CHANGE_RGB_CAMERA_WHITE_BALANCE(m_devices[0], req.white_balance);
        return true;
    }

    bool L3Cam::enableRgbCameraAutoExposureTime(l3cam_ros::EnableRgbCameraAutoExposureTime::Request &req, l3cam_ros::EnableRgbCameraAutoExposureTime::Response &res)
    {
        res.error = ENABLE_RGB_CAMERA_AUTO_EXPOSURE_TIME(m_devices[0], req.enabled);
        return true;
    }

    bool L3Cam::changeRgbCameraExposureTime(l3cam_ros::ChangeRgbCameraExposureTime::Request &req, l3cam_ros::ChangeRgbCameraExposureTime::Response &res)
    {
        res.error = CHANGE_RGB_CAMERA_EXPOSURE_TIME(m_devices[0], req.exposure_time);
        return true;
    }

    // Thermal
    bool L3Cam::changeThermalCameraColormap(l3cam_ros::ChangeThermalCameraColormap::Request &req, l3cam_ros::ChangeThermalCameraColormap::Response &res)
    {
        res.error = CHANGE_THERMAL_CAMERA_COLORMAP(m_devices[0], (thermalTypes)req.colormap);
        return true;
    }

    bool L3Cam::enableThermalCameraTemperatureFilter(l3cam_ros::EnableThermalCameraTemperatureFilter::Request &req, l3cam_ros::EnableThermalCameraTemperatureFilter::Response &res)
    {
        res.error = ENABLE_THERMAL_CAMERA_TEMPERATURE_FILTER(m_devices[0], req.enabled);
        return true;
    }

    bool L3Cam::changeThermalCameraTemperatureFilter(l3cam_ros::ChangeThermalCameraTemperatureFilter::Request &req, l3cam_ros::ChangeThermalCameraTemperatureFilter::Response &res)
    {
        res.error = CHANGE_THERMAL_CAMERA_TEMPERATURE_FILTER(m_devices[0], req.min_temperature, req.max_temperature);
        return true;
    }

    bool L3Cam::changeThermalCameraProcessingPipeline(l3cam_ros::ChangeThermalCameraProcessingPipeline::Request &req, l3cam_ros::ChangeThermalCameraProcessingPipeline::Response &res)
    {
        res.error = CHANGE_THERMAL_CAMERA_PROCESSING_PIPELINE(m_devices[0], req.pipeline);
        return true;
    }

    bool L3Cam::enableThermalCameraTemperatureDataUdp(l3cam_ros::EnableThermalCameraTemperatureDataUdp::Request &req, l3cam_ros::EnableThermalCameraTemperatureDataUdp::Response &res)
    {
        res.error = ENABLE_THERMAL_CAMERA_TEMPERATURE_DATA_UDP(m_devices[0], req.enabled);
        return true;
    }

    // Allied
    bool L3Cam::changeAlliedCameraExposureTime(l3cam_ros::ChangeAlliedCameraExposureTime::Request &req, l3cam_ros::ChangeAlliedCameraExposureTime::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = CHANGE_ALLIED_CAMERA_EXPOSURE_TIME_US(m_devices[0], *m_allied_wide_sensor, req.exposure_time);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = CHANGE_ALLIED_CAMERA_EXPOSURE_TIME_US(m_devices[0], *m_allied_narrow_sensor, req.exposure_time);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::enableAlliedCameraAutoExposureTime(l3cam_ros::EnableAlliedCameraAutoExposureTime::Request &req, l3cam_ros::EnableAlliedCameraAutoExposureTime::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = ENABLE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME(m_devices[0], *m_allied_wide_sensor, req.enabled);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = ENABLE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME(m_devices[0], *m_allied_narrow_sensor, req.enabled);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::changeAlliedCameraAutoExposureTimeRange(l3cam_ros::ChangeAlliedCameraAutoExposureTimeRange::Request &req, l3cam_ros::ChangeAlliedCameraAutoExposureTimeRange::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = CHANGE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME_RANGE(m_devices[0], *m_allied_wide_sensor, req.auto_exposure_time_range_min, req.auto_exposure_time_range_max);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = CHANGE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME_RANGE(m_devices[0], *m_allied_narrow_sensor, req.auto_exposure_time_range_min, req.auto_exposure_time_range_max);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::changeAlliedCameraGain(l3cam_ros::ChangeAlliedCameraGain::Request &req, l3cam_ros::ChangeAlliedCameraGain::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = CHANGE_ALLIED_CAMERA_GAIN(m_devices[0], *m_allied_wide_sensor, req.gain);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = CHANGE_ALLIED_CAMERA_GAIN(m_devices[0], *m_allied_narrow_sensor, req.gain);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::enableAlliedCameraAutoGain(l3cam_ros::EnableAlliedCameraAutoGain::Request &req, l3cam_ros::EnableAlliedCameraAutoGain::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = ENABLE_ALLIED_CAMERA_AUTO_GAIN(m_devices[0], *m_allied_wide_sensor, req.enabled);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = ENABLE_ALLIED_CAMERA_AUTO_GAIN(m_devices[0], *m_allied_narrow_sensor, req.enabled);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::changeAlliedCameraAutoGainRange(l3cam_ros::ChangeAlliedCameraAutoGainRange::Request &req, l3cam_ros::ChangeAlliedCameraAutoGainRange::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = CHANGE_ALLIED_CAMERA_AUTO_GAIN_RANGE(m_devices[0], *m_allied_wide_sensor, (float)req.auto_gain_range_min, (float)req.auto_gain_range_max);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = CHANGE_ALLIED_CAMERA_AUTO_GAIN_RANGE(m_devices[0], *m_allied_narrow_sensor, (float)req.auto_gain_range_min, (float)req.auto_gain_range_max);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::changeAlliedCameraGamma(l3cam_ros::ChangeAlliedCameraGamma::Request &req, l3cam_ros::ChangeAlliedCameraGamma::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = CHANGE_ALLIED_CAMERA_GAMMA(m_devices[0], *m_allied_wide_sensor, req.gamma);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = CHANGE_ALLIED_CAMERA_GAMMA(m_devices[0], *m_allied_narrow_sensor, req.gamma);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::changeAlliedCameraSaturation(l3cam_ros::ChangeAlliedCameraSaturation::Request &req, l3cam_ros::ChangeAlliedCameraSaturation::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = CHANGE_ALLIED_CAMERA_SATURATION(m_devices[0], *m_allied_wide_sensor, req.saturation);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = CHANGE_ALLIED_CAMERA_SATURATION(m_devices[0], *m_allied_narrow_sensor, req.saturation);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::changeAlliedCameraHue(l3cam_ros::ChangeAlliedCameraHue::Request &req, l3cam_ros::ChangeAlliedCameraHue::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = CHANGE_ALLIED_CAMERA_HUE(m_devices[0], *m_allied_wide_sensor, req.hue);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = CHANGE_ALLIED_CAMERA_HUE(m_devices[0], *m_allied_narrow_sensor, req.hue);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::changeAlliedCameraIntensityAutoPrecedence(l3cam_ros::ChangeAlliedCameraIntensityAutoPrecedence::Request &req, l3cam_ros::ChangeAlliedCameraIntensityAutoPrecedence::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = CHANGE_ALLIED_CAMERA_INTENSITY_AUTO_PRECEDENCE(m_devices[0], *m_allied_wide_sensor, req.intensity_auto_precedence);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = CHANGE_ALLIED_CAMERA_INTENSITY_AUTO_PRECEDENCE(m_devices[0], *m_allied_narrow_sensor, req.intensity_auto_precedence);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::enableAlliedCameraAutoWhiteBalance(l3cam_ros::EnableAlliedCameraAutoWhiteBalance::Request &req, l3cam_ros::EnableAlliedCameraAutoWhiteBalance::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = ENABLE_ALLIED_CAMERA_AUTO_WHITE_BALANCE(m_devices[0], *m_allied_wide_sensor, req.enabled);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = ENABLE_ALLIED_CAMERA_AUTO_WHITE_BALANCE(m_devices[0], *m_allied_narrow_sensor, req.enabled);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::changeAlliedCameraBalanceRatioSelector(l3cam_ros::ChangeAlliedCameraBalanceRatioSelector::Request &req, l3cam_ros::ChangeAlliedCameraBalanceRatioSelector::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = CHANGE_ALLIED_CAMERA_BALANCE_RATIO_SELECTOR(m_devices[0], *m_allied_wide_sensor, req.white_balance_ratio_selector);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = CHANGE_ALLIED_CAMERA_BALANCE_RATIO_SELECTOR(m_devices[0], *m_allied_narrow_sensor, req.white_balance_ratio_selector);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::changeAlliedCameraBalanceRatio(l3cam_ros::ChangeAlliedCameraBalanceRatio::Request &req, l3cam_ros::ChangeAlliedCameraBalanceRatio::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = CHANGE_ALLIED_CAMERA_BALANCE_RATIO(m_devices[0], *m_allied_wide_sensor, req.balance_ratio);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = CHANGE_ALLIED_CAMERA_BALANCE_RATIO(m_devices[0], *m_allied_narrow_sensor, req.balance_ratio);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::changeAlliedCameraBalanceWhiteAutoRate(l3cam_ros::ChangeAlliedCameraBalanceWhiteAutoRate::Request &req, l3cam_ros::ChangeAlliedCameraBalanceWhiteAutoRate::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_RATE(m_devices[0], *m_allied_wide_sensor, req.white_balance_auto_rate);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_RATE(m_devices[0], *m_allied_narrow_sensor, req.white_balance_auto_rate);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::changeAlliedCameraBalanceWhiteAutoTolerance(l3cam_ros::ChangeAlliedCameraBalanceWhiteAutoTolerance::Request &req, l3cam_ros::ChangeAlliedCameraBalanceWhiteAutoTolerance::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_TOLERANCE(m_devices[0], *m_allied_wide_sensor, req.white_balance_auto_tolerance);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_TOLERANCE(m_devices[0], *m_allied_narrow_sensor, req.white_balance_auto_tolerance);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::changeAlliedCameraIntensityControllerRegion(l3cam_ros::ChangeAlliedCameraIntensityControllerRegion::Request &req, l3cam_ros::ChangeAlliedCameraIntensityControllerRegion::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_REGION(m_devices[0], *m_allied_wide_sensor, req.intensity_controller_region);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_REGION(m_devices[0], *m_allied_narrow_sensor, req.intensity_controller_region);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::changeAlliedCameraIntensityControllerTarget(l3cam_ros::ChangeAlliedCameraIntensityControllerTarget::Request &req, l3cam_ros::ChangeAlliedCameraIntensityControllerTarget::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_TARGET(m_devices[0], *m_allied_wide_sensor, req.intensity_controller_target);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_TARGET(m_devices[0], *m_allied_narrow_sensor, req.intensity_controller_target);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::getAlliedCameraBlackLevel(l3cam_ros::GetAlliedCameraBlackLevel::Request &req, l3cam_ros::GetAlliedCameraBlackLevel::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = GET_ALLIED_CAMERA_BLACK_LEVEL(m_devices[0], *m_allied_wide_sensor, &res.black_level);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = GET_ALLIED_CAMERA_BLACK_LEVEL(m_devices[0], *m_allied_narrow_sensor, &res.black_level);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::getAlliedCameraExposureTime(l3cam_ros::GetAlliedCameraExposureTime::Request &req, l3cam_ros::GetAlliedCameraExposureTime::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = GET_ALLIED_CAMERA_EXPOSURE_TIME_US(m_devices[0], *m_allied_wide_sensor, &res.exposure_time);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = GET_ALLIED_CAMERA_EXPOSURE_TIME_US(m_devices[0], *m_allied_narrow_sensor, &res.exposure_time);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::getAlliedCameraAutoExposureTime(l3cam_ros::GetAlliedCameraAutoExposureTime::Request &req, l3cam_ros::GetAlliedCameraAutoExposureTime::Response &res)
    {
        bool enabled;
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = GET_ALLIED_CAMERA_AUTO_EXPOSURE_TIME(m_devices[0], *m_allied_wide_sensor, &enabled);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = GET_ALLIED_CAMERA_AUTO_EXPOSURE_TIME(m_devices[0], *m_allied_narrow_sensor, &enabled);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        res.enabled = enabled;

        return true;
    }

    bool L3Cam::getAlliedCameraAutoExposureTimeRange(l3cam_ros::GetAlliedCameraAutoExposureTimeRange::Request &req, l3cam_ros::GetAlliedCameraAutoExposureTimeRange::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = GET_ALLIED_CAMERA_AUTO_EXPOSURE_TIME_RANGE(m_devices[0], *m_allied_wide_sensor, &res.auto_exposure_time_range_min, &res.auto_exposure_time_range_max);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = GET_ALLIED_CAMERA_AUTO_EXPOSURE_TIME_RANGE(m_devices[0], *m_allied_narrow_sensor, &res.auto_exposure_time_range_min, &res.auto_exposure_time_range_max);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::getAlliedCameraGain(l3cam_ros::GetAlliedCameraGain::Request &req, l3cam_ros::GetAlliedCameraGain::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = GET_ALLIED_CAMERA_GAIN(m_devices[0], *m_allied_wide_sensor, &res.gain);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = GET_ALLIED_CAMERA_GAIN(m_devices[0], *m_allied_narrow_sensor, &res.gain);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::getAlliedCameraAutoGain(l3cam_ros::GetAlliedCameraAutoGain::Request &req, l3cam_ros::GetAlliedCameraAutoGain::Response &res)
    {
        bool enabled;
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = GET_ALLIED_CAMERA_AUTO_GAIN(m_devices[0], *m_allied_wide_sensor, &enabled);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = GET_ALLIED_CAMERA_AUTO_GAIN(m_devices[0], *m_allied_narrow_sensor, &enabled);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        res.enabled = enabled;

        return true;
    }

    bool L3Cam::getAlliedCameraAutoGainRange(l3cam_ros::GetAlliedCameraAutoGainRange::Request &req, l3cam_ros::GetAlliedCameraAutoGainRange::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = GET_ALLIED_CAMERA_AUTO_GAIN_RANGE(m_devices[0], *m_allied_wide_sensor, &res.auto_gain_range_min, &res.auto_gain_range_max);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = GET_ALLIED_CAMERA_AUTO_GAIN_RANGE(m_devices[0], *m_allied_narrow_sensor, &res.auto_gain_range_min, &res.auto_gain_range_max);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::getAlliedCameraGamma(l3cam_ros::GetAlliedCameraGamma::Request &req, l3cam_ros::GetAlliedCameraGamma::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = GET_ALLIED_CAMERA_GAMMA(m_devices[0], *m_allied_wide_sensor, &res.gamma);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = GET_ALLIED_CAMERA_GAMMA(m_devices[0], *m_allied_narrow_sensor, &res.gamma);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::getAlliedCameraSaturation(l3cam_ros::GetAlliedCameraSaturation::Request &req, l3cam_ros::GetAlliedCameraSaturation::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = GET_ALLIED_CAMERA_SATURATION(m_devices[0], *m_allied_wide_sensor, &res.saturation);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = GET_ALLIED_CAMERA_SATURATION(m_devices[0], *m_allied_narrow_sensor, &res.saturation);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::getAlliedCameraSharpness(l3cam_ros::GetAlliedCameraSharpness::Request &req, l3cam_ros::GetAlliedCameraSharpness::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = GET_ALLIED_CAMERA_SHARPNESS(m_devices[0], *m_allied_wide_sensor, &res.sharpness);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = GET_ALLIED_CAMERA_SHARPNESS(m_devices[0], *m_allied_narrow_sensor, &res.sharpness);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::getAlliedCameraHue(l3cam_ros::GetAlliedCameraHue::Request &req, l3cam_ros::GetAlliedCameraHue::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = GET_ALLIED_CAMERA_HUE(m_devices[0], *m_allied_wide_sensor, &res.hue);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = GET_ALLIED_CAMERA_HUE(m_devices[0], *m_allied_narrow_sensor, &res.hue);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::getAlliedCameraIntensityAutoPrecedence(l3cam_ros::GetAlliedCameraIntensityAutoPrecedence::Request &req, l3cam_ros::GetAlliedCameraIntensityAutoPrecedence::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = GET_ALLIED_CAMERA_INTENSITY_AUTO_PRECEDENCE(m_devices[0], *m_allied_wide_sensor, &res.intensity_auto_precedence);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = GET_ALLIED_CAMERA_INTENSITY_AUTO_PRECEDENCE(m_devices[0], *m_allied_narrow_sensor, &res.intensity_auto_precedence);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::getAlliedCameraAutoWhiteBalance(l3cam_ros::GetAlliedCameraAutoWhiteBalance::Request &req, l3cam_ros::GetAlliedCameraAutoWhiteBalance::Response &res)
    {
        bool enabled;
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = GET_ALLIED_CAMERA_AUTO_WHITE_BALANCE(m_devices[0], *m_allied_wide_sensor, &enabled);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = GET_ALLIED_CAMERA_AUTO_WHITE_BALANCE(m_devices[0], *m_allied_narrow_sensor, &enabled);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        res.enabled = enabled;

        return true;
    }

    bool L3Cam::getAlliedCameraBalanceRatioSelector(l3cam_ros::GetAlliedCameraBalanceRatioSelector::Request &req, l3cam_ros::GetAlliedCameraBalanceRatioSelector::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = GET_ALLIED_CAMERA_BALANCE_RATIO_SELECTOR(m_devices[0], *m_allied_wide_sensor, &res.white_balance_ratio_selector);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = GET_ALLIED_CAMERA_BALANCE_RATIO_SELECTOR(m_devices[0], *m_allied_narrow_sensor, &res.white_balance_ratio_selector);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::getAlliedCameraBalanceRatio(l3cam_ros::GetAlliedCameraBalanceRatio::Request &req, l3cam_ros::GetAlliedCameraBalanceRatio::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = GET_ALLIED_CAMERA_BALANCE_RATIO(m_devices[0], *m_allied_wide_sensor, &res.balance_ratio);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = GET_ALLIED_CAMERA_BALANCE_RATIO(m_devices[0], *m_allied_narrow_sensor, &res.balance_ratio);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::getAlliedCameraBalanceWhiteAutoRate(l3cam_ros::GetAlliedCameraBalanceWhiteAutoRate::Request &req, l3cam_ros::GetAlliedCameraBalanceWhiteAutoRate::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = GET_ALLIED_CAMERA_BALANCE_WHITE_AUTO_RATE(m_devices[0], *m_allied_wide_sensor, &res.white_balance_auto_rate);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = GET_ALLIED_CAMERA_BALANCE_WHITE_AUTO_RATE(m_devices[0], *m_allied_narrow_sensor, &res.white_balance_auto_rate);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::getAlliedCameraBalanceWhiteAutoTolerance(l3cam_ros::GetAlliedCameraBalanceWhiteAutoTolerance::Request &req, l3cam_ros::GetAlliedCameraBalanceWhiteAutoTolerance::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = GET_ALLIED_CAMERA_BALANCE_WHITE_AUTO_TOLERANCE(m_devices[0], *m_allied_wide_sensor, &res.white_balance_auto_tolerance);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = GET_ALLIED_CAMERA_BALANCE_WHITE_AUTO_TOLERANCE(m_devices[0], *m_allied_narrow_sensor, &res.white_balance_auto_tolerance);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::getAlliedCameraAutoModeRegion(l3cam_ros::GetAlliedCameraAutoModeRegion::Request &req, l3cam_ros::GetAlliedCameraAutoModeRegion::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = GET_ALLIED_CAMERA_AUTO_MODE_REGION(m_devices[0], *m_allied_wide_sensor, &res.width, &res.height);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = GET_ALLIED_CAMERA_AUTO_MODE_REGION(m_devices[0], *m_allied_narrow_sensor, &res.width, &res.height);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::getAlliedCameraIntensityControllerRegion(l3cam_ros::GetAlliedCameraIntensityControllerRegion::Request &req, l3cam_ros::GetAlliedCameraIntensityControllerRegion::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = GET_ALLIED_CAMERA_INTENSITY_CONTROLLER_REGION(m_devices[0], *m_allied_wide_sensor, &res.intensity_controller_region);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = GET_ALLIED_CAMERA_INTENSITY_CONTROLLER_REGION(m_devices[0], *m_allied_narrow_sensor, &res.intensity_controller_region);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::getAlliedCameraIntensityControllerTarget(l3cam_ros::GetAlliedCameraIntensityControllerTarget::Request &req, l3cam_ros::GetAlliedCameraIntensityControllerTarget::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = GET_ALLIED_CAMERA_INTENSITY_CONTROLLER_TARGET(m_devices[0], *m_allied_wide_sensor, &res.intensity_controller_target);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = GET_ALLIED_CAMERA_INTENSITY_CONTROLLER_TARGET(m_devices[0], *m_allied_narrow_sensor, &res.intensity_controller_target);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    bool L3Cam::getAlliedCameraMaxDriverBuffersCount(l3cam_ros::GetAlliedCameraMaxDriverBuffersCount::Request &req, l3cam_ros::GetAlliedCameraMaxDriverBuffersCount::Response &res)
    {
        switch (req.allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res.error = GET_ALLIED_CAMERA_MAX_DRIVER_BUFFERS_COUNT(m_devices[0], *m_allied_wide_sensor, &res.max_driver_buffers_count);
            break;
        case alliedCamerasIds::narrow_camera:
            res.error = GET_ALLIED_CAMERA_MAX_DRIVER_BUFFERS_COUNT(m_devices[0], *m_allied_narrow_sensor, &res.max_driver_buffers_count);
            break;
        default:
            res.error = L3CAM_VALUE_OUT_OF_RANGE;
        }
        return true;
    }

    // Sensors Disconnection
    void L3Cam::networkDisconnected(int code)
    {
        srv_network_disconnected_.request.code = code;
        if (!client_network_disconnected_.call(srv_network_disconnected_))
        {
            ROS_ERROR_STREAM(this->getNamespace() << " error: Failed to call service " << client_network_disconnected_.getService());
        }
    }

    void L3Cam::lidarDisconnected(int code)
    {
        // Stream
        srv_pointcloud_stream_disconnected_.request.code = code;
        if (!client_lidar_stream_disconnected_.call(srv_pointcloud_stream_disconnected_))
        {
            ROS_ERROR_STREAM(this->getNamespace() << " error: Failed to call service " << client_lidar_stream_disconnected_.getService());
        }

        // Configuration
        srv_pointcloud_configuration_disconnected_.request.code = code;
        if (!client_lidar_configuration_disconnected_.call(srv_pointcloud_configuration_disconnected_))
        {
            ROS_ERROR_STREAM(this->getNamespace() << " error: Failed to call service " << client_lidar_configuration_disconnected_.getService());
        }
    }

    void L3Cam::polDisconnected(int code)
    {
        // Stream
        srv_pol_wide_stream_disconnected_.request.code = code;
        if (!client_pol_wide_stream_disconnected_.call(srv_pol_wide_stream_disconnected_))
        {
            ROS_ERROR_STREAM(this->getNamespace() << " error: Failed to call service " << client_pol_wide_stream_disconnected_.getService());
        }

        // Configuration
        srv_pol_configuration_disconnected_.request.code = code;
        if (!client_pol_configuration_disconnected_.call(srv_pol_configuration_disconnected_))
        {
            ROS_ERROR_STREAM(this->getNamespace() << " error: Failed to call service " << client_pol_configuration_disconnected_.getService());
        }
    }

    void L3Cam::rgbDisconnected(int code)
    {
        // Stream
        srv_rgb_narrow_stream_disconnected_.request.code = code;
        if (!client_rgb_narrow_stream_disconnected_.call(srv_rgb_narrow_stream_disconnected_))
        {
            ROS_ERROR_STREAM(this->getNamespace() << " error: Failed to call service " << client_rgb_narrow_stream_disconnected_.getService());
        }

        // Configuration
        srv_rgb_configuration_disconnected_.request.code = code;
        if (!client_rgb_configuration_disconnected_.call(srv_rgb_configuration_disconnected_))
        {
            ROS_ERROR_STREAM(this->getNamespace() << " error: Failed to call service " << client_rgb_configuration_disconnected_.getService());
        }
    }

    void L3Cam::thermalDisconnected(int code)
    {
        // Stream
        srv_thermal_stream_disconnected_.request.code = code;
        if (!client_thermal_stream_disconnected_.call(srv_thermal_stream_disconnected_))
        {
            ROS_ERROR_STREAM(this->getNamespace() << " error: Failed to call service " << client_thermal_stream_disconnected_.getService());
        }

        // Configuration
        srv_thermal_configuration_disconnected_.request.code = code;
        if (!client_thermal_configuration_disconnected_.call(srv_thermal_configuration_disconnected_))
        {
            ROS_ERROR_STREAM(this->getNamespace() << " error: Failed to call service " << client_thermal_configuration_disconnected_.getService());
        }
    }

    void L3Cam::alliedwideDisconnected(int code)
    {
        // Stream
        srv_pol_wide_stream_disconnected_.request.code = code;
        if (!client_pol_wide_stream_disconnected_.call(srv_pol_wide_stream_disconnected_))
        {
            ROS_ERROR_STREAM(this->getNamespace() << " error: Failed to call service " << client_pol_wide_stream_disconnected_.getService());
        }

        // Configuration
        srv_wide_configuration_disconnected_.request.code = code;
        if (!client_wide_configuration_disconnected_.call(srv_wide_configuration_disconnected_))
        {
            ROS_ERROR_STREAM(this->getNamespace() << " error: Failed to call service " << client_wide_configuration_disconnected_.getService());
        }
    }

    void L3Cam::alliedNarrowDisconnect(int code)
    {
        // Stream
        srv_rgb_narrow_stream_disconnected_.request.code = code;
        if (!client_rgb_narrow_stream_disconnected_.call(srv_rgb_narrow_stream_disconnected_))
        {
            ROS_ERROR_STREAM(this->getNamespace() << " error: Failed to call service " << client_rgb_narrow_stream_disconnected_.getService());
        }

        // Configuration
        srv_narrow_configuration_disconnected_.request.code = code;
        if (!client_narrow_configuration_disconnected_.call(srv_narrow_configuration_disconnected_))
        {
            ROS_ERROR_STREAM(this->getNamespace() << " error: Failed to call service " << client_narrow_configuration_disconnected_.getService());
        }
    }

    void L3Cam::errorNotification(const int32_t *error)
    {
        // ROS_INFO("Error notification received");
        int errort = *error;

        switch (errort)
        {
        case ERROR_LIDAR_TIMED_OUT:
            node->lidarDisconnected(errort);
            break;
        case ERROR_THERMAL_CAMERA_TIMEOUT:
            node->thermalDisconnected(errort);
            break;
        }
    }

} // namespace l3cam_ros

int main(int argc, char **argv)
{
    ros::init(argc, argv, "l3cam_ros_node");

    ROS_INFO_STREAM("L3Cam version " << GET_VERSION() << "\n");

    node = new l3cam_ros::L3Cam();

    int error = L3CAM_OK;
    error = node->initializeDevice();
    if (error)
    {
        ROS_ERROR_STREAM(node->getNamespace() << " error " << error << " while initializing device: " << getErrorDescription(error));
        ROS_INFO("Terminating...");
        node->disconnectAll(error);
        TERMINATE(node->m_devices[0]);
        node->m_status = LibL3CamStatus::terminated;
        ROS_INFO("Terminated.");
        ros::shutdown();
        return error;
    }

    error = node->startDeviceStream();
    if (error)
    {
        ROS_ERROR_STREAM(node->getNamespace() << " error " << error << " while starting device and stream: " << getErrorDescription(error));
        if (error == L3CAM_TIMEOUT_ERROR)
        {
            ROS_INFO_STREAM("Device is not " << (node->m_status == connected ? "started." : "streaming."));
        }
        else
        {
            ROS_INFO("Terminating...");
            node->disconnectAll(error);
            STOP_STREAM(node->m_devices[0]);
            STOP_DEVICE(node->m_devices[0]);
            TERMINATE(node->m_devices[0]);
            node->m_status = LibL3CamStatus::terminated;
            ROS_INFO("Terminated.");
            ros::shutdown();
            return error;
        }
    }

    node->spin();

    ros::shutdown();
    return 0;
}
