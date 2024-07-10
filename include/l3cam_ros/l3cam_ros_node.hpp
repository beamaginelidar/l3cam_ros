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

#include <beamagine.h>
#include "l3cam_ros_utils.hpp"

#include "l3cam_ros/LibL3camStatus.h"
#include "l3cam_ros/GetVersion.h"
#include "l3cam_ros/Initialize.h"
#include "l3cam_ros/Terminate.h"
#include "l3cam_ros/FindDevices.h"
#include "l3cam_ros/GetLocalServerAddress.h"
#include "l3cam_ros/GetDeviceInfo.h"
#include "l3cam_ros/GetDeviceStatus.h"
#include "l3cam_ros/GetSensorsAvailable.h"
#include "l3cam_ros/ChangeStreamingProtocol.h"
#include "l3cam_ros/GetRtspPipeline.h"
#include "l3cam_ros/GetNetworkConfiguration.h"
#include "l3cam_ros/ChangeNetworkConfiguration.h"
#include "l3cam_ros/PowerOffDevice.h"
#include "l3cam_ros/StartDevice.h"
#include "l3cam_ros/StopDevice.h"
#include "l3cam_ros/StartStream.h"
#include "l3cam_ros/StopStream.h"
#include "l3cam_ros/GetDeviceTemperatures.h"

#include "l3cam_ros/ChangePointcloudColor.h"
#include "l3cam_ros/ChangePointcloudColorRange.h"
#include "l3cam_ros/ChangeDistanceRange.h"
#include "l3cam_ros/SetBiasShortRange.h"
#include "l3cam_ros/EnableAutoBias.h"
#include "l3cam_ros/ChangeBiasValue.h"
#include "l3cam_ros/ChangeAutobiasValue.h"
#include "l3cam_ros/GetAutobiasValue.h"

#include "l3cam_ros/SetPolarimetricCameraDefaultSettings.h"
#include "l3cam_ros/ChangePolarimetricCameraBrightness.h"
#include "l3cam_ros/ChangePolarimetricCameraBlackLevel.h"
#include "l3cam_ros/EnablePolarimetricCameraAutoGain.h"
#include "l3cam_ros/ChangePolarimetricCameraAutoGainRange.h"
#include "l3cam_ros/ChangePolarimetricCameraGain.h"
#include "l3cam_ros/EnablePolarimetricCameraAutoExposureTime.h"
#include "l3cam_ros/ChangePolarimetricCameraAutoExposureTimeRange.h"
#include "l3cam_ros/ChangePolarimetricCameraExposureTime.h"

#include "l3cam_ros/SetRgbCameraDefaultSettings.h"
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

#include "l3cam_ros/ChangeThermalCameraColormap.h"
#include "l3cam_ros/EnableThermalCameraTemperatureFilter.h"
#include "l3cam_ros/ChangeThermalCameraTemperatureFilter.h"
#include "l3cam_ros/ChangeThermalCameraProcessingPipeline.h"
#include "l3cam_ros/EnableThermalCameraTemperatureDataUdp.h"

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

#include "l3cam_ros/GetAlliedCameraBlackLevel.h"
#include "l3cam_ros/GetAlliedCameraExposureTime.h"
#include "l3cam_ros/GetAlliedCameraAutoExposureTime.h"
#include "l3cam_ros/GetAlliedCameraAutoExposureTimeRange.h"
#include "l3cam_ros/GetAlliedCameraGain.h"
#include "l3cam_ros/GetAlliedCameraAutoGain.h"
#include "l3cam_ros/GetAlliedCameraAutoGainRange.h"
#include "l3cam_ros/GetAlliedCameraGamma.h"
#include "l3cam_ros/GetAlliedCameraSaturation.h"
#include "l3cam_ros/GetAlliedCameraSharpness.h"
#include "l3cam_ros/GetAlliedCameraHue.h"
#include "l3cam_ros/GetAlliedCameraIntensityAutoPrecedence.h"
#include "l3cam_ros/GetAlliedCameraAutoWhiteBalance.h"
#include "l3cam_ros/GetAlliedCameraBalanceRatioSelector.h"
#include "l3cam_ros/GetAlliedCameraBalanceRatio.h"
#include "l3cam_ros/GetAlliedCameraBalanceWhiteAutoRate.h"
#include "l3cam_ros/GetAlliedCameraBalanceWhiteAutoTolerance.h"
#include "l3cam_ros/GetAlliedCameraAutoModeRegion.h"
#include "l3cam_ros/GetAlliedCameraIntensityControllerRegion.h"
#include "l3cam_ros/GetAlliedCameraIntensityControllerTarget.h"
#include "l3cam_ros/GetAlliedCameraMaxDriverBuffersCount.h"

#include "l3cam_ros/SensorDisconnected.h"

namespace l3cam_ros
{
    class L3Cam : public ros::NodeHandle
    {
    public:
        explicit L3Cam();

        void spin();

        int initializeDevice();
        int startDeviceStream();
        void disconnectAll(int code);

        l3cam m_devices[1];
        LibL3CamStatus m_status;

    private:
        void initializeServices();
        void initializeLidarServices();
        void initializePolarimetricServices();
        void initializeRgbServices();
        void initializeThermalServices();
        void initializeAlliedWideServices();
        void initializeAlliedNarrowServices();

        inline void printDefaultError(int error, std::string param);

        template <typename T>
        void loadParam(const std::string &param_name, T &param_var, const T &default_val);
        void loadDefaultParams();
        void loadNetworkDefaultParams();
        void loadLidarDefaultParams();
        void loadPolarimetricDefaultParams();
        void loadRgbDefaultParams();
        void loadThermalDefaultParams();
        void loadAlliedWideDefaultParams();
        void loadAlliedNarrowDefaultParams();

        // Service callbacks
        bool libL3camStatus(l3cam_ros::LibL3camStatus::Request &req, l3cam_ros::LibL3camStatus::Response &res);
        bool getVersion(l3cam_ros::GetVersion::Request &req, l3cam_ros::GetVersion::Response &res);
        bool initialize(l3cam_ros::Initialize::Request &req, l3cam_ros::Initialize::Response &res);
        bool terminate(l3cam_ros::Terminate::Request &req, l3cam_ros::Terminate::Response &res);
        bool findDevices(l3cam_ros::FindDevices::Request &req, l3cam_ros::FindDevices::Response &res);
        bool getLocalServerAddress(l3cam_ros::GetLocalServerAddress::Request &req, l3cam_ros::GetLocalServerAddress::Response &res);
        bool getDeviceInfo(l3cam_ros::GetDeviceInfo::Request &req, l3cam_ros::GetDeviceInfo::Response &res);
        bool getDeviceStatus(l3cam_ros::GetDeviceStatus::Request &req, l3cam_ros::GetDeviceStatus::Response &res);
        bool getSensorsAvailable(l3cam_ros::GetSensorsAvailable::Request &req, l3cam_ros::GetSensorsAvailable::Response &res);
        bool changeStreamingProtocol(l3cam_ros::ChangeStreamingProtocol::Request &req, l3cam_ros::ChangeStreamingProtocol::Response &res);
        bool getRtspPipeline(l3cam_ros::GetRtspPipeline::Request &req, l3cam_ros::GetRtspPipeline::Response &res);
        bool getNetworkConfiguration(l3cam_ros::GetNetworkConfiguration::Request &req, l3cam_ros::GetNetworkConfiguration::Response &res);
        bool changeNetworkConfiguration(l3cam_ros::ChangeNetworkConfiguration::Request &req, l3cam_ros::ChangeNetworkConfiguration::Response &res);
        bool powerOffDevice(l3cam_ros::PowerOffDevice::Request &req, l3cam_ros::PowerOffDevice::Response &res);
        bool startDevice(l3cam_ros::StartDevice::Request &req, l3cam_ros::StartDevice::Response &res);
        bool stopDevice(l3cam_ros::StopDevice::Request &req, l3cam_ros::StopDevice::Response &res);
        bool startStream(l3cam_ros::StartStream::Request &req, l3cam_ros::StartStream::Response &res);
        bool stopStream(l3cam_ros::StopStream::Request &req, l3cam_ros::StopStream::Response &res);
        bool getDeviceTemperatures(l3cam_ros::GetDeviceTemperatures::Request &req, l3cam_ros::GetDeviceTemperatures::Response &res);
        bool changePointcloudColor(l3cam_ros::ChangePointcloudColor::Request &req, l3cam_ros::ChangePointcloudColor::Response &res);
        bool changePointcloudColorRange(l3cam_ros::ChangePointcloudColorRange::Request &req, l3cam_ros::ChangePointcloudColorRange::Response &res);
        bool changeDistanceRange(l3cam_ros::ChangeDistanceRange::Request &req, l3cam_ros::ChangeDistanceRange::Response &res);
        bool setBiasShortRange(l3cam_ros::SetBiasShortRange::Request &req, l3cam_ros::SetBiasShortRange::Response &res);
        bool enableAutoBias(l3cam_ros::EnableAutoBias::Request &req, l3cam_ros::EnableAutoBias::Response &res);
        bool changeBiasValue(l3cam_ros::ChangeBiasValue::Request &req, l3cam_ros::ChangeBiasValue::Response &res);
        bool changeAutobiasValue(l3cam_ros::ChangeAutobiasValue::Request &req, l3cam_ros::ChangeAutobiasValue::Response &res);
        bool getAutobiasValue(l3cam_ros::GetAutobiasValue::Request &req, l3cam_ros::GetAutobiasValue::Response &res);
        bool setPolarimetricCameraDefaultSettings(l3cam_ros::SetPolarimetricCameraDefaultSettings::Request &req, l3cam_ros::SetPolarimetricCameraDefaultSettings::Response &res);
        bool changePolarimetricCameraBrightness(l3cam_ros::ChangePolarimetricCameraBrightness::Request &req, l3cam_ros::ChangePolarimetricCameraBrightness::Response &res);
        bool changePolarimetricCameraBlackLevel(l3cam_ros::ChangePolarimetricCameraBlackLevel::Request &req, l3cam_ros::ChangePolarimetricCameraBlackLevel::Response &res);
        bool enablePolarimetricCameraAutoGain(l3cam_ros::EnablePolarimetricCameraAutoGain::Request &req, l3cam_ros::EnablePolarimetricCameraAutoGain::Response &res);
        bool changePolarimetricCameraAutoGainRange(l3cam_ros::ChangePolarimetricCameraAutoGainRange::Request &req, l3cam_ros::ChangePolarimetricCameraAutoGainRange::Response &res);
        bool changePolarimetricCameraGain(l3cam_ros::ChangePolarimetricCameraGain::Request &req, l3cam_ros::ChangePolarimetricCameraGain::Response &res);
        bool enablePolarimetricCameraAutoExposureTime(l3cam_ros::EnablePolarimetricCameraAutoExposureTime::Request &req, l3cam_ros::EnablePolarimetricCameraAutoExposureTime::Response &res);
        bool changePolarimetricCameraAutoExposureTimeRange(l3cam_ros::ChangePolarimetricCameraAutoExposureTimeRange::Request &req, l3cam_ros::ChangePolarimetricCameraAutoExposureTimeRange::Response &res);
        bool changePolarimetricCameraExposureTime(l3cam_ros::ChangePolarimetricCameraExposureTime::Request &req, l3cam_ros::ChangePolarimetricCameraExposureTime::Response &res);
        bool setRgbCameraDefaultSettings(l3cam_ros::SetRgbCameraDefaultSettings::Request &req, l3cam_ros::SetRgbCameraDefaultSettings::Response &res);
        bool changeRgbCameraBrightness(l3cam_ros::ChangeRgbCameraBrightness::Request &req, l3cam_ros::ChangeRgbCameraBrightness::Response &res);
        bool changeRgbCameraContrast(l3cam_ros::ChangeRgbCameraContrast::Request &req, l3cam_ros::ChangeRgbCameraContrast::Response &res);
        bool changeRgbCameraSaturation(l3cam_ros::ChangeRgbCameraSaturation::Request &req, l3cam_ros::ChangeRgbCameraSaturation::Response &res);
        bool changeRgbCameraSharpness(l3cam_ros::ChangeRgbCameraSharpness::Request &req, l3cam_ros::ChangeRgbCameraSharpness::Response &res);
        bool changeRgbCameraGamma(l3cam_ros::ChangeRgbCameraGamma::Request &req, l3cam_ros::ChangeRgbCameraGamma::Response &res);
        bool changeRgbCameraGain(l3cam_ros::ChangeRgbCameraGain::Request &req, l3cam_ros::ChangeRgbCameraGain::Response &res);
        bool enableRgbCameraAutoWhiteBalance(l3cam_ros::EnableRgbCameraAutoWhiteBalance::Request &req, l3cam_ros::EnableRgbCameraAutoWhiteBalance::Response &res);
        bool changeRgbCameraWhiteBalance(l3cam_ros::ChangeRgbCameraWhiteBalance::Request &req, l3cam_ros::ChangeRgbCameraWhiteBalance::Response &res);
        bool enableRgbCameraAutoExposureTime(l3cam_ros::EnableRgbCameraAutoExposureTime::Request &req, l3cam_ros::EnableRgbCameraAutoExposureTime::Response &res);
        bool changeRgbCameraExposureTime(l3cam_ros::ChangeRgbCameraExposureTime::Request &req, l3cam_ros::ChangeRgbCameraExposureTime::Response &res);
        bool changeThermalCameraColormap(l3cam_ros::ChangeThermalCameraColormap::Request &req, l3cam_ros::ChangeThermalCameraColormap::Response &res);
        bool enableThermalCameraTemperatureFilter(l3cam_ros::EnableThermalCameraTemperatureFilter::Request &req, l3cam_ros::EnableThermalCameraTemperatureFilter::Response &res);
        bool changeThermalCameraTemperatureFilter(l3cam_ros::ChangeThermalCameraTemperatureFilter::Request &req, l3cam_ros::ChangeThermalCameraTemperatureFilter::Response &res);
        bool changeThermalCameraProcessingPipeline(l3cam_ros::ChangeThermalCameraProcessingPipeline::Request &req, l3cam_ros::ChangeThermalCameraProcessingPipeline::Response &res);
        bool enableThermalCameraTemperatureDataUdp(l3cam_ros::EnableThermalCameraTemperatureDataUdp::Request &req, l3cam_ros::EnableThermalCameraTemperatureDataUdp::Response &res);
        bool changeAlliedCameraExposureTime(l3cam_ros::ChangeAlliedCameraExposureTime::Request &req, l3cam_ros::ChangeAlliedCameraExposureTime::Response &res);
        bool enableAlliedCameraAutoExposureTime(l3cam_ros::EnableAlliedCameraAutoExposureTime::Request &req, l3cam_ros::EnableAlliedCameraAutoExposureTime::Response &res);
        bool changeAlliedCameraAutoExposureTimeRange(l3cam_ros::ChangeAlliedCameraAutoExposureTimeRange::Request &req, l3cam_ros::ChangeAlliedCameraAutoExposureTimeRange::Response &res);
        bool changeAlliedCameraGain(l3cam_ros::ChangeAlliedCameraGain::Request &req, l3cam_ros::ChangeAlliedCameraGain::Response &res);
        bool enableAlliedCameraAutoGain(l3cam_ros::EnableAlliedCameraAutoGain::Request &req, l3cam_ros::EnableAlliedCameraAutoGain::Response &res);
        bool changeAlliedCameraAutoGainRange(l3cam_ros::ChangeAlliedCameraAutoGainRange::Request &req, l3cam_ros::ChangeAlliedCameraAutoGainRange::Response &res);
        bool changeAlliedCameraGamma(l3cam_ros::ChangeAlliedCameraGamma::Request &req, l3cam_ros::ChangeAlliedCameraGamma::Response &res);
        bool changeAlliedCameraSaturation(l3cam_ros::ChangeAlliedCameraSaturation::Request &req, l3cam_ros::ChangeAlliedCameraSaturation::Response &res);
        bool changeAlliedCameraHue(l3cam_ros::ChangeAlliedCameraHue::Request &req, l3cam_ros::ChangeAlliedCameraHue::Response &res);
        bool changeAlliedCameraIntensityAutoPrecedence(l3cam_ros::ChangeAlliedCameraIntensityAutoPrecedence::Request &req, l3cam_ros::ChangeAlliedCameraIntensityAutoPrecedence::Response &res);
        bool enableAlliedCameraAutoWhiteBalance(l3cam_ros::EnableAlliedCameraAutoWhiteBalance::Request &req, l3cam_ros::EnableAlliedCameraAutoWhiteBalance::Response &res);
        bool changeAlliedCameraBalanceRatioSelector(l3cam_ros::ChangeAlliedCameraBalanceRatioSelector::Request &req, l3cam_ros::ChangeAlliedCameraBalanceRatioSelector::Response &res);
        bool changeAlliedCameraBalanceRatio(l3cam_ros::ChangeAlliedCameraBalanceRatio::Request &req, l3cam_ros::ChangeAlliedCameraBalanceRatio::Response &res);
        bool changeAlliedCameraBalanceWhiteAutoRate(l3cam_ros::ChangeAlliedCameraBalanceWhiteAutoRate::Request &req, l3cam_ros::ChangeAlliedCameraBalanceWhiteAutoRate::Response &res);
        bool changeAlliedCameraBalanceWhiteAutoTolerance(l3cam_ros::ChangeAlliedCameraBalanceWhiteAutoTolerance::Request &req, l3cam_ros::ChangeAlliedCameraBalanceWhiteAutoTolerance::Response &res);
        bool changeAlliedCameraIntensityControllerRegion(l3cam_ros::ChangeAlliedCameraIntensityControllerRegion::Request &req, l3cam_ros::ChangeAlliedCameraIntensityControllerRegion::Response &res);
        bool changeAlliedCameraIntensityControllerTarget(l3cam_ros::ChangeAlliedCameraIntensityControllerTarget::Request &req, l3cam_ros::ChangeAlliedCameraIntensityControllerTarget::Response &res);
        bool getAlliedCameraBlackLevel(l3cam_ros::GetAlliedCameraBlackLevel::Request &req, l3cam_ros::GetAlliedCameraBlackLevel::Response &res);
        bool getAlliedCameraExposureTime(l3cam_ros::GetAlliedCameraExposureTime::Request &req, l3cam_ros::GetAlliedCameraExposureTime::Response &res);
        bool getAlliedCameraAutoExposureTime(l3cam_ros::GetAlliedCameraAutoExposureTime::Request &req, l3cam_ros::GetAlliedCameraAutoExposureTime::Response &res);
        bool getAlliedCameraAutoExposureTimeRange(l3cam_ros::GetAlliedCameraAutoExposureTimeRange::Request &req, l3cam_ros::GetAlliedCameraAutoExposureTimeRange::Response &res);
        bool getAlliedCameraGain(l3cam_ros::GetAlliedCameraGain::Request &req, l3cam_ros::GetAlliedCameraGain::Response &res);
        bool getAlliedCameraAutoGain(l3cam_ros::GetAlliedCameraAutoGain::Request &req, l3cam_ros::GetAlliedCameraAutoGain::Response &res);
        bool getAlliedCameraAutoGainRange(l3cam_ros::GetAlliedCameraAutoGainRange::Request &req, l3cam_ros::GetAlliedCameraAutoGainRange::Response &res);
        bool getAlliedCameraGamma(l3cam_ros::GetAlliedCameraGamma::Request &req, l3cam_ros::GetAlliedCameraGamma::Response &res);
        bool getAlliedCameraSaturation(l3cam_ros::GetAlliedCameraSaturation::Request &req, l3cam_ros::GetAlliedCameraSaturation::Response &res);
        bool getAlliedCameraSharpness(l3cam_ros::GetAlliedCameraSharpness::Request &req, l3cam_ros::GetAlliedCameraSharpness::Response &res);
        bool getAlliedCameraHue(l3cam_ros::GetAlliedCameraHue::Request &req, l3cam_ros::GetAlliedCameraHue::Response &res);
        bool getAlliedCameraIntensityAutoPrecedence(l3cam_ros::GetAlliedCameraIntensityAutoPrecedence::Request &req, l3cam_ros::GetAlliedCameraIntensityAutoPrecedence::Response &res);
        bool getAlliedCameraAutoWhiteBalance(l3cam_ros::GetAlliedCameraAutoWhiteBalance::Request &req, l3cam_ros::GetAlliedCameraAutoWhiteBalance::Response &res);
        bool getAlliedCameraBalanceRatioSelector(l3cam_ros::GetAlliedCameraBalanceRatioSelector::Request &req, l3cam_ros::GetAlliedCameraBalanceRatioSelector::Response &res);
        bool getAlliedCameraBalanceRatio(l3cam_ros::GetAlliedCameraBalanceRatio::Request &req, l3cam_ros::GetAlliedCameraBalanceRatio::Response &res);
        bool getAlliedCameraBalanceWhiteAutoRate(l3cam_ros::GetAlliedCameraBalanceWhiteAutoRate::Request &req, l3cam_ros::GetAlliedCameraBalanceWhiteAutoRate::Response &res);
        bool getAlliedCameraBalanceWhiteAutoTolerance(l3cam_ros::GetAlliedCameraBalanceWhiteAutoTolerance::Request &req, l3cam_ros::GetAlliedCameraBalanceWhiteAutoTolerance::Response &res);
        bool getAlliedCameraAutoModeRegion(l3cam_ros::GetAlliedCameraAutoModeRegion::Request &req, l3cam_ros::GetAlliedCameraAutoModeRegion::Response &res);
        bool getAlliedCameraIntensityControllerRegion(l3cam_ros::GetAlliedCameraIntensityControllerRegion::Request &req, l3cam_ros::GetAlliedCameraIntensityControllerRegion::Response &res);
        bool getAlliedCameraIntensityControllerTarget(l3cam_ros::GetAlliedCameraIntensityControllerTarget::Request &req, l3cam_ros::GetAlliedCameraIntensityControllerTarget::Response &res);
        bool getAlliedCameraMaxDriverBuffersCount(l3cam_ros::GetAlliedCameraMaxDriverBuffersCount::Request &req, l3cam_ros::GetAlliedCameraMaxDriverBuffersCount::Response &res);

        void networkDisconnected(int code);
        void lidarDisconnected(int code);
        void polDisconnected(int code);
        void rgbDisconnected(int code);
        void thermalDisconnected(int code);
        void alliedwideDisconnected(int code);
        void alliedNarrowDisconnect(int code);

        static void errorNotification(const int32_t *error);

        ros::ServiceServer srv_libl3cam_status_;
        ros::ServiceServer srv_get_version_;
        ros::ServiceServer srv_initialize_;
        ros::ServiceServer srv_terminate_;
        ros::ServiceServer srv_find_devices_;
        ros::ServiceServer srv_get_local_server_address_;
        ros::ServiceServer srv_get_device_info_;
        ros::ServiceServer srv_get_device_status_;
        ros::ServiceServer srv_get_sensors_available_;
        ros::ServiceServer srv_change_streaming_protocol_;
        ros::ServiceServer srv_get_rtsp_pipeline_;
        ros::ServiceServer srv_get_network_configuration_;
        ros::ServiceServer srv_change_network_configuration_;
        ros::ServiceServer srv_power_off_device_;
        ros::ServiceServer srv_start_device_;
        ros::ServiceServer srv_stop_device_;
        ros::ServiceServer srv_start_stream_;
        ros::ServiceServer srv_stop_stream_;
        ros::ServiceServer srv_get_device_temperatures_;

        ros::ServiceServer srv_change_pointcloud_color_;
        ros::ServiceServer srv_change_pointcloud_color_range_;
        ros::ServiceServer srv_change_distance_range_;
        ros::ServiceServer srv_set_bias_short_range_;
        ros::ServiceServer srv_enable_auto_bias_;
        ros::ServiceServer srv_change_bias_value_;
        ros::ServiceServer srv_change_autobias_value_;
        ros::ServiceServer srv_get_autobias_value_;

        ros::ServiceServer srv_set_polarimetric_default_settings_;
        ros::ServiceServer srv_change_polarimetric_brightness_;
        ros::ServiceServer srv_change_polarimetric_black_level_;
        ros::ServiceServer srv_enable_polarimetric_auto_gain_;
        ros::ServiceServer srv_change_polarimetric_auto_gain_range_;
        ros::ServiceServer srv_change_polarimetric_gain_;
        ros::ServiceServer srv_enable_polarimetric_auto_exposure_time_;
        ros::ServiceServer srv_change_polarimetric_auto_exposure_time_range_;
        ros::ServiceServer srv_change_polarimetric_exposure_time_;

        ros::ServiceServer srv_set_rgb_default_settings_;
        ros::ServiceServer srv_change_rgb_brightness_;
        ros::ServiceServer srv_change_rgb_contrast_;
        ros::ServiceServer srv_change_rgb_saturation_;
        ros::ServiceServer srv_change_rgb_sharpness_;
        ros::ServiceServer srv_change_rgb_gamma_;
        ros::ServiceServer srv_change_rgb_gain_;
        ros::ServiceServer srv_enable_rgb_auto_white_balance_;
        ros::ServiceServer srv_change_rgb_white_balance_;
        ros::ServiceServer srv_enable_rgb_auto_exposure_time_;
        ros::ServiceServer srv_change_rgb_exposure_time_;

        ros::ServiceServer srv_change_thermal_colormap_;
        ros::ServiceServer srv_enable_thermal_temperature_filter_;
        ros::ServiceServer srv_change_thermal_temperature_filter_;
        ros::ServiceServer srv_change_thermal_processing_pipeline_;
        ros::ServiceServer srv_enable_thermal_temperature_data_udp_;

        ros::ServiceServer srv_change_allied_exposure_time_;
        ros::ServiceServer srv_enable_allied_auto_exposure_time_;
        ros::ServiceServer srv_change_allied_auto_exposure_time_range_;
        ros::ServiceServer srv_change_allied_gain_;
        ros::ServiceServer srv_enable_allied_auto_gain_;
        ros::ServiceServer srv_change_allied_auto_gain_range_;
        ros::ServiceServer srv_change_allied_gamma_;
        ros::ServiceServer srv_change_allied_saturation_;
        ros::ServiceServer srv_change_allied_hue_;
        ros::ServiceServer srv_change_allied_intensity_auto_precedence_;
        ros::ServiceServer srv_enable_allied_auto_white_balance_;
        ros::ServiceServer srv_change_allied_balance_ratio_selector_;
        ros::ServiceServer srv_change_allied_balance_ratio_;
        ros::ServiceServer srv_change_allied_balance_white_auto_rate_;
        ros::ServiceServer srv_change_allied_balance_white_auto_tolerance_;
        ros::ServiceServer srv_change_allied_intensity_controller_region_;
        ros::ServiceServer srv_change_allied_intensity_controller_target_;

        ros::ServiceServer srv_get_allied_black_level_;
        ros::ServiceServer srv_get_allied_exposure_time_;
        ros::ServiceServer srv_get_allied_auto_exposure_time_;
        ros::ServiceServer srv_get_allied_auto_exposure_time_range_;
        ros::ServiceServer srv_get_allied_gain_;
        ros::ServiceServer srv_get_allied_auto_gain_;
        ros::ServiceServer srv_get_allied_auto_gain_range_;
        ros::ServiceServer srv_get_allied_gamma_;
        ros::ServiceServer srv_get_allied_saturation_;
        ros::ServiceServer srv_get_allied_sharpness_;
        ros::ServiceServer srv_get_allied_hue_;
        ros::ServiceServer srv_get_allied_intensity_auto_precedence_;
        ros::ServiceServer srv_get_allied_auto_white_balance_;
        ros::ServiceServer srv_get_allied_balance_ratio_selector_;
        ros::ServiceServer srv_get_allied_balance_ratio_;
        ros::ServiceServer srv_get_allied_balance_white_auto_rate_;
        ros::ServiceServer srv_get_allied_balance_white_auto_tolerance_;
        ros::ServiceServer srv_get_allied_auto_mode_region_;
        ros::ServiceServer srv_get_allied_intensity_controller_region_;
        ros::ServiceServer srv_get_allied_intensity_controller_target_;
        ros::ServiceServer srv_get_allied_max_driver_buffers_count_;

        ros::ServiceClient client_network_disconnected_;
        l3cam_ros::SensorDisconnected srv_network_disconnected_;
        ros::ServiceClient client_lidar_stream_disconnected_;
        l3cam_ros::SensorDisconnected srv_pointcloud_stream_disconnected_;
        ros::ServiceClient client_lidar_configuration_disconnected_;
        l3cam_ros::SensorDisconnected srv_pointcloud_configuration_disconnected_;
        ros::ServiceClient client_pol_wide_stream_disconnected_;
        l3cam_ros::SensorDisconnected srv_pol_wide_stream_disconnected_;
        ros::ServiceClient client_pol_configuration_disconnected_;
        l3cam_ros::SensorDisconnected srv_pol_configuration_disconnected_;
        ros::ServiceClient client_rgb_narrow_stream_disconnected_;
        l3cam_ros::SensorDisconnected srv_rgb_narrow_stream_disconnected_;
        ros::ServiceClient client_rgb_configuration_disconnected_;
        l3cam_ros::SensorDisconnected srv_rgb_configuration_disconnected_;
        ros::ServiceClient client_thermal_stream_disconnected_;
        l3cam_ros::SensorDisconnected srv_thermal_stream_disconnected_;
        ros::ServiceClient client_thermal_configuration_disconnected_;
        l3cam_ros::SensorDisconnected srv_thermal_configuration_disconnected_;
        ros::ServiceClient client_wide_configuration_disconnected_;
        l3cam_ros::SensorDisconnected srv_wide_configuration_disconnected_;
        ros::ServiceClient client_narrow_configuration_disconnected_;
        l3cam_ros::SensorDisconnected srv_narrow_configuration_disconnected_;

        sensor m_av_sensors[6];

        sensor *m_lidar_sensor = NULL;
        sensor *m_rgb_sensor = NULL;
        sensor *m_thermal_sensor = NULL;
        sensor *m_polarimetric_sensor = NULL;
        sensor *m_allied_wide_sensor = NULL;
        sensor *m_allied_narrow_sensor = NULL;

        int m_num_devices;
        int timeout_secs_;

        bool m_shutdown_requested;

    }; // class L3Cam

} // namespace l3cam_ros