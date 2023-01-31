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

//==============================================================================
//
// Title:		libL3Cam
// Purpose:		Interface of the library for communications with L3CAM devices
//
// Created on:	22/11/2021 at 12:01:11 by Beamagine.
// Copyright:	. All Rights Reserved.
//
//==============================================================================

#ifndef __libL3Cam_H__
#define __libL3Cam_H__

#ifdef __cplusplus
    extern "C" {
#endif

//==============================================================================
// Include files
#ifdef _WIN32
#include "cvidef.h"
#endif
#include "beamagine.h"
#include "beamErrors.h"

//! @brief  Returns the error description for the error specified
//! @param  error_code Integer with the error received
//! @return pointer with the error description
const char* getBeamErrorDescription(int error_code);

//! @brief  Returns the library version
//! @param  none
//! @return pointer with the library version
const char* GET_VERSION();

//! @brief  Initializes the library and internal communication system
//! @param  none
//! @return 0 if OK otherwise Error, check error definitions
int INITIALIZE();

//! @brief  Closes the library communications, call this function to properly relase resources
//! @param  none
//! @return 0 if OK otherwise Error.
int TERMINATE(l3cam device);

//! @brief  Search for L3CAM devices in the network, this function is blocking call
//! @param  devices[] Array where L3CAM device information is stored
//! @param  num_devices Pointer where the number of devices found is returned
//! @return 0 if OK otherwise Error, check error definition
int FIND_DEVICES(l3cam devices[], int *num_devices);

//! @brief  Returns the local IP address of the NIC where the L3CAM is connected
//! @param  none
//! @return Pointer with the IP address
const char *GET_LOCAL_SERVER_ADDRESS(l3cam device);

//! @brief  Gets the device internal status
//! @param  device The device to execute the function
//! @param  status Pointer where the satatus is saved, check satus definition
//! @return 0 if OK otherwise Error, check error definition
int GET_DEVICE_STATUS(l3cam device, int32_t *system_status);

//! @brief  Gets the available sensors in the L3CAM device
//! @param  device The device to execute the function
//! @param  sensors Array where the sensors information is stored
//! @param  num_sensors Pointer where the number of sensors is returned
//! @return 0 if OK otherwise Error, check error definition
int GET_SENSORS_AVAILABLE(l3cam device, sensor sensors[], int *num_sensors);

//! @brief  Changes the streaming protocol of a desired sensor
//! @param  device The device to execute the function
//! @param  sensor Pointer with the sensor to change the protocol
//! @return 0 if OK otherwise Error, check error definition
int CHANGE_STREAMING_PROTOCOL(l3cam device, sensor *sensor_id);

//! @brief  Returns the RTSP pipeline for a specific sensor
//! @param  device The device to execute the function
//! @param  sensor Pointer with the sensor to retrieve the RTSP pipeline
//! @param  pipeline Pointer to store the pipeline
//! @return 0 if OK otherwise Error, check error definition
int GET_RTSP_PIPELINE(l3cam device, sensor sensor_id, char **pipeline); 

//! @brief  Changes the RTSP pipeline for the specified sensor, the pipeline is not stored 
//! @param  device The device to execute the function
//! @param  sensor Pointer with the sensor to change the RTSP pipeline
//! @param  pipeline Pointer with the new pipeline
//! @return 0 if OK otherwise Error, check error definition
int CHANGE_RTSP_PIPELINE(l3cam device, sensor sensor_id, char *pipeline);

//! @brief  Returns the network configuration of the device
//! @param  device The device to execute the function
//! @param  ip_address pointer to store the device ip address
//! @param  netmask pointer to store the device ip address netmask
//! @param  gateway pointer to store the device gateway address
//! @return 0 if OK otherwise Error, check error definition
int GET_NETWORK_CONFIGURATION(l3cam device, char **ip_address, char **netmask, char **gateway);

//! @brief  Changes the network configuration of the device, this change is permanent
//! @param  device The device to execute the function
//! @param  ip_address pointer with the new device ip address
//! @param  netmask pointer with the new device ip address netmask
//! @param  gateway pointer with the device gateway address
//! @param  enable_dhcp Boolean that enables or disabes the DHCP configuration, if false proper network configuration is required
//! @return 0 if OK otherwise Error, check error definition
int CHANGE_NETWORK_CONFIGURATION(l3cam device, char *ip_address, char *netmask, char *gateway, bool enable_dhcp);

//! @brief  Power offs the device, use this function to properly shut down the device
//! @param  device The device to execute the function
//! @return none
void POWER_OFF_DEVICE(l3cam device);

//! @brief  Starts the device, initializes internal sensors
//! @param  device The device to execute the function
//! @return 0 if OK otherwise Error
int START_DEVICE(l3cam device);

//! @brief  Stops the device, stops internal sensors and threads
//! @param  device The device to execute the function
//! @return 0 if OK otherwise Error, check error definition
int STOP_DEVICE(l3cam device);

//! @brief  Starts the streaming threads and functionality in the L3CAM device
//! @param  device The device to execute the function
//! @return 0 if OK otherwise Error, check error definition
int START_STREAM(l3cam device);

//! @brief  Stops the streaming threads and functionality in the L3CAM device
//! @param  device The device to execute the function
//! @return 0 if OK otherwise Error, check error definition
int STOP_STREAM(l3cam device);

//! @brief  Changes the laser class, this changes the emited laser power
//! @param  device The device to execute the function
//! @param  laser_class Integer that indicates the desired laser class
//! @return 0 if OK otherwise Error, check error definition
int CHANGE_LASER_CLASS(l3cam device, int laser_class);

//! @brief  Changes the pointcloud color representation
//! @param  device The device to execute the function
//! @param  visualization_color the desired color visualization, check color types definition
//! @return 0 if OK otherwise Error, check error definition
int CHANGE_POINT_CLOUD_COLOR(l3cam device, int visualization_color);

//! @brief  Changes the color ranges for the pointcloud representation
//! @param  device The device to execute the function
//! @param  max_value Max distance/intensity value for color representation
//! @param  min_value Min distance/intensity value for color representation
//! @return 0 if OK otherwise Error, check error definition
int CHANGE_POINT_CLOUD_COLOR_RANGE(l3cam device, int min_value, int max_value);

//! @brief  Changes the range of LiDAR data
//! @param  device The device to execute the function
//! @param  min_distance The minimum distance of the LiDAR data
//! @param  max_distance The maximum distance of the LiDAR data
//! @return 0 if OK otherwise Error, check error definition
int CHANGE_DISTANCE_RANGE(l3cam device, int min_distance, int max_distance);

//! @brief  Restarts de configuration values of the polarimetric cameras
//! @param  device The device to execute the function
//! @return 0 if OK otherwise Error, check error definition
int SET_POLARIMETRIC_CAMERA_DEFAULT_SETTINGS(l3cam device);

//! @brief  Changes the brightness of the polarimetric camera
//! @param  device The device to execute the function
//! @param  brightness Value to set in the camera 0 to 255
//! @return 0 if OK otherwise Error, check error definition
int CHANGE_POLARIMETRIC_CAMERA_BRIGHTNESS(l3cam device, int brightness);

//! @brief  Changes the black level of the polarimetric camera
//! @param  device The device to execute the function
//! @param  black_level Value to set in the camera to 0 to 12.5
//! @return 0 if OK otherwise Error, check error definition
int CHANGE_POLARIMETRIC_CAMERA_BLACK_LEVEL(l3cam device, float black_level);

//! @brief  Changes the gain of the polarimetric camera
//! @param  device The device to execute the function
//! @param  gain_value Gain to set in the camera 0-48.0 db
//! @return 0 if OK otherwise Error, check error definition
int CHANGE_POLARIMETRIC_CAMERA_GAIN(l3cam device, float gain_value);

//! @brief  Enables/disables the automatic gain of the polarimetric camera
//! @param  device The device to execute the function
//! @param  enabled boolean to enable/disable the function
//! @return 0 if OK otherwise Error, check error definition
int ENABLE_POLARIMETRIC_CAMERA_AUTO_GAIN(l3cam device, bool enabled);

//! @brief  Changes the gain range of the auto gain parameter of the polarimetric camera
//! @param  device The device to execute the function
//! @param  min_gain Value to set as the minimum gain value 0 to 48.0
//! @param  max_gain Value to set as the maximum gain value 0 to 48.0
//! @return 0 if OK otherwise Error, check error definition
int CHANGE_POLARIMETRIC_CAMERA_AUTO_GAIN_RANGE(l3cam device, float min_gain, float max_gain);

//! @brief  Changes the exposure time of the polarimetric camera
//! @param  device The device to execute the function
//! @param  exposure_time Value in usecs to set the exposure time 33.456us to 1000000.0us
//! @return 0 if OK otherwise Error, check error definition
int CHANGE_POLARIMETRIC_CAMERA_EXPOSURE_TIME(l3cam device, float exposure_time);

//! @brief  Enables/disables the automatic exposure time of the polarimetric camera
//! @param  device The device to execute the function
//! @param  enabled boolean to enable/disable the function
//! @return 0 if OK otherwise Error, check error definition
int ENABLE_POLARIMETRIC_CAMERA_AUTO_EXPOSURE_TIME(l3cam device, bool enabled);

//! @brief  Changes the exposure time range of the auto exposure time parameter of the polarimetric camera
//! @param  device The device to execute the function
//! @param  min_exposure Value to set as the minimum exposure time value 33.456us to 1000000.0us
//! @param  max_exposure Value to set as the maximum exposure time value 33.456us to 1000000.0us
//! @return 0 if OK otherwise Error, check error definition
int CHANGE_POLARIMETRIC_CAMERA_AUTO_EXPOSURE_TIME_RANGE(l3cam device, float min_exposure, float max_exposure);

//! @brief  Restarts de configuration values of the RGB camera
//! @param  device The device to execute the function
//! @return 0 if OK otherwise Error, check error definition
int SET_RGB_CAMERA_DEFAULT_SETTINGS(l3cam device);

//! @brief  Changes the brightness of the RGB camera sensor
//! @param  device The device to execute the function
//! @param  brightness The value of the brightness ECON -15 to 15
//! @return 0 if OK otherwise Error, check error definition
int CHANGE_RGB_CAMERA_BRIGHTNESS(l3cam device, int brightness);

//! @brief  Changes the contrast of the RGB camera sensor
//! @param  device The device to execute the function
//! @param  constrast The value of the contrast ECON 0 to 30
//! @return 0 if OK otherwise Error, check error definition
int CHANGE_RGB_CAMERA_CONTRAST(l3cam device, int contrast);

//! @brief  Changes the saturation of the RGB camera sensor
//! @param  device The device to execute the function
//! @param  saturation The value of the saturation ECON 0 to 60
//! @return 0 if OK otherwise Error, check error definition
int CHANGE_RGB_CAMERA_SATURATION(l3cam device, int saturation);

//! @brief  Changes the sharpness of the RGB camera sensor
//! @param  device The device to execute the function
//! @param  sharpness The value of the sharpness ECON 0 to 127
//! @return 0 if OK otherwise Error, check error definition
int CHANGE_RGB_CAMERA_SHARPNESS(l3cam device, int sharpness);

//! @brief  Changes the gamma of the RGB camera sensor
//! @param  device The device to execute the function
//! @param  gamma The value of the gamam ECON 40 to 500
//! @return 0 if OK otherwise Error, check error definition
int CHANGE_RGB_CAMERA_GAMMA(l3cam device, int gamma);

//! @brief  Changes the gain of the RGB camera sensor
//! @param  device The device to execute the function
//! @param  gain The value of the gain ECON 0 to 63
//! @return 0 if OK otherwise Error, check error definition
int CHANGE_RGB_CAMERA_GAIN(l3cam device, int gain);

//! @brief  Changes the white balance of the RGB camera sensor
//! @param  device The device to execute the function
//! @param  white_balance The value of the white balance ECON 1000 to 10000
//! @return 0 if OK otherwise Error, check error definition
int CHANGE_RGB_CAMERA_WHITE_BALANCE(l3cam device, int white_balance);

//! @brief  Changes the exposure time of the RGB camera sensor
//! @param  device The device to execute the function
//! @param  exposure_time The value of the exposure time ECON 1 to 10000
//! @return 0 if OK otherwise Error, check error definition
int CHANGE_RGB_CAMERA_EXPOSURE_TIME(l3cam device, int exposure_time);

//! @brief  Enables/disables the automatic white balance of the RGB camera sensor
//! @param  device The device to execute the function
//! @param  enabled boolean to enabe/disable the camera function
//! @return 0 if OK otherwise Error, check error definition
int ENABLE_RGB_CAMERA_AUTO_WHITE_BALANCE(l3cam device, bool enabled);

//! @brief  Enables/disables the automatic exposure time of the RGB camera sensor
//! @param  device The device to execute the function
//! @param  enabled boolean to enabe/disable the camera function
//! @return 0 if OK otherwise Error, check error definition
int ENABLE_RGB_CAMERA_AUTO_EXPOSURE_TIME(l3cam device, bool enabled);

//! @brief  Changes the resolution of the RGB camera sensor image
//! @param  device The device to execute the function
//! @param  resolution The desired resolution see beamagine.h
//! @return 0 if OK otherwise Error, check error definition
int CHANGE_RGB_CAMERA_RESOLUTION(l3cam device, econResolutions resolution);

//! @brief  Changes the framerate of the RGB camera sensor
//! @param  device The device to execute the function
//! @param  framerate Integer that indicates the framerate 1fps - 16fps
//! @return 0 if OK otherwise Error, check error definition
int CHANGE_RGB_CAMERA_FRAMERATE(l3cam device, int framerate);


//! @brief  Changes the colormap of the thermal image
//! @param  device The device to execute the function
//! @param  colormap The desired colormap
//! @return 0 if OK otherwise Error, check error definition
int CHANGE_THERMAL_CAMERA_COLORMAP(l3cam device, thermalTypes colormap);

//! @brief  Enables/disables the temperature filter of the thermal image
//! @param  device The device to execute the function
//! @param  enabled Boolean to enable or disable the filter
//! @return 0 if OK otherwise Error, check error definition
int ENABLE_THERMAL_CAMERA_TEMPERATURE_FILTER(l3cam device, bool enabled);

//! @brief  Changes the thermal temperature filter values
//! @param  device The device to execute the function
//! @param  min_temperature Min value in ºC to show -40ºC to 200ºC
//! @param  max_temperature Max value in ºC to show -40ºC to 200ºC
//! @return 0 if OK otherwise Error, check error definition
int CHANGE_THERMAL_CAMERA_TEMPERATURE_FILTER(l3cam device, float min_temperature, float max_temperature);


#ifdef __cplusplus
    }
#endif

#endif  /* ndef __libL3Cam_H__ */
