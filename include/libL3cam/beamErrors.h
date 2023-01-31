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

/*
LIST OF BEAMAGINE ERRORS
*/

#define L3CAM_OK    0

//!INTERNAL ERRORS
#define L3CAM_NO_SENSORS_AVAILABLE              11
#define L3CAM_INVALID_PARAMETERS                12

#define L3CAM_TIMEOUT_ERROR                     210
#define L3CAM_ERROR_OUT_OF_MEMORY               211

#define L3CAM_UNDEFINED_LASER_CLASS             220
#define L3CAM_UNDEFINED_SENSOR                  222
#define L3CAM_VALUE_OUT_OF_RANGE                223
#define L3CAM_SENSOR_NOT_AVAILABLE              224

#define L3CAM_ERROR_CREATING_TCP_CLIENT_SOCKET  230
#define L3CAM_ERROR_INITALIZING_WSA             231
#define L3CAM_ERROR_CONNECTING_WITH_TCP_SERVER  232
#define L3CAM_ERROR_SENDING_TCP_MESSAGE         233
#define L3CAM_ERROR_CREATING_TCP_SERVER_SOCKET  234
#define L3CAM_ERROR_BINDING_SOCKET              235
#define L3CAM_ERROR_STARTING_TCP_SERVER         236
#define L3CAM_ERROR_ACCEPTING_TCP_CLIENT        237

//!LIDAR ERRORS

//!ECON CAMERA ERRORS
#define ERROR_OPENING_ECON_CAMERA               20
#define ERROR_SETTING_ECON_CAMERA_PARAMETER     21
#define ERROR_SETTING_ECON_DEFAULT              22


//!POLARIMETRIC ERRORS
#define ERROR_POL_STD_EXCEPTION             30
#define ERROR_POL_RUNTIME_EXCEPTION         31
#define ERROR_POL_GENERIC_EXCEPTION         32
#define ERROR_POL_TIMEOUT_EXCEPTION         33
#define ERROR_POL_UNDEFINED_ERROR           34
#define ERROR_POL_PERCEPTION_EXCEPTION      35

//!THERMAL ERRORS
#define ERROR_OPENING_THERMAL_CAMERA    40
#define ERROR_THERMAL_IMAGE_SIZE        41
#define ERROR_CLOSING_THERMAL_CAMERA    42
#define ERROR_SETTING_THERMAL_LUT       43
#define ERROR_GETTING_THERMAL_LUT       44
#define ERROR_SETTING_THERMAL_SHARPEN   45
#define ERROR_SETTING_THERMAL_SMOOTH    46
#define ERROR_GETTING_THERMAL_SYSTEM    47
#define ERROR_SETTING_THERMAL_SYSTEM    48


