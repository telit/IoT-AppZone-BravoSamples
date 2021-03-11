/*===============================================================================================*/
/*         >>> Copyright (C) Telit Communications S.p.A. Italy All Rights Reserved. <<<          */
/*!
  @file
    sensors_demo.h

  @brief
    <brief description>
  @details
    <detailed description>
  @note
    Dependencies:
    m2mb_types.h

  @author
		FabioPi
  @date
    01/12/2020
 */

/*=================================================================
#Telit Extensions
#
#Copyright (C) 2019, Telit Communications S.p.A.
#All rights reserved.
#
#Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#
#Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#
#Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in
#the documentation and/or other materials provided with the distribution.
#
#
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS``AS IS'' AND ANY EXPRESS OR IMPLIED
#WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
#PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
#DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
#HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
#NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#POSSIBILITY OF SUCH DAMAGE.
#
==============================================================*/


#ifndef HDR_SENSORS_DEMO_H_
#define HDR_SENSORS_DEMO_H_

/* Global declarations ==========================================================================*/
#define INT_GPIO_PIN_NUM 6
#define LED_PIN_NUM 10

#define SENSOR_AR_TOUT  100  // 10 = 1 sec

#define TAMPER_XML_NAME "object_26242.xml"
#define ROTATION_XML_NAME "object_26250.xml"
#define ENVIRONMENT_XML_NAME "object_26251.xml"



#define FIFO_SIZE                      300
#define MAX_PACKET_LENGTH              18
#define TICKS_IN_ONE_SECOND            32000.0F

#define ROTATION_VECTOR_SAMPLE_RATE    10  //Hz


/*BME*/

#define BME680_W_SELF_TEST_FAILED 3

#define MIN_TEMPERATURE INT16_C(0)    /* 0 degree Celsius */
#define MAX_TEMPERATURE INT16_C(6000)   /* 60 degree Celsius */

#define MIN_PRESSURE UINT32_C(90000)  /* 900 hecto Pascals */
#define MAX_PRESSURE UINT32_C(110000)   /* 1100 hecto Pascals */

#define MIN_HUMIDITY UINT32_C(20000)  /* 20% relative humidity */
#define MAX_HUMIDITY UINT32_C(80000)  /* 80% relative humidity*/

#define HEATR_DUR 2000
#define N_MEAS    6
#define LOW_TEMP  150
#define HIGH_TEMP   350


/* Global typedefs ==============================================================================*/
typedef enum
{
  BSENS_SENSOR_ENVIRONM_ID = 1,
  BSENS_SENSOR_3D_VECT_ID = 2,
  BSENS_SENSOR_TAMPER_ID = 3
} BSENS_SENSOR_ID_E;


typedef struct
{
  float temperature;
  float pressure;
  float humidity;
  int airQ;
} BSENS_ENV_T;


typedef struct
{
  float w;
  float x;
  float y;
  float z;
  INT16 accuracy;
} BSENS_3DVECT_T;


typedef struct
{
  INT16 status;
  UINT32 timestamp;
} BSENS_TAMPER_T;



typedef enum
{
  STATUS_INVALID = -1,
  STILL_ENDED = 0,
  WALKING_ENDED = 1,
  RUNNING_ENDED = 2,
  BYCYCLE_ENDED = 3,
  VEHICLE_ENDED = 4,
  TILTING_ENDED = 5,
  STILL_STARTED = 8,
  WALKING_STARTED = 9,
  RUNNING_STARTED = 10,
  BICYCLE_STARTED = 11,
  VEHICLE_STARTED = 12,
  TILTING_STARTED = 13,

} BHI_TAMPER_STATUS_E;

/* Global functions =============================================================================*/
/**
   @brief This function is used to run bhy hub
*/
int init_sensors( void );
int read_sensor(BSENS_SENSOR_ID_E id, void **data);

#endif /* HDR_SENSORS_DEMO_H_ */
