/*!
  * Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
  * 
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions are met:
  * 
  * Redistributions of source code must retain the above copyright
  * notice, this list of conditions and the following disclaimer.
  * 
  * Redistributions in binary form must reproduce the above copyright
  * notice, this list of conditions and the following disclaimer in the
  * documentation and/or other materials provided with the distribution.
  * 
  * Neither the name of the copyright holder nor the names of the
  * contributors may be used to endorse or promote products derived from
  * this software without specific prior written permission.
  * 
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
  * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
  * OR CONTRIBUTORS BE LIABLE FOR ANY
  * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
  * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
  * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
  * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  * ANY WAY OUT OF THE USE OF THIS
  * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
  * 
  * The information provided is believed to be accurate and reliable.
  * The copyright holder assumes no responsibility
  * for the consequences of use
  * of such information nor for any infringement of patents or
  * other rights of third parties which may result from its use.
  * No license is granted by implication or otherwise under any patent or
  * patent rights of the copyright holder.
  *
  * @file          bhy_uc_driver_constants.h
  *
  * @date          12/15/2016
  *
  * @brief         header file of bhy_uc_driver.c
  *
  */


#ifndef BHY_UC_DRIVER_CONSTANTS_H_
#define BHY_UC_DRIVER_CONSTANTS_H_
#ifdef __cplusplus
extern "C"{
#endif
#include "bhy_uc_driver_config.h"

/****************************************************************************/
/*                      Constants definitions                               */
/****************************************************************************/

/* definition of all the known IDs. User can add their own IDs to the       */
/* bhy_parse_next_fifo_packet function. follow section 15 table 28 of the   */
/* BHI160 datasheet.                                                        */
#define VS_ID_PADDING                           0
#define VS_ID_ACCELEROMETER                     1
#define VS_ID_MAGNETOMETER                      2
#define VS_ID_ORIENTATION                       3
#define VS_ID_GYROSCOPE                         4
#define VS_ID_LIGHT                             5
#define VS_ID_BAROMETER                         6
#define VS_ID_TEMPERATURE                       7
#define VS_ID_PROXIMITY                         8
#define VS_ID_GRAVITY                           9
#define VS_ID_LINEAR_ACCELERATION               10
#define VS_ID_ROTATION_VECTOR                   11
#define VS_ID_HUMIDITY                          12
#define VS_ID_AMBIENT_TEMPERATURE               13
#define VS_ID_UNCALIBRATED_MAGNETOMETER         14
#define VS_ID_GAME_ROTATION_VECTOR              15
#define VS_ID_UNCALIBRATED_GYROSCOPE            16
#define VS_ID_SIGNIFICANT_MOTION                17
#define VS_ID_STEP_DETECTOR                     18
#define VS_ID_STEP_COUNTER                      19
#define VS_ID_GEOMAGNETIC_ROTATION_VECTOR       20
#define VS_ID_HEART_RATE                        21
#define VS_ID_TILT_DETECTOR                     22
#define VS_ID_WAKE_GESTURE                      23
#define VS_ID_GLANCE_GESTURE                    24
#define VS_ID_PICKUP_GESTURE                    25
#define VS_ID_CUS1                              26
#define VS_ID_CUS2                              27
#define VS_ID_CUS3                              28
#define VS_ID_CUS4                              29
#define VS_ID_CUS5                              30
#define VS_ID_ACTIVITY                          31

#define VS_ID_ACCELEROMETER_WAKEUP                  (VS_ID_ACCELEROMETER+32)
#define VS_ID_MAGNETOMETER_WAKEUP                   (VS_ID_MAGNETOMETER+32)
#define VS_ID_ORIENTATION_WAKEUP                    (VS_ID_ORIENTATION+32)
#define VS_ID_GYROSCOPE_WAKEUP                      (VS_ID_GYROSCOPE+32)
#define VS_ID_LIGHT_WAKEUP                          (VS_ID_LIGHT+32)
#define VS_ID_BAROMETER_WAKEUP                      (VS_ID_BAROMETER+32)
#define VS_ID_TEMPERATURE_WAKEUP                    (VS_ID_TEMPERATURE+32)
#define VS_ID_PROXIMITY_WAKEUP                      (VS_ID_PROXIMITY+32)
#define VS_ID_GRAVITY_WAKEUP                        (VS_ID_GRAVITY+32)
#define VS_ID_LINEAR_ACCELERATION_WAKEUP            (VS_ID_LINEAR_ACCELERATION+32)
#define VS_ID_ROTATION_VECTOR_WAKEUP                (VS_ID_ROTATION_VECTOR+32)
#define VS_ID_HUMIDITY_WAKEUP                       (VS_ID_HUMIDITY+32)
#define VS_ID_AMBIENT_TEMPERATURE_WAKEUP            (VS_ID_AMBIENT_TEMPERATURE+32)
#define VS_ID_UNCALIBRATED_MAGNETOMETER_WAKEUP      (VS_ID_UNCALIBRATED_MAGNETOMETER+32)
#define VS_ID_GAME_ROTATION_VECTOR_WAKEUP           (VS_ID_GAME_ROTATION_VECTOR+32)
#define VS_ID_UNCALIBRATED_GYROSCOPE_WAKEUP         (VS_ID_UNCALIBRATED_GYROSCOPE+32)
#define VS_ID_SIGNIFICANT_MOTION_WAKEUP             (VS_ID_SIGNIFICANT_MOTION+32)
#define VS_ID_STEP_DETECTOR_WAKEUP                  (VS_ID_STEP_DETECTOR+32)
#define VS_ID_STEP_COUNTER_WAKEUP                   (VS_ID_STEP_COUNTER+32)
#define VS_ID_GEOMAGNETIC_ROTATION_VECTOR_WAKEUP    (VS_ID_GEOMAGNETIC_ROTATION_VECTOR+32)
#define VS_ID_HEART_RATE_WAKEUP                     (VS_ID_HEART_RATE+32)
#define VS_ID_TILT_DETECTOR_WAKEUP                  (VS_ID_TILT_DETECTOR+32)
#define VS_ID_WAKE_GESTURE_WAKEUP                   (VS_ID_WAKE_GESTURE+32)
#define VS_ID_GLANCE_GESTURE_WAKEUP                 (VS_ID_GLANCE_GESTURE+32)
#define VS_ID_PICKUP_GESTURE_WAKEUP                 (VS_ID_PICKUP_GESTURE+32)
#define VS_ID_CUS1_WAKEUP              				(VS_ID_CUS1+32)
#define VS_ID_CUS2_WAKEUP 			                (VS_ID_CUS2+32)
#define VS_ID_CUS3_WAKEUP           			    (VS_ID_CUS3+32)
#define VS_ID_CUS4_WAKEUP                 			(VS_ID_CUS4+32)
#define VS_ID_CUS5_WAKEUP                 			(VS_ID_CUS5+32)
#define VS_ID_ACTIVITY_WAKEUP                       (VS_ID_ACTIVITY+32)

#define VS_ID_DEBUG                         245
#define VS_ID_TIMESTAMP_LSW_WAKEUP          246
#define VS_ID_TIMESTAMP_MSW_WAKEUP          247
#define VS_ID_META_EVENT_WAKEUP             248
#define VS_ID_BSX_C                         249
#define VS_ID_BSX_B                         250
#define VS_ID_BSX_A                         251
#define VS_ID_TIMESTAMP_LSW                 252
#define VS_ID_TIMESTAMP_MSW                 253
#define VS_ID_META_EVENT                    254

#ifdef __cplusplus
}
#endif
#endif /* BHY_UC_DRIVER_CONSTANTS_H_ */
