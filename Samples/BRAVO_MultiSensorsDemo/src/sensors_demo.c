/*===============================================================================================*/
/*         >>> Copyright (C) Telit Communications S.p.A. Italy All Rights Reserved. <<<          */
/*!
  @file
    sensors_demo.c

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


/* Include files ================================================================================*/

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

#include "m2mb_types.h"
#include "m2mb_os_api.h"
#include "m2mb_fs_stdio.h"
#include "m2mb_i2c.h"
#include "m2mb_gpio.h"
#include "m2mb_lwm2m.h"
#include "m2mb_power.h"

#include "m2mb_fs_posix.h"
#include "m2mb_fs_errno.h"

#include "azx_log.h"
#include "azx_utils.h"
#include "azx_tasks.h"

#include "app_cfg.h"

#include "bsec_integration.h"
#include "bsec_serialized_configurations_iaq.h"

#include "bhy_support.h"
#include "bhy_uc_driver.h"
#include "bosch_pcb_7183_di03_bmi160_bmm150-7183_di03-2-1-11696_20180502.h"
#include "bme680.h"

#include "gpio.h"
#include "i2c.h"
#include "i2c_bme680.h"

#include "lwm2m.h"


#include "sensors_demo.h"


/* Local defines ================================================================================*/
/*
   Bosch device
 */


/* Local typedefs ===============================================================================*/
/* Local statics ================================================================================*/

static uint32_t bhy_system_timestamp = 0;
static uint8_t fifo[FIFO_SIZE];


static BSENS_ENV_T gENVIRON_data;
static BSENS_TAMPER_T gTAMPER_data;
static BSENS_3DVECT_T g3DVECT_data;


/* Local function prototypes ====================================================================*/

/*-----------------------------------------------------------------------------------------------*/
/**
   @brief This is the callback function used to acquire sensor data

   @param[in]   sensor_data
   @param[in]   sensor_id

 */
static void sensors_tamper_callback( bhy_data_generic_t *sensor_data, bhy_virtual_sensor_t sensor_id );

static void sensors_callback_rotation_vector( bhy_data_generic_t *sensor_data,
    bhy_virtual_sensor_t sensor_id );
/*-----------------------------------------------------------------------------------------------*/
/**
   @brief This function is time stamp callback function that process data in fifo.

   @param[in]   new_timestamp
 */
static void timestamp_callback( bhy_data_scalar_u16_t *new_timestamp );
/*-----------------------------------------------------------------------------------------------*/

/**
   @brief           Capture the system time in microseconds

   @return          system_current_time    current system timestamp in microseconds
*/
static int64_t get_timestamp_us( void );

/**
   @brief           Handling of the ready outputs

   @param[in]       timestamp       time in nanoseconds
   @param[in]       iaq             IAQ signal
   @param[in]       iaq_accuracy    accuracy of IAQ signal
   @param[in]       temperature     temperature signal
   @param[in]       humidity        humidity signal
   @param[in]       pressure        pressure signal
   @param[in]       raw_temperature raw temperature signal
   @param[in]       raw_humidity    raw humidity signal
   @param[in]       gas             raw gas sensor signal
   @param[in]       bsec_status     value returned by the bsec_do_steps() call

   @return          none
*/
static void bsec_output_ready( int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature,
                          float humidity,
                          float pressure, float raw_temperature, float raw_humidity, float gas,
                          bsec_library_return_t bsec_status,
                          float static_iaq, float co2_equivalent, float breath_voc_equivalent );


/**
   @brief           Load previous library state from non-volatile memory

   @param[in,out]   state_buffer    buffer to hold the loaded state string
   @param[in]       n_buffer        size of the allocated state buffer

   @return          number of bytes copied to state_buffer
*/
static uint32_t state_load( uint8_t *state_buffer, uint32_t n_buffer );


/**
   @brief           Save library state to non-volatile memory

   @param[in]       state_buffer    buffer holding the state to be stored
   @param[in]       length          length of the state string to be stored

   @return          none
*/
static void state_save( const uint8_t *state_buffer, uint32_t length );


/**
   @brief           Load library config from non-volatile memory

   @param[in,out]   config_buffer    buffer to hold the loaded state string
   @param[in]       n_buffer        size of the allocated state buffer

   @return          number of bytes copied to config_buffer
*/
static uint32_t config_load( uint8_t *config_buffer, uint32_t n_buffer );


static INT32 bsec_loop_taskCB(INT32 type, INT32 param1, INT32 param2);
/*-----------------------------------------------------------------------------------------------*/


/* Static functions =============================================================================*/

static void sensors_tamper_callback( bhy_data_generic_t *sensor_data, bhy_virtual_sensor_t sensor_id )
{
  int16_t activity = sensor_data->data_scalar_s16.data;
  float time_stamp = ( float )( bhy_system_timestamp ) / TICKS_IN_ONE_SECOND;

  write_LED( M2MB_GPIO_HIGH_VALUE );
  BHI_TAMPER_STATUS_E status = STATUS_INVALID;
  switch( ( INT32 )sensor_id )
  {
  case VS_ID_ACTIVITY:
  case VS_ID_ACTIVITY_WAKEUP:
    AZX_LOG_INFO( "activity recognized %.2f\r\n", time_stamp );

    if( activity & 0x1 ) // bit 0
    {
      AZX_LOG_INFO( "Still activity ended\r\n" );
      status = STILL_ENDED;

    }

    if( activity & 0x2 ) // bit 1
    {
      AZX_LOG_INFO( "Walking activity ended\r\n" );
      status = WALKING_ENDED;
    }

    if( activity & 0x4 ) // bit 2
    {
      AZX_LOG_INFO( "Running activity ended\r\n" );
      status = RUNNING_ENDED;
    }

    if( activity & 0x8 ) // bit 3
    {
      AZX_LOG_INFO( "On Bicycle activity ended\r\n" );
      status = BYCYCLE_ENDED;
    }

    if( activity & 0x10 ) // bit 4
    {
      AZX_LOG_INFO( "In Vehicle activity ended\r\n" );
      status = VEHICLE_ENDED;
    }

    if( activity & 0x20 ) // bit 5
    {
      AZX_LOG_INFO( "Tilting activity ended\r\n" );
      status = TILTING_ENDED;
    }

    if( activity & 0x100 ) // bit 8
    {
      AZX_LOG_INFO( "Still activity started\r\n" );
      status = STILL_STARTED;
    }

    if( activity & 0x200 ) // bit 9
    {
      AZX_LOG_INFO( "Walking activity started\r\n" );
      status = WALKING_STARTED;
    }

    if( activity & 0x400 ) // bit 10
    {
      AZX_LOG_INFO( "Running activity started\r\n" );
      status = RUNNING_STARTED;
    }

    if( activity & 0x800 ) // bit 11
    {
      AZX_LOG_INFO( "On Bicycle activity started\r\n" );
      status = BICYCLE_STARTED;
    }

    if( activity & 0x1000 ) // bit 12
    {
      AZX_LOG_INFO( "In Vehicle activity started\r\n" );
      status = VEHICLE_STARTED;
    }

    if( activity & 0x2000 ) // bit 13
    {
      AZX_LOG_INFO( "Tilting activity started\r\n" );
      status = TILTING_STARTED;
    }
    break;

  default:
    AZX_LOG_INFO( "unknown id = %d\r\n", sensor_id );
      status = STATUS_INVALID;
    break;
  }

  if(status  != STATUS_INVALID)
  {
    gTAMPER_data.status = status;
    gTAMPER_data.timestamp = time_stamp;

    update_tamper_LWM2MObject( (int) status );
  }

  /* activity recognition is not time critical, so let's wait a little bit */
  //    int tamper_data = 1;
  //    updateLWM2MObject(_obj_tamper_uri, &tamper_data, sizeof(int));
  //    sensor_ar_timer = SENSOR_AR_TOUT;
  azx_sleep_ms( 200 );
  write_LED( M2MB_GPIO_LOW_VALUE );
}


static void sensors_callback_rotation_vector( bhy_data_generic_t *sensor_data,
    bhy_virtual_sensor_t sensor_id )
{
  (void)sensor_id;
  static int16_t old_w = 0;
  static int16_t old_x = 0;
  static int16_t old_y = 0;
  static int16_t old_z = 0;

  static int i = 0;
  uint8_t is_new_value = 0;
  is_new_value |= ( old_w != sensor_data->data_quaternion.w );
  is_new_value |= ( old_x != sensor_data->data_quaternion.x );
  is_new_value |= ( old_y != sensor_data->data_quaternion.y );
  is_new_value |= ( old_z != sensor_data->data_quaternion.z );

  /* change the data unit by dividing by 16384 */
  float w = ( float )sensor_data->data_quaternion.w / 16384.0f;
  float x = ( float )sensor_data->data_quaternion.x / 16384.0f;
  float y = ( float )sensor_data->data_quaternion.y / 16384.0f;
  float z = ( float )sensor_data->data_quaternion.z / 16384.0f;

  /* emit message only if value is different from previous one and it is new */


  if( ( i++ >= 50 ) && is_new_value )
  {
    update_rotation_LWM2MObject( w, x, y, z, sensor_data->data_quaternion.estimated_accuracy );
    i = 0;
    AZX_LOG_INFO( "------->  w=%f, x=%f, y=%f, z=%f; acc=%d\r\n",
        w, x, y, z, sensor_data->data_quaternion.estimated_accuracy );
    g3DVECT_data.w = w;
    g3DVECT_data.x = x;
    g3DVECT_data.y = y;
    g3DVECT_data.z = z;
    g3DVECT_data.accuracy = sensor_data->data_quaternion.estimated_accuracy;

  }

  old_w = sensor_data->data_quaternion.w;
  old_x = sensor_data->data_quaternion.x;
  old_y = sensor_data->data_quaternion.y;
  old_z = sensor_data->data_quaternion.z;
}

/*-----------------------------------------------------------------------------------------------*/
static void timestamp_callback( bhy_data_scalar_u16_t *new_timestamp )
{
  //AZX_LOG_DEBUG("new timestamp for id %u: 0x%08X\r\n", new_timestamp->sensor_id, new_timestamp->data);
  /* updates the system timestamp */
  bhy_update_system_timestamp( new_timestamp, &bhy_system_timestamp );
  //AZX_LOG_DEBUG("updated timestamp: %u\r\n", bhy_system_timestamp);
}




static int64_t get_timestamp_us( void )
{
  int64_t system_current_time = ( int64_t )m2mb_os_getSysTicks() * 10000L;
  return system_current_time;
}

/*-----------------------------------------------------------------------------------------------*/
static void bsec_output_ready( int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature,
                          float humidity,
                          float pressure, float raw_temperature, float raw_humidity, float gas,
                          bsec_library_return_t bsec_status,
                          float static_iaq, float co2_equivalent, float breath_voc_equivalent )
{
  ( void ) timestamp;
  ( void ) iaq_accuracy;
  ( void ) raw_temperature;
  ( void ) raw_humidity;
  ( void ) gas;
  ( void ) bsec_status;
  ( void ) static_iaq;
  ( void ) co2_equivalent;
  ( void ) breath_voc_equivalent;
  /* =============
    Please insert system specific code to further process or display the BSEC outputs
    ================*/
  static int demult = 0;

  gENVIRON_data.temperature = temperature;
  gENVIRON_data.humidity = humidity;
  gENVIRON_data.pressure = pressure / 100;
  gENVIRON_data.airQ = iaq;

  if( demult++ > 2 )
  {
    update_environment_LWM2MObject( temperature, pressure / 100., humidity, ( INT16 )iaq );
    demult = 0;
    AZX_LOG_INFO( "------>" );
  }


  AZX_LOG_INFO( "T %f H %f P %f IAQ %f\r\n", temperature, humidity, pressure / 100., iaq );
}

/*-----------------------------------------------------------------------------------------------*/
static uint32_t state_load( uint8_t *state_buffer, uint32_t n_buffer )
{
  ( void ) state_buffer;
  ( void ) n_buffer;
  // ...
  // Load a previous library state from non-volatile memory, if available.
  //
  // Return zero if loading was unsuccessful or no state was available,
  // otherwise return length of loaded state string.
  // ...
  return 0;
}

/*-----------------------------------------------------------------------------------------------*/
static void state_save( const uint8_t *state_buffer, uint32_t length )
{
  ( void ) state_buffer;
  ( void ) length;
  // ...
  // Save the string some form of non-volatile memory, if possible.
  // ...
}

/*-----------------------------------------------------------------------------------------------*/
static uint32_t config_load( uint8_t *config_buffer, uint32_t n_buffer )
{
  // ...
  // Load a library config from non-volatile memory, if available.
  //
  // Return zero if loading was unsuccessful or no config was available,
  // otherwise return length of loaded config string.
  // ...
  if( n_buffer < sizeof( bsec_config_iaq ) )
  {
    memcpy( config_buffer, bsec_config_iaq, n_buffer );
    return n_buffer;
  }
  else
  {
    memcpy( config_buffer, bsec_config_iaq, sizeof( bsec_config_iaq ) );
    return sizeof( bsec_config_iaq );
  }

  return 0;
}



static INT32 bsec_loop_taskCB(INT32 type, INT32 param1, INT32 param2)
{
  (void) type;
  (void) param1;
  (void) param2;
  AZX_LOG_DEBUG("BSEC task started\r\n");
  /* Call to endless loop function which reads and processes data based on sensor settings */
  /* State is saved every 10.000 samples, which means every 10.000 * 3 secs = 500 minutes  */
  bsec_iot_loop( azx_sleep_ms, get_timestamp_us, bsec_output_ready, state_save, 10000 );
  return 0;
}








/*-----------------------------------------------------------------------------------------------*/
int init_sensors( void )
{
  /* BHY Variable*/
   uint8_t                    *fifoptr           = NULL;
   uint8_t             bytes_left_in_fifo = 0;
   uint16_t            bytes_remaining    = 0;
   uint16_t                   bytes_read         = 0;

   bhy_data_generic_t         fifo_packet;
   bhy_data_type_t            packet_type;
   BHY_RETURN_FUNCTION_TYPE   result;


   return_values_init ret;
   INT32 bsecTaskId;

  /* the remapping matrix for BHA or BHI here should be configured according to its placement on customer's PCB. */
  /* for details, please check 'Application Notes Axes remapping of BHA250(B)/BHI160(B)' document. */
  int8_t                     bhy_mapping_matrix_config[3 * 3] = {0, 1, 0, -1, 0, 0, 0, 0, 1};
  /* the remapping matrix for Magnetometer should be configured according to its placement on customer's PCB.  */
  /* for details, please check 'Application Notes Axes remapping of BHA250(B)/BHI160(B)' document. */
  int8_t                     mag_mapping_matrix_config[3 * 3] = {0, 1, 0, 1, 0, 0, 0, 0, -1};
  /* the sic matrix should be calculated for customer platform by logging uncalibrated magnetometer data. */
  /* the sic matrix here is only an example array (identity matrix). Customer should generate their own matrix. */
  /* This affects magnetometer fusion performance. */
  float sic_array[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  /* To get the customized version number in firmware, it is necessary to read Parameter Page 2, index 125 */
  /* to get this information. This feature is only supported for customized firmware. To get this customized */
  /* firmware, you need to contact your local FAE of Bosch Sensortec. */

  //struct cus_version_t      bhy_cus_version;

  /* init the bhy chip */
  if( bhy_driver_init( bhy_firmware_image ) )
  {
    AZX_LOG_CRITICAL( "Fail to init bhy\r\n" );
    return BHY_ERROR;
  }

  /* wait for the bhy trigger the interrupt pin go down and up again */
  while( read_gpio() )
  {
    azx_sleep_ms( 10 );
  }

  while( !read_gpio() )
  {
    azx_sleep_ms( 10 );
  }

  /* To get the customized version number in firmware, it is necessary to read Parameter Page 2, index 125 */
  /* to get this information. This feature is only supported for customized firmware. To get this customized */
  /* firmware, you need to contact your local FAE of Bosch Sensortec. */
  //bhy_read_parameter_page(BHY_PAGE_2, PAGE2_CUS_FIRMWARE_VERSION, (uint8_t*)&bhy_cus_version, sizeof(struct cus_version_t));
  //DEBUG("cus version base:%d major:%d minor:%d\n", bhy_cus_version.base, bhy_cus_version.major, bhy_cus_version.minor);

  /* enables the activity recognition and assigns the callback */
  bhy_enable_virtual_sensor( VS_TYPE_ACTIVITY_RECOGNITION, VS_NON_WAKEUP, 1, 0, VS_FLUSH_NONE, 0, 0 );
  bhy_install_sensor_callback( VS_TYPE_ACTIVITY_RECOGNITION, VS_NON_WAKEUP, sensors_tamper_callback );
  bhy_install_timestamp_callback( VS_NON_WAKEUP, timestamp_callback );



  /* The remapping matrix for BHI and Magmetometer should be configured here to make sure rotation vector is */
  /* calculated in a correct coordinates system. */
  bhy_mapping_matrix_set( PHYSICAL_SENSOR_INDEX_ACC, bhy_mapping_matrix_config );
  bhy_mapping_matrix_set( PHYSICAL_SENSOR_INDEX_MAG, mag_mapping_matrix_config );
  bhy_mapping_matrix_set( PHYSICAL_SENSOR_INDEX_GYRO, bhy_mapping_matrix_config );
  /* This sic matrix setting affects magnetometer fusion performance. */
  bhy_set_sic_matrix( sic_array );

  /* install the callback function for parse fifo data */
  if( bhy_install_sensor_callback( VS_TYPE_ROTATION_VECTOR, VS_NON_WAKEUP,
      sensors_callback_rotation_vector ) )
  {
    AZX_LOG_ERROR( "Fail to install rotation sensor callback\r\n" );
    return BHY_ERROR;
  }

  /* enables the virtual sensor */
  if( bhy_enable_virtual_sensor( VS_TYPE_ROTATION_VECTOR, VS_NON_WAKEUP, ROTATION_VECTOR_SAMPLE_RATE, 0,
      VS_FLUSH_NONE, 0, 0 ) )
  {
    AZX_LOG_ERROR( "Fail to enable rotation sensor id=%d\r\n", VS_TYPE_ROTATION_VECTOR );
    return BHY_ERROR;
  }


  /*INIT BSEC for environmental*/
  ret = bsec_iot_init( BSEC_SAMPLE_RATE_LP, 0.0f, bme680_i2c_write, bme680_i2c_read, azx_sleep_ms,
                         state_load, config_load );

  if( ret.bme680_status )
  {
    /* Could not intialize BME680 */
    AZX_LOG_CRITICAL( "Could not intialize BME680: %d\r\n", ret.bme680_status  );
    return BHY_ERROR; // (int)ret.bme680_status;
  }
  else
  {
    if( ret.bsec_status )
    {
      /* Could not intialize BSEC library */
      AZX_LOG_ERROR( "Could not intialize BSEC library\r\n" );
      return BHY_ERROR; // (int)ret.bsec_status;
    }
  }

  bsecTaskId = azx_tasks_createTask((char *)"BSEC_LOOP", AZX_TASKS_STACK_XL, 5, AZX_TASKS_MBOX_S, bsec_loop_taskCB);
  if(bsecTaskId <= 0)
  {
    AZX_LOG_CRITICAL("cannot create BSEC task!\r\n");
    return BHY_ERROR;
  }


  write_LED( M2MB_GPIO_HIGH_VALUE );
  azx_sleep_ms( 100 );
  write_LED( M2MB_GPIO_LOW_VALUE );
  azx_sleep_ms( 100 );
  write_LED( M2MB_GPIO_HIGH_VALUE );
  azx_sleep_ms( 100 );
  write_LED( M2MB_GPIO_LOW_VALUE );
  azx_sleep_ms( 100 );

  AZX_LOG_INFO( "System is now monitoring activity for tampering and rotation changes, move the board to update data.\r\n" );

  //START BSEC task
  azx_tasks_sendMessageToTask(bsecTaskId, 0, 0, 0);

  /* continuously read and parse the fifo after the push of the button*/
  while(1)
  {
    /* wait until the interrupt fires */
      /* unless we already know there are bytes remaining in the fifo */

      while( !read_gpio()  && !bytes_remaining )
      {
        azx_sleep_ms( 10 );
      }

      bhy_read_fifo( fifo + bytes_left_in_fifo, FIFO_SIZE - bytes_left_in_fifo, &bytes_read,
          &bytes_remaining );
      bytes_read         += bytes_left_in_fifo;
      fifoptr             = fifo;
      packet_type        = BHY_DATA_TYPE_PADDING;


      do
      {
        /* this function will call callbacks that are registered */
        result = bhy_parse_next_fifo_packet( &fifoptr, &bytes_read, &fifo_packet, &packet_type );

        /* prints all the debug packets */
        if( packet_type == BHY_DATA_TYPE_DEBUG )
        {
          bhy_print_debug_packet( &fifo_packet.data_debug, bhy_printf );
        }

        /* the logic here is that if doing a partial parsing of the fifo, then we should not parse        */
        /* the last 18 bytes (max length of a packet) so that we don't try to parse an incomplete   */
        /* packet                                                                                                            */
      }
      while( ( result == BHY_SUCCESS ) && ( bytes_read > ( bytes_remaining ? MAX_PACKET_LENGTH : 0 ) ) );

      bytes_left_in_fifo = 0;

      if( bytes_remaining )
      {
        /* shifts the remaining bytes to the beginning of the buffer */
        while( bytes_left_in_fifo < bytes_read )
        {
          fifo[bytes_left_in_fifo++] = *( fifoptr++ );
        }
      }


  }
  return BHY_SUCCESS;
}


int read_sensor(BSENS_SENSOR_ID_E id, void **data)
{

  switch(id)
  {
  case BSENS_SENSOR_ENVIRONM_ID:
    AZX_LOG_TRACE("asked to read Environment data.\r\n");
    *data = &gENVIRON_data;
    AZX_LOG_TRACE( "LAST DATA: t: %f, hum: %f, p: %f, airq: %d\r\n",
               gENVIRON_data.temperature,
               gENVIRON_data.humidity,
               gENVIRON_data.pressure,
               gENVIRON_data.airQ
               );
    break;
  case BSENS_SENSOR_3D_VECT_ID:
    AZX_LOG_TRACE("asked to read 3DVECT data.\r\n");

    AZX_LOG_TRACE( "LAST DATA: x=%f, y=%f, z=%f, w=%f; acc=%d\r\n",
            g3DVECT_data.x, g3DVECT_data.y, g3DVECT_data.z, g3DVECT_data.w,
            g3DVECT_data.accuracy );
    *data = &g3DVECT_data;
    break;
  case BSENS_SENSOR_TAMPER_ID:
    AZX_LOG_TRACE("asked to read Tampering status data.\r\n");
    AZX_LOG_TRACE( "LAST DATA: %d at %u\r\n",
               gTAMPER_data.status, gTAMPER_data.timestamp );
    *data = &gTAMPER_data;


    break;
  default:
    break;
  }
  return  BHY_SUCCESS;
}


/**
   @brief This is the log function that bhy uses as extern.

   @param[in]   fmt string format
   @param[in]

 */
void trace_log( const char *fmt, ... )
{
  char log_buffer[256];
  va_list arg;
  va_start( arg, fmt );
  vsnprintf( log_buffer, sizeof( log_buffer ), fmt, arg );
  va_end( arg );
  AZX_LOG_INFO( log_buffer );
}

/* Global functions =============================================================================*/

