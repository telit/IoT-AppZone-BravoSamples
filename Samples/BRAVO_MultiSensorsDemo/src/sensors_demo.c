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
#include <limits.h>
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
#include "m2mb_wDog.h"

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

#ifndef SKIP_LWM2M
#include "lwm2m.h"
#endif

#include "sensors_demo.h"



extern INT32 bsens_taskId;
/* Local defines ================================================================================*/
#define WAKE_UP_TICKS 10       //to be used in m2mb_wDog_enable
#define CTRL_TICKS_TO_REBOOT 6 //to be used in m2mb_wDog_enable
#define WD_TOUT_COUNT 3        //to be used in m2mb_wDog_addTask

/* Local typedefs ===============================================================================*/
/* Local statics ================================================================================*/

static uint32_t bhy_system_timestamp = 0;
static uint8_t fifo[FIFO_SIZE];


static BSENS_ENV_T gENVIRON_data;
static BSENS_TAMPER_T gTAMPER_data;
static BSENS_3DVECT_T g3DVECT_data;



#ifdef BSEC_THREAD
INT32 bsecTaskId;
#endif

M2MB_WDOG_HANDLE h_wDog;
UINT32 wd_tick_s;
UINT32 wd_kick_delay_ms;



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


#ifdef BSEC_THREAD
static INT32 bsec_loop_taskCB(INT32 type, INT32 param1, INT32 param2);
#endif


#ifndef BSEC_THREAD
/*!
 * @brief       custom version of bsec_iot_loop (defined in bsec_integration.c).

                it has no while loop, so it can be called to run only once, but it is up to caller to define when it must be executed next.
                the sleep function pointer can be used to provide caller with the amount of time that needs to pass before next execution
 *
 * @param[in]   get_timestamp_us    pointer to the system specific timestamp derivation function
 * @param[in]   output_ready        pointer to the function processing obtained BSEC outputs
 * @param[in]   state_save          pointer to the system-specific state save function
 * @param[in]   save_intvl          interval at which BSEC state should be saved (in samples)
 *
 * @return      none
 */
static void bsec_iot_custom_loop(get_timestamp_us_fct get_timestamp_us, output_ready_fct output_ready,
                    state_save_fct state_save, uint32_t save_intvl);
#endif

static void WDcallback(M2MB_WDOG_HANDLE hDog, M2MB_WDOG_IND_E wDog_event,UINT16 resp_size, void *resp_struct, void *userdata);

/*-----------------------------------------------------------------------------------------------*/


/* Static functions =============================================================================*/
/**
   @brief           returns the current uptime in ms as an unsigned long long int

   @return          number of milliseconds elapsed from startup
*/
static UINT64 get_u64_uptime_ms(void)
{

  UINT32 sysTicks = m2mb_os_getSysTicks();

  FLOAT32 ms_per_tick = m2mb_os_getSysTickDuration_ms();

  return (UINT64) (sysTicks * ms_per_tick); //milliseconds
}

/*-----------------------------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------------------------*/

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
#ifndef SKIP_LWM2M
    update_tamper_LWM2MObject( (int) status );
#endif
  }

  write_LED( M2MB_GPIO_LOW_VALUE );
}

/*-----------------------------------------------------------------------------------------------*/

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


  if( ( i++ >= (5 * ROTATION_VECTOR_SAMPLE_RATE) ) && is_new_value )
  {
#ifndef SKIP_LWM2M
    update_rotation_LWM2MObject( w, x, y, z, sensor_data->data_quaternion.estimated_accuracy );
#endif
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
  /* updates the system timestamp */
  bhy_update_system_timestamp( new_timestamp, &bhy_system_timestamp );

}




static int64_t get_timestamp_us( void )
{
  return get_u64_uptime_ms() * 1000L;
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
#ifndef SKIP_LWM2M
  static int demult = 0;
#endif

  gENVIRON_data.temperature = temperature;
  gENVIRON_data.humidity = humidity;
  gENVIRON_data.pressure = pressure / 100;
  gENVIRON_data.airQ = iaq;

#ifndef SKIP_LWM2M
  if( demult++ > 2 )
  {
    update_environment_LWM2MObject( temperature, pressure / 100., humidity, ( INT16 )iaq );
    demult = 0;
    AZX_LOG_INFO( "------>" );
  }
#endif

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
  uint32_t ret = 0;
  // ...
  // Load a library config from non-volatile memory, if available.
  //
  // Return zero if loading was unsuccessful or no config was available,
  // otherwise return length of loaded config string.
  // ...
  if( n_buffer < sizeof( bsec_config_iaq ) )
  {
    memcpy( config_buffer, bsec_config_iaq, n_buffer );
    ret = n_buffer;
  }
  else
  {
    memcpy( config_buffer, bsec_config_iaq, sizeof( bsec_config_iaq ) );
    ret = sizeof( bsec_config_iaq );
  }

  return ret;
}


/*-----------------------------------------------------------------------------------------------*/
static void WDcallback(M2MB_WDOG_HANDLE hDog, M2MB_WDOG_IND_E wDog_event,UINT16 resp_size, void *resp_struct, void *userdata)
{
	(void)hDog;
	(void)resp_size;
	(void)resp_struct;

  M2MB_OS_TASK_HANDLE TaskWD_H = (M2MB_OS_TASK_HANDLE)userdata;

	switch (wDog_event)
	{
		case M2MB_WDOG_TIMEOUT_IND:
		{
			AZX_LOG_INFO("Watchdog expired!\r\n");
			//kill the task and restart it.
      AZX_LOG_INFO("Terminate task and restart it.\r\n");
      m2mb_os_taskTerminate(TaskWD_H);
      m2mb_os_taskRestart(TaskWD_H);
      AZX_LOG_INFO("Send a message %d\r\n", azx_tasks_sendMessageToTask(bsens_taskId, TASK_REINIT, 0, 0));
      ;
		}
		break;

		default:
			break;
	}
}



#ifndef BSEC_THREAD
static UINT64 bsec_sleep_ms_resume_ts = 0;
/*-----------------------------------------------------------------------------------------------*/
static void bsec_sleep(UINT32 ms)
{
  UINT64 tmp_ts = get_u64_uptime_ms() + ms;
  if(tmp_ts < bsec_sleep_ms_resume_ts )
  {
    /*overflow in internal counter, compute the right amount*/
    AZX_LOG_ERROR("OVERFLOWING when asking to sleep %u ms! current value: %llu, it would become %llu\r\n", ms, bsec_sleep_ms_resume_ts, tmp_ts);
    bsec_sleep_ms_resume_ts = (ULONG_LONG_MAX - bsec_sleep_ms_resume_ts) + tmp_ts;
  }
  else
  {

    bsec_sleep_ms_resume_ts = tmp_ts;
  }
}
#endif

/*-----------------------------------------------------------------------------------------------*/
#ifdef BSEC_THREAD
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
#endif

/*-----------------------------------------------------------------------------------------------*/
#ifndef BSEC_THREAD
/*all defined in bsec_integration.c*/
extern void bme680_bsec_trigger_measurement(bsec_bme_settings_t *sensor_settings, sleep_fct sleep);
extern void bme680_bsec_read_data(int64_t time_stamp_trigger, bsec_input_t *inputs, uint8_t *num_bsec_inputs,
    int32_t bsec_process_data);
extern void bme680_bsec_process_data(bsec_input_t *bsec_inputs, uint8_t num_bsec_inputs, output_ready_fct output_ready);

/*!
 * @brief       custom version of bsec_iot_loop (defined in bsec_integration.c).

                it has no while loop, so it can be called to run only once, but it is up to caller to define when it must be executed next.
                the sleep function pointer can be used to provide caller with the amount of time that needs to pass before next execution
 *
 * @param[in]   get_timestamp_us    pointer to the system specific timestamp derivation function
 * @param[in]   output_ready        pointer to the function processing obtained BSEC outputs
 * @param[in]   state_save          pointer to the system-specific state save function
 * @param[in]   save_intvl          interval at which BSEC state should be saved (in samples)
 *
 * @return      none
 */
static void bsec_iot_custom_loop(get_timestamp_us_fct get_timestamp_us, output_ready_fct output_ready,
                    state_save_fct state_save, uint32_t save_intvl)
{
    /* Timestamp variables */
    int64_t time_stamp = 0;
    int64_t time_stamp_interval_ms = 0;

    /* Allocate enough memory for up to BSEC_MAX_PHYSICAL_SENSOR physical inputs*/
    bsec_input_t bsec_inputs[BSEC_MAX_PHYSICAL_SENSOR];

    /* Number of inputs to BSEC */
    uint8_t num_bsec_inputs = 0;

    /* BSEC sensor settings struct */
    bsec_bme_settings_t sensor_settings;

    /* Save state variables */
    uint8_t bsec_state[BSEC_MAX_STATE_BLOB_SIZE];
    uint8_t work_buffer[BSEC_MAX_WORKBUFFER_SIZE];
    uint32_t bsec_state_len = 0;

    static uint32_t n_samples = 0;

    bsec_library_return_t bsec_status = BSEC_OK;

    {
        /* get the timestamp in nanoseconds before calling bsec_sensor_control() */
        time_stamp = get_timestamp_us() * 1000;

        /* Retrieve sensor settings to be used in this time instant by calling bsec_sensor_control */
        bsec_sensor_control(time_stamp, &sensor_settings);

        /* Trigger a measurement if necessary */
        bme680_bsec_trigger_measurement(&sensor_settings, azx_sleep_ms);

        /* Read data from last measurement */
        num_bsec_inputs = 0;
        bme680_bsec_read_data(time_stamp, bsec_inputs, &num_bsec_inputs, sensor_settings.process_data);

        /* Time to invoke BSEC to perform the actual processing */
        bme680_bsec_process_data(bsec_inputs, num_bsec_inputs, output_ready);

        /* Increment sample counter */
        n_samples++;

        /* Retrieve and store state if the passed save_intvl */
        if (n_samples >= save_intvl)
        {
            bsec_status = bsec_get_state(0, bsec_state, sizeof(bsec_state), work_buffer, sizeof(work_buffer), &bsec_state_len);
            if (bsec_status == BSEC_OK)
            {
                state_save(bsec_state, bsec_state_len);
            }
            n_samples = 0;
        }


        /* Compute how long we can sleep until we need to call bsec_sensor_control() next */
        /* Time_stamp is converted from microseconds to nanoseconds first and then the difference to milliseconds */
        time_stamp_interval_ms = (sensor_settings.next_call - get_timestamp_us() * 1000) / 1000000;
        if (time_stamp_interval_ms > 0)
        {
            bsec_sleep((uint32_t)time_stamp_interval_ms);
        }
    }
}
#endif


/*-----------------------------------------------------------------------------------------------*/
int run_sensors_loop(int reset)
{
  /* BHY Variables*/
  uint8_t                       *fifoptr           = NULL;
  static uint16_t               bytes_left_in_fifo = 0;
  static uint16_t               bytes_remaining    = 0;
  static uint16_t               bytes_read         = 0;

  UINT64 kick_ts = 0;

  bhy_data_generic_t            fifo_packet;
  bhy_data_type_t               packet_type;
  BHY_RETURN_FUNCTION_TYPE      result;
  uint8_t                       reset_BHI = reset;
   /* continuously read and parse the fifo after the push of the button*/
  AZX_LOG_DEBUG("Starting the loop...\r\n");
  while(1)
  {

      if(get_u64_uptime_ms() > kick_ts)
      {
        kick_ts = get_u64_uptime_ms() + wd_kick_delay_ms;
        AZX_LOG_INFO("WD kick!\r\n");
        m2mb_wDog_kick(h_wDog, m2mb_os_taskGetId());
      }
      if(reset_BHI == DO_RESET)
      {
        AZX_LOG_INFO("Reset parameters and BHI160 FIFO\r\n");
        reset_BHI = DO_NOT_RESET;
        memset(fifo, 0, FIFO_SIZE);
        bytes_left_in_fifo = 0;
        bytes_remaining    = 0;
        bytes_read         = 0;

        result =  bhy_set_host_interface_control(BHY_HOST_ABORT_TRANSFER,1);
        if(result != BHY_SUCCESS)
        {
          AZX_LOG_WARN("Abort transfer result: %d\r\n", result);
        }
      }
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

        /* the logic here is that if doing a partial parsing of the fifo, then we should not parse  */
        /* the last 18 bytes (max length of a packet) so that we don't try to parse an incomplete   */
        /* packet                                                                                   */
      }
      while( ( result == BHY_SUCCESS ) && ( bytes_read > ( bytes_remaining ? MAX_PACKET_LENGTH : 0 ) ) );

      if(result == BHY_OUT_OF_RANGE && bytes_read > FIFO_SIZE )
      {
        AZX_LOG_WARN("Out of sync, reset everything\r\n");
        uint16_t to_be_flushed = bytes_remaining;
        while(to_be_flushed > 0)
        {
          bhy_read_fifo( fifo, FIFO_SIZE, &bytes_read, &bytes_remaining );
          to_be_flushed -= bytes_read;
          azx_sleep_ms( 10 );
        }
        
        memset(fifo, 0, FIFO_SIZE);
        bytes_left_in_fifo = 0;
        bytes_remaining    = 0;
        bytes_read         = 0;

        azx_sleep_ms( 10 );
      }

      bytes_left_in_fifo = 0;

      if( bytes_remaining )
      {
        /* shifts the remaining bytes to the beginning of the buffer */
        while( bytes_left_in_fifo < bytes_read )
        {
          fifo[bytes_left_in_fifo++] = *( fifoptr++ );
        }

      }
#ifndef BSEC_THREAD
    if(get_u64_uptime_ms() > bsec_sleep_ms_resume_ts)
    {
      bsec_iot_custom_loop( get_timestamp_us, bsec_output_ready, state_save, 10000 );
    }
#endif


  }

  return BHY_SUCCESS;
}
/*-----------------------------------------------------------------------------------------------*/
int init_sensors( void )
{
   return_values_init ret;


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

#ifdef BSEC_THREAD
  bsecTaskId = azx_tasks_createTask((char *)"BSEC_LOOP", AZX_TASKS_STACK_XL, 28, AZX_TASKS_MBOX_S, bsec_loop_taskCB);
  if(bsecTaskId <= 0)
  {
    AZX_LOG_CRITICAL("cannot create BSEC task!\r\n");
    return BHY_ERROR;
  }
#endif

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
#ifdef BSEC_THREAD
  azx_tasks_sendMessageToTask(bsecTaskId, 0, 0, 0);
#endif

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
  char log_buffer[512];
  va_list arg;
  va_start( arg, fmt );
  vsnprintf( log_buffer, sizeof( log_buffer ), fmt, arg );
  va_end( arg );
  AZX_LOG_INFO( "###%s", log_buffer );
}


/*-----------------------------------------------------------------------------------------------*/
void WDog_Init(M2MB_OS_TASK_HANDLE TaskWD_H)
{
M2MB_RESULT_E res;
MEM_W time_ms;


	AZX_LOG_INFO("\r\nInit WatchDog\r\n");
	res = m2mb_wDog_init(&h_wDog, WDcallback, TaskWD_H);
	if (res == M2MB_RESULT_SUCCESS)
	{
		AZX_LOG_INFO("m2mb_wDog_init OK\r\n");
	}
	else
	{
		AZX_LOG_ERROR("m2mb_wDog_init Fail, error: %d\r\n", res);
		return;
	}

	/* Verifying tick duration */
	res = m2mb_wDog_getItem(h_wDog, M2MB_WDOG_SELECT_CMD_TICK_DURATION_MS, 0, &time_ms);
	if (res == M2MB_RESULT_SUCCESS)
	{
		wd_tick_s = time_ms/1000;
		AZX_LOG_INFO("Tick duration: %ds\r\n", wd_tick_s);
	}
	else
	{
		AZX_LOG_ERROR("Get tick duration Fail, error: %d\r\n", res);
	}

  wd_kick_delay_ms = (WAKE_UP_TICKS - 1 ) * wd_tick_s * 1000;

	AZX_LOG_INFO("Adding Task under WD control with inactivity timeout of %ds\r\n", WD_TOUT_COUNT * WAKE_UP_TICKS * wd_tick_s);
	/* wdTimeout (inactivity timeout of the task) is set to WD_TOUT_COUNT (3 in this case).
	 * This counter is decreased every time a control is done and no kick have been received. Control is done every WAKE_UP_TICKS.
	 * When the counter reaches 0 a further control is done and if it's still 0 then callback is called,
	 * so task inactivity timeout will be more or less  WD_TOUT_COUNT * WAKE_UP_TICKS * 1s
	*/
	res = m2mb_wDog_addTask(h_wDog, TaskWD_H, WD_TOUT_COUNT);

	if (res == M2MB_RESULT_SUCCESS)
	{
		AZX_LOG_INFO("m2mb_wDog_addTask OK\r\n");
	}
	else
	{
		AZX_LOG_ERROR("m2mb_wDog_addTask Fail\r\n");
	}

	AZX_LOG_INFO("Enabling the WatchDog\r\n");
	/* WAKE_UP_TICKS defines the number of ticks of every control, by default the tick is every 1s
	 * CTRL_TICKS_TO_REBOOT this defines the number of controls the wd does before rebooting the app if no kick are received (or no action is done in watchdog callback )
	 * so timeout to reboot is  WAKE_UP_TICKS * CTRL_TICKS_TO_REBOOT * 1s
	 */
	res = m2mb_wDog_enable(h_wDog, WAKE_UP_TICKS, CTRL_TICKS_TO_REBOOT);
	if (res == M2MB_RESULT_SUCCESS)
	{
		AZX_LOG_INFO("m2mb_wDog_enable OK\r\n");
	}
	else
	{
		AZX_LOG_ERROR("m2mb_wDog_enable Fail, error: %d\r\n", res);
	}

}



/* Global functions =============================================================================*/

