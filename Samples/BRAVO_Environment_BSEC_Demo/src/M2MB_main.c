/*=========================================================================*/
/*   Copyright (C) Telit Communications S.p.A. Italy All Rights Reserved.  */
/**
  @file
    M2MB_main.c

  @brief
    The file contains the main user entry point of Appzone

  @details

  @description
    Environment Demo application. Debug prints on MAIN UART
  @version
    1.0.2
  @note
    Start of Appzone: Entry point
    User code entry is in function M2MB_main()

  @author WhiteBeard
  @author FabioPi

  @date
    2020-02-26
*/

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


/* Local defines ================================================================================*/

/*
   Bosch device
*/
#define INT_GPIO_PIN_NUM 6
#define LED_PIN_NUM 10


#define BME680_W_SELF_TEST_FAILED 3

#define XML_NAME "object_26251.xml"


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


/* Local typedefs ===============================================================================*/
/* Local statics ================================================================================*/


/* Local function prototypes ====================================================================*/

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
static void output_ready( int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature,
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

/* Static functions =============================================================================*/


static int64_t get_timestamp_us( void )
{
  int64_t system_current_time = ( int64_t )m2mb_os_getSysTicks() * 10000L;
  return system_current_time;
}

/*-----------------------------------------------------------------------------------------------*/
static void output_ready( int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature,
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


/* Global functions =============================================================================*/

/*-----------------------------------------------------------------------------------------------*/
/***************************************************************************************************
   \User Entry Point of Appzone

   \param [in] Module Id

   \details Main of the appzone user
 **************************************************************************************************/
void M2MB_main( int argc, char **argv )
{
  ( void )argc;
  ( void )argv;
  bsec_version_t bsec_version;
  return_values_init ret;
  /* SET output channel */
  AZX_LOG_INIT();
  AZX_LOG_INFO( "Starting Environmental BSEC Demo. This is v%s built on %s %s.\r\n",
                VERSION, __DATE__, __TIME__ );

INT32 ids[] = { ENVIRONMENT_OBJ_ID };
  /* Open GPIO */
  if( open_LED( LED_PIN_NUM ) != 0 )
  {
    AZX_LOG_ERROR( "Cannot open gpio channel.\r\n" );
    return;
  }

  write_LED( M2MB_GPIO_HIGH_VALUE );
  azx_sleep_ms( 5000 );
  write_LED( M2MB_GPIO_LOW_VALUE );

  /* Copy xml file if not exixting */
  if( 0 != check_xml_file( XML_NAME ) )
  {
    if( 0 != copy_xml_file( XML_NAME ) )
    {
      AZX_LOG_CRITICAL( "Failed copying file!\r\n" );
      
      for( int i = 0; i < 10; i++ )
      {
        write_LED( M2MB_GPIO_HIGH_VALUE );
        azx_sleep_ms( 200 );
        write_LED( M2MB_GPIO_LOW_VALUE );
        azx_sleep_ms( 200 );
      }

      return;
    }
    else
    {
      M2MB_POWER_HANDLE handle;
      AZX_LOG_DEBUG( "Rebooting to apply xml file\r\n" );

      if( M2MB_RESULT_SUCCESS != m2mb_power_init( &handle, NULL, ( void * ) NULL ) )
      {
        AZX_LOG_CRITICAL( "cannot init power module!\r\n" );
      }
      else
      {
        for( int i = 0; i < 3; i++ )
        {
          write_LED( M2MB_GPIO_HIGH_VALUE );
          azx_sleep_ms( 1000 );
          write_LED( M2MB_GPIO_LOW_VALUE );
          azx_sleep_ms( 1000 );
        }

        m2mb_power_reboot( handle );
        m2mb_power_deinit( handle );
        return;
      }
    }
  }


  /*Initialize one edge lwm2m configuration*/
  if(oneedge_init( ids, sizeof(ids) / sizeof(ids[0]) ) != 0)
  {
    AZX_LOG_ERROR("Failed enabling LWM2M!\r\n");
    return;
  }
  

  bsec_get_version( &bsec_version );
  AZX_LOG_INFO( "BSEC Version %d %d %d %d\r\n", bsec_version.major, bsec_version.minor,
                bsec_version.major_bugfix, bsec_version.minor_bugfix );
  /* Call to the function which initializes the BSEC library
     Switch on low-power mode and provide no temperature offset */
  ret = bsec_iot_init( BSEC_SAMPLE_RATE_LP, 0.0f, bme680_i2c_write, bme680_i2c_read, azx_sleep_ms,
                       state_load, config_load );

  if( ret.bme680_status )
  {
    /* Could not intialize BME680 */
    AZX_LOG_CRITICAL( "Could not intialize BME680: %d\r\n", ret.bme680_status  );
    return;// (int)ret.bme680_status;
  }
  else
  {
    if( ret.bsec_status )
    {
      /* Could not intialize BSEC library */
      AZX_LOG_ERROR( "Could not intialize BSEC library\r\n" );
      return;// (int)ret.bsec_status;
    }
  }
  
  write_LED( M2MB_GPIO_HIGH_VALUE );
  azx_sleep_ms( 100 );
  write_LED( M2MB_GPIO_LOW_VALUE );
  azx_sleep_ms( 100 );
  write_LED( M2MB_GPIO_HIGH_VALUE );
  azx_sleep_ms( 100 );
  write_LED( M2MB_GPIO_LOW_VALUE );
  azx_sleep_ms( 100 );
  
  /* Call to endless loop function which reads and processes data based on sensor settings */
  /* State is saved every 10.000 samples, which means every 10.000 * 3 secs = 500 minutes  */
  bsec_iot_loop( azx_sleep_ms, get_timestamp_us, output_ready, state_save, 10000 );
}


