/*=========================================================================*/
/*   Copyright (C) Telit Communications S.p.A. Italy All Rights Reserved.  */
/**
  @file
    M2MB_main.c

  @brief
    The file contains the main user entry point of Appzone

  @details

  @description
    Tampering Demo application. Debug prints on MAIN UART
  @version
    1.0.0
  @note
    Start of Appzone: Entry point
    User code entry is in function M2MB_main()

  @author WhiteBeard
  @author FabioPi

  @date
    2020-02-26
*/

/* Include files ================================================================================*/
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

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


#include "bhy_support.h"
#include "bhy_uc_driver.h"
#include "Bosch_PCB_7183_di03_BMI160-7183_di03.2.1.11696_170103.h"

#include "gpio.h"
#include "i2c.h"
#include "lwm2m.h"


/* Local defines ================================================================================*/

/*
   Bosch device
*/
#define INT_GPIO_PIN_NUM 6
#define LED_PIN_NUM 10

#define SENSOR_AR_TOUT  100  // 10 = 1 sec

#define XML_NAME "object_26242.xml"




#define FIFO_SIZE                      300
#define MAX_PACKET_LENGTH              18
#define TICKS_IN_ONE_SECOND            32000.0F

/* Local typedefs ===============================================================================*/
/* Local statics ================================================================================*/
static uint32_t bhy_system_timestamp = 0;
static uint8_t fifo[FIFO_SIZE];


/* Local function prototypes ====================================================================*/

/*-----------------------------------------------------------------------------------------------*/
/**
   @brief This is the callback function used to acquire sensor data

   @param[in]   sensor_data
   @param[in]   sensor_id

*/
static void sensors_callback( bhy_data_generic_t *sensor_data, bhy_virtual_sensor_t sensor_id );

/*-----------------------------------------------------------------------------------------------*/
/**
   @brief This function is time stamp callback function that process data in fifo.

   @param[in]   new_timestamp
*/
static void timestamp_callback( bhy_data_scalar_u16_t *new_timestamp );

/*-----------------------------------------------------------------------------------------------*/
/**
   @brief This function is used to run bhy hub
*/
static void demo_sensor( void );


/* Static functions =============================================================================*/

static void sensors_callback( bhy_data_generic_t *sensor_data, bhy_virtual_sensor_t sensor_id )
{
  int16_t activity = sensor_data->data_scalar_s16.data;
  float time_stamp = ( float )( bhy_system_timestamp ) / TICKS_IN_ONE_SECOND;
  
  write_LED( M2MB_GPIO_HIGH_VALUE );

  switch( ( INT32 )sensor_id )
  {
    case VS_ID_ACTIVITY:
    case VS_ID_ACTIVITY_WAKEUP:
      AZX_LOG_INFO( "activity recognized %.2f\r\n", time_stamp );

      if( activity & 0x1 ) // bit 0
      {
        AZX_LOG_INFO( "Still activity ended\r\n" );
        update_tamper_LWM2MObject( 0 );
      }

      if( activity & 0x2 ) // bit 1
      {
        AZX_LOG_INFO( "Walking activity ended\r\n" );
        update_tamper_LWM2MObject( 1 );
      }

      if( activity & 0x4 ) // bit 2
      {
        AZX_LOG_INFO( "Running activity ended\r\n" );
        update_tamper_LWM2MObject( 2 );
      }

      if( activity & 0x8 ) // bit 3
      {
        AZX_LOG_INFO( "On Bicycle activity ended\r\n" );
        update_tamper_LWM2MObject( 3 );
      }

      if( activity & 0x10 ) // bit 4
      {
        AZX_LOG_INFO( "In Vehicle activity ended\r\n" );
        update_tamper_LWM2MObject( 4 );
      }

      if( activity & 0x20 ) // bit 5
      {
        AZX_LOG_INFO( "Tilting activity ended\r\n" );
        update_tamper_LWM2MObject( 5 );
      }

      if( activity & 0x100 ) // bit 8
      {
        AZX_LOG_INFO( "Still activity started\r\n" );
        update_tamper_LWM2MObject( 8 );
      }

      if( activity & 0x200 ) // bit 9
      {
        AZX_LOG_INFO( "Walking activity started\r\n" );
        update_tamper_LWM2MObject( 9 );
      }

      if( activity & 0x400 ) // bit 10
      {
        AZX_LOG_INFO( "Running activity started\r\n" );
        update_tamper_LWM2MObject( 10 );
      }

      if( activity & 0x800 ) // bit 11
      {
        AZX_LOG_INFO( "On Bicycle activity started\r\n" );
        update_tamper_LWM2MObject( 11 );
      }

      if( activity & 0x1000 ) // bit 12
      {
        AZX_LOG_INFO( "In Vehicle activity started\r\n" );
        update_tamper_LWM2MObject( 12 );
      }

      if( activity & 0x2000 ) // bit 13
      {
        AZX_LOG_INFO( "Tilting activity started\r\n" );
        update_tamper_LWM2MObject( 13 );
      }

      break;

    default:
      AZX_LOG_INFO( "unknown id = %d\r\n", sensor_id );
      break;
  }

  /* activity recognition is not time critical, so let's wait a little bit */
  //    int tamper_data = 1;
  //    updateLWM2MObject(_obj_tamper_uri, &tamper_data, sizeof(int));
  //    sensor_ar_timer = SENSOR_AR_TOUT;
  azx_sleep_ms( 200 );
  write_LED( M2MB_GPIO_LOW_VALUE );
}


/*-----------------------------------------------------------------------------------------------*/
static void timestamp_callback( bhy_data_scalar_u16_t *new_timestamp )
{
  //AZX_LOG_DEBUG("new timestamp for id %u: 0x%08X\r\n", new_timestamp->sensor_id, new_timestamp->data);
  /* updates the system timestamp */
  bhy_update_system_timestamp( new_timestamp, &bhy_system_timestamp );
  //AZX_LOG_DEBUG("updated timestamp: %u\r\n", bhy_system_timestamp);
}


/*-----------------------------------------------------------------------------------------------*/
static void demo_sensor( void )
{
  /* BHY Variable*/
  uint8_t                    *fifoptr           = NULL;
  uint8_t                    bytes_left_in_fifo = 0;
  uint16_t                   bytes_remaining    = 0;
  uint16_t                   bytes_read         = 0;
  bhy_data_generic_t         fifo_packet;
  bhy_data_type_t            packet_type;
  BHY_RETURN_FUNCTION_TYPE   result;

  /* To get the customized version number in firmware, it is necessary to read Parameter Page 2, index 125 */
  /* to get this information. This feature is only supported for customized firmware. To get this customized */
  /* firmware, you need to contact your local FAE of Bosch Sensortec. */
  //struct cus_version_t      bhy_cus_version;

  /* init the bhy chip */
  if( bhy_driver_init( bhy1_fw ) )
  {
    AZX_LOG_CRITICAL( "Fail to init bhy\r\n" );
    return;
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
  bhy_install_sensor_callback( VS_TYPE_ACTIVITY_RECOGNITION, VS_NON_WAKEUP, sensors_callback );
  bhy_install_timestamp_callback( VS_NON_WAKEUP, timestamp_callback );


  write_LED( M2MB_GPIO_HIGH_VALUE );
  azx_sleep_ms( 100 );
  write_LED( M2MB_GPIO_LOW_VALUE );
  azx_sleep_ms( 100 );
  write_LED( M2MB_GPIO_HIGH_VALUE );
  azx_sleep_ms( 100 );
  write_LED( M2MB_GPIO_LOW_VALUE );
  azx_sleep_ms( 100 );
  
  AZX_LOG_INFO( "System is now monitoring activity.\r\n" );
  /* wait for the push-button to be pressed */

  /* continuously read and parse the fifo after the push of the button*/
  while( 1 )
  {
    bhy_read_fifo( fifo + bytes_left_in_fifo, FIFO_SIZE - bytes_left_in_fifo, &bytes_read,
                   &bytes_remaining );
    bytes_read         += bytes_left_in_fifo;
    fifoptr             = fifo;
    packet_type        = BHY_DATA_TYPE_PADDING;

    if( bytes_read )
    {
      do
      {
        /* this function will call callbacks that are registered */
        result = bhy_parse_next_fifo_packet( &fifoptr, &bytes_read, &fifo_packet, &packet_type );
        //AZX_LOG_DEBUG("fifo_parse_packet result: %d. Bytes read: %u; remaining: %u\r\n", result, bytes_read, bytes_remaining);
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
    else
    {
      //AZX_LOG_DEBUG("nothing to be read\r\n");
      /* activity recognition is not time critical, so let's wait a little bit */
      azx_sleep_ms( 100 );
      /*            if(sensor_ar_timer == 1)
            {
              sensor_ar_timer = 0;
                int tamper_data = 0;
                updateLWM2MObject(_obj_tamper_uri, &tamper_data, sizeof(int));
            }
            else if(sensor_ar_timer == 0)
            {
            }
            else
              sensor_ar_timer--;
      */
    }
  }

  //    return BHY_SUCCESS;
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
  /* SET output channel */
  AZX_LOG_INIT();
  AZX_LOG_INFO( "Starting Tampering Demo app. This is v%s built on %s %s.\r\n",
                VERSION, __DATE__, __TIME__ );

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
        return;
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

  /* Open I2C */
  if( open_I2C() != 0 )
  {
    AZX_LOG_ERROR( "cannot open I2C channel.\r\n" );
    return;
  }

  /* Open GPIO */
  if( open_gpio( INT_GPIO_PIN_NUM ) != 0 )
  {
    AZX_LOG_ERROR( "cannot open gpio channel.\r\n" );
    return;
  }

  if(oneedge_init( TAMPERING_OBJ_ID ) != 0)
  {
    AZX_LOG_ERROR("Failed enabling LWM2M!\r\n");
    return;
  }
  demo_sensor();
}
