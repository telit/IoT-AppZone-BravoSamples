/*=========================================================================*/
/*   Copyright (C) Telit Communications S.p.A. Italy All Rights Reserved.  */
/**
  @file
    M2MB_main.c

  @brief
    The file contains the main user entry point of Appzone

  @details

  @description
    SmartLock Demo application. Debug prints on MAIN UART
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
#include "Bosch_BHI160_Orientation.h"

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

#define XML_NAME "object_26247.xml"




#define FIFO_SIZE                      300
#define MAX_PACKET_LENGTH              18
#define TICKS_IN_ONE_SECOND            32000.0F

/* Local typedefs ===============================================================================*/
/* Local statics ================================================================================*/

static uint8_t fifo[FIFO_SIZE];

static struct accel_physical_status_t phy_acc;
static struct gyro_physical_status_t phy_gyro;
static struct mag_physical_status_t phy_mag;
static uint8_t physical_sensor_present_bitmap[8];

/* Local function prototypes ====================================================================*/
/*-----------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------*/
/**
   @brief This function is used to run bhy hub
*/
static void demo_sensor( void );

/*-----------------------------------------------------------------------------------------------*/
/**
   @brief This function is a callback function to acquire sensor data

   @param[in]   sensor_data
   @param[in]   sensor_id
*/
static void sensors_callback( bhy_data_generic_t *sensor_data, bhy_virtual_sensor_t sensor_id );


/*-----------------------------------------------------------------------------------------------*/
/*!
   @brief This function is a callback function to get meta event info

   @param[in]   event_data
   @param[in]   event_type
*/
static void meta_event_callback( bhy_data_meta_event_t *event_data,
                                 bhy_meta_event_type_t event_type );


/* Static functions =============================================================================*/
static void sensors_callback( bhy_data_generic_t *sensor_data, bhy_virtual_sensor_t sensor_id )
{
  write_LED( M2MB_GPIO_HIGH_VALUE );

  switch( ( INT32 )sensor_id )
  {
    case VS_ID_CUS2:      // cus2 id 0x1B indicates VirtAnyNoMotion virtual sensor driver
    case VS_ID_CUS2_WAKEUP:
    {
      uint8_t motion_event = sensor_data->data_custom.data[0];

      if( motion_event == 0 )
      {
        AZX_LOG_INFO( "No Motion\r\n" );
        update_smartlock_LWM2MObject( 0 );
      }
      else
        if( motion_event == 1 )
        {
          AZX_LOG_INFO( "Any Motion\r\n" );
          update_smartlock_LWM2MObject( 5 );
        }
    }
    break;

    case VS_ID_CUS3:      // cus3 id 0x1C indicates VirtSmartLock virtual sensor driver
    case VS_ID_CUS3_WAKEUP:
    {
      uint8_t door_status = sensor_data->data_custom.data[0];

      if( door_status == 1 )
      {
        AZX_LOG_INFO( "Door is Closed\r\n" );
        update_smartlock_LWM2MObject( 1 );
      }
      else
        if( door_status == 2 )
        {
          AZX_LOG_INFO( "Door is Open\r\n" );
          update_smartlock_LWM2MObject( 2 );
        }
        else
          if( door_status == 3 )
          {
            AZX_LOG_INFO( "Calibration Alert\r\n" );
            update_smartlock_LWM2MObject( 3 );
          }
          else
            if( door_status == 4 )
            {
              // "Door open too long time" alert, timer can be configured by BHY_PAGE_13_PARAMETER_DOOR_OPEN_TIMER
              AZX_LOG_INFO( "Door Open too long\r\n" );
              update_smartlock_LWM2MObject( 4 );
            }
    }
    break;

    default:
      AZX_LOG_DEBUG( "unknown id = %d\r\n", sensor_id );
      break;
  }

  write_LED( M2MB_GPIO_LOW_VALUE );
}


static void meta_event_callback( bhy_data_meta_event_t *event_data,
                                 bhy_meta_event_type_t event_type )
{
  switch( event_type )
  {
    case BHY_META_EVENT_TYPE_INITIALIZED:
      AZX_LOG_DEBUG( "Initialize success!\r\n" );
      break;

    case BHY_META_EVENT_TYPE_FIFO_OVERFLOW:
      AZX_LOG_INFO( "FIFO overflow: %d %d %d %d\r\n", event_data->meta_event_id, event_data->event_number,
                    event_data->sensor_type, event_data->event_specific );
      break;

    case BHY_META_EVENT_TYPE_ERROR:
      AZX_LOG_INFO( "Error: %d %d %d %d\r\n", event_data->meta_event_id, event_data->event_number,
                    event_data->sensor_type, event_data->event_specific );
      break;

    case BHY_META_EVENT_TYPE_SENSOR_ERROR:
      AZX_LOG_INFO( "Sensor error: %d %d %d %d\r\n", event_data->meta_event_id, event_data->event_number,
                    event_data->sensor_type, event_data->event_specific );
      break;

    case BHY_META_EVENT_TYPE_SELF_TEST_RESULTS:
      if( event_data->event_specific == BHY_SUCCESS )
      {
        AZX_LOG_DEBUG( "self test result success!  sensor_type=%d\r\n", event_data->sensor_type );
      }
      else
      {
        AZX_LOG_DEBUG( "self test result fail!  sensor_type=%d \r\n", event_data->sensor_type );
      }

      break;

    default:
      AZX_LOG_DEBUG( "Unknown meta event\r\n" );
      break;
  }
}

/*-----------------------------------------------------------------------------------------------*/

static void demo_sensor( void )
{
  /* BHY Variable*/
  uint8_t                    *fifoptr           = NULL;
  uint16_t                   bytes_left_in_fifo = 0;
  uint16_t                   bytes_remaining    = 0;
  uint16_t                   bytes_read         = 0;
  uint32_t                   i                  = 0;
  bhy_data_generic_t         fifo_packet;
  bhy_data_type_t            packet_type;
  BHY_RETURN_FUNCTION_TYPE   result;
  struct sensor_information_non_wakeup_t sensor_info_non_wakeup;
  struct sensor_information_wakeup_t sensor_info_wakeup;
  /* To get the customized version number in firmware, it is necessary to read Parameter Page 2, index 125 */
  /* to get this information. This feature is only supported for customized firmware. To get this customized */
  /* firmware, you need to contact your local FAE of Bosch Sensortec. */
  struct cus_version_t      bhy_cus_version;
  /* If custom sensor is related to IMU sensor, then the remapping matrix for BHA or BHI here should */
  /* be configured according to its placement on customer's PCB. */
  /* for details, please check 'Application Notes Axes remapping of BHA250(B)/BHI160(B)' document. */
  /* For detauls on how to configure remapping matrix, please check example of 'accelerometer_remapping_example.c'. */
  bhy_install_meta_event_callback( BHY_META_EVENT_TYPE_INITIALIZED, meta_event_callback );
  bhy_install_meta_event_callback( BHY_META_EVENT_TYPE_SELF_TEST_RESULTS, meta_event_callback );
  if (BHY_SUCCESS != (result = bhy_install_sensor_callback( VS_TYPE_CUS2, VS_WAKEUP, sensors_callback )))
  {
    AZX_LOG_ERROR("cannot register callback! %d\r\n", result);
    return;
  }
  if (BHY_SUCCESS != (result = bhy_install_sensor_callback( VS_TYPE_CUS3, VS_WAKEUP, sensors_callback )))
  {
    AZX_LOG_ERROR("cannot register callback! %d\r\n", result);
    return;
  }

  /* init the bhy chip */
  if( bhy_driver_init( bhy1_fw ) )
  {
    AZX_LOG_ERROR( "Fail to init bhy\r\n" );
    return;
  }
  else
  {
    AZX_LOG_INFO("BHY Correctly Initialized\r\n");
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
  bhy_read_parameter_page( BHY_PAGE_2, PAGE2_CUS_FIRMWARE_VERSION, ( uint8_t * )&bhy_cus_version,
                           sizeof( struct cus_version_t ) );
  AZX_LOG_TRACE( "CUS version - base:%d major:%d minor:%d\n", bhy_cus_version.base,
                bhy_cus_version.major, bhy_cus_version.minor );
  /* get physical sensor presence from sensor hub */
  bhy_read_parameter_page( BHY_PAGE_1, BHY_PARAM_SYSTEM_PHYSICAL_SENSOR_PRESENT,
                           &physical_sensor_present_bitmap[0], 8 );
  AZX_LOG_INFO( "Physical Sensor is present\r\n" );

  for( i = 0; i < 8; i++ )
  {
    AZX_LOG_TRACE( "bitmap[%d] = 0x%x\r\n", i, physical_sensor_present_bitmap[i] );
  }


    /* get physical sensor status from sensor hub */
    bhy_get_physical_sensor_status( &phy_acc, &phy_gyro, &phy_mag );
    /*  Physical Sensor Status:
      Flags[bit 0]:  interrupt enable
      Flags[bits 5-7]: Sensor Power Mode values:
        0: Sensor Not Present
        1: Power Down
        2: Suspend
        3: Self-Test
        4: Interrupt Motion
        5: One Shot
        6: Low Power Active
        7: Active
    */
    AZX_LOG_INFO( "Physical Sensor Status:\r\n" );
    AZX_LOG_INFO( "Acc : sample rate %d, range %d, int %d pwr %d\r\n",
                  phy_acc.accel_sample_rate, phy_acc.accel_dynamic_range, phy_acc.accel_flag & 0x01,
                  ( phy_acc.accel_flag & 0xE0 ) >> 5 );
    AZX_LOG_INFO( "Gyro: sample rate %d, range %d, int %d pwr %d\r\n",
                  phy_gyro.gyro_sample_rate, phy_gyro.gyro_dynamic_range, phy_gyro.gyro_flag & 0x01,
                  ( phy_gyro.gyro_flag & 0xE0 ) >> 5 );
    AZX_LOG_INFO( "Mag : sample rate %d, range %d, int %d pwr %d\r\n",
                  phy_mag.mag_sample_rate, phy_mag.mag_dynamic_range, phy_mag.mag_flag & 0x01,
                  ( phy_mag.mag_flag & 0xE0 ) >> 5 );
    AZX_LOG_INFO( "" );

    /* read custom sensor event size from hub for later fifo parse */
    bhy_sync_cus_evt_size();
    AZX_LOG_INFO( "CUS evt size = %d %d %d %d %d\r\n", bhy_get_cus_evt_size( VS_TYPE_CUS1 ),
                   bhy_get_cus_evt_size( VS_TYPE_CUS2 ), \
                   bhy_get_cus_evt_size( VS_TYPE_CUS3 ), bhy_get_cus_evt_size( VS_TYPE_CUS4 ), \
                   bhy_get_cus_evt_size( VS_TYPE_CUS5 ) );


  /* get virtual sensor information from sensor hub */
  AZX_LOG_TRACE( "Supported Virtual Sensor Information:\r\n" );

  for( i = 1; i < 32; i++ )
  {
    bhy_get_wakeup_sensor_information( i, &sensor_info_wakeup );

    if( sensor_info_wakeup.wakeup_sensor_type == i )
    {
      AZX_LOG_TRACE( "id=%2d\r\n", i );
    }
  }

  for( i = 33; i < 64; i++ )
  {
    bhy_get_non_wakeup_sensor_information( i, &sensor_info_non_wakeup );

    if( sensor_info_non_wakeup.non_wakeup_sensor_type == i )
    {
      AZX_LOG_TRACE( "id=%2d\r\n", i );
    }
  }

  /* enables the virtual sensor */
  bhy_enable_virtual_sensor( VS_TYPE_CUS2, VS_WAKEUP, 50, 0, VS_FLUSH_ALL, 0,
                             0 ); // 50Hz for VirtAnyNoMotion
  bhy_enable_virtual_sensor( VS_TYPE_CUS3, VS_WAKEUP, 200, 0, VS_FLUSH_ALL, 0,
                             0 ); // 200Hz for VirtSmartLock
  AZX_LOG_INFO( "Virtual Sensors Enabled\r\n" );


  /* get physical sensor status from sensor hub */
  bhy_get_physical_sensor_status( &phy_acc, &phy_gyro, &phy_mag );
  /*  Physical Sensor Status:
    Flags[bit 0]:  interrupt enable
    Flags[bits 5-7]: Sensor Power Mode values:
      0: Sensor Not Present
      1: Power Down
      2: Suspend
      3: Self-Test
      4: Interrupt Motion
      5: One Shot
      6: Low Power Active
      7: Active
  */
  AZX_LOG_INFO( "Physical Sensor Status:\r\n" );
  AZX_LOG_INFO( "Acc : sample rate %d, range %d, int %d pwr %d\r\n",
                phy_acc.accel_sample_rate, phy_acc.accel_dynamic_range, phy_acc.accel_flag & 0x01,
                ( phy_acc.accel_flag & 0xE0 ) >> 5 );
  AZX_LOG_INFO( "Gyro: sample rate %d, range %d, int %d pwr %d\r\n",
                phy_gyro.gyro_sample_rate, phy_gyro.gyro_dynamic_range, phy_gyro.gyro_flag & 0x01,
                ( phy_gyro.gyro_flag & 0xE0 ) >> 5 );
  AZX_LOG_INFO( "Mag : sample rate %d, range %d, int %d pwr %d\r\n",
                phy_mag.mag_sample_rate, phy_mag.mag_dynamic_range, phy_mag.mag_flag & 0x01,
                ( phy_mag.mag_flag & 0xE0 ) >> 5 );
  AZX_LOG_INFO( "" );

  /* read custom sensor event size from hub for later fifo parse */
  bhy_sync_cus_evt_size();
  AZX_LOG_INFO( "CUS evt size = %d %d %d %d %d\r\n", bhy_get_cus_evt_size( VS_TYPE_CUS1 ),
                 bhy_get_cus_evt_size( VS_TYPE_CUS2 ), \
                 bhy_get_cus_evt_size( VS_TYPE_CUS3 ), bhy_get_cus_evt_size( VS_TYPE_CUS4 ), \
                 bhy_get_cus_evt_size( VS_TYPE_CUS5 ) );

  write_LED( M2MB_GPIO_HIGH_VALUE );
  azx_sleep_ms( 100 );
  write_LED( M2MB_GPIO_LOW_VALUE );
  azx_sleep_ms( 100 );
  write_LED( M2MB_GPIO_HIGH_VALUE );
  azx_sleep_ms( 100 );
  write_LED( M2MB_GPIO_LOW_VALUE );
  azx_sleep_ms( 100 );

  while( 1 )
  {
    /* wait until the interrupt fires */
    /* unless we already know there are bytes remaining in the fifo */
    while( !read_gpio() && !bytes_remaining )
    {
      azx_sleep_ms( 10 );
    }

    bhy_read_fifo( fifo + bytes_left_in_fifo, FIFO_SIZE - bytes_left_in_fifo, &bytes_read,
                   &bytes_remaining );
    bytes_read           += bytes_left_in_fifo;
    fifoptr              = fifo;
    packet_type          = BHY_DATA_TYPE_PADDING;

    do
    {
      /* this function will call callbacks that are registered */
      result = bhy_parse_next_fifo_packet( &fifoptr, &bytes_read, &fifo_packet, &packet_type );
      AZX_LOG_TRACE( "Result %d, PackType :%d", result, packet_type );

      if( AZX_LOG_LEVEL_TRACE >= azx_log_getLevel() )
      {
        /* prints all the debug packets */
        if( packet_type == BHY_DATA_TYPE_PADDING )
        {
          /* padding data only added at the end of each FIFO dump, discard it. */

            AZX_LOG_INFO( ">Padding\r\n" );

        }
        else
        if( packet_type == BHY_DATA_TYPE_DEBUG )
        {

            AZX_LOG_INFO( ">DebugString       : " );
            bhy_print_debug_packet( &fifo_packet.data_debug, bhy_printf );
            AZX_LOG_INFO( "\r\n" );

        }
        else
        {
            AZX_LOG_INFO( "\r\n" );
        }
      }
      /* the logic here is that if doing a partial parsing of the fifo, then we should not parse  */
      /* the last 18 bytes (max length of a packet) so that we don't try to parse an incomplete   */
      /* packet */
      azx_sleep_ms( 10 );
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

    azx_sleep_ms( 10 );
  }
}



/* Global functions =============================================================================*/

/*-----------------------------------------------------------------------------------------------*/
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
  AZX_LOG_INFO( "Starting SmartLock Demo app. This is v%s built on %s %s.\r\n",
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

  if( oneedge_init( SMARTLOCK_OBJ_ID ) != 0 )
  {
    AZX_LOG_ERROR( "Failed enabling LWM2M!\r\n" );
    return;
  }
  else
  {
    AZX_LOG_INFO("LWM2M Ready, start sensors demo.\r\n\r\n");
    demo_sensor();
  }

}
