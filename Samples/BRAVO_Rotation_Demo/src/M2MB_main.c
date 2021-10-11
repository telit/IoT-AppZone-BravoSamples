/*=========================================================================*/
/*   Copyright (C) Telit Communications S.p.A. Italy All Rights Reserved.  */
/**
  @file
    M2MB_main.c

  @brief
    The file contains the main user entry point of Appzone

  @details

  @description
    Rotation Demo application. Debug prints on MAIN UART
  @version
    1.0.4
  @note
    Start of Appzone: Entry point
    User code entry is in function M2MB_main()

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


#include "bhy_support.h"
#include "bhy_uc_driver.h"
#include "bosch_pcb_7183_di03_bmi160_bmm150-7183_di03-2-1-11696_20180502.h"

#include "gpio.h"
#include "i2c.h"

#ifndef SKIP_LWM2M
#include "lwm2m.h"
#endif

/* Local defines ================================================================================*/

/*
   Bosch device
*/
#define INT_GPIO_PIN_NUM 6
#define LED_INDEX_NUM 2 /* GPIO 10 */

#define SENSOR_AR_TOUT  100  /* 10 = 1 sec  */

#ifndef SKIP_LWM2M
#define XML_NAME "object_26250.xml"
#endif



#define FIFO_SIZE                      300
#define MAX_PACKET_LENGTH              18
#define TICKS_IN_ONE_SECOND            32000.0F

#define ROTATION_VECTOR_SAMPLE_RATE    10  /* Hz */


/* Local typedefs ===============================================================================*/
/* Local statics ================================================================================*/
static uint8_t fifo[FIFO_SIZE];


/* Local function prototypes ====================================================================*/
/*-----------------------------------------------------------------------------------------------*/
/**
   @brief This is the callback function used to acquire sensor data

   @param[in]   sensor_data
   @param[in]   sensor_id

*/
static void sensors_callback_rotation_vector( bhy_data_generic_t *sensor_data,
                                              bhy_virtual_sensor_t sensor_id );


/*-----------------------------------------------------------------------------------------------*/
/**
   @brief This function is used to run bhy hub
*/
static void demo_sensor( void );


/* Static functions =============================================================================*/

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
#ifndef SKIP_LWM2M
    update_rotation_LWM2MObject( w, x, y, z, sensor_data->data_quaternion.estimated_accuracy );
#endif
    i = 0;
    AZX_LOG_INFO( "-------> x=%f, y=%f, z=%f, w=%f; acc=%d\r\n", 
      x, y, z, w, sensor_data->data_quaternion.estimated_accuracy );  
  }

  old_w = sensor_data->data_quaternion.w;
  old_x = sensor_data->data_quaternion.x;
  old_y = sensor_data->data_quaternion.y;
  old_z = sensor_data->data_quaternion.z;
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
  //struct cus_version_t       bhy_cus_version;
  AZX_LOG_INFO( "version=%s\r\n", bhy_get_version() );

  /* init the bhy chip */
  if( bhy_driver_init( bhy_firmware_image ) )
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
 
  /* The remapping matrix for BHI and Magmetometer should be configured here to make sure rotation vector is */
  /* calculated in a correct coordinates system. */
  bhy_mapping_matrix_set( PHYSICAL_SENSOR_INDEX_ACC, bhy_mapping_matrix_config );
  bhy_mapping_matrix_set( PHYSICAL_SENSOR_INDEX_MAG, mag_mapping_matrix_config );
  bhy_mapping_matrix_set( PHYSICAL_SENSOR_INDEX_GYRO, bhy_mapping_matrix_config );
  /* This sic matrix setting affects magnetometer fusion performance. */
  bhy_set_sic_matrix( sic_array );

  /* install the callback function for parse fifo data */
  if( bhy_install_sensor_callback( VS_TYPE_ROTATION_VECTOR, VS_WAKEUP,
                                   sensors_callback_rotation_vector ) )
  {
    AZX_LOG_ERROR( "Fail to install rotation sensor callback\r\n" );
    return;
  }

  /* enables the virtual sensor */
  if( bhy_enable_virtual_sensor( VS_TYPE_ROTATION_VECTOR, VS_WAKEUP, ROTATION_VECTOR_SAMPLE_RATE, 0,
                                 VS_FLUSH_NONE, 0, 0 ) )
  {
    AZX_LOG_ERROR( "Fail to enable rotation sensor id=%d\r\n", VS_TYPE_ROTATION_VECTOR );
    return;
  }
  write_LED( M2MB_GPIO_HIGH_VALUE );
  azx_sleep_ms( 100 );
  write_LED( M2MB_GPIO_LOW_VALUE );
  azx_sleep_ms( 100 );
  write_LED( M2MB_GPIO_HIGH_VALUE );
  azx_sleep_ms( 100 );
  write_LED( M2MB_GPIO_LOW_VALUE );
  azx_sleep_ms( 100 );
  
  AZX_LOG_INFO("Ready, move the board to update data\r\n");
  
  while( 1 )
  {
    /* wait until the interrupt fires */
    /* unless we already know there are bytes remaining in the fifo */
    while( !read_gpio()  && !bytes_remaining )
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

      /* prints all the debug packets */
      if( packet_type == BHY_DATA_TYPE_DEBUG )
      {
        bhy_print_debug_packet( &fifo_packet.data_debug, bhy_printf );
      }

      /* the logic here is that if doing a partial parsing of the fifo, then we should not parse  */
      /* the last 18 bytes (max length of a packet) so that we don't try to parse an incomplete   */
      /* packet */
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

  //    return BHY_SUCCESS;
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
  AZX_LOG_INFO( "%s", log_buffer );
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
  
#ifndef SKIP_LWM2M
  INT16 instances[] = {0};
  LWM2M_OBJ_REG_T obj = {ROTATION_OBJ_ID, 1, instances };
#endif

  /* SET output channel */
  AZX_LOG_INIT();
  AZX_LOG_INFO( "Starting Rotation Demo app. This is v%s built on %s %s.\r\n",
                VERSION, __DATE__, __TIME__ );

  /* Open GPIO */
  if( open_LED( LED_INDEX_NUM ) != 0 )
  {
    AZX_LOG_ERROR( "Cannot open gpio channel.\r\n" );
    return;
  }

  write_LED( M2MB_GPIO_HIGH_VALUE );
  azx_sleep_ms( 5000 );
  write_LED( M2MB_GPIO_LOW_VALUE );
  
#ifndef SKIP_LWM2M
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
#endif 
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

#ifndef SKIP_LWM2M
  if(oneedge_init( &obj, 1, NULL)  != 0)
  {
    AZX_LOG_ERROR("Failed enabling LWM2M!\r\n");
    return;
  }
#else
  AZX_LOG_INFO( "Will run without LWM2M data publishing\r\n");
#endif

  demo_sensor();
}
