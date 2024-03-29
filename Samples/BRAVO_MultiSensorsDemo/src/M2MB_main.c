/*=========================================================================*/
/*   Copyright (C) Telit Communications S.p.A. Italy All Rights Reserved.  */
/**
  @file
    M2MB_main.c

  @brief
    The file contains the main user entry point of Appzone

  @details

  @description
    MultiSensors Demo application. Debug prints on MAIN UART
  @version
    1.0.3
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
#include "azx_tasks.h"

#include "app_cfg.h"



#include "gpio.h"
#include "i2c.h"
#include "lwm2m.h"


#include "sensors_demo.h"


#ifndef BRAVO_REV_B
#include "i2c_bme680.h"
#endif


/* Local defines ================================================================================*/

/* Local typedefs ===============================================================================*/
/* Local statics ================================================================================*/
INT32 bsens_taskId;
M2MB_OS_TASK_HANDLE Task_h;
/* Local function prototypes ====================================================================*/

/**
   @brief           callback function for main demo task, executed by main function.

   @return          0 in case of success, -1 in case of failure
*/
static INT32 demoTaskCb(INT32 type, INT32 param1, INT32 param2);


/* Static functions =============================================================================*/

static INT32 demoTaskCb(INT32 type, INT32 param1, INT32 param2)
{
  (void) param1;
  (void) param2;

  switch(type)
  {
    case TASK_INIT:
    {
#ifndef SKIP_LWM2M
      int reboot_needed = 0;

      INT16 instances[] = {0};
      LWM2M_OBJ_REG_T objs[] = {
      {TAMPERING_OBJ_ID, 1, instances },
      {ROTATION_OBJ_ID, 1, instances },
      {ENVIRONMENT_OBJ_ID, 1, instances }
      };
#endif
      /* Open GPIO */
      if( open_LED( LED_INDEX_NUM ) != 0 )
      {
        AZX_LOG_ERROR( "Cannot open gpio channel.\r\n" );
        return -1;
      }

      write_LED( M2MB_GPIO_HIGH_VALUE );
      azx_sleep_ms( 5000 );
      write_LED( M2MB_GPIO_LOW_VALUE );

#ifndef SKIP_LWM2M
      /* Copy xml file if not exixting */
      if( 0 != check_xml_file( TAMPER_XML_NAME ) )
      {
        if( 0 != copy_xml_file( TAMPER_XML_NAME ) )
        {
          AZX_LOG_CRITICAL( "Failed copying file!\r\n" );

          for( int i = 0; i < 10; i++ )
          {
            write_LED( M2MB_GPIO_HIGH_VALUE );
            azx_sleep_ms( 200 );
            write_LED( M2MB_GPIO_LOW_VALUE );
            azx_sleep_ms( 200 );
          }

          return -1;
        }
        reboot_needed = 1;
      }

      /* Copy xml file if not exixting */
      if( 0 != check_xml_file( ROTATION_XML_NAME ) )
      {
        if( 0 != copy_xml_file( ROTATION_XML_NAME ) )
        {
          AZX_LOG_CRITICAL( "Failed copying file!\r\n" );

          for( int i = 0; i < 10; i++ )
          {
            write_LED( M2MB_GPIO_HIGH_VALUE );
            azx_sleep_ms( 200 );
            write_LED( M2MB_GPIO_LOW_VALUE );
            azx_sleep_ms( 200 );
          }

          return -1;
        }
        reboot_needed = 1;
      }

      if( 0 != check_xml_file( ENVIRONMENT_XML_NAME ) )
      {
        if( 0 != copy_xml_file( ENVIRONMENT_XML_NAME ) )
        {
          AZX_LOG_CRITICAL( "Failed copying file!\r\n" );

          for( int i = 0; i < 10; i++ )
          {
            write_LED( M2MB_GPIO_HIGH_VALUE );
            azx_sleep_ms( 200 );
            write_LED( M2MB_GPIO_LOW_VALUE );
            azx_sleep_ms( 200 );
          }
          return -1;
        }
        reboot_needed = 1;
      }

      if(reboot_needed)
      {
        M2MB_POWER_HANDLE handle;
        AZX_LOG_DEBUG( "Rebooting to apply xml file\r\n" );

        if( M2MB_RESULT_SUCCESS != m2mb_power_init( &handle, NULL, ( void * ) NULL ) )
        {
          AZX_LOG_CRITICAL( "cannot init power module!\r\n" );
          return -1;
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
          return 0;
        }
      }

#endif

      /* Open I2C */
      if( open_I2C() != 0 )
      {
        AZX_LOG_ERROR( "cannot open I2C channel.\r\n" );
        return -1;
      }

      /* Open GPIO */
      if( open_gpio( INT_GPIO_PIN_NUM ) != 0 )
      {
        AZX_LOG_ERROR( "cannot open gpio channel.\r\n" );
        return -1;
      }
#ifndef SKIP_LWM2M
      if(oneedge_init( objs, 3, NULL ) != 0)
      {
        AZX_LOG_ERROR("Failed enabling LWM2M!\r\n");
        return -1;
      }
#else
      AZX_LOG_INFO( "Will run without LWM2M data publishing\r\n");
#endif
      Task_h = m2mb_os_taskGetId();
      AZX_LOG_DEBUG("init sensors...\r\n");
      init_sensors();

       /*Store task handle for watchdog*/
      WDog_Init( Task_h);
      azx_tasks_sendMessageToTask(bsens_taskId, TASK_LOOP, DO_NOT_RESET, 0);
    }
    break;
  case TASK_LOOP:
    run_sensors_loop(param1);
    break;
  case TASK_REINIT:
    close_I2C();
    /* Open I2C */
    if( open_I2C() != 0 )
    {
      AZX_LOG_ERROR( "cannot open I2C channel.\r\n" );
      return -1;
    }
    #ifndef BRAVO_REV_B
    bme680_close_I2C();
    /*I2C will be reopened by read/write operations*/
    #endif
    azx_tasks_sendMessageToTask(bsens_taskId, TASK_LOOP, DO_RESET /*reset FIFO and parameters*/, 0);
    break;
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

  azx_tasks_init();

  /* SET output channel */
  AZX_LOG_INIT();
  AZX_LOG_INFO( "Starting Tampering + Rotation + Environment Demo app. This is v%s built on %s %s.\r\n",
      VERSION, __DATE__, __TIME__ );


  bsens_taskId = azx_tasks_createTask((char*) "BSENS" , AZX_TASKS_STACK_XL, 4, AZX_TASKS_MBOX_S, demoTaskCb);
  if(bsens_taskId > 0)
  {

    azx_tasks_sendMessageToTask(bsens_taskId, TASK_INIT, 0, 0);
  }
  else
  {
    return;
  }
}


