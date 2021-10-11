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
    1.0.2
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
#include "m2mb_socket.h"

#include "m2mb_rtc.h"

#include "m2mb_fs_posix.h"
#include "m2mb_fs_errno.h"

#include "azx_log.h"
#include "azx_utils.h"
#include "azx_tasks.h"

#include "app_cfg.h"



#include "gpio.h"
#include "i2c.h"
#ifndef SKIP_LWM2M
#include "lwm2m.h"
#endif

#include "sensors_demo.h"

/* Local defines ================================================================================*/

#define PDU_SIZE 14
#define OPAQUE_SIZE 1024


#define PRINT_PDU 0
#define PRINT_PDU_HEX 1
#define PRINT_OPAQUE_HEX 0

#ifndef SKIP_LWM2M
  #define ONE_EDGE 1
#else
  #define ONE_EDGE 0
#endif


#define DEFAULT_SAMPLE_PERIOD 60000 //60000
#define DEFAULT_SENDING_PERIOD 300000 //300000


#define EV_B_SENSORS_BIT 0x00000001


#ifndef BOSCH_BSEC
#define BOSCH_BSEC 1
#endif


/* Local typedefs ===============================================================================*/

typedef struct
{
  UINT32 timestamp;
  float temperature;
  UINT8 humidity;
  UINT16 pressure;
  UINT16 engine_rot;
  UINT8 event_counter;
} DATA_ENTRY_T;

typedef struct
{
  UINT8 buffer[OPAQUE_SIZE];
  UINT16 p;
  M2MB_OS_SEM_HANDLE CSSemH;
} DATA_BUFFER_T;

/* Local statics ================================================================================*/
static BOOLEAN is_a_bravo = TRUE;
static M2MB_OS_EV_HANDLE gBoschInitEvH = NULL;


static UINT32 gSamplingRate = DEFAULT_SAMPLE_PERIOD;
#if ONE_EDGE
static UINT32 gSendingRate = DEFAULT_SENDING_PERIOD;


static M2MB_LWM2M_OBJ_URI_T _obj_telit_app_data_uri = { .uriLen = 4,
    .obj = 33205, .objInst = 0, .resource = 0, .resourceInst = 0 };
#endif

static DATA_BUFFER_T opaque = {0};

/* Local function prototypes ====================================================================*/

static UINT32 get_uptime(void);

/**
   @brief           callback function for main demo task, executed by main function.

   @return          0 in case of success, -1 in case of failure
 */
static INT32 demoTaskCb(INT32 type, INT32 param1, INT32 param2);

static INT32 entryToHexBuf(DATA_ENTRY_T *entry, unsigned char *tmp);
#if ONE_EDGE
static INT32 dataSendTaskCb(INT32 type, INT32 param1, INT32 param2);
static INT32 lwm2m_monDataTaskCb(INT32 type, INT32 param1, INT32 param2);
#endif

static INT32 dataReadTaskCb(INT32 type, INT32 param1, INT32 param2);
static INT32 sensorsInitCB(INT32 res, void *arg);
/* Static functions =============================================================================*/

static UINT32 get_uptime(void)
{

  UINT32 sysTicks = m2mb_os_getSysTicks();

  FLOAT32 ms_per_tick = m2mb_os_getSysTickDuration_ms();

  return (UINT32) (sysTicks * ms_per_tick); //milliseconds
}

static INT32 entryToHexBuf(DATA_ENTRY_T *entry, unsigned char *tmp)
{
  UINT32 word_tmp;
  UINT16 nibble_tmp;

  union
  {
    float f;
    UINT32 u;
  } f2u = { .f = entry->temperature };

  word_tmp = m2mb_socket_bsd_htonl(entry->timestamp);
  memcpy(&tmp[0], &word_tmp, sizeof(word_tmp));

  word_tmp = m2mb_socket_bsd_htonl(f2u.u);
  memcpy(&tmp[4], &word_tmp, sizeof(word_tmp));

  memcpy(&tmp[8], &entry->humidity, sizeof(entry->humidity));

  nibble_tmp = m2mb_socket_bsd_htons(entry->pressure);
  memcpy(&tmp[9], &nibble_tmp, sizeof(nibble_tmp));

  nibble_tmp = m2mb_socket_bsd_htons(entry->engine_rot);
  memcpy(&tmp[11], &nibble_tmp, sizeof(nibble_tmp));

  memcpy(&tmp[13], &entry->event_counter, sizeof(entry->event_counter));

  return 0;
}

#if ONE_EDGE
static INT32 dataSendTaskCb(INT32 type, INT32 param1, INT32 param2)
{
  (void) type;
  (void) param1;
  (void) param2;

  UINT32                  curEvBits;
  UINT32 delay = 0;

  AZX_LOG_INFO("Send task. Wait for Bosch sensors check.\r\n");
  m2mb_os_ev_get(gBoschInitEvH, EV_B_SENSORS_BIT, M2MB_OS_EV_GET_ANY, &curEvBits, M2MB_OS_WAIT_FOREVER);


  while(1)
  {
    UINT32 starting_time = get_uptime();

    AZX_LOG_TRACE("Starting_time: %u\r\n", starting_time);

    UINT32 sleep_time = gSendingRate - delay;

    /* sleep_time could be "negative"*/
    if( delay > gSendingRate )
    {
      sleep_time = DEFAULT_SENDING_PERIOD;
    }
    AZX_LOG_TRACE("Waiting for %u ms\r\n", sleep_time);

    azx_sleep_ms(sleep_time);

    m2mb_os_sem_get(opaque.CSSemH, M2MB_OS_WAIT_FOREVER );

    delay =  (get_uptime() - starting_time ) - gSendingRate;

    AZX_LOG_INFO("Time to send data!\r\n");
    /*send opaque with lwm2m and wait ack*/
    /*todo check result value */
    set_opaque_resource(getLwM2mHandle(), &_obj_telit_app_data_uri, opaque.buffer, opaque.p);

    /*reset*/
    memset(opaque.buffer, 0, OPAQUE_SIZE);
    opaque.p = 0;

    m2mb_os_sem_put(opaque.CSSemH);
  }
}
#endif

static INT32 dataReadTaskCb(INT32 type, INT32 param1, INT32 param2)
{
  (void) type;
  (void) param1;
  (void) param2;

  DATA_ENTRY_T entry = {0};
  INT32 rtcfd;
  unsigned char tmp[PDU_SIZE + 1 ] = {0};

  UINT32                  curEvBits;
  UINT32 delay = 0;


  AZX_LOG_INFO("Read task. Waiting for Bosch sensors check.\r\n");
  m2mb_os_ev_get(gBoschInitEvH, EV_B_SENSORS_BIT, M2MB_OS_EV_GET_ANY, &curEvBits, M2MB_OS_WAIT_FOREVER);

  while (1)
  {

    UINT32 starting_time = get_uptime();
    AZX_LOG_TRACE("starting_time: %u\r\n", starting_time);


    UINT32 sleep_time = gSamplingRate - delay;

    /* sleep_time could be "negative"*/
    if( delay > gSamplingRate )
    {
      sleep_time = DEFAULT_SAMPLE_PERIOD;
    }

    AZX_LOG_TRACE("Waiting for %u ms\r\n", sleep_time);
    azx_sleep_ms(sleep_time);

    if(is_a_bravo)
    {
      void * pdata = NULL;
      BSENS_ENV_T *pENVIRON_data;
      BSENS_TAMPER_T *pTAMPER_data;
      BSENS_3DVECT_T *pVECT_data;

      AZX_LOG_INFO("\r\n\r\n==== Reading data from sensors...\r\n\r\n");

      read_sensor(BSENS_SENSOR_ENVIRONM_ID, (void **)&pdata);

      pENVIRON_data = (BSENS_ENV_T *) pdata;
      entry.temperature = pENVIRON_data->temperature;
      entry.humidity = (UINT8) pENVIRON_data->humidity;
      entry.pressure = (UINT16) pENVIRON_data->pressure;


      read_sensor(BSENS_SENSOR_TAMPER_ID, (void **)&pdata);
      pTAMPER_data = (BSENS_TAMPER_T *) pdata;
      entry.event_counter = pTAMPER_data->still_ended_count;

      read_sensor(BSENS_SENSOR_3D_VECT_ID, (void **)&pdata);
      pVECT_data = (BSENS_3DVECT_T *) pdata;

      entry.engine_rot = (UINT16) (pVECT_data->w * 1000); /*axis is a float from 0 to 1.*/

    }
    else
    {
      static int sign = 1;
      static BOOLEAN is_init = FALSE;
      if (!is_init)
      {
        AZX_LOG_INFO("Initialize simulated data...\r\n");
        entry.timestamp = 0;
        entry.temperature = 25;
        entry.humidity = 100;
        entry.pressure = 1000;
        entry.engine_rot = 300;
        entry.event_counter = 125;
        is_init = TRUE;
      }
      else
      {
        AZX_LOG_INFO("Simulating data...\r\n");
        entry.temperature++;
        if(entry.temperature >= 100.0)
        {
          entry.temperature = 25.0;
        }

        if(entry.humidity >= 2)
        {
          entry.humidity -= 2;
        }
        if(entry.humidity <= 0)
        {
          entry.humidity = 100;
        }
        
        entry.pressure += sign;
        if(entry.pressure > 1100)
        {
          sign = -1;
        }
        if(entry.pressure < 900)
        {
          sign = 1;
        }

        entry.engine_rot +=25;
        if(entry.engine_rot > 600)
        {
          entry.engine_rot = 300;
        }

        entry.event_counter += 5;
        if (entry.event_counter >= 255)
        {
          entry.event_counter = 125;
        }
      }
    }

    rtcfd = m2mb_rtc_open( "/dev/rtc0", 0 );
    if(-1 == rtcfd)
    {
      AZX_LOG_CRITICAL("cannot open rtc to get timestamp!!!\r\n");
    }
    else
    {
      M2MB_RTC_TIMEVAL_T timeval;
      M2MB_RTC_TIME_T time;
      m2mb_rtc_ioctl( rtcfd, M2MB_RTC_IOCTL_GET_TIMEVAL, &timeval );
      m2mb_rtc_ioctl( rtcfd, M2MB_RTC_IOCTL_GET_SYSTEM_TIME, &time );
      m2mb_rtc_close(rtcfd);

      //AZX_LOG_DEBUG("timezone: %d, dst: %u\r\n", time.tz, time.dlst);

      entry.timestamp = timeval.sec - (time.tz * 15 * 60); /*compensate timezone*/
    }

    if(PRINT_PDU)
    {
      AZX_LOG_INFO("\r\nData entry: \r\n");
      AZX_LOG_INFO("time : %u\r\n", entry.timestamp);
      AZX_LOG_INFO("temp : %.2f\r\n", entry.temperature);
      AZX_LOG_INFO("humi : %u\r\n", entry.humidity);
      AZX_LOG_INFO("pres : %u\r\n", entry.pressure);
      AZX_LOG_INFO("engi : %u\r\n", entry.engine_rot);
      AZX_LOG_INFO("event: %u\r\n", entry.event_counter);
      AZX_LOG_INFO("\r\n");
    }

    /* Store data into final buffer */

    entryToHexBuf(&entry, tmp);

    if(PRINT_PDU_HEX){
      int i;
      AZX_LOG_INFO("\r\n \r\nData: ");
      for (i=0; i<PDU_SIZE; i++)
      {
        AZX_LOG_INFO("%02X", tmp[i]);
      }
      AZX_LOG_INFO("\r\n\r\n");
    }

    m2mb_os_sem_get(opaque.CSSemH, M2MB_OS_WAIT_FOREVER );

    delay =  (get_uptime() - starting_time ) - gSamplingRate;
    AZX_LOG_TRACE("Delay: %u\r\n", delay);
    memcpy(&opaque.buffer[opaque.p], tmp, PDU_SIZE);
    opaque.p += PDU_SIZE;

    /*check if the remaining size fits other PDUs*/
    if((OPAQUE_SIZE - opaque.p) < PDU_SIZE)
    {
      AZX_LOG_INFO("Reset buffer counter\r\n");
      /*reset*/
      opaque.p = 0;
    }

    if(PRINT_OPAQUE_HEX)
    {
      int i;
      AZX_LOG_INFO("\r\nOPAQUE: ");
      for (i=0; i<OPAQUE_SIZE; i++)
      {
        AZX_LOG_INFO("%02X", opaque.buffer[i]);
      }
      AZX_LOG_INFO("\r\n");
    }

    m2mb_os_sem_put(opaque.CSSemH);

  }
}

#if ONE_EDGE
static INT32 lwm2m_monDataTaskCb(INT32 type, INT32 param1, INT32 param2)
{
  (void) param2;

  switch(type)
  {
  case EV_MON_URC_RECEIVED:
  {
    INT32 val;
    M2MB_LWM2M_OBJ_URI_T *p_uri = (M2MB_LWM2M_OBJ_URI_T *) param1;
    M2MB_LWM2M_OBJ_URI_T uri;
    uri.obj = p_uri->obj;
    uri.objInst = 0;
    uri.resourceInst = 0;
    uri.uriLen = M2MB_LWM2M_URI_3_FIELDS;

    uri.resource = p_uri->resource;
    
    
    if(M2MB_RESULT_SUCCESS != read_integer_from_uri(getLwM2mHandle(), &uri, &val))
    {
      AZX_LOG_ERROR("cannot read integer from uri %u/%u/%u/%u !\r\n",
          p_uri->obj, p_uri->objInst, p_uri->resource, p_uri->resourceInst );
    }
    else
    {
      AZX_LOG_INFO("Received new integer value %d for uri %u/%u/%u/%u\r\n", val,
          p_uri->obj, p_uri->objInst, p_uri->resource, p_uri->resourceInst);

      switch(p_uri->obj)
      {
      case TIME_SERIES_METERING_RATES_OBJ_ID:
        switch( p_uri->resource)
        {
        case 1:
          AZX_LOG_INFO("Sampling rate: %d\r\n", val);
          if(val != 0)
          {
            gSamplingRate = val * 1000;
          }
          else
          {
            /* overwrite the value with the default */
            write_integer_resource(getLwM2mHandle(), p_uri, DEFAULT_SAMPLE_PERIOD / 1000);
            gSamplingRate = DEFAULT_SAMPLE_PERIOD;
          }
          break;
        case 2:
          AZX_LOG_INFO("Sending rate: %d\r\n", val);
          if(val != 0)
          {
            gSendingRate = val * 1000;
          }
          else
          {
            /* overwrite the value with the default */
            write_integer_resource(getLwM2mHandle(), p_uri, DEFAULT_SENDING_PERIOD / 1000);
            gSendingRate = DEFAULT_SENDING_PERIOD;
          }
          break;
        default:
          break;
        }

        break;
        default:
          break;
      }

    }
  }break;
  }

  return 0;

}
#endif

static INT32 sensorsInitCB(INT32 res, void *arg)
{
  (void) arg;

  if(0 != res)
  {
    AZX_LOG_WARN("Not a Bravo board, sensors data will be simulated\r\n");
    is_a_bravo = FALSE;
  }
  m2mb_os_ev_set(gBoschInitEvH, EV_B_SENSORS_BIT, M2MB_OS_EV_SET);
  return 0;
}


static INT32 demoTaskCb(INT32 type, INT32 param1, INT32 param2)
{
  (void) type;
  (void) param1;
  (void) param2;
  INT32 taskId;

#if ONE_EDGE
  int reboot_needed = 0;
  INT16 instances[] = {0};

  LWM2M_OBJ_REG_T objs[] = {
      {TIME_SERIES_METERING_INFO_OBJ_ID, 1, instances },
      {TIME_SERIES_METERING_RATES_OBJ_ID, 1, instances }
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

#if ONE_EDGE
  /* Copy xml file if not existing */
  if( 0 != check_xml_file( SENDING_OPTS_XML_NAME ) )
  {
    if( 0 != copy_xml_file( SENDING_OPTS_XML_NAME ) )
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

  /* Copy xml file if not existing */
  if( 0 != check_xml_file( SAMPLING_OPTS_XML_NAME ) )
  {
    if( 0 != copy_xml_file( SAMPLING_OPTS_XML_NAME ) )
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
    AZX_LOG_DEBUG( "Rebooting to apply xml file(s)\r\n" );

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


#if BOSCH_BSEC

  /* Open I2C */
  if( open_I2C() != 0 )
  {
    AZX_LOG_ERROR( "cannot open I2C channel.\r\n" );
    return -1;
  }
#endif

  /* Open GPIO */
  if( open_gpio( INT_GPIO_PIN_NUM ) != 0 )
  {
    AZX_LOG_ERROR( "cannot open gpio channel.\r\n" );
    return -1;
  }
  
#if ONE_EDGE

  /*skip the instances creation*/
  setAsBootstrapped();

  if(oneedge_init( objs, 2, NULL ) != 0)
  {
    AZX_LOG_ERROR("Failed enabling LWM2M!\r\n");
    return -1;
  }

  taskId = azx_tasks_createTask((char*) "MON_DATA", AZX_TASKS_STACK_L, 5, AZX_TASKS_MBOX_S, lwm2m_monDataTaskCb);
  if(taskId > 0)
  {
    lwm2m_ReadTaskRegister(taskId);
  }
  else
  {
    return -1;
  }


  /*read values from time series objects for sampling and sending rates*/
  {
    M2MB_LWM2M_OBJ_URI_T uri;
    INT32 val;
    uri.obj = TIME_SERIES_METERING_RATES_OBJ_ID;
    uri.objInst = 0;
    uri.resourceInst = 0;
    uri.uriLen = M2MB_LWM2M_URI_3_FIELDS;

    uri.resource = 1;
    if(M2MB_RESULT_SUCCESS != read_integer_from_uri(getLwM2mHandle(), &uri, &val))
    {
      AZX_LOG_ERROR("cannot read integer from uri %u/%u/%u/%u !\r\n", uri.obj, uri.objInst, uri.resource, uri.resourceInst );
    }
    else
    {
      AZX_LOG_INFO("sampling rate : %d\r\n", val);
      if(val != 0)
      {
        gSamplingRate = val * 1000;
      }
      else
      {
        /* overwrite the value with the default */
        write_integer_resource(getLwM2mHandle(), &uri, DEFAULT_SAMPLE_PERIOD / 1000);
        gSamplingRate = DEFAULT_SAMPLE_PERIOD;
      }

    }

    azx_sleep_ms(3000);

    uri.resource = 2;
    if(M2MB_RESULT_SUCCESS != read_integer_from_uri(getLwM2mHandle(), &uri, &val))
    {
      AZX_LOG_ERROR("cannot read integer from uri %u/%u/%u/%u !\r\n", uri.obj, uri.objInst, uri.resource, uri.resourceInst );
    }
    else
    {
      AZX_LOG_INFO("sending rate : %d\r\n", val);
      if(val != 0)
      {
        gSendingRate = val * 1000;
      }
      else
      {
        /* overwrite the value with the default */
        write_integer_resource(getLwM2mHandle(), &uri, DEFAULT_SENDING_PERIOD / 1000);
        gSendingRate = DEFAULT_SENDING_PERIOD;
      }

    }
  }

#else
  AZX_LOG_INFO( "Will run without LWM2M data publishing\r\n");
#endif



  {
    /*Critical Section for opaque buffer*/

    M2MB_OS_SEM_ATTR_HANDLE semAttrHandle;

    m2mb_os_sem_setAttrItem(&semAttrHandle,
        CMDS_ARGS(M2MB_OS_SEM_SEL_CMD_CREATE_ATTR, NULL,
            M2MB_OS_SEM_SEL_CMD_COUNT, 1 /*CS*/,
            M2MB_OS_SEM_SEL_CMD_TYPE, M2MB_OS_SEM_GEN));
    m2mb_os_sem_init( &opaque.CSSemH, &semAttrHandle );

    /*Init opaque*/
    memset(opaque.buffer, 0, OPAQUE_SIZE);
    opaque.p = 0;
  }

  {
    M2MB_OS_RESULT_E        osRes;
    M2MB_OS_EV_ATTR_HANDLE evAttrHandle;
    m2mb_os_ev_setAttrItem( &evAttrHandle, CMDS_ARGS(M2MB_OS_EV_SEL_CMD_CREATE_ATTR, NULL));
    osRes = m2mb_os_ev_init( &gBoschInitEvH, &evAttrHandle );
    if ( osRes != M2MB_OS_SUCCESS )
    {
      m2mb_os_ev_setAttrItem( &evAttrHandle, M2MB_OS_EV_SEL_CMD_DEL_ATTR, NULL );
      AZX_LOG_CRITICAL("m2mb_os_ev_init failed!\r\n");
      return -1;
    }
  }


  taskId = azx_tasks_createTask((char*) "read" , AZX_TASKS_STACK_L, 2, AZX_TASKS_MBOX_S, dataReadTaskCb);
  if(taskId > 0)
  {
    azx_tasks_sendMessageToTask(taskId, 0, 0, 0);
  }
  else
  {
    return -1;
  }
#if ONE_EDGE
  taskId = azx_tasks_createTask((char*) "read" , AZX_TASKS_STACK_L, 3, AZX_TASKS_MBOX_S, dataSendTaskCb);
  if(taskId > 0)
  {
    azx_tasks_sendMessageToTask(taskId, 0, 0, 0);
  }
  else
  {
    return -1;
  }
#endif

#if BOSCH_BSEC
  AZX_LOG_DEBUG("Init sensors...\r\n");
  init_sensors(sensorsInitCB);
#else
  /*skip the sensors initialization*/
  sensorsInitCB(-1, NULL);
#endif
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
  INT32 taskId;
  azx_tasks_init();

  /* SET output channel */
  AZX_LOG_INIT();
  AZX_LOG_INFO( "Starting Time Series Demo app. This is v%s built on %s %s.\r\n",
      VERSION, __DATE__, __TIME__ );


  taskId = azx_tasks_createTask((char*) "BSENS" , AZX_TASKS_STACK_XL, 1, AZX_TASKS_MBOX_S, demoTaskCb);
  if(taskId > 0)
  {

    azx_tasks_sendMessageToTask(taskId, 0, 0, 0);
  }
  else
  {
    return;
  }
}


