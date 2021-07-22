/*Copyright (C) 2020 Telit Communications S.p.A. Italy - All Rights Reserved.*/
/*    See LICENSE file in the project root for full license information.     */

/**
  @file
    M2MB_main.c

  @brief
    The file contains the main user entry point of Appzone

  @details
  
  @description
	LED management through IPSO object 3311 Demo application. Debug prints on MAIN UART
  @version 
    1.0.0
  @note
    Start of Appzone: Entry point
    User code entry is in function M2MB_main()

  @author NormanAr


  @date
    27/05/2020
*/

/* Include files ================================================================================*/
#include <string.h>
#include "m2mb_types.h"
#include "m2mb_os_types.h"
#include "m2mb_os_api.h"
#include "m2mb_lwm2m.h"

#include "m2mb_net.h"
#include "m2mb_pdp.h"
#include "m2mb_socket.h"
#include "m2mb_gpio.h"

#include "azx_log.h"
#include "azx_utils.h"
#include "azx_tasks.h"

#include "app_cfg.h"

#include "lwm2m.h"
#include "gpio.h"


/* Local defines ================================================================================*/

/*cellular event bits*/
#define EV_NET_BIT         (UINT32)0x1
#define EV_PDP_BIT         (UINT32)0x2


/* Local typedefs ===============================================================================*/
typedef enum {
  INIT=0,
  APPLICATION_EXIT
} APP_STATES;

/* Local statics ================================================================================*/

static INT8 lwm2m_taskID;

/*Handles*/
static M2MB_PDP_HANDLE    gPdpHandle = NULL;
M2MB_OS_EV_HANDLE         net_pdp_evHandle = NULL;

/* Global function prototypes ====================================================================*/

UINT8         gpioInit      ( void );
M2MB_RESULT_E setGpio       ( UINT16 index, INT32 v );
M2MB_RESULT_E restoreGpio   ( void );
M2MB_RESULT_E closeAllGPIO  ( void );


void NetCallback(M2MB_NET_HANDLE h, M2MB_NET_IND_E net_event, UINT16 resp_size, void *resp_struct, void *myUserdata);
void PdpCallback(M2MB_PDP_HANDLE h, M2MB_PDP_IND_E pdp_event, UINT8 cid, void *userdata);


/* Local function prototypes ====================================================================*/


/**
   @brief           callback function for main demo task, executed by main function.

   @return          0 in case of success, -1 in case of failure
*/
static INT32 demoAppTask ( INT32 type, INT32 param1, INT32 param2);
static INT32 lwm2m_taskCB( INT32 event, INT32 i_uri, INT32 param2);


/* Static functions =============================================================================*/
static INT32 lwm2m_taskCB( INT32 event, INT32 i_uri, INT32 param2)
{
  (void) param2;
  AZX_LOG_DEBUG("LWM2M Task CB: <%d>\r\n", event);

  MEM_W data_buffer[256] = {0};
  INT32 data_int = 0;

  M2MB_LWM2M_OBJ_URI_T *pUri = (M2MB_LWM2M_OBJ_URI_T *) i_uri;
  M2MB_LWM2M_OBJ_URI_T uri = *pUri;

  UINT32           curEvBits;

  M2MB_RESULT_E    retVal;
  M2MB_OS_RESULT_E osRes;

  m2mb_os_free(pUri);

  switch(event)
  {
  case EV_MON_URC_RECEIVED:
  {
    AZX_LOG_INFO("Asking a read operation for {%u/%u/%u/%u (%u)}\r\n",
      uri.obj, uri.objInst, uri.resource, uri.resourceInst, uri.uriLen );


    memset(data_buffer, 0, sizeof(data_buffer));

    M2MB_LWM2M_HANDLE lwm2mHandle = getLwM2mHandle();
    retVal = m2mb_lwm2m_read( lwm2mHandle, &(uri), data_buffer, sizeof(data_buffer));
    if ( retVal == M2MB_RESULT_SUCCESS )
    {
      AZX_LOG_TRACE( "LWM2M read request succeeded\r\n" );
    }
    else
    {
      AZX_LOG_ERROR("Read request failed\r\n");
      return retVal;
    }

    /*wait for event*/
    AZX_LOG_DEBUG("Waiting LWM2M read complete (10 seconds)...\r\n");

    M2MB_OS_EV_HANDLE eventsHandleLwm2m = getEventHandle();
    osRes = m2mb_os_ev_get(eventsHandleLwm2m, EV_LWM2M_READ_RES_BIT, M2MB_OS_EV_GET_ANY_AND_CLEAR, &curEvBits, M2MB_OS_MS2TICKS(10000));
    if(osRes != M2MB_OS_SUCCESS)
    {
      AZX_LOG_ERROR("LWM2M read timeout!\r\n");
      return M2MB_RESULT_FAIL;
    }

    AZX_LOG_TRACE("uri.obj = %d\r\n", uri.obj);
    
	if (uri.obj == M2MB_LWM2M_LIGHT_CONTROL_OBJ_ID)
    {
      switch(uri.resource)
      {
        case LIGHT_CONTROL_OBJ_ON_OFF_ID:
          AZX_LOG_TRACE("LIGHT_CONTROL_OBJ_ON_OFF_ID");

		  /*Cast the data buffer as an integers array and take the first element*/
          data_int = !!((INT32) ( (INT32 *)data_buffer )[0]);

          AZX_LOG_DEBUG("data_int <%d> \r\n", data_int);
          AZX_LOG_DEBUG("\r\nBoolean data in {%u/%u/%u/%u} resource was updated to new value: %s\r\n",
          uri.obj, uri.objInst, uri.resource, uri.resourceInst,
            (data_int > 0)? "true":"false");

          if (setGpio(uri.objInst, data_int) != M2MB_RESULT_SUCCESS)
          {
            AZX_LOG_ERROR("Error setting GPIOs \r\n");
          }
          break;
        case LIGHT_CONTROL_OBJ_DIMMER_ID:
          AZX_LOG_INFO("LIGHT_CONTROL_OBJ_DIMMER_ID");
          break;

        case LIGHT_CONTROL_OBJ_TIME_ID:
          AZX_LOG_INFO("LIGHT_CONTROL_OBJ_TIME_ID");

          break;
        case LIGHT_CONTROL_OBJ_CUMULATIVE_ACTIVE_POWER_ID:
          AZX_LOG_INFO("LIGHT_CONTROL_OBJ_CUMULATIVE_ACTIVE_POWER_ID");

          break;
        case LIGHT_CONTROL_OBJ_POWER_FACTOR_ID:
          AZX_LOG_INFO("LIGHT_CONTROL_OBJ_POWER_FACTOR_ID");

          break;
        case LIGHT_CONTROL_OBJ_COLOR_ID:
          AZX_LOG_INFO("LIGHT_CONTROL_OBJ_COLOR_ID");

          break;
        case LIGHT_CONTROL_OBJ_SENSOR_UNITSR_ID:
          AZX_LOG_INFO("LIGHT_CONTROL_OBJ_SENSOR_UNITSR_ID");

          break;
        case LIGHT_CONTROL_OBJ_APPLICATION_TYPE_ID:
          AZX_LOG_INFO("LIGHT_CONTROL_OBJ_APPLICATION_TYPE_ID");
          break;

        default:
          AZX_LOG_WARN("\r\nUnexpected resource URI {%u/%u/%u/%u (%u)}!\r\n",
          uri.obj, uri.objInst, uri.resource, uri.resourceInst, uri.uriLen );
          break;
      } /* switch(uri.resource) */
    }
    else
    {
      AZX_LOG_WARN("Unexpected object ID %u\r\n", uri.obj);
    }

  }
    break;
  default:
    AZX_LOG_WARN("Unmanaged task event %d\r\n", event);
    break;
  } /* switch(event) */

  return 1;
}


static INT32 demoAppTask(INT32 type, INT32 param1, INT32 param2)
{
  (void)type;
  (void)param1;
  (void)param2;

  M2MB_RESULT_E retVal = M2MB_RESULT_SUCCESS;

  M2MB_OS_RESULT_E        osRes;
  M2MB_OS_EV_ATTR_HANDLE  evAttrHandle = NULL;
  UINT32                  curEvBits;

  M2MB_NET_HANDLE netHandle;
  INT32 ret;

  int task_status = type;

  void *myUserdata = NULL;

  LWM2M_OBJ_USERDATA_T *lwm2mUserData = NULL;

  do
  {
    AZX_LOG_TRACE("Initializing resources...\r\n");

    /* Init events handler */
    osRes  = m2mb_os_ev_setAttrItem( &evAttrHandle,
    CMDS_ARGS(M2MB_OS_EV_SEL_CMD_CREATE_ATTR, NULL,
    M2MB_OS_EV_SEL_CMD_NAME, "net_pdp_ev"));
    if ( osRes != M2MB_OS_SUCCESS )
    {
      AZX_LOG_CRITICAL("Error setting attribute event!\r\n");
      task_status = APPLICATION_EXIT;
      break;
    }

    osRes = m2mb_os_ev_init( &net_pdp_evHandle, &evAttrHandle );
    if ( osRes != M2MB_OS_SUCCESS )
    {
      m2mb_os_ev_setAttrItem( &evAttrHandle, M2MB_OS_EV_SEL_CMD_DEL_ATTR, NULL );
      AZX_LOG_CRITICAL("m2mb_os_ev_init failed!\r\n");
      task_status = APPLICATION_EXIT;
      break;
    }
    else
    {
      AZX_LOG_TRACE("m2mb_os_ev_init success\r\n");
    }
    /* ------------------- */

    /* check network registration and configure PDP context */
    retVal = m2mb_net_init(&netHandle, NetCallback, myUserdata);
    if ( retVal == M2MB_RESULT_SUCCESS )
    {
      AZX_LOG_TRACE( "m2mb_net_init returned M2MB_RESULT_SUCCESS\r\n");
    }
    else
    {
      AZX_LOG_ERROR( "m2mb_net_init did not return M2MB_RESULT_SUCCESS\r\n" );
      task_status = APPLICATION_EXIT;
      break;
    }

    AZX_LOG_INFO("Waiting for registration...\r\n");

    //gets information about regitration status
    retVal = m2mb_net_get_reg_status_info(netHandle);
    if ( retVal != M2MB_RESULT_SUCCESS )
    {
      AZX_LOG_ERROR( "m2mb_net_get_reg_status_info did not return M2MB_RESULT_SUCCESS\r\n" );
      task_status = APPLICATION_EXIT;
      break;
    }

    /*Wait for network registration event to occur (released in NetCallback function) */
    m2mb_os_ev_get(net_pdp_evHandle, EV_NET_BIT, M2MB_OS_EV_GET_ANY_AND_CLEAR, &curEvBits, M2MB_OS_WAIT_FOREVER);

    azx_sleep_ms(8000);

    /*GPIO handles initialization*/
    AZX_LOG_INFO( "Initialize GPIO\r\n");
    ret = gpioInit();
    if (ret != 0)
    {
      AZX_LOG_ERROR( "Initialization GPIO error\r\n");
      task_status = APPLICATION_EXIT;
      break;
    }

    lwm2mUserData = (LWM2M_OBJ_USERDATA_T *)m2mb_os_malloc(sizeof(LWM2M_OBJ_USERDATA_T));
    if(!lwm2mUserData)
    {
      AZX_LOG_ERROR( "Cannot allocate memory\r\n");
      task_status = APPLICATION_EXIT;
      break;
    }
    lwm2mUserData->objID = M2MB_LWM2M_LIGHT_CONTROL_OBJ_ID;


    INT16 instances[] = {0, 1, 2};
    LWM2M_OBJ_REG_T obj = {M2MB_LWM2M_LIGHT_CONTROL_OBJ_ID, 3, instances };

    /* Check XML file*/
    AZX_LOG_INFO("Looking for <%s> file..\r\n", OBJECT_XML_NAME);
    if( 0 != check_xml_file(OBJECT_XML_NAME) )
    {
      AZX_LOG_CRITICAL("%s file is not present in XML folder!\r\n", OBJECT_XML_NAME);
      task_status = APPLICATION_EXIT;
      break;
    }

    /* Initialize LWM2M */
    AZX_LOG_INFO( "Initialize LWM2M\r\n");
    ret = oneedge_init(&obj, 1, lwm2mUserData);
    if (ret != 0)
    {
      AZX_LOG_ERROR( "Initialization LWM2M error\r\n");
      task_status = APPLICATION_EXIT;
      break;
    }


    //monitoring con gli handle restituiti dali getter
    lwm2m_taskID = azx_tasks_createTask((char*) "LWM2M_TASK", AZX_TASKS_STACK_XL,
    1, AZX_TASKS_MBOX_M, lwm2m_taskCB);
    lwm2m_ReadTaskRegister(lwm2m_taskID);


    /*Register a monitor on a resource by creating a URI object and passing it to m2mb_lwm2m_mon*/

    M2MB_LWM2M_MON_REQ_T mon;
    mon.mode   = M2MB_LWM2M_MON_MODE_SET_CMD;
    mon.action = M2MB_LWM2M_MON_ENABLE;

    M2MB_LWM2M_OBJ_URI_T uri_mon = {
      M2MB_LWM2M_URI_3_FIELDS, 			// UINT8  uriLen;
      M2MB_LWM2M_LIGHT_CONTROL_OBJ_ID, 	// UINT16 obj;
      0, 								// UINT16 objInst;
      0, 								// UINT16 resource;
      0									// UINT16 resourceInst;
    };

    AZX_LOG_INFO( "Register monitor resources\r\n");
    registerMonitorResource(uri_mon, mon);


    /* Restore GPIOs value */
    AZX_LOG_INFO( "Restore previous GPIO values\r\n");
    if (restoreGpio() != M2MB_RESULT_SUCCESS)
    {
      AZX_LOG_ERROR( "Restoring GPIO values error\r\n");
      task_status = APPLICATION_EXIT;
      break;
    }

    AZX_LOG_INFO("\r\nWaiting for events from portal. Write on monitored resource or call an exec\r\n\r\n");

  }
  while (0);

  if (task_status == APPLICATION_EXIT)
  {
    if(gPdpHandle)
    {
      m2mb_pdp_deinit(gPdpHandle);
    }

    if(netHandle)
    {
      m2mb_net_deinit(netHandle);
    }

    if(net_pdp_evHandle)
    {
      m2mb_os_ev_deinit(net_pdp_evHandle);
    }

    if (lwm2mUserData)
    {
      m2mb_os_free(lwm2mUserData);
    }

    if (closeAllGPIO() != M2MB_RESULT_SUCCESS)
    {
      AZX_LOG_ERROR("Not all GPIO are closed!.\r\n");
    }  


    AZX_LOG_DEBUG("Application complete.\r\n");
  }

  return 0;

}

/* Global functions =============================================================================*/

M2MB_RESULT_E closeAllGPIO	( void )
{
  LIGHT_CONTROL_OBJ_INSTANCE_S *lightObjs;
  if (getLightControlObjTable(&lightObjs) < 0)
  {
    AZX_LOG_ERROR("Cannot get the objs table!\r\n");
  }

  int error = 0;

  for (; lightObjs->index != -1; lightObjs++)
  {
    if (lightObjs->fd != -1)
    {
      if(close_gpio(&lightObjs->fd) != 0)
      {
        AZX_LOG_WARN("Closing GPIO <%d> error!\r\n", lightObjs->fd);
        error ++;
      }
    }
  }

  if (error != 0)
  {
    return M2MB_RESULT_FAIL;
  }

  return M2MB_RESULT_SUCCESS;
}


M2MB_RESULT_E restoreGpio	( void )
{
  MEM_W data_buffer[256] = {0};
  M2MB_RESULT_E retVal;

  M2MB_LWM2M_OBJ_URI_T uri;
  M2MB_OS_RESULT_E    osRes;

  UINT32 curEvBits;
  INT32 data_int = 0;


  /* Fill resource URI with required parameters */
  uri.obj = M2MB_LWM2M_LIGHT_CONTROL_OBJ_ID;
  uri.resourceInst = 0;
  uri.uriLen = M2MB_LWM2M_URI_3_FIELDS;
  uri.resource = LIGHT_CONTROL_OBJ_ON_OFF_ID;

  LIGHT_CONTROL_OBJ_INSTANCE_S *lightObjs;
  if (getLightControlObjTable(&lightObjs) < 0)
  {
    AZX_LOG_ERROR("Cannot get the objs table!\r\n");
  }

  M2MB_LWM2M_HANDLE lwm2mHandle = getLwM2mHandle();
  M2MB_OS_EV_HANDLE eventsHandleLwm2m = getEventHandle();

  for (; lightObjs->index != -1; lightObjs++)
  {

    uri.objInst = lightObjs->index;
    memset(data_buffer, 0, sizeof(data_buffer));

    retVal = m2mb_lwm2m_read( lwm2mHandle, &(uri), data_buffer, sizeof(data_buffer));
    if ( retVal == M2MB_RESULT_SUCCESS )
    {
      AZX_LOG_TRACE( "LWM2M read request succeeded\r\n" );
    }
    else
    {
      AZX_LOG_ERROR("Read request failed\r\n");
      return retVal;
    }

    /*wait for event*/
    AZX_LOG_DEBUG("Waiting LWM2M ON_OFF read complete (10 seconds)...\r\n");


    osRes = m2mb_os_ev_get(eventsHandleLwm2m, EV_LWM2M_READ_RES_BIT, M2MB_OS_EV_GET_ANY_AND_CLEAR, &curEvBits,     M2MB_OS_MS2TICKS(10000));
    if(osRes != M2MB_OS_SUCCESS)
    {
      AZX_LOG_ERROR("LWM2M read timeout!\r\n");
      return M2MB_RESULT_FAIL;
    }
    AZX_LOG_TRACE("uri.obj = %d\r\n", uri.obj);


    /*Cast the data buffer as an integers array and take the first element*/
    data_int = !!((INT32) ( (INT32 *)data_buffer )[0]);

    AZX_LOG_TRACE("data_int <%d> \r\n", data_int);

    AZX_LOG_TRACE("\r\nrestoreGPIO - Boolean data in {%u/%u/%u/%u} resource was updated to new value: %s\r\n",
    uri.obj, uri.objInst, uri.resource, uri.resourceInst,
    (data_int > 0)? "true":"false");

    if(setGpio(uri.objInst, data_int) != M2MB_RESULT_SUCCESS )
    {
      AZX_LOG_ERROR("Error setting GPIOs \r\n");
    }
  }
  
  return M2MB_RESULT_SUCCESS;
}

M2MB_RESULT_E setGpio(UINT16 index, INT32 v)
{
  LIGHT_CONTROL_OBJ_INSTANCE_S *instance;

  if (getLightControlObjTable(&instance) < 0)
  {
    AZX_LOG_ERROR("Cannot get the objs table!\r\n");
  }

  AZX_LOG_TRACE("index = %d, value = %d\r\n", index, v);
  AZX_LOG_TRACE("instance.index = %d, instance->fd = %d, pin = %d\r\n",
  instance[index].index, instance[index].fd, instance[index].pin);

  //Write value on the GPIO
  write_gpio(instance[index].fd,  (M2MB_GPIO_VALUE_E) v);

  return M2MB_RESULT_SUCCESS;
}

UINT8 gpioInit(void)
{

  LIGHT_CONTROL_OBJ_INSTANCE_S *lightObjs;    


#if 1	/* only for debug */
  if (getLightControlObjTable(&lightObjs) < 0)
  {
    AZX_LOG_ERROR("Cannot get the objs table!\r\n");
  }
  for (; lightObjs->index != -1; lightObjs++)
  {
    INT32 pin = getGpioPinByIndex(lightObjs->index);
    AZX_LOG_TRACE("pin <%d>!\r\n", pin);
  }
#endif


  if (getLightControlObjTable(&lightObjs) < 0)
  {
    AZX_LOG_ERROR("Cannot get the objs table!\r\n");
  }
  for (; lightObjs->index != -1; lightObjs++)
  {
    INT32 fd;
    INT32 ret;

    ret = open_LED(lightObjs->index);
    if (ret)
    {
      AZX_LOG_CRITICAL("Cannot set GPIO descriptors <%d>!\r\n", lightObjs->pin);
      return -1;
    }

    fd = getGpioDescriptor(lightObjs->index);
    if (fd == -1)
    {
      AZX_LOG_ERROR("GPIO pin is not valid!\r\n");
      return -1;
    }

    lightObjs->fd = fd;

    AZX_LOG_TRACE("GPIO %d opened: <%d>!\r\n", lightObjs->pin, fd);
  }

  AZX_LOG_INFO("GPIOs initialized!\r\n");

  return 0;
}

void PdpCallback(M2MB_PDP_HANDLE h, M2MB_PDP_IND_E pdp_event, UINT8 cid, void *userdata)
{
  (void)userdata;
  struct M2MB_SOCKET_BSD_SOCKADDR_IN CBtmpAddress;
  CHAR CBtmpIPaddr[32];

  switch (pdp_event)
  {
  case M2MB_PDP_UP:

    AZX_LOG_DEBUG ("Context activated!\r\n");
    m2mb_pdp_get_my_ip(h, cid, M2MB_PDP_IPV4, &CBtmpAddress.sin_addr.s_addr);
    m2mb_socket_bsd_inet_ntop( M2MB_SOCKET_BSD_AF_INET, &CBtmpAddress.sin_addr.s_addr, ( CHAR * )&( CBtmpIPaddr ), sizeof( CBtmpIPaddr ) );
    AZX_LOG_INFO( "IP address: %s\r\n", CBtmpIPaddr);
    m2mb_os_ev_set(net_pdp_evHandle, EV_PDP_BIT, M2MB_OS_EV_SET);

    break;

  case M2MB_PDP_DOWN:

    AZX_LOG_INFO ("Context deactivated!\r\n");
    m2mb_os_ev_set(net_pdp_evHandle, EV_PDP_BIT, M2MB_OS_EV_CLEAR);

    AZX_LOG_INFO ("Context deactivated!\r\n");
    break;

  default:
    AZX_LOG_DEBUG("unexpected pdp_event: %d\r\n", pdp_event);
    break;

  }
}

void NetCallback(M2MB_NET_HANDLE h, M2MB_NET_IND_E net_event, UINT16 resp_size, void *resp_struct, void *myUserdata)
{
  (void)resp_size;
  (void)myUserdata;

  M2MB_NET_REG_STATUS_T *stat_info;

  switch (net_event)
  {
  case M2MB_NET_GET_REG_STATUS_INFO_RESP:
    stat_info = (M2MB_NET_REG_STATUS_T*)resp_struct;

    if  (stat_info->stat == 1 || stat_info->stat == 5)
    {
      AZX_LOG_INFO("Module is registered to cell 0x%X!\r\n", stat_info->cellID);
      //Release the task blocked on EV_NET_BIT
      m2mb_os_ev_set(net_pdp_evHandle, EV_NET_BIT, M2MB_OS_EV_SET);
    }
    else
    {
      //gets information about regitration status
      m2mb_net_get_reg_status_info(h); //try again
    }
  break;

  default:

    AZX_LOG_DEBUG("Unexpected net_event: %d\r\n", net_event);
    break;
  }
}
/*-----------------------------------------------------------------------------------------------*/

/***************************************************************************************************
\User Entry Point of Appzone

\param [in] Module Id

\details Main of the appzone user
**************************************************************************************************/
void M2MB_main( int argc, char **argv )
{
  (void)argc;
  (void)argv;

  INT32 taskID;

  azx_tasks_init();

  azx_sleep_ms(5000);


  /* SET output channel */
  AZX_LOG_INIT();
  AZX_LOG_INFO("Starting lwm2m demo. This is v%s built on %s %s.\r\n",
  VERSION, __DATE__, __TIME__);

  AZX_LOG_INFO("On OneEdge portal.\r\n\r\n");

  taskID = azx_tasks_createTask((char*) "LWM2M_TASK",
  AZX_TASKS_STACK_XL, 1,
  AZX_TASKS_MBOX_M, demoAppTask);

  AZX_LOG_DEBUG("Task ID: %d.\r\n", taskID);

  azx_sleep_ms(1000);

  if (taskID > 0)
  {
    azx_tasks_sendMessageToTask( taskID, INIT, 0, 0);
  }
  else
  {
    AZX_LOG_ERROR("Cannot create task!\r\n");
    return;
  }
}

