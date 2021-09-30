/*=========================================================================*/
/*   Copyright (C) Telit Communications S.p.A. Italy All Rights Reserved.  */
/**
 @file
 lwm2m.c

 @brief
 The file contains lwm2m callbacks and utilities

 @details


 @author WhiteBeard
 @author FabioPi

 @date
 2021-01-20
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

#include "azx_tasks.h"

#include "azx_log.h"
#include "azx_utils.h"
#include "app_cfg.h"

#include "gpio.h"
#include "lwm2m.h"

/* Local defines ================================================================================*/
#define GPIO_INDEX_RED     0
#define GPIO_INDEX_YELLOW   1
#define GPIO_INDEX_GREEN  2

/* Local typedefs ===============================================================================*/

/* Local statics ================================================================================*/
static INT32 dataTaskID = -1;
static LWM2M_OBJ_USERDATA_T *lwm2mUserData = NULL;

static BOOLEAN bootstrapped = FALSE;

/* HANDLER */
static M2MB_OS_EV_HANDLE eventsHandleLwm2m  = NULL;
static M2MB_OS_SEM_HANDLE lwm2m_CSSemHandle = NULL;
static M2MB_LWM2M_HANDLE lwm2mHandle     = NULL;

/* ONEEDGE */
static M2MB_LWM2M_OBJ_URI_T _obj_telit_service_uri = { .uriLen = 4,
    .obj = 33211, .objInst = 0, .resource = 0, .resourceInst = 1 };

static M2MB_LWM2M_OBJ_URI_T _obj_tamper_uri = { 3, TAMPERING_OBJ_ID, 0, 1 };

static M2MB_LWM2M_OBJ_URI_T _obj_environment_uri_t = { 3, ENVIRONMENT_OBJ_ID, 0,
    1, 0 };
static M2MB_LWM2M_OBJ_URI_T _obj_environment_uri_p = { 3, ENVIRONMENT_OBJ_ID, 0,
    2, 0 };
static M2MB_LWM2M_OBJ_URI_T _obj_environment_uri_h = { 3, ENVIRONMENT_OBJ_ID, 0,
    3, 0 };
static M2MB_LWM2M_OBJ_URI_T _obj_environment_uri_iaq = { 3, ENVIRONMENT_OBJ_ID,
    0, 4, 0 };

static M2MB_LWM2M_OBJ_URI_T _obj_rotation_uri_w =
    { 3, ROTATION_OBJ_ID, 0, 1, 0 };
static M2MB_LWM2M_OBJ_URI_T _obj_rotation_uri_x =
    { 3, ROTATION_OBJ_ID, 0, 2, 0 };
static M2MB_LWM2M_OBJ_URI_T _obj_rotation_uri_y =
    { 3, ROTATION_OBJ_ID, 0, 3, 0 };
static M2MB_LWM2M_OBJ_URI_T _obj_rotation_uri_z =
    { 3, ROTATION_OBJ_ID, 0, 4, 0 };
static M2MB_LWM2M_OBJ_URI_T _obj_rotation_uri_a =
    { 3, ROTATION_OBJ_ID, 0, 5, 0 };

static LIGHT_CONTROL_OBJ_INSTANCE_S light_control_obj_table[] = {
  LIGHT_CONTROL_OBJ_INFO("Red", "LD1", GPIO_INDEX_RED, -1, RED_LED_GPIO),
  LIGHT_CONTROL_OBJ_INFO("Yellow", "LD2", GPIO_INDEX_YELLOW, -1, YELLOW_LED_GPIO),
  LIGHT_CONTROL_OBJ_INFO("Green", "LD3", GPIO_INDEX_GREEN, -1, GREEN_LED_GPIO),
  { .description = (char*) NULL, .appType = (char*) NULL, -1, -1, -1 }
};

/* Local function prototypes ====================================================================*/
static M2MB_RESULT_E writeLightControlApplicationType(M2MB_LWM2M_EVENT_E event, INT16 index);
static INT32 writeLightControlAppTypeTask(INT32 event, INT32 index,
    INT32 taskId);

static INT32 set_string_resource(M2MB_LWM2M_HANDLE h, M2MB_LWM2M_OBJ_URI_T *pUri, CHAR *data);

/* Static functions =============================================================================*/
static INT32 writeLightControlAppTypeTask(INT32 event, INT32 index,
    INT32 taskId) {
  (void)event;

  AZX_LOG_DEBUG("LWM2M_AT_TASK Task : <%d>\r\n", (INT8 ) taskId);

  UINT32 curEvBits;
  M2MB_LWM2M_OBJ_URI_T resource_uri;

  /* Fill resource URI with required parameters */
  resource_uri.obj = LWM2M_LIGHT_CONTROL_OBJ_ID;
  resource_uri.objInst = index;
  resource_uri.resourceInst = 0;
  resource_uri.uriLen = M2MB_LWM2M_URI_3_FIELDS;
  resource_uri.resource = LIGHT_CONTROL_OBJ_APPLICATION_TYPE_ID;

  AZX_LOG_INFO("Instance <%d> with app type <%s>\r\n", index,
      light_control_obj_table[index].appType);

  M2MB_RESULT_E retVal = m2mb_lwm2m_write(lwm2mHandle, &resource_uri,
      light_control_obj_table[index].appType,
      strlen(light_control_obj_table[index].appType));
  //retVal is just the return value of the API, not the completed operation
  //the completed operation ends when an event is raised into the callback
  if (retVal != M2MB_RESULT_SUCCESS) {
    AZX_LOG_ERROR("m2mb_lwm2m_write returned error %d\r\n", retVal);
    return -1;
  }

  if (M2MB_OS_SUCCESS
      != m2mb_os_ev_get(eventsHandleLwm2m,
          EV_LWM2M_WRITE_RES_BIT, M2MB_OS_EV_GET_ANY_AND_CLEAR, &curEvBits,
          M2MB_OS_MS2TICKS(10000) /*wait 10 seconds for the event to occur*/
      )) {
    AZX_LOG_ERROR("LWM2M write timeout!\r\n");
    return -2;
  }

  /* resource created and updated, proceed with the next m2mb_lwm2m_newinst */
  m2mb_os_ev_set(eventsHandleLwm2m, EV_LWM2M_NEW_RES_BIT,  M2MB_OS_EV_SET);


  AZX_LOG_INFO("LWM2M_AT_TASK Task : <%d> destroying...\r\n", taskId);
  INT32 r = azx_tasks_destroyTask((INT8) taskId);
  AZX_LOG_ERROR("Error code <%d> destroying LWM2M_AT_TASK Task ID <%d>\r\n", r,
      taskId);

  return 1;
}

static M2MB_RESULT_E writeLightControlApplicationType(M2MB_LWM2M_EVENT_E event, INT16 index) {
  INT8 appTypeTaskID;
  appTypeTaskID = azx_tasks_createTask((char*) "LWM2M_AT_TASK",
      AZX_TASKS_STACK_S, 2, AZX_TASKS_MBOX_S,
      writeLightControlAppTypeTask);

  AZX_LOG_INFO("Task App Type ID: %d.\r\n", appTypeTaskID);
  if (appTypeTaskID <= 0) {
    AZX_LOG_ERROR(
        "Cannot create lwm2m application type managing task!\r\n");
    return M2MB_RESULT_FAIL;
  }

  azx_tasks_sendMessageToTask(appTypeTaskID, event, (INT32) index,
      (INT32) appTypeTaskID);

  return M2MB_RESULT_SUCCESS;
}



static INT32 set_string_resource(M2MB_LWM2M_HANDLE h, M2MB_LWM2M_OBJ_URI_T *pUri, CHAR *data)
{
  UINT32                  curEvBits;
  if(pUri->uriLen == M2MB_LWM2M_URI_3_FIELDS)
  {
    AZX_LOG_INFO("\r\nSetting string resource {%u/%u/%u} value to %s on LWM2M client.\r\n",
        pUri->obj, pUri->objInst, pUri->resource,
        data);
  }
  else
  {
    AZX_LOG_INFO("\r\nSetting string resource {%u/%u/%u/%u} value to %s on LWM2M client.\r\n",
        pUri->obj, pUri->objInst, pUri->resource, pUri->resourceInst,
        data);
  }

  M2MB_RESULT_E retVal = m2mb_lwm2m_set( h, pUri, data, strlen(data));

  /* retVal is just the return value of the API, not the completed operation
  the completed operation ends when an event is raised into the callback */
  if ( retVal != M2MB_RESULT_SUCCESS )
  {
    AZX_LOG_ERROR( "m2mb_lwm2m_write returned error %d\r\n",retVal );
    return -1;
  }

  if( M2MB_OS_SUCCESS != m2mb_os_ev_get(
      eventsHandleLwm2m,
      EV_LWM2M_SET_RES_BIT,
      M2MB_OS_EV_GET_ANY_AND_CLEAR,
      &curEvBits,
      M2MB_OS_MS2TICKS(10000) /*wait 10 seconds for the event to occur*/
  )
  )
  {
    AZX_LOG_ERROR("LWM2M set timeout!\r\n");
    return -2;
  }
  else
  {
    return 0;
  }

}
/* Global functions =============================================================================*/
M2MB_RESULT_E read_integer_from_uri(M2MB_LWM2M_HANDLE h, M2MB_LWM2M_OBJ_URI_T *pUri, INT32 *result)
{
  int data[2];
  M2MB_RESULT_E retVal;
  M2MB_OS_RESULT_E osRes;
  UINT32 curEvBits;


  if(NULL == h)
  {
    AZX_LOG_ERROR("oneedge not initialized yet\r\n");
    return M2MB_RESULT_FAIL;
  }

  AZX_LOG_TRACE("m2mb_lwm2m_read for integer from {%d/%d/%d}\r\n",
      pUri->obj, pUri->objInst, pUri->resource );

  retVal = m2mb_lwm2m_read( h, pUri, data, sizeof(data));
  if ( retVal == M2MB_RESULT_SUCCESS )
  {
    AZX_LOG_TRACE( "m2mb_lwm2m_read request succeeded\r\n" );
  }

  /*wait for event*/
  AZX_LOG_TRACE("Waiting LWM2M read complete (10 seconds)...\r\n");
  osRes = m2mb_os_ev_get(eventsHandleLwm2m, EV_LWM2M_READ_RES_BIT, M2MB_OS_EV_GET_ANY_AND_CLEAR, &curEvBits, M2MB_OS_MS2TICKS(10000));
  if(osRes != M2MB_OS_SUCCESS)
  {
    AZX_LOG_ERROR("LWM2M read timeout!\r\n");
    return M2MB_RESULT_FAIL;
  }

  *result = data[0];
  AZX_LOG_TRACE("Received data <%d> from portal\r\n", *result);
  return M2MB_RESULT_SUCCESS;
}

INT32 write_integer_resource(M2MB_LWM2M_HANDLE h, M2MB_LWM2M_OBJ_URI_T *pUri, INT32 value)
{
  UINT32                  curEvBits;
  INT32 _value = value;
  if(pUri->uriLen == M2MB_LWM2M_URI_3_FIELDS)
  {
    AZX_LOG_INFO("\r\nWriting integer resource {%u/%u/%u} value to %d on LWM2M client.\r\n",
        pUri->obj, pUri->objInst, pUri->resource,
        _value);
  }
  else
  {
    AZX_LOG_INFO("\r\nWriting integer resource {%u/%u/%u/%u} value to %d on LWM2M client.\r\n",
        pUri->obj, pUri->objInst, pUri->resource, pUri->resourceInst,
        _value);
  }


  M2MB_RESULT_E retVal = m2mb_lwm2m_write( h, pUri, &_value, sizeof( INT32 ));

  //retVal is just the return value of the API, not the completed operation
  //the completed operation ends when an event is raised into the callback
  if ( retVal != M2MB_RESULT_SUCCESS )
  {
    AZX_LOG_ERROR( "m2mb_lwm2m_write returned error %d\r\n",retVal );
    return -1;
  }

  if( M2MB_OS_SUCCESS != m2mb_os_ev_get(
      eventsHandleLwm2m,
      EV_LWM2M_WRITE_RES_BIT,
      M2MB_OS_EV_GET_ANY_AND_CLEAR,
      &curEvBits,
      M2MB_OS_MS2TICKS(10000) /*wait 10 seconds for the event to occur*/
  )
  )
  {
    AZX_LOG_ERROR("LWM2M write timeout!\r\n");
    return -2;
  }
  else
  {
    return 0;
  }

}


INT32 set_opaque_resource(M2MB_LWM2M_HANDLE h, M2MB_LWM2M_OBJ_URI_T *pUri, UINT8 *data, UINT32 datalen)
{
  UINT32                  curEvBits;
  M2MB_RESULT_E retVal;
  if(pUri->uriLen == M2MB_LWM2M_URI_3_FIELDS)
  {
    AZX_LOG_INFO("\r\nSetting opaque resource {%u/%u/%u} on LWM2M client.\r\n",
        pUri->obj, pUri->objInst, pUri->resource
    );
  }
  else
  {
    AZX_LOG_INFO("\r\nSetting opaque resource {%u/%u/%u/%u} on LWM2M client.\r\n",
        pUri->obj, pUri->objInst, pUri->resource, pUri->resourceInst
    );
  }
  AZX_LOG_INFO("%u bytes will be set\r\n", datalen);

  retVal = m2mb_lwm2m_set( h, pUri, (void *) data, (UINT16) datalen);

  //retVal is just the return value of the API, not the completed operation
  //the completed operation ends when an event is raised into the callback
  if ( retVal != M2MB_RESULT_SUCCESS )
  {
    AZX_LOG_ERROR( "m2mb_lwm2m_set returned error %d\r\n", retVal );
    return -1;
  }

  if( M2MB_OS_SUCCESS != m2mb_os_ev_get(
      eventsHandleLwm2m,
      EV_LWM2M_SET_RES_BIT,
      M2MB_OS_EV_GET_ANY_AND_CLEAR,
      &curEvBits,
      M2MB_OS_MS2TICKS(10000) /*wait 10 seconds for the event to occur*/
  )
  )
  {
    AZX_LOG_ERROR("LWM2M set timeout!\r\n");
    return -2;
  }
  else
  {
    /* wait notify*/
    if (M2MB_OS_SUCCESS != m2mb_os_ev_get( eventsHandleLwm2m,
        EV_LWM2M_NFYACK_BIT, M2MB_OS_EV_GET_ANY_AND_CLEAR, &curEvBits,
        M2MB_OS_MS2TICKS(30000) /*wait 30 seconds for the event to occur*/
    ))
    {
      AZX_LOG_ERROR("lwm2m notify ack timeout!\r\n");
      return -3;
    }
    else
    {
      return 0;
    }
  }

}



void lwm2mIndicationCB(M2MB_LWM2M_HANDLE h, M2MB_LWM2M_EVENT_E event,
    UINT16 resp_size, void *resp_struct, void *userdata) {
  (void) h;
  (void) resp_size;
  (void) userdata;

  /* Client generated events */
  switch (event) {

  case M2MB_LWM2M_ENABLE_RES: {
    M2MB_LWM2M_ENABLE_RES_T *resp = (M2MB_LWM2M_ENABLE_RES_T *) resp_struct;
    if (resp->result == M2MB_LWM2M_RES_SUCCESS) {

      AZX_LOG_TRACE("LWM2M enable result OK\r\n");
      m2mb_os_ev_set(eventsHandleLwm2m, EV_LWM2M_ENABLE_RES_BIT,
          M2MB_OS_EV_SET);
    } else {

      AZX_LOG_WARN("Enable result %d\r\n", resp->result);
    }
    break;
  }

  case M2MB_LWM2M_DISABLE_RES: {
    M2MB_LWM2M_DISABLE_RES_T *resp = (M2MB_LWM2M_DISABLE_RES_T *) resp_struct;
    if (resp->result == M2MB_LWM2M_RES_SUCCESS) {

      AZX_LOG_TRACE("LWM2M disable result OK\r\n");
      m2mb_os_ev_set(eventsHandleLwm2m, EV_LWM2M_DISABLE_RES_BIT,
          M2MB_OS_EV_SET);
    } else {

      AZX_LOG_WARN("Enable result %d\r\n", resp->result);
    }
    break;
  }

  case M2MB_LWM2M_SET_RES: {
    M2MB_LWM2M_SET_RES_T *resp = (M2MB_LWM2M_SET_RES_T *) resp_struct;
    if (resp->result == M2MB_LWM2M_RES_SUCCESS) {
      AZX_LOG_DEBUG("LWM2M set result OK\r\n");
      m2mb_os_ev_set(eventsHandleLwm2m, EV_LWM2M_SET_RES_BIT,
          M2MB_OS_EV_SET);
    } else {
      AZX_LOG_WARN("set result %d\r\n", resp->result);
    }
    break;
  }

  /* event in response to m2mb_lwm2m_read() */
  case M2MB_LWM2M_READ_RES: {
    M2MB_LWM2M_READ_RES_T *resp = (M2MB_LWM2M_READ_RES_T *) resp_struct;
    AZX_LOG_TRACE("M2MB_LWM2M_READ_RES, result: %d\r\n", resp->result);
    AZX_LOG_TRACE("Read type: %d; size: %u\r\n", resp->resType, resp->len);
    if (resp->result == M2MB_LWM2M_RES_SUCCESS) {
      m2mb_os_ev_set(eventsHandleLwm2m, EV_LWM2M_READ_RES_BIT,
          M2MB_OS_EV_SET);
    } else {
      AZX_LOG_ERROR("READ failed with error %d\r\n", resp->result);
    }
    break;
  }

  case M2MB_LWM2M_WRITE_RES: {
    M2MB_LWM2M_WRITE_RES_T *resp = (M2MB_LWM2M_WRITE_RES_T *) resp_struct;
    if (resp->result == M2MB_LWM2M_RES_SUCCESS) {
      AZX_LOG_TRACE("LWM2M write result OK\r\n");
      m2mb_os_ev_set(eventsHandleLwm2m, EV_LWM2M_WRITE_RES_BIT,
          M2MB_OS_EV_SET);
    } else
    {
      AZX_LOG_WARN("Enable write %d\r\n", resp->result);
    }
    break;
  }


  /* event in case a resource changed (if monitoring is enabled) */
  case M2MB_LWM2M_MON_INFO_IND:
  {

    /*
    An URC was received, like
      #LWM2MMON: UPD,"/3311/x/5850/0
    */
    /* event that brings a resource change information */
    M2MB_LWM2M_OBJ_URI_T * p_uri = (M2MB_LWM2M_OBJ_URI_T *)m2mb_os_malloc(sizeof(M2MB_LWM2M_OBJ_URI_T));
    M2MB_LWM2M_MON_INFO_IND_T *pInfo = ( M2MB_LWM2M_MON_INFO_IND_T * ) resp_struct;
    AZX_LOG_TRACE("\r\nM2MB_LWM2M_MON_INFO_IND\r\n");

    if(p_uri && dataTaskID != -1)
    {
      memcpy(p_uri, &(pInfo->uri), sizeof(M2MB_LWM2M_OBJ_URI_T));
      AZX_LOG_INFO( "\r\nResource /%u/%u/%u/%u changed!\r\n",
          p_uri->obj, p_uri->objInst,
          p_uri->resource, p_uri->resourceInst);
      p_uri->uriLen = M2MB_LWM2M_URI_3_FIELDS;

      azx_tasks_sendMessageToTask( dataTaskID, EV_MON_URC_RECEIVED, (INT32)p_uri, 0);
    }
    else
    {
      AZX_LOG_ERROR("Cannot allocate uri struct\r\n");
    }

    break;
  }

  case M2MB_LWM2M_NEW_INST_RES:
  {
    // event in response to the m2mb_lwm2m_newinst
    M2MB_LWM2M_NEW_INST_RES_T *resp =
        (M2MB_LWM2M_NEW_INST_RES_T *) resp_struct;

    LWM2M_OBJ_USERDATA_T *data = (LWM2M_OBJ_USERDATA_T*) userdata;

    switch( resp->result )
    {
    case M2MB_LWM2M_RES_SUCCESS:

      AZX_LOG_TRACE("New Instance created successfully\r\n");

      if (userdata)
      {
        INT32 objId = data->objID;
        INT16 *index =  (INT16*) data->data;
        AZX_LOG_TRACE( "objId: %d, index: %d \r\n", objId, *index);

        switch (objId)
        {
        case LWM2M_LIGHT_CONTROL_OBJ_ID:

          azx_sleep_ms(4000);

          // start task to write the resource
          if (writeLightControlApplicationType(
              M2MB_LWM2M_NEW_INST_RES, *index) != M2MB_RESULT_SUCCESS)
          {
            AZX_LOG_ERROR("Error writing application Type resource\r\n");
          }

          /* free the resource allocated in to oneedge_init */
          m2mb_os_free((INT16*) data->data);

          break;

        default:
          AZX_LOG_INFO("unexpected object instance. <%d>\r\n", objId);
          /* resource created and updated, proceed with the next m2mb_lwm2m_newinst */
          break;
        }
      }
      else /*null userdata*/
      {
        m2mb_os_ev_set(eventsHandleLwm2m, EV_LWM2M_NEW_RES_BIT, M2MB_OS_EV_SET);
      }
      break;

    case M2MB_LWM2M_RES_FAIL_NOT_ALLOWED:
      AZX_LOG_WARN( "New instance creation not allowed (already present?)\r\n" );

      m2mb_os_ev_set(eventsHandleLwm2m, EV_LWM2M_NEW_RES_BIT,  M2MB_OS_EV_SET);
      break;

    default:
      AZX_LOG_DEBUG( "Creating object instance specified: generic result %d\r\n", resp->result );
      break;
    }

    break;
  }

  case M2MB_LWM2M_SRV_INFO_IND: {

    AZX_LOG_TRACE("M2MB_LWM2M_SRV_INFO_IND\r\n");
    M2MB_LWM2M_SRV_INFO_IND_T *resp =
        (M2MB_LWM2M_SRV_INFO_IND_T *) resp_struct;

    switch (resp->info) {
    case M2MB_LWM2M_CL_STATE_DISABLED:
      AZX_LOG_DEBUG(
          "resp->info == M2MB_LWM2M_CL_STATE_DISABLED\r\n");
      m2mb_os_ev_set(eventsHandleLwm2m, EV_LWM2M_SRV_DISABLED_BIT,
          M2MB_OS_EV_SET);
      break;
    case M2MB_LWM2M_CL_STATE_BOOTSTRAPPING:
      AZX_LOG_DEBUG(
          "resp->info == M2MB_LWM2M_CL_STATE_BOOTSTRAPPING\r\n");
      break;
    case M2MB_LWM2M_CL_STATE_BOOTSTRAPPED:
      AZX_LOG_DEBUG("resp->info == M2MB_LWM2M_CL_STATE_BOOTSTRAPPED\r\n");
      break;
    case M2MB_LWM2M_CL_STATE_REGISTERING:
      AZX_LOG_DEBUG("resp->info == M2MB_LWM2M_CL_STATE_REGISTERING\r\n");
      break;
    case M2MB_LWM2M_CL_STATE_REGISTERED:
      AZX_LOG_DEBUG("resp->info == M2MB_LWM2M_CL_STATE_REGISTERED\r\n");
      m2mb_os_ev_set(eventsHandleLwm2m, EV_LWM2M_SRV_REG_BIT,
          M2MB_OS_EV_SET);
      break;
    case M2MB_LWM2M_CL_STATE_DEREGISTERING:
      AZX_LOG_DEBUG(
          "resp->info == M2MB_LWM2M_CL_STATE_DEREGISTERING\r\n");
      break;
    case M2MB_LWM2M_CL_STATE_SUSPENDED:
      AZX_LOG_DEBUG("resp->info == M2MB_LWM2M_CL_STATE_SUSPENDED\r\n");
      break;
    default:
      AZX_LOG_WARN("resp->info: unexpected value!! %d\r\n", resp->info);
      break;
    }
    break;
  }

  case M2MB_LWM2M_NFYACK_URI_RES:
  { // event in response to the m2mb_lwm2m_nfy_ack_uri
    M2MB_LWM2M_NFYACK_URI_RES_T *resp = ( M2MB_LWM2M_NFYACK_URI_RES_T * )resp_struct;
    if( ( resp ) && ( resp->result == M2MB_LWM2M_RES_SUCCESS ) )
    {
      // code
      AZX_LOG_TRACE( "m2mb_lwm2m_nfy_ack_uri response OK\r\n" );
      m2mb_os_ev_set(eventsHandleLwm2m, EV_LWM2M_NFYADD_RES_BIT, M2MB_OS_EV_SET);
    }
    break;
  }

  case M2MB_LWM2M_NFYACK_STATUS_RES:
  { // event in response to the m2mb_lwm2m_nfy_ack_status
    M2MB_LWM2M_NFYACK_STATUS_RES_T *resp = ( M2MB_LWM2M_NFYACK_STATUS_RES_T * )resp_struct;

    if( ( resp ) && ( resp->result == M2MB_LWM2M_RES_SUCCESS ) )
    {
      AZX_LOG_TRACE( "Current status for Notify Ack reporting is %s\r\n\r\n", ( ( resp->enabled == TRUE ) ? ( "ENABLED" ) : ( "DISABLED" ) ) );
      m2mb_os_ev_set(eventsHandleLwm2m, EV_LWM2M_NFYSTAT_RES_BIT, M2MB_OS_EV_SET);
    }
    break;
  }
  case M2MB_LWM2M_NFYACK_LIST_RES:
  {
    // event in response to the m2mb_lwm2m_nfy_ack_list
    M2MB_LWM2M_NFYACK_LIST_RES_T *resp = ( M2MB_LWM2M_NFYACK_LIST_RES_T * )resp_struct;

    if( ( resp ) && ( resp->result == M2MB_LWM2M_RES_SUCCESS ) )
    {
      // code
      AZX_LOG_TRACE( "m2mb_lwm2m_nfy_ack_list %hu elements read\r\n", resp->listElementsNumber );
      m2mb_os_ev_set(eventsHandleLwm2m, EV_LWM2M_NFYACK_LIST_RES_BIT, M2MB_OS_EV_SET);
    }
    break;
  }
  case M2MB_LWM2M_NFYACK_INFO_IND:
  { // event that brings a Notify Ack information
    M2MB_LWM2M_NFYACK_INFO_IND_T *pInfo = ( M2MB_LWM2M_NFYACK_INFO_IND_T * )resp_struct;
    
    // code; i.e.:
    // uriLen value is the number of valid fields of the URI, starting from the 1st and going towards
    // the last one in the printf below
    if( ( pInfo ) /*&& ( pInfo->uri.uriLen == 3 )*/ )
    {
      AZX_LOG_DEBUG( "#LWM2MNFYACK: agentId %hu, SSID %hu, uri \"/%hu/%hu/%hu\", %s\r\n",
          pInfo->agent,
          pInfo->shServerId,
          pInfo->uri.obj,
          pInfo->uri.objInst,
          pInfo->uri.resource,
          ( pInfo->nfyState == M2MB_LWM2M_NFY_STATE_ACK_RECEIVED ) ? ( "ACK RECEIVED" ) : ( "ACK NOT RECEIVED" ) );

      if(pInfo->nfyState == M2MB_LWM2M_NFY_STATE_ACK_RECEIVED )
      {
        m2mb_os_ev_set(eventsHandleLwm2m, EV_LWM2M_NFYACK_BIT, M2MB_OS_EV_SET);
      }
    }

    break;
  }


  case M2MB_LWM2M_SESSION_INFO_IND:
  {
    AZX_LOG_TRACE("M2MB_LWM2M_SESSION_INFO_IND\r\n");
    M2MB_LWM2M_INFO_IND_T *resp = (M2MB_LWM2M_INFO_IND_T *) resp_struct;

    if (resp->info == M2MB_LWM2M_INFO_RESOURCE_EXECUTE)
    {
      M2MB_LWM2M_OBJ_URI_T *currData = (M2MB_LWM2M_OBJ_URI_T *) resp->data;
      AZX_LOG_DEBUG("Info Exec Ind: %d/%d/%d\r\n", currData->obj,  currData->objInst, currData->resource);
      /*   example
       if( currData->obj == 3303 && currData->resource == 5605)
       {
       AZX_LOG_DEBUG( "Reset Min/Max Value of temperature\r\n");
       }
       else
       {
       AZX_LOG_DEBUG("executed another resource");
       }*/
    }

    break;
  }

  /* event in response to m2mb_lwm2m_mon() */
  case M2MB_LWM2M_MON_RES:
  {
    AZX_LOG_TRACE( "M2MB_LWM2M_MON_RES\r\n" );

    M2MB_LWM2M_MON_RES_T *resp = ( M2MB_LWM2M_MON_RES_T * )resp_struct;
    AZX_LOG_TRACE( "Resource change monitor setting result: %d\r\n", resp->result );

    if( resp->result == M2MB_LWM2M_RES_SUCCESS )
    {
      AZX_LOG_DEBUG("Monitoring enabled.\r\n\r\n");
      m2mb_os_ev_set(eventsHandleLwm2m, EV_LWM2M_MON_RES_BIT, M2MB_OS_EV_SET);
    }
    else
    {
      AZX_LOG_DEBUG("Monitoring result error.\r\n\r\n");
    }

    break;
  }

  /* event in response to m2mb_lwm2m_get_stat() */
  case M2MB_LWM2M_GET_STAT_RES:
  {
    AZX_LOG_DEBUG( "M2MB_LWM2M_GET_STAT_RES\r\n" );
    break;
  }

  default:
    AZX_LOG_DEBUG("LWM2M EVENT %d\r\n", event);
    break;
  }
}

/*-----------------------------------------------------------------------------------------------*/
void setAsBootstrapped(void)
{
  bootstrapped = TRUE;
}

/*-----------------------------------------------------------------------------------------------*/
void setAsNotBootstrapped(void)
{
  bootstrapped = FALSE;
}

/*-----------------------------------------------------------------------------------------------*/
uint8_t oneedge_init(LWM2M_OBJ_REG_T *objs, INT16 obj_num, void *userData) {
  M2MB_RESULT_E retVal;
  M2MB_LWM2M_ENABLE_REQ_T pars;
  INT32 service_enable = 1;

  M2MB_OS_RESULT_E osRes;
  M2MB_OS_EV_ATTR_HANDLE evAttrHandle;
  UINT32 curEvBits;

  lwm2mUserData = (LWM2M_OBJ_USERDATA_T *) userData;


  int i;
  M2MB_OS_SEM_ATTR_HANDLE semAttrHandle;

  if (NULL == lwm2m_CSSemHandle) {
    m2mb_os_sem_setAttrItem(&semAttrHandle,
        CMDS_ARGS(M2MB_OS_SEM_SEL_CMD_CREATE_ATTR, NULL,
            M2MB_OS_SEM_SEL_CMD_COUNT, 1 /*CS*/,
            M2MB_OS_SEM_SEL_CMD_TYPE, M2MB_OS_SEM_GEN));
    m2mb_os_sem_init(&lwm2m_CSSemHandle, &semAttrHandle);
  }

  AZX_LOG_TRACE("Asked to register %d object instances.\r\n", obj_num);

  /* Init events handler */
  osRes = m2mb_os_ev_setAttrItem(&evAttrHandle,
      CMDS_ARGS(M2MB_OS_EV_SEL_CMD_CREATE_ATTR, NULL,
          M2MB_OS_EV_SEL_CMD_NAME, "lwm2m_ev"));
  osRes = m2mb_os_ev_init(&eventsHandleLwm2m, &evAttrHandle);

  if (osRes != M2MB_OS_SUCCESS) 
  {
    m2mb_os_ev_setAttrItem(&evAttrHandle, M2MB_OS_EV_SEL_CMD_DEL_ATTR,
        NULL);
    AZX_LOG_CRITICAL("m2mb_os_ev_init failed!\r\n");
    return 1;
  } else {
    AZX_LOG_TRACE("m2mb_os_ev_init success\r\n");
  }

  //get the handle of the lwm2m client on lwm2mHandle
  retVal = m2mb_lwm2m_init(&lwm2mHandle, lwm2mIndicationCB,  (void *) lwm2mUserData);

  if (retVal != M2MB_RESULT_SUCCESS) {
    AZX_LOG_ERROR("m2mb_lwm2m_init returned error %d\r\n", retVal);
    m2mb_os_ev_deinit(eventsHandleLwm2m);
    return 1;
  }


  retVal = m2mb_lwm2m_agent_config(lwm2mHandle, 0 /*Telit Agent ID*/);
  if (retVal != M2MB_RESULT_SUCCESS)
  {
    AZX_LOG_ERROR("m2mb_lwm2m_agent_config request succeeded");
    m2mb_os_ev_deinit(eventsHandleLwm2m);
    m2mb_os_sem_deinit(lwm2m_CSSemHandle);
    m2mb_lwm2m_deinit(lwm2mHandle);
    return 1;
  }

  if(!bootstrapped)
  {
    retVal = m2mb_lwm2m_write(lwm2mHandle, &_obj_telit_service_uri,
        &service_enable, sizeof(INT32));

    if (retVal != M2MB_RESULT_SUCCESS)
    {
      AZX_LOG_ERROR("m2mb_lwm2m_write returned error %d\r\n", retVal);
      m2mb_os_ev_deinit(eventsHandleLwm2m);
      m2mb_os_sem_deinit(lwm2m_CSSemHandle);

      m2mb_lwm2m_deinit(lwm2mHandle);
      return 1;
    }
  }


  m2mb_os_ev_set(eventsHandleLwm2m, EV_LWM2M_SET_RES_BIT, M2MB_OS_EV_CLEAR);

  //AT#LWM2MENA=1
  memset(&pars, 0, sizeof(M2MB_LWM2M_ENABLE_REQ_T));

  pars.apnclass = CTX_ID; /*CID*/
  pars.guardRequestEventSecs = 5;
  pars.guardReleaseEventSecs = 5;
  pars.commandType = M2MB_LWM2MENA_CMD_TYPE_SET;
  pars.mode = M2MB_LWM2M_MODE_NO_ACK;

  retVal = m2mb_lwm2m_enable(lwm2mHandle, &pars);
  if (retVal != M2MB_RESULT_SUCCESS)
  {
    AZX_LOG_ERROR("m2mb_lwm2m_enable returned error %d\r\n", retVal);
    m2mb_os_ev_deinit(eventsHandleLwm2m);
    m2mb_os_sem_deinit(lwm2m_CSSemHandle);
    m2mb_lwm2m_deinit(lwm2mHandle);
    return 1;
  }

  if (M2MB_OS_SUCCESS
      != m2mb_os_ev_get(eventsHandleLwm2m,
          EV_LWM2M_ENABLE_RES_BIT, M2MB_OS_EV_GET_ANY_AND_CLEAR, &curEvBits,
          M2MB_OS_MS2TICKS(10000) /*wait 10 seconds for the event to occur*/
      ))
  {
    AZX_LOG_ERROR("m2mb_lwm2m_enable timeout!\r\n");
    m2mb_os_ev_deinit(eventsHandleLwm2m);
    m2mb_os_sem_deinit(lwm2m_CSSemHandle);
    m2mb_lwm2m_disable(lwm2mHandle);
    azx_sleep_ms(2000);
    m2mb_lwm2m_deinit(lwm2mHandle);
    return 1;
  }

  azx_sleep_ms(1000);


  AZX_LOG_DEBUG("Waiting LWM2M Registering (120 seconds)...\r\n");
  osRes = m2mb_os_ev_get(eventsHandleLwm2m, EV_LWM2M_SRV_REG_BIT,
      M2MB_OS_EV_GET_ANY_AND_CLEAR, &curEvBits, M2MB_OS_MS2TICKS(120000));
  if (osRes != M2MB_OS_SUCCESS)
  {
    AZX_LOG_ERROR("LWM2M Register timeout!\r\n");
    m2mb_os_ev_deinit(eventsHandleLwm2m);
    m2mb_os_sem_deinit(lwm2m_CSSemHandle);
    m2mb_lwm2m_disable(lwm2mHandle);
    azx_sleep_ms(2000);
    m2mb_lwm2m_deinit(lwm2mHandle);
    return 1;
  }

  AZX_LOG_DEBUG("LWM2M registered\r\n");

  azx_sleep_ms(1000);

  for (i = 0; i < obj_num; i++)
  {
    M2MB_LWM2M_OBJ_URI_T uri;
    M2MB_LWM2M_NEW_INST_REQ_T new_inst_params;

    /*new object instance information*/
    uri.obj = objs[i].obj_id;

    new_inst_params.agent = 0; /*Telit Agent*/

    int j;
    for (j = 0; j < objs[i].inst_num; j++)
    {

      if(!bootstrapped)
        {
        AZX_LOG_TRACE( "Creating new instance %d for object %d\r\n", j, objs[i].obj_id);

        uri.objInst = objs[i].instances_ids[j];
        uri.resource = 0;
        uri.resourceInst = 0;
        uri.uriLen = M2MB_LWM2M_URI_3_FIELDS;

        /*If OK the instance was not present, and so it was created. If an error
         is received, it is likely because the instance already exists.*/
      }
      INT16 *index = NULL;

      if (lwm2mUserData)
      {

        switch( lwm2mUserData->objID )
        {
        case LWM2M_LIGHT_CONTROL_OBJ_ID:
        {
          index = (INT16 *) m2mb_os_malloc(sizeof(INT16));
          if(!index)
          {
            AZX_LOG_ERROR( "Cannot allocate memory\r\n");
            //TODO check if needed free resources
            return M2MB_RESULT_FAIL;
          }
          *index = objs[i].instances_ids[j];
          lwm2mUserData->data = (void*) index;

          break;
        }
        default:
          break;
        }
      }
      if(!bootstrapped)
      {
        retVal = m2mb_lwm2m_newinst(lwm2mHandle, &uri, &new_inst_params);
        if (retVal != M2MB_RESULT_SUCCESS)
        {
          AZX_LOG_WARN("m2mb_lwm2m_newinst returned error %d\r\n", retVal);
          m2mb_os_ev_deinit(eventsHandleLwm2m);
          m2mb_os_sem_deinit(lwm2m_CSSemHandle);
          m2mb_lwm2m_deinit(lwm2mHandle);
          if (index)
          {
            m2mb_os_free(index);
          }
          return M2MB_RESULT_FAIL;
        }

        osRes = m2mb_os_ev_get(
              eventsHandleLwm2m,
              EV_LWM2M_NEW_RES_BIT,
              M2MB_OS_EV_GET_ANY_AND_CLEAR,
              &curEvBits,
              M2MB_OS_MS2TICKS(10000) /*wait 10 seconds for the event to occur*/
              );
        if (osRes != M2MB_OS_SUCCESS)
        {
          AZX_LOG_ERROR("m2mb_lwm2m_newinst timeout!\r\n");
          m2mb_os_ev_deinit(eventsHandleLwm2m);
          m2mb_os_sem_deinit(lwm2m_CSSemHandle);
          m2mb_lwm2m_deinit(lwm2mHandle);
          if (index)
          {
            m2mb_os_free(index);
          }
          azx_sleep_ms(2000);
          return M2MB_RESULT_FAIL;
        }

        AZX_LOG_TRACE("Created NewInst[%d]  uri_new_inst->uriLen: %d \r\n", uri.objInst, uri.uriLen);
      }

    } //for (j = 0; j < objs[i].inst_num; j++)

    switch(uri.obj)
    {
      case TIME_SERIES_METERING_INFO_OBJ_ID:
      {
        uri.objInst = objs[i].instances_ids[0];
        uri.resourceInst = 0;
        uri.uriLen = M2MB_LWM2M_URI_3_FIELDS;

        uri.resource = 1;
        set_string_resource(lwm2mHandle, &uri, (CHAR *)"dummy_meter_serial_number");

        uri.resource = 2;
        set_string_resource(lwm2mHandle, &uri, (CHAR *)"dummy_meter_fw_version");
        break;
      }
      case TIME_SERIES_METERING_RATES_OBJ_ID:
      {
        M2MB_LWM2M_MON_REQ_T mon;
        uri.objInst = objs[i].instances_ids[0];
        uri.resource = 0;
        uri.resourceInst = 0;
        uri.uriLen = M2MB_LWM2M_URI_3_FIELDS;

        mon.mode   = M2MB_LWM2M_MON_MODE_SET_CMD;
        mon.action = M2MB_LWM2M_MON_ENABLE;
        AZX_LOG_INFO( "Register monitor resources\r\n");
        registerMonitorResource(uri, mon);
      }
      break;
      default:
        break;
    }
  } //for (i = 0; i < obj_num; i++)

  {

    /*adding an URI to the list */
    M2MB_LWM2M_OBJ_URI_T uri;

    uri.uriLen       = M2MB_LWM2M_URI_3_FIELDS;
    uri.obj          = TELIT_APPLICATION_DATA_OBJ_ID;
    uri.objInst      = 0;
    uri.resource     = 0;
    uri.resourceInst = 0;  // better set it to 0, even if M2MB_LWM2M_URI_3_FIELDS

    AZX_LOG_DEBUG("enable notify ack\r\n");
    retVal = m2mb_lwm2m_nfy_ack_uri( lwm2mHandle, TRUE, &uri );
    if ( retVal != M2MB_RESULT_SUCCESS )
    {
      AZX_LOG_ERROR( "m2mb_lwm2m_nfy_ack_uri request failed!" );
      return 1;
    }
  }
  if (M2MB_OS_SUCCESS
      != m2mb_os_ev_get(eventsHandleLwm2m,
          EV_LWM2M_NFYADD_RES_BIT, M2MB_OS_EV_GET_ANY_AND_CLEAR, &curEvBits,
          M2MB_OS_MS2TICKS(10000) /*wait 10 seconds for the event to occur*/
      ))
  {
    AZX_LOG_ERROR("m2mb_lwm2m_nfy_ack_uri timeout!\r\n");
    m2mb_os_ev_deinit(eventsHandleLwm2m);
    m2mb_os_sem_deinit(lwm2m_CSSemHandle);
    m2mb_lwm2m_disable(lwm2mHandle);
    azx_sleep_ms(2000);
    m2mb_lwm2m_deinit(lwm2mHandle);
    return 1;
  }
  /* setting the status to ENABLED */
  retVal = m2mb_lwm2m_nfy_ack_status( lwm2mHandle, TRUE, TRUE );
  if ( retVal != M2MB_RESULT_SUCCESS )
  {
    AZX_LOG_ERROR( "m2mb_lwm2m_nfy_ack_status request failed" );
    return 1;
  }
  if (M2MB_OS_SUCCESS
      != m2mb_os_ev_get(eventsHandleLwm2m,
          EV_LWM2M_NFYSTAT_RES_BIT, M2MB_OS_EV_GET_ANY_AND_CLEAR, &curEvBits,
          M2MB_OS_MS2TICKS(10000) /*wait 10 seconds for the event to occur*/
      ))
  {
    AZX_LOG_ERROR("m2mb_lwm2m_nfy_ack_list timeout!\r\n");
    m2mb_os_ev_deinit(eventsHandleLwm2m);
    m2mb_os_sem_deinit(lwm2m_CSSemHandle);
    m2mb_lwm2m_disable(lwm2mHandle);
    azx_sleep_ms(2000);
    m2mb_lwm2m_deinit(lwm2mHandle);

    return 1;
  }
  return 0;
}

/*-----------------------------------------------------------------------------------------------*/
void update_tamper_LWM2MObject(int value)
{
  M2MB_OS_RESULT_E osRes;
  m2mb_os_sem_get(lwm2m_CSSemHandle, M2MB_OS_WAIT_FOREVER);
  M2MB_RESULT_E retVal = m2mb_lwm2m_write(lwm2mHandle, &_obj_tamper_uri,
      &value, sizeof(int));

  osRes = m2mb_os_ev_get(eventsHandleLwm2m, EV_LWM2M_WRITE_RES_BIT,
      M2MB_OS_EV_GET_ANY_AND_CLEAR, NULL, M2MB_OS_MS2TICKS(10000));
  if (osRes != M2MB_OS_SUCCESS) {
    AZX_LOG_ERROR("LWM2M write failure\r\n");

  }

  m2mb_os_sem_put(lwm2m_CSSemHandle);
  if (retVal != M2MB_RESULT_SUCCESS) {
    AZX_LOG_ERROR("m2mb_lwm2m_set returned error %d\r\n", retVal);
    return;
  }
}

/*-----------------------------------------------------------------------------------------------*/
void update_environment_LWM2MObject(float _t, float _p, float _rh, INT16 _iaq) {
  M2MB_OS_RESULT_E osRes;
  m2mb_os_sem_get(lwm2m_CSSemHandle, M2MB_OS_WAIT_FOREVER);
  M2MB_RESULT_E retVal = m2mb_lwm2m_set(lwm2mHandle, &_obj_environment_uri_t,
      &_t, sizeof(float));
  azx_sleep_ms(10);
  retVal = m2mb_lwm2m_set(lwm2mHandle, &_obj_environment_uri_p, &_p,
      sizeof(float));
  azx_sleep_ms(10);
  if (retVal != M2MB_RESULT_SUCCESS) 
  {
    AZX_LOG_ERROR("m2mb_lwm2m_set returned error %d\r\n", retVal);
  }
  retVal = m2mb_lwm2m_set(lwm2mHandle, &_obj_environment_uri_h, &_rh,
      sizeof(float));
  azx_sleep_ms(10);
  if (retVal != M2MB_RESULT_SUCCESS) 
  {
    AZX_LOG_ERROR("m2mb_lwm2m_set returned error %d\r\n", retVal);
  }
  retVal = m2mb_lwm2m_set(lwm2mHandle, &_obj_environment_uri_iaq, &_iaq,
      sizeof(INT16));
  if (retVal != M2MB_RESULT_SUCCESS) 
  {
    AZX_LOG_ERROR("m2mb_lwm2m_set returned error %d\r\n", retVal);
  }

  osRes = m2mb_os_ev_get(eventsHandleLwm2m, EV_LWM2M_SET_RES_BIT,
      M2MB_OS_EV_GET_ANY_AND_CLEAR, NULL, M2MB_OS_MS2TICKS(10000));
  if (osRes != M2MB_OS_SUCCESS) {
    AZX_LOG_ERROR("LWM2M write failure\r\n");
  }

  m2mb_os_sem_put(lwm2m_CSSemHandle);

  if (retVal != M2MB_RESULT_SUCCESS) {
    AZX_LOG_ERROR("m2mb_lwm2m_set returned error %d\r\n", retVal);
    return;
  }
}

/*-----------------------------------------------------------------------------------------------*/
void update_rotation_LWM2MObject(float _w, float _x, float _y, float _z,
    INT16 _acc) {
  M2MB_RESULT_E retVal;
  M2MB_OS_RESULT_E osRes;
  m2mb_os_sem_get(lwm2m_CSSemHandle, M2MB_OS_WAIT_FOREVER);
  retVal = m2mb_lwm2m_set(lwm2mHandle, &_obj_rotation_uri_w, &_w,
      sizeof(float));
  //azx_sleep_ms(100);
  if (retVal != M2MB_RESULT_SUCCESS) 
  {
    AZX_LOG_ERROR("m2mb_lwm2m_set returned error %d\r\n", retVal);
  }
  retVal = m2mb_lwm2m_set(lwm2mHandle, &_obj_rotation_uri_x, &_x,
      sizeof(float));
  //azx_sleep_ms(100);
  if (retVal != M2MB_RESULT_SUCCESS) 
  {
    AZX_LOG_ERROR("m2mb_lwm2m_set returned error %d\r\n", retVal);
  }
  retVal = m2mb_lwm2m_set(lwm2mHandle, &_obj_rotation_uri_y, &_y,
      sizeof(float));
  //azx_sleep_ms(100);
  if (retVal != M2MB_RESULT_SUCCESS) 
  {
    AZX_LOG_ERROR("m2mb_lwm2m_set returned error %d\r\n", retVal);
  }
  retVal = m2mb_lwm2m_set(lwm2mHandle, &_obj_rotation_uri_z, &_z,
      sizeof(float));
  //azx_sleep_ms(100);
  retVal = m2mb_lwm2m_set(lwm2mHandle, &_obj_rotation_uri_a, &_acc,
      sizeof(INT16));
  if (retVal != M2MB_RESULT_SUCCESS) 
  {
    AZX_LOG_ERROR("m2mb_lwm2m_set returned error %d\r\n", retVal);
  }
  osRes = m2mb_os_ev_get(eventsHandleLwm2m, EV_LWM2M_SET_RES_BIT,
      M2MB_OS_EV_GET_ANY_AND_CLEAR, NULL, M2MB_OS_MS2TICKS(10000));
  if (osRes != M2MB_OS_SUCCESS) {
    AZX_LOG_ERROR("LWM2M write failure\r\n");

  }

  m2mb_os_sem_put(lwm2m_CSSemHandle);
  if (retVal != M2MB_RESULT_SUCCESS)
  {
    AZX_LOG_ERROR("m2mb_lwm2m_set returned error %d\r\n", retVal);
    return;
  }
}

/*-----------------------------------------------------------------------------------------------*/
int check_xml_file(const char *name) {
  char path[64] = { 0 };
  struct M2MB_STAT info;
  sprintf(path, "/XML/%s", name);
  AZX_LOG_DEBUG("Looking for <%s> file..\r\n", path);

  if (0 == m2mb_fs_stat(path, &info)) {
    AZX_LOG_DEBUG("File is present, continue...\r\n");
    return 0;
  } else {
    INT32 last_errno = m2mb_fs_get_errno_value();

    if (last_errno == M2MB_FS_ENOENT) {
      AZX_LOG_WARN("File not found.\r\n");
      return -1;
    } else {
      AZX_LOG_ERROR("Error with m2mb_fs_stat, errno is: %d\r\n",
          last_errno);
      return -2;
    }
  }
}

/*-----------------------------------------------------------------------------------------------*/
int getLightControlObjTable(LIGHT_CONTROL_OBJ_INSTANCE_S **lightObjs)
{
  if(NULL == lightObjs)
  {
    return 0;
  }
  else
  {
    *lightObjs = light_control_obj_table;
    return 1;
  }
}
/*-----------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------*/
int copy_xml_file(const char *name) {
  char inpath[64] = { 0 };
  char outpath[64] = { 0 };
  char data[256];
  INT32 in_fd, out_fd;
  sprintf(inpath, "/mod/%s", name);
  sprintf(outpath, "/XML/%s", name);
  in_fd = m2mb_fs_open(inpath, M2MB_O_RDONLY);

  if (-1 == in_fd) {
    AZX_LOG_ERROR("cannot find input file! abort.\r\n");
    return -1;
  } else {
    out_fd = m2mb_fs_open(outpath, M2MB_O_WRONLY | M2MB_O_CREAT);

    if (-1 == out_fd) {
      AZX_LOG_ERROR("cannot create output file! abort.\r\n");
      return -2;
    } else {
      struct M2MB_STAT stats;
      UINT32 filesize;
      UINT32 written = 0;
      UINT32 chunk = sizeof(data);
      AZX_LOG_DEBUG("%s output file opened\r\n", outpath);

      if (-1 == m2mb_fs_fstat(in_fd, &stats)) {
        AZX_LOG_ERROR("cannot stat input file! abort\r\n");
        return -3;
      } else {
        filesize = stats.st_size;
        AZX_LOG_DEBUG("File size: %u\r\n", filesize);

        if (chunk > filesize) {
          AZX_LOG_DEBUG(
              "chunk is %u, bigger than filesize (%u). Decrease it.\r\n",
              chunk, filesize);
          chunk = filesize;
        }

        while (1) {
          SSIZE_T res = 0;
          res = m2mb_fs_read(in_fd, data, chunk);

          if (res != (SSIZE_T) chunk) {
            AZX_LOG_ERROR(
                "read is less than expected: %d instead of %d\r\n",
                res, chunk);
            return -4;
          }

          res = m2mb_fs_write(out_fd, data, chunk);

          if (res != (SSIZE_T) chunk) {
            AZX_LOG_ERROR(
                "written is less than expected: %d instead of %d\r\n",
                res, chunk);
            return -5;
          }

          AZX_LOG_DEBUG("%u bytes written into output file.\r\n",
              chunk);
          written += chunk;

          if ((filesize - written) < chunk) {
            chunk = filesize - written;
          }

          if (written == filesize) {
            AZX_LOG_DEBUG("file written. \r\n");
            break;
          }
        }

        return 0;
      }
    }
  }
}

int registerMonitorResource(M2MB_LWM2M_OBJ_URI_T uri_mon, M2MB_LWM2M_MON_REQ_T mon)
{
  M2MB_RESULT_E     retVal;
  M2MB_OS_RESULT_E  osRes;
  UINT32              curEvBits;

  if (lwm2mHandle == NULL)
  {
    AZX_LOG_ERROR("lwm2mHandle is NULL!\r\n");
    return -1;
  }
  retVal = m2mb_lwm2m_mon(lwm2mHandle, &uri_mon, &mon );
  if ( retVal != M2MB_RESULT_SUCCESS )
  {
    AZX_LOG_ERROR( "m2mb_lwm2m_mon request fails!\r\n" );
    return -1;
  }
  AZX_LOG_TRACE( "m2mb_lwm2m_mon request succeeded!\r\n" );
  osRes = m2mb_os_ev_get(
      eventsHandleLwm2m,
      EV_LWM2M_MON_RES_BIT,
      M2MB_OS_EV_GET_ANY_AND_CLEAR,
      &curEvBits,
      M2MB_OS_MS2TICKS(10000) /*wait 10 seconds for the event to occur*/
  );

  if (osRes != M2MB_OS_SUCCESS)
  {
    AZX_LOG_ERROR("m2mb_lwm2m_mon timeout!\r\n");
    m2mb_os_ev_deinit( eventsHandleLwm2m );

    m2mb_lwm2m_disable(lwm2mHandle);
    azx_sleep_ms(2000);
    m2mb_lwm2m_deinit(lwm2mHandle);
    return -1;
  }

  return 1;
}

int lwm2m_ReadTaskRegister(INT32 taskID) {
  if (taskID > 0) {
    dataTaskID = taskID;
    return 1;
  }
  return -1;
}

M2MB_LWM2M_HANDLE getLwM2mHandle()
{
  return lwm2mHandle;
}

M2MB_OS_EV_HANDLE getEventHandle()
{
  return eventsHandleLwm2m;
}

