
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

#include "azx_log.h"
#include "azx_utils.h"
#include "app_cfg.h"

#include "lwm2m.h"


/* Local defines ================================================================================*/
/* Local typedefs ===============================================================================*/
/* Local statics ================================================================================*/

/* ONEEDGE */
static M2MB_LWM2M_HANDLE lwm2mHandle;

static M2MB_LWM2M_OBJ_URI_T _obj_telit_service_uri = {.uriLen = 4, .obj = 33211, .objInst = 0, .resource = 0, .resourceInst = 1};


static M2MB_LWM2M_OBJ_URI_T _obj_tamper_uri = {3, TAMPERING_OBJ_ID, 0, 1};

static M2MB_LWM2M_OBJ_URI_T _obj_environment_uri_t = {3, ENVIRONMENT_OBJ_ID, 0, 1, 0};
static M2MB_LWM2M_OBJ_URI_T _obj_environment_uri_p = {3, ENVIRONMENT_OBJ_ID, 0, 2, 0};
static M2MB_LWM2M_OBJ_URI_T _obj_environment_uri_h = {3, ENVIRONMENT_OBJ_ID, 0, 3, 0};
static M2MB_LWM2M_OBJ_URI_T _obj_environment_uri_iaq = {3, ENVIRONMENT_OBJ_ID, 0, 4, 0};

static M2MB_LWM2M_OBJ_URI_T _obj_rotation_uri_w = {3, ROTATION_OBJ_ID, 0, 1, 0};
static M2MB_LWM2M_OBJ_URI_T _obj_rotation_uri_x = {3, ROTATION_OBJ_ID, 0, 2, 0};
static M2MB_LWM2M_OBJ_URI_T _obj_rotation_uri_y = {3, ROTATION_OBJ_ID, 0, 3, 0};
static M2MB_LWM2M_OBJ_URI_T _obj_rotation_uri_z = {3, ROTATION_OBJ_ID, 0, 4, 0};
static M2MB_LWM2M_OBJ_URI_T _obj_rotation_uri_a = {3, ROTATION_OBJ_ID, 0, 5, 0};

static M2MB_LWM2M_OBJ_URI_T _obj_smartlock_uri = { 3, SMARTLOCK_OBJ_ID, 0, 1};



static M2MB_OS_EV_HANDLE lwm2m_evHandle = NULL;

/* Local function prototypes ====================================================================*/
/* Static functions =============================================================================*/
/* Global functions =============================================================================*/

void lwm2mIndCB( M2MB_LWM2M_HANDLE h, M2MB_LWM2M_EVENT_E event, UINT16 resp_size, void *resp_struct,
                 void *userdata )
{
  ( void ) h;
  ( void ) resp_size;
  ( void ) userdata;

  /* Client generated events */
  switch( event )
  {
    
    case M2MB_LWM2M_ENABLE_RES:
    {
      M2MB_LWM2M_ENABLE_RES_T *resp = ( M2MB_LWM2M_ENABLE_RES_T * )resp_struct;
      if(resp->result == M2MB_LWM2M_RES_SUCCESS)
      {
        AZX_LOG_DEBUG( "LWM2M enable result OK\r\n");
        m2mb_os_ev_set(lwm2m_evHandle, EV_LWM2M_ENABLE_RES_BIT, M2MB_OS_EV_SET);
      }
      else
      {
        AZX_LOG_WARN( "Enable result %d\r\n", resp->result );
      }
      break;
    }
    
    case M2MB_LWM2M_SET_RES:
    {
      M2MB_LWM2M_SET_RES_T *resp = ( M2MB_LWM2M_SET_RES_T * )resp_struct;
      if(resp->result == M2MB_LWM2M_RES_SUCCESS)
      {
        AZX_LOG_DEBUG( "LWM2M set result OK\r\n");
        m2mb_os_ev_set(lwm2m_evHandle, EV_LWM2M_SET_RES_BIT, M2MB_OS_EV_SET);
      }
      else
      {
        AZX_LOG_WARN( "set result %d\r\n", resp->result );
      }
      break;
    }

    case M2MB_LWM2M_WRITE_RES:
    {
      M2MB_LWM2M_WRITE_RES_T *resp = ( M2MB_LWM2M_WRITE_RES_T * )resp_struct;
      if(resp->result == M2MB_LWM2M_RES_SUCCESS)
      {
        AZX_LOG_TRACE( "LWM2M write result OK\r\n");
        m2mb_os_ev_set(lwm2m_evHandle, EV_LWM2M_WRITE_RES_BIT, M2MB_OS_EV_SET);
      }
      else
      {
        AZX_LOG_WARN( "Enable write %d\r\n", resp->result );
      }
      break;
    }

    case M2MB_LWM2M_NEW_INST_RES:
    {
      // event in response to the m2mb_lwm2m_newinst
      M2MB_LWM2M_NEW_INST_RES_T *resp = ( M2MB_LWM2M_NEW_INST_RES_T * )resp_struct;

      switch( resp->result )
      {
        case M2MB_LWM2M_RES_SUCCESS:
          AZX_LOG_TRACE( "New Instance created successfully\r\n" );
          break;

        case M2MB_LWM2M_RES_FAIL_NOT_ALLOWED:
          AZX_LOG_WARN( "New instance creation not allowed (already present?)\r\n" );
          break;

        default:
          AZX_LOG_DEBUG( "Creating object instance specified: generic result %d\r\n", resp->result );
          break;
      }

      break;
    }

    case  M2MB_LWM2M_SRV_INFO_IND:
    {

      AZX_LOG_TRACE("M2MB_LWM2M_SRV_INFO_IND\r\n");
      M2MB_LWM2M_SRV_INFO_IND_T *resp = ( M2MB_LWM2M_SRV_INFO_IND_T * )resp_struct;


      switch(resp->info)
      {
        case M2MB_LWM2M_CL_STATE_DISABLED:
          break;
        case M2MB_LWM2M_CL_STATE_BOOTSTRAPPING:
          AZX_LOG_DEBUG( "resp->info == M2MB_LWM2M_CL_STATE_BOOTSTRAPPING\r\n" );
          break;
        case M2MB_LWM2M_CL_STATE_BOOTSTRAPPED:
          AZX_LOG_DEBUG( "resp->info == M2MB_LWM2M_CL_STATE_BOOTSTRAPPED\r\n" );
          break;
        case M2MB_LWM2M_CL_STATE_REGISTERING:
          AZX_LOG_DEBUG( "resp->info == M2MB_LWM2M_CL_STATE_REGISTERING\r\n" );
          break;
        case M2MB_LWM2M_CL_STATE_REGISTERED:
          AZX_LOG_DEBUG( "resp->info == M2MB_LWM2M_CL_STATE_REGISTERED\r\n" );
          m2mb_os_ev_set(lwm2m_evHandle, EV_LWM2M_SRV_REG_BIT, M2MB_OS_EV_SET);
          break;
        case M2MB_LWM2M_CL_STATE_DEREGISTERING:
          AZX_LOG_DEBUG( "resp->info == M2MB_LWM2M_CL_STATE_DEREGISTERING\r\n" );
          break;
        case M2MB_LWM2M_CL_STATE_SUSPENDED:
          AZX_LOG_DEBUG( "resp->info == M2MB_LWM2M_CL_STATE_SUSPENDED\r\n" );
          break;
        default:
          AZX_LOG_WARN( "resp->info: unexpected value!! %d\r\n", resp->info);
          break;
      }
      break;
    }

    case  M2MB_LWM2M_SESSION_INFO_IND:
    {
      AZX_LOG_DEBUG( "M2MB_LWM2M_SESSION_INFO_IND\r\n" );
      M2MB_LWM2M_INFO_IND_T *resp = ( M2MB_LWM2M_INFO_IND_T * )resp_struct;

      if( resp->info == M2MB_LWM2M_INFO_RESOURCE_EXECUTE )
      {
        M2MB_LWM2M_OBJ_URI_T *currData = ( M2MB_LWM2M_OBJ_URI_T * )resp->data;
        AZX_LOG_DEBUG( "Info Exec Ind: %d/%d/%d\r\n", currData->obj, currData->objInst,
                       currData->resource );
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

    default:
      AZX_LOG_DEBUG( "LWM2M EVENT %d\r\n", event );
      break;
  }
}

/*-----------------------------------------------------------------------------------------------*/
uint8_t oneedge_init( INT32 obj_id )
{
  M2MB_RESULT_E retVal;
  M2MB_LWM2M_ENABLE_REQ_T pars;
  M2MB_LWM2M_OBJ_URI_T uri;
  M2MB_LWM2M_NEW_INST_REQ_T new_inst_params;
  INT32 service_enable = 1;
  

  M2MB_OS_RESULT_E        osRes;
  M2MB_OS_EV_ATTR_HANDLE  evAttrHandle;
  UINT32                  curEvBits;
  


  /* Init events handler */
  osRes = m2mb_os_ev_setAttrItem( &evAttrHandle, CMDS_ARGS(M2MB_OS_EV_SEL_CMD_CREATE_ATTR, NULL, M2MB_OS_EV_SEL_CMD_NAME, "lwm2m_ev"));
  osRes = m2mb_os_ev_init( &lwm2m_evHandle, &evAttrHandle );

  if ( osRes != M2MB_OS_SUCCESS )
  {
    m2mb_os_ev_setAttrItem( &evAttrHandle, M2MB_OS_EV_SEL_CMD_DEL_ATTR, NULL );
    AZX_LOG_CRITICAL("m2mb_os_ev_init failed!\r\n");
    return -1;
  }
  else
  {
    AZX_LOG_TRACE("m2mb_os_ev_init success\r\n");
  }

  
  //get the handle of the lwm2m client on lwm2mHandle
  retVal = m2mb_lwm2m_init( &lwm2mHandle, lwm2mIndCB, ( void * )NULL );

  if( retVal != M2MB_RESULT_SUCCESS )
  {
    AZX_LOG_ERROR( "m2mb_lwm2m_init returned error %d\r\n", retVal );
    m2mb_os_ev_deinit( lwm2m_evHandle );
    return -1;
  }


  retVal = m2mb_lwm2m_write( lwm2mHandle, &_obj_telit_service_uri, &service_enable, sizeof( INT32 ) );
  if( retVal != M2MB_RESULT_SUCCESS )
  {
    AZX_LOG_ERROR( "m2mb_lwm2m_write returned error %d\r\n", retVal );
    m2mb_os_ev_deinit( lwm2m_evHandle );

    m2mb_lwm2m_deinit( lwm2mHandle );
    return -1;
  }

  //AT#LWM2MENA=1
  memset( &pars, 0, sizeof( M2MB_LWM2M_ENABLE_REQ_T ) );

  pars.apnclass = 1; /*CID*/
  pars.guardRequestEventSecs = 5;
  pars.guardReleaseEventSecs = 5;
  pars.commandType = M2MB_LWM2MENA_CMD_TYPE_SET;
  pars.mode = M2MB_LWM2M_MODE_NO_ACK;

  retVal = m2mb_lwm2m_enable( lwm2mHandle, &pars );
  if( retVal != M2MB_RESULT_SUCCESS )
  {
    AZX_LOG_ERROR( "m2mb_lwm2m_enable returned error %d\r\n", retVal );
    m2mb_os_ev_deinit( lwm2m_evHandle );
    m2mb_lwm2m_deinit( lwm2mHandle );
    return -1;
  }

  if(M2MB_OS_SUCCESS != m2mb_os_ev_get(
      lwm2m_evHandle,
      EV_LWM2M_ENABLE_RES_BIT,
      M2MB_OS_EV_GET_ANY_AND_CLEAR,
      &curEvBits,
      M2MB_OS_MS2TICKS(10000) /*wait 10 seconds for the event to occur*/
  )
  )
  {
    AZX_LOG_ERROR("m2mb_lwm2m_enable timeout!\r\n");
    m2mb_os_ev_deinit( lwm2m_evHandle );

    azx_sleep_ms(2000);
    m2mb_lwm2m_deinit(lwm2mHandle);
    return -1;
  }



  AZX_LOG_DEBUG("Waiting LWM2M Registering (120 seconds timeout)...\r\n");
  osRes = m2mb_os_ev_get(lwm2m_evHandle, EV_LWM2M_SRV_REG_BIT, M2MB_OS_EV_GET_ANY_AND_CLEAR, &curEvBits, M2MB_OS_MS2TICKS(120000));
  if(osRes != M2MB_OS_SUCCESS)
  {
    AZX_LOG_ERROR("LWM2M Register timeout!\r\n");
    m2mb_os_ev_deinit( lwm2m_evHandle );

    m2mb_lwm2m_disable(lwm2mHandle);
    azx_sleep_ms(2000);
    m2mb_lwm2m_deinit(lwm2mHandle);
    return -1;
  }

  azx_sleep_ms(1000);
  
  /*new object instance information*/
  uri.obj = obj_id;
  uri.objInst = 0;

  new_inst_params.agent = 0; /*Telit Agent*/
  /*If OK the instance was not present, and so it was created. If an error
     is received, it is likely because the instance already exists.*/
  retVal = m2mb_lwm2m_newinst( lwm2mHandle, &uri, &new_inst_params );

  if( retVal != M2MB_RESULT_SUCCESS )
  {
    AZX_LOG_ERROR( "m2mb_lwm2m_newinst returned error %d\r\n", retVal );
    m2mb_os_ev_deinit( lwm2m_evHandle );
    m2mb_lwm2m_deinit( lwm2mHandle );
    return -1;
  }

  return 0;
}


/*-----------------------------------------------------------------------------------------------*/
void update_tamper_LWM2MObject( int value )
{
  M2MB_RESULT_E retVal = m2mb_lwm2m_write( lwm2mHandle, &_obj_tamper_uri, &value, sizeof( int ) );

  if( retVal != M2MB_RESULT_SUCCESS )
  {
    AZX_LOG_ERROR( "m2mb_lwm2m_set returned error %d\r\n", retVal );
    return;
  }
}


/*-----------------------------------------------------------------------------------------------*/
void update_environment_LWM2MObject( float _t, float _p, float _rh, INT16 _iaq )
{
  M2MB_RESULT_E retVal = m2mb_lwm2m_set( lwm2mHandle, &_obj_environment_uri_t, &_t, sizeof( float ) );
  azx_sleep_ms( 10 );
  retVal = m2mb_lwm2m_set( lwm2mHandle, &_obj_environment_uri_p, &_p, sizeof( float ) );
  azx_sleep_ms( 10 );
  retVal = m2mb_lwm2m_set( lwm2mHandle, &_obj_environment_uri_h, &_rh, sizeof( float ) );
  azx_sleep_ms( 10 );
  retVal = m2mb_lwm2m_set( lwm2mHandle, &_obj_environment_uri_iaq, &_iaq, sizeof( INT16 ) );

  if( retVal != M2MB_RESULT_SUCCESS )
  {
    AZX_LOG_ERROR( "m2mb_lwm2m_set returned error %d\r\n", retVal );
    return;
  }
}


/*-----------------------------------------------------------------------------------------------*/
void update_rotation_LWM2MObject( float _w, float _x, float _y, float _z, INT16 _acc )
{
  M2MB_RESULT_E retVal;
  retVal = m2mb_lwm2m_set( lwm2mHandle, &_obj_rotation_uri_w, &_w, sizeof( float ) );
  //azx_sleep_ms(100);
  retVal = m2mb_lwm2m_set( lwm2mHandle, &_obj_rotation_uri_x, &_x, sizeof( float ) );
  //azx_sleep_ms(100);
  retVal = m2mb_lwm2m_set( lwm2mHandle, &_obj_rotation_uri_y, &_y, sizeof( float ) );
  //azx_sleep_ms(100);
  retVal = m2mb_lwm2m_set( lwm2mHandle, &_obj_rotation_uri_z, &_z, sizeof( float ) );
  //azx_sleep_ms(100);
  retVal = m2mb_lwm2m_set( lwm2mHandle, &_obj_rotation_uri_a, &_acc, sizeof( INT16 ) );

  if( retVal != M2MB_RESULT_SUCCESS )
  {
    AZX_LOG_ERROR( "m2mb_lwm2m_set returned error %d\r\n", retVal );
    return;
  }
}


/*-----------------------------------------------------------------------------------------------*/
void update_smartlock_LWM2MObject( int value )
{
  M2MB_RESULT_E retVal = m2mb_lwm2m_write( lwm2mHandle, &_obj_smartlock_uri, &value, sizeof( int ) );

  if( retVal != M2MB_RESULT_SUCCESS )
  {
    AZX_LOG_ERROR( "m2mb_lwm2m_set returned error %d\r\n", retVal );
    return;
  }
}


/*-----------------------------------------------------------------------------------------------*/
int check_xml_file( const char *name )
{
  char path[64] = {0};
  struct M2MB_STAT info;
  sprintf( path, "/XML/%s", name );
  AZX_LOG_DEBUG( "Looking for <%s> file..\r\n", path );

  if( 0 == m2mb_fs_stat( path, &info ) )
  {
    AZX_LOG_DEBUG( "File is present, continue...\r\n" );
    return 0;
  }
  else
  {
    INT32 last_errno = m2mb_fs_get_errno_value();

    if( last_errno == M2MB_FS_ENOENT )
    {
      AZX_LOG_WARN( "File not found.\r\n" );
      return -1;
    }
    else
    {
      AZX_LOG_ERROR( "Error with m2mb_fs_stat, errno is: %d\r\n", last_errno );
      return -2;
    }
  }
}


/*-----------------------------------------------------------------------------------------------*/
int copy_xml_file( const char *name )
{
  char inpath[64] = {0};
  char outpath[64] = {0};
  char data[256];
  INT32 in_fd, out_fd;
  sprintf( inpath, "/mod/%s", name );
  sprintf( outpath, "/XML/%s", name );
  in_fd = m2mb_fs_open( inpath, M2MB_O_RDONLY );

  if( -1 == in_fd )
  {
    AZX_LOG_ERROR( "cannot find input file! abort.\r\n" );
    return -1;
  }
  else
  {
    out_fd = m2mb_fs_open( outpath, M2MB_O_WRONLY | M2MB_O_CREAT );

    if( -1 == out_fd )
    {
      AZX_LOG_ERROR( "cannot create output file! abort.\r\n" );
      return -2;
    }
    else
    {
      struct M2MB_STAT stats;
      UINT32 filesize;
      UINT32 written = 0;
      UINT32 chunk = sizeof( data );
      AZX_LOG_DEBUG( "%s output file opened\r\n", outpath );

      if( -1 == m2mb_fs_fstat( in_fd, &stats ) )
      {
        AZX_LOG_ERROR( "cannot stat input file! abort\r\n" );
        return -3;
      }
      else
      {
        filesize = stats.st_size;
        AZX_LOG_DEBUG( "File size: %u\r\n", filesize );

        if( chunk > filesize )
        {
          AZX_LOG_DEBUG( "chunk is %u, bigger than filesize (%u). Decrease it.\r\n", chunk, filesize );
          chunk = filesize;
        }

        while( 1 )
        {
          SSIZE_T res = 0;
          res = m2mb_fs_read( in_fd, data, chunk );

          if( res != ( SSIZE_T ) chunk )
          {
            AZX_LOG_ERROR( "read is less than expected: %d instead of %d\r\n", res, chunk );
            return -4;
          }

          res = m2mb_fs_write( out_fd, data, chunk );

          if( res != ( SSIZE_T ) chunk )
          {
            AZX_LOG_ERROR( "written is less than expected: %d instead of %d\r\n", res, chunk );
            return -5;
          }

          AZX_LOG_DEBUG( "%u bytes written into output file.\r\n", chunk );
          written += chunk;

          if( ( filesize - written ) < chunk )
          {
            chunk = filesize - written;
          }

          if( written == filesize )
          {
            AZX_LOG_DEBUG( "file written. \r\n" );
            break;
          }
        }

        return 0;
      }
    }
  }
}
