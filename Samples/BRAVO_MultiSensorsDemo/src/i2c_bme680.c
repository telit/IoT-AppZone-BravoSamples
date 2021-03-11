
/*=========================================================================*/
/*   Copyright (C) Telit Communications S.p.A. Italy All Rights Reserved.  */
/**
  @file
    i2c_bme680.c

  @brief
    The file contains i2c utilities for BME680 device

  @details


  @author WhiteBeard
  @author FabioPi
  
  @date
    2020-02-15
*/
/* Include files ================================================================================*/
#include <stdio.h>
#include <string.h>

#include "m2mb_types.h"

#include "m2mb_os_api.h"
#include "m2mb_i2c.h"

#include "azx_log.h"
#include "azx_utils.h"
#include "app_cfg.h"

#include "bhy_uc_driver.h"

#include "i2c_bme680.h"

#include "i2c.h"

/* Local defines ================================================================================*/
/* Local typedefs ===============================================================================*/
/* Local statics ================================================================================*/
static INT32 i2c_fd = -1;
/* Local function prototypes ====================================================================*/
/* Static functions =============================================================================*/
/* Global functions =============================================================================*/

#ifdef BRAVO_REV_B
int8_t bme680_i2c_read( uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len )
{
  AZX_LOG_TRACE( "bme680_i2c_read\r\n" );
  AZX_LOG_TRACE( "bme680_i2c_read LEN = %d\r\n", len );
  int8_t rc = bhy_soft_passthru_read( dev_id, reg_addr, data, len, 1 ); // increment reg

  if( rc )
  {
    AZX_LOG_ERROR( "bme680_i2c_read ERROR %d\r\n", rc );
  }
  return rc;
}

/*-----------------------------------------------------------------------------------------------*/
int8_t bme680_i2c_write( uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len )
{
  AZX_LOG_TRACE( "bme680_i2c_write\r\n" );
  AZX_LOG_TRACE( "bme680_i2c_write LEN = %d\r\n", len );
  int8_t rc = bhy_soft_passthru_write( dev_id, reg_addr, data, len, 1 ); // increment reg

  if( rc )
  {
    AZX_LOG_ERROR( "bme680_i2c_read ERROR %d\r\n", rc );
  }

  return rc;
}
#else

int bme680_openI2C( uint8_t dev_id )
{
  INT32 res;
  CHAR dev_ID[64];
  M2MB_I2C_CFG_T config;
  M2MB_OS_SEM_ATTR_HANDLE semAttrHandle;

  if (NULL == I2C_CSSemHandle)
  {
    m2mb_os_sem_setAttrItem(&semAttrHandle,
        CMDS_ARGS(M2MB_OS_SEM_SEL_CMD_CREATE_ATTR, NULL,
            M2MB_OS_SEM_SEL_CMD_COUNT, 1 /*CS*/,
            M2MB_OS_SEM_SEL_CMD_TYPE, M2MB_OS_SEM_GEN));
    m2mb_os_sem_init( &I2C_CSSemHandle, &semAttrHandle );
  }

  /**************
      Configuring the IIC device.
   **************/
  AZX_LOG_INFO( "\r\nConfiguring the Bosch BME680 device...\r\n" );
  //Create device name using device address in decimal base, left shifted by 1 bit
  sprintf( dev_ID, "/dev/I2C-%d",
           ( UINT16 ) dev_id << 1 ); //I2C API require an already left-shifted device address
  AZX_LOG_INFO( "Opening channel %s\r\n", dev_ID );
  i2c_fd = m2mb_i2c_open( dev_ID, 0 );

  if( -1 == i2c_fd )
  {
    AZX_LOG_ERROR( "cannot open I2C channel!\r\n" );
    return 1;
  }

  config.sclPin = I2C_SCL;
  config.sdaPin = I2C_SDA;
  config.registerId = 0x00; //dummy register
  res = m2mb_i2c_ioctl( i2c_fd, M2MB_I2C_IOCTL_SET_CFG, ( void * )&config );

  if( res != 0 )
  {
    AZX_LOG_ERROR( "cannot configure I2C channel\r\n" );
    return 1;
  }

  return 0;
}

/*-----------------------------------------------------------------------------------------------*/
int8_t bme680_i2c_write( uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len  )
{
  INT32 i2c_res;
  M2MB_I2C_CFG_T config;

  AZX_LOG_TRACE( "bme680_i2c_write\r\n" );
  AZX_LOG_TRACE( "bme680_i2c_write LEN = %d\r\n", len );

  if( i2c_fd == -1 )
  {
    i2c_res = bme680_openI2C(dev_id);
    if(i2c_res != 0)
    {
      AZX_LOG_ERROR( "bme680_i2c_write ERROR %d\r\n", i2c_res );
      return 1;
    }
  }

  i2c_res = m2mb_i2c_ioctl( i2c_fd, M2MB_I2C_IOCTL_GET_CFG, ( void * )&config );

  if( i2c_res != 0 )
  {
    AZX_LOG_ERROR( "cannot get I2C channel configuration\r\n" );
    return 1;
  }

  config.registerId = reg_addr;
  i2c_res = m2mb_i2c_ioctl( i2c_fd, M2MB_I2C_IOCTL_SET_CFG, ( void * )&config );

  if( i2c_res != 0 )
  {
    AZX_LOG_ERROR( "cannot set I2C channel configuration\r\n" );
    return 1;
  }

  AZX_LOG_TRACE( "Configuring I2C Registers - Writing %d bytes into 0x%02X register...\r\n", len,
                 reg_addr );

  m2mb_os_sem_get(I2C_CSSemHandle, M2MB_OS_WAIT_FOREVER);
  i2c_res = m2mb_i2c_write( i2c_fd, data, len );
  m2mb_os_sem_put(I2C_CSSemHandle);

  if( i2c_res != len )
  {
    AZX_LOG_ERROR( "cannot write data! error: %d\r\n", i2c_res );
    return 1;
  }
  else
  {
    AZX_LOG_TRACE( "Write: success\r\n" );
    return 0;
  }
}

/*-----------------------------------------------------------------------------------------------*/
int8_t bme680_i2c_read_16( uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size,
                           M2MB_I2C_CFG_T *pConfig )
{
  ( void )addr;

  for( int i = 0; size > 0; i++ )
  {
    pConfig->registerId = reg;
    INT32 i2c_res = m2mb_i2c_ioctl( i2c_fd, M2MB_I2C_IOCTL_SET_CFG, ( void * )pConfig );

    if( i2c_res != 0 )
    {
      AZX_LOG_ERROR( "Cannot set I2C channel configuration\r\n" );
      return 1;
    }

    uint16_t tmpsize;

    if( size >= 16 )
    {
      tmpsize = 16;
    }
    else
    {
      tmpsize = size;
    }

    i2c_res = m2mb_i2c_read( i2c_fd, p_buf, tmpsize ); //reading 16 bytes max at a time

    if( i2c_res == tmpsize )
    {
      size = size - tmpsize;
      reg = reg + tmpsize;
      p_buf = p_buf + tmpsize;
      AZX_LOG_TRACE( "Reading Success.\r\n" );
      AZX_LOG_TRACE( "i2c->" );

      for( int i = 1; i < i2c_res; i++ )
      {
        AZX_LOG_TRACE( " %02x", p_buf[i] );
      }

      AZX_LOG_TRACE( "\r\n" );
    }
    else
    {
      AZX_LOG_ERROR( "Reading FAIL for %d bytes on register 0x%02X! - Exit Value: %d.\r\n", size, reg,
                     i2c_res );
      return 1;
    }
  }

  return 0;
}


/*-----------------------------------------------------------------------------------------------*/
int8_t bme680_i2c_read( uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len  )
{

  INT32 i2c_res;
  M2MB_I2C_CFG_T config;

  AZX_LOG_TRACE( "bme680_i2c_read\r\n" );
  AZX_LOG_TRACE( "bme680_i2c_read LEN = %d\r\n", len );

  if( i2c_fd == -1 )
  {
    i2c_res = bme680_openI2C(dev_id);
    if(i2c_res != 0)
    {
      AZX_LOG_ERROR( "bme680_i2c_read ERROR %d\r\n", i2c_res );
      return 1;
    }
  }


  AZX_LOG_TRACE( "I2C read register..." );
  i2c_res = m2mb_i2c_ioctl( i2c_fd, M2MB_I2C_IOCTL_GET_CFG, ( void * )&config );

  if( i2c_res != 0 )
  {
    AZX_LOG_ERROR( "Cannot get I2C channel configuration\r\n" );
    return 1;
  }

  m2mb_os_sem_get(I2C_CSSemHandle, M2MB_OS_WAIT_FOREVER);
  if( reg_addr == 0 ) // if read FIFO double buffer
  {
    for( ; len > 50; )
    {
      i2c_res = bme680_i2c_read_16( dev_id, reg_addr, data, 50, &config );
      len = len - 50;
      data = data + 50;
    }

    i2c_res += bme680_i2c_read_16( dev_id, reg_addr, data, len, &config );
  }
  else
  {
    i2c_res = bme680_i2c_read_16( dev_id, reg_addr, data, len, &config );
  }
  m2mb_os_sem_put(I2C_CSSemHandle);
  return i2c_res;
}

#endif
