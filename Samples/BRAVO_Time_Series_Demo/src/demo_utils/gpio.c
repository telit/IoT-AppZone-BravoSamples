
/*=========================================================================*/
/*   Copyright (C) Telit Communications S.p.A. Italy All Rights Reserved.  */
/**
  @file
    gpio.c

  @brief
    The file contains gpio utilities

  @details


  @author WhiteBeard
  @author FabioPi

  @date
    2020-02-15
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

#include "gpio.h"

/* Local defines ================================================================================*/

/* Local typedefs ===============================================================================*/

/* Local statics ================================================================================*/
static INT8 LED_GPIOS[3] = {RED_LED_GPIO, YELLOW_LED_GPIO, GREEN_LED_GPIO};

/* GPIO files descriptors */
static INT32 gpio_fd = -1;
static INT32 led_fd[3] = {-1,-1,-1};
/* Local function prototypes ====================================================================*/
/* Static functions =============================================================================*/
/* Global functions =============================================================================*/


void gpio_interr_cb( UINT32 fd, void *userdata )
{
  ( void )fd;
  ( void )userdata;
}

/*-----------------------------------------------------------------------------------------------*/
int close_gpio( int *pin )
{
  INT32 ret = -1;
  ret = m2mb_gpio_close(*pin);
  if(ret != 0)
  {
    return -1;
  }

  *pin = 0;
  return 0;
}

/*-----------------------------------------------------------------------------------------------*/
int open_gpio( int pin )
{
  INT32 ret = -1;
  char path[32];
  memset( path, 0, sizeof( path ) );
  sprintf( path, "/dev/GPIO%d", pin );
  /* Open file descriptor for GPIO*/
  gpio_fd = m2mb_gpio_open( path, 0 );

  if( gpio_fd != -1 )
  {
    ret = m2mb_gpio_multi_ioctl( gpio_fd,
                                 CMDS_ARGS( M2MB_GPIO_IOCTL_SET_DIR, M2MB_GPIO_MODE_INPUT,      /*Set gpio in input mode*/
                                            M2MB_GPIO_IOCTL_SET_PULL,
                                            M2MB_GPIO_PULL_DOWN,                      /*Set pull configuration as pull down*/
                                            M2MB_GPIO_IOCTL_SET_DRIVE, M2MB_GPIO_MEDIUM_DRIVE,                /*Pull drive set to medium*/
                                            M2MB_GPIO_IOCTL_SET_INTR_TYPE,
                                            INTR_CB_SET,                        /*Select interrupt type as callback function*/
                                            M2MB_GPIO_IOCTL_SET_INTR_TRIGGER,
                                            M2MB_GPIO_INTR_ANYEDGE,        /*Select interrupt event on both edges*/
                                            M2MB_GPIO_IOCTL_SET_INTR_CB, ( UINT32 )gpio_interr_cb,             /*Register interrupt callback*/
                                            M2MB_GPIO_IOCTL_SET_INTR_ARG, ( UINT32 )
                                            pin,                 /*Register interrupt callback parameter*/
                                            M2MB_GPIO_IOCTL_INIT_INTR, ( UINT32 )NULL ) /*enable interrupts*/
                               );

    if( ret != -1 )
    {
      ret = 0;
    }
  }
  else
  {
    ret = 1;
  }

  return ret;
}

/*-----------------------------------------------------------------------------------------------*/
int open_LED( int index )
{
  INT32 ret = -1;
  char path[32];
  memset( path, 0, sizeof( path ) );

  if (index < 0 || index > 3)
  {
    return ret;
  }

  sprintf( path, "/dev/GPIO%d", LED_GPIOS[index] );

  /* Open file descriptor for GPIO*/
  led_fd[index] = m2mb_gpio_open( path, 0 );

  if( led_fd[index] != -1 )
  {
    ret = m2mb_gpio_multi_ioctl( led_fd[index],
                                 CMDS_ARGS( M2MB_GPIO_IOCTL_SET_DIR, M2MB_GPIO_MODE_OUTPUT,      /*Set gpio in output mode*/
                                            M2MB_GPIO_IOCTL_SET_PULL, M2MB_GPIO_PULL_DOWN,          /*Set pull configuration as pull down*/
                                            M2MB_GPIO_IOCTL_SET_DRIVE, M2MB_GPIO_MEDIUM_DRIVE )      /*Pull drive set to medium*/
                               );
    if( ret != -1 )
    {
      ret = 0;
    }
  }
  else
  {
    ret = 1;
  }

  return ret;
}

/*-----------------------------------------------------------------------------------------------*/
M2MB_GPIO_VALUE_E read_gpio( INT32 fd )
{
  M2MB_GPIO_VALUE_E value;

  if( 0 == m2mb_gpio_read( fd, &value ) )
  {
    return value;
  }
  else
  {
    return M2MB_GPIO_LOW_VALUE;
  }
}

/*-----------------------------------------------------------------------------------------------*/
void write_gpio( INT32 fd, M2MB_GPIO_VALUE_E value )
{
  m2mb_gpio_write( fd, value );
}

/*-----------------------------------------------------------------------------------------------*/
M2MB_GPIO_VALUE_E read_gpio( void )
{
  return read_gpio( gpio_fd );
}

/*-----------------------------------------------------------------------------------------------*/
void write_LED( M2MB_GPIO_VALUE_E value )
{
  m2mb_gpio_write( led_fd[0], value );
}

/*-----------------------------------------------------------------------------------------------*/
void writeLEDbyIndex( INT32 index, M2MB_GPIO_VALUE_E value )
{
  m2mb_gpio_write( led_fd[index], value );
}

/*-----------------------------------------------------------------------------------------------*/
INT32 getGpioDescriptor(INT32 index)
{
  if (index < 0 || index > 2)
  {
    return -1;
  }
  return led_fd[index];
}

/*-----------------------------------------------------------------------------------------------*/

INT32 getGpioPinByIndex(INT32 index)
{
  if (index < 0 || index > 2)
  {
    return -1;
  }
  return LED_GPIOS[index];
}

/*-----------------------------------------------------------------------------------------------*/
