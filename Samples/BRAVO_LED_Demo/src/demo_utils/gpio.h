/*===============================================================================================*/
/*         >>> Copyright (C) Telit Communications S.p.A. Italy All Rights Reserved. <<<          */
/**
  @file
    gpio.h

  @brief
    demo gpio utilities

  @details


  @note
    Dependencies:
    m2mb_types.h

  @author WhiteBeard
  @author FabioPi

  @date
    2020-02-15
*/

#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_

/* Global declarations ==========================================================================*/

#define RED_LED_GPIO 	1
#define YELLOW_LED_GPIO 9
#define GREEN_LED_GPIO	10

/* Global typedefs ==============================================================================*/

/* Global functions =============================================================================*/

/**
  @brief        Callback for GPIO interrupts

  @param[in]    fd          File descriptor of GPIO device
  @param[in]    userdata    Address of user data buffer

*/
void gpio_interr_cb( UINT32 fd, void *userdata );



/**
  @brief        Close GPIO input file/device

  @param[in]    *pin         GPIO pin number
  @retval                   0 = device close, error other values

*/
int close_gpio( int *pin );

/**
  @brief        Open GPIO input file/device

  @param[in]    pin         GPIO pin number
  @retval                   0 = device open, error other values

*/
int open_gpio( int pin );

/**
  @brief        Open GPIO output file/device for LED

  @param[in]    pin         GPIO pin number
  @retval                   0 = device open, error other values

*/
int open_LED( int pin );

/**
  @brief        Read value from GPIO file/device

  @param[in]    fd         file/device descriptor
  @retval                  GPIO status

*/
M2MB_GPIO_VALUE_E read_gpio( INT32 fd );

M2MB_GPIO_VALUE_E read_gpio( void );


/**
  @brief        Write value to GPIO file/device

  @param[in]    fd         file/device descriptor
  @param[in]    value      GPIO status to be written

*/
void write_gpio( INT32 fd, M2MB_GPIO_VALUE_E value );

/**
  @brief        Write value to LED 1

  @param[in]    value      GPIO status to be written

*/
void write_LED( M2MB_GPIO_VALUE_E value );


/**
  @brief        Write value to LED by pin

  @param[in]    value      GPIO status to be written

*/
void writeLEDbyIndex( INT32 index, M2MB_GPIO_VALUE_E value );
/**
  @brief        Return the GPIO descriptor

  @param[in]    value      GPIO index
  return -1 in case of error

*/
INT32 getGpioDescriptor(INT32 pin);

/**
  @brief        Return the GPIO pin

  @param[in]    value      GPIO index
  return -1 in case of error

*/
INT32 getGpioPinByIndex(INT32 index);

#endif /* SRC_GPIO_H_ */
