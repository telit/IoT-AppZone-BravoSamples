/*===============================================================================================*/
/*         >>> Copyright (C) Telit Communications S.p.A. Italy All Rights Reserved. <<<          */
/**
  @file
    i2c.h

  @brief
    demo i2c utilities

  @details


  @note
    Dependencies:
    m2mb_types.h

  @author WhiteBeard
  @author FabioPi

  @date
    2020-02-15
*/
#ifndef SRC_I2C_H_
#define SRC_I2C_H_

/* Global declarations ==========================================================================*/
/* default I2C configuration */
#define I2C_SDA       (UINT8) 2  //GPIO 2
#define I2C_SCL       (UINT8) 3  //GPIO 3
#define BHY_I2C_160B_ADDR1 (UINT16) 0x28


/* Global typedefs ==============================================================================*/
/* Global functions =============================================================================*/


/**
  @brief        Opens I2C device at BHI160B default address

*/
int open_I2C( void );


/**
  @brief        Writes data from buffer using I2C

  @param[in]    addr    Device address on I2C bus (not used)
  @param[in]    reg     Start register address
  @param[in]    p_buf   Pointer to buffer with data to be written
  @param[in]    size    Size of the data to be written
  @return               1 on error, 0 if OK

*/
int8_t sensor_i2c_write( uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size );

/**
  @brief        Reads data to buffer using I2C 16 bytes a t a time - internal function

  @param[in]    addr    Device address on I2C bus (not used)
  @param[in]    reg     Start register address
  @param[in]    p_buf   Pointer to buffer with data to be read
  @param[in]    size    Size of the data to be written
  @return               1 on error, 0 if OK

*/
int8_t sensor_i2c_read( uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size );

/**
  @brief        Closes I2C device channel

*/
int close_I2C( void );

#endif /* SRC_I2C_H_ */
