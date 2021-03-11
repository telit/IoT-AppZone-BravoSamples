/*===============================================================================================*/
/*         >>> Copyright (C) Telit Communications S.p.A. Italy All Rights Reserved. <<<          */
/**
  @file
    i2c_bme680.h

  @brief
    demo i2c utilities for BME680 device

  @details


  @note
    Dependencies:
    m2mb_types.h

  @author WhiteBeard
  @author FabioPi
  
  @date
    2020-02-15
*/

#ifndef SRC_I2C_BME680_H_
#define SRC_I2C_BME680_H_


/* Global declarations ==========================================================================*/
/* Global typedefs ==============================================================================*/
/* Global functions =============================================================================*/

/**
  @brief        Read data from buffer using I2C

  @param[in]    dev_id    Device address on I2C bus
  @param[in]    reg_addr  Start register address
  @param[in]    data      Pointer to buffer where data will be stored
  @param[in]    len       Size of the data to be read
  @return                 1 on error, 0 if OK

*/
int8_t bme680_i2c_read( uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len );

/**
  @brief        Write data to buffer using I2C

  @param[in]    dev_id    Device address on I2C bus
  @param[in]    reg_addr  Start register address
  @param[in]    data      Pointer to buffer with data to be read
  @param[in]    len       Size of the data to be written
  @return                 1 on error, 0 if OK

*/
int8_t bme680_i2c_write( uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len );


#endif /* SRC_I2C_BME680_H_ */
