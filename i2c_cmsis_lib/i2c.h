/*
 * i2c.h
 *
 *  Created on: Nov 19, 2024
 *      Author: dell
 */

#ifndef I2C_H_
#define I2C_H_

#include "main.h"
#include <stdbool.h>

void i2c_I2C1_GPIO_config(void);
void i2c_I2C1_config(void);
bool i2c_I2C1_isSlaveAddressExist(uint8_t Addr);

bool i2c_I2C1_masterTransmit_IT(uint8_t Addr, uint8_t *pData, uint8_t len, uint32_t timeout);
bool i2c_I2C1_masterTransmit(uint8_t Addr, uint8_t *pData, uint8_t len, uint32_t timeout);
bool i2c_I2C1_masterReceive(uint8_t Addr, uint8_t *pData, uint8_t len, uint32_t timeout);
bool I2C1_masterReceive(uint8_t Addr, uint8_t *pData, uint8_t len, uint32_t timeout);
void receive_handler();
bool I2C1_masterReceive_IT(uint8_t Addr, uint8_t *pData, uint8_t len, uint32_t timeout);
void error_i2c_handler();
bool masterReceive(uint8_t addrs, uint8_t *pData, uint8_t len, uint32_t timeout);
uint8_t DS3231_Read_custom(uint8_t Addr, uint8_t *pData, uint8_t reg,uint8_t len, uint32_t timeout);

bool I2C_WriteEEPROM(uint8_t slave_addr,uint8_t mem_addr, uint8_t *data, uint8_t length,uint32_t timeout);
bool Write_eeprom(uint8_t slave_addr, uint8_t mem_addr, uint8_t *pData, uint8_t mem_size, uint8_t len, uint32_t timeout);
bool Read_eeprom(uint8_t slave_addr, uint8_t *pData, uint8_t mem_addr,uint8_t mem_size,uint8_t len, uint32_t timeout);
#endif /* I2C_H_ */
