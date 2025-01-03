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
uint8_t DS3231_Read(uint8_t Addr, uint8_t *pData, uint8_t len, uint32_t timeout);
#endif /* I2C_H_ */
