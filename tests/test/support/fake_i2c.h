#ifndef __FAKE_I2C_H__
#define __FAKE_I2C_H__

#include "hmc_5883l_driver.h"

int fakeRead(hmc_5883l_driver_t* driver, uint8_t* data, uint8_t len);
int fakeWrite(hmc_5883l_driver_t* driver, uint8_t* data, uint8_t len);

#endif