#include "hmc_5883l_driver.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// Public functions ==============================================================

int hmc_driverInit( hmc_5883l_driver_t* driver, void* instance, 
                        int (*read) (hmc_5883l_driver_t*, uint8_t*, uint8_t),
                        int (*write)(hmc_5883l_driver_t*, uint8_t*, uint8_t))
{
	if (driver == NULL || instance == NULL || read == NULL || write == NULL)
		return -1;

	driver->instance = instance;
	driver->read = read;
	driver->write = write;

	memset(driver->registers, 0, eHMC5883L_MAX_REGISTERS);

	return 0;
}


int hmc_configureContinuosMode(hmc_5883l_driver_t* driver)
{
	uint8_t hmc_command[2];

	// Write CRA (00) – send 0x3C 0x00 0x70 (8-average, 15 Hz default, normal measurement)
	hmc_command[0] = eHMC5883L_REG_CONFIG_A;
	hmc_command[1] = 0x70;

	if (driver->write(driver, hmc_command, sizeof(hmc_command)) != 0)
		return -1;

	// Write CRB (01) – send 0x3C 0x01 0xA0 (Gain=5, or any other desired gain)
	hmc_command[0] = eHMC5883L_REG_CONFIG_B;
	hmc_command[1] = 0xE0;

	if (driver->write(driver, hmc_command, sizeof(hmc_command)) != 0) 
		return -1;

	// Write Mode (02) – send 0x3C 0x02 0x00 (Continuous-measurement mode)
	hmc_command[0] = eHMC5883L_REG_MODE;
	hmc_command[1] = 0x00;

	if (driver->write(driver, hmc_command, sizeof(hmc_command)) != 0)
		return -1;
	
	return 0;
}


uint16_t hmc_readX_raw(hmc_5883l_driver_t* driver)
{
	uint8_t hmc_command = eHMC5883L_REG_DATA_OUTPUT_X_MSB;
	driver->write(driver, &hmc_command, sizeof(hmc_command));

	driver->read(driver, &driver->registers[eHMC5883L_REG_DATA_OUTPUT_X_MSB], 2);

	return (uint16_t) driver->registers[eHMC5883L_REG_DATA_OUTPUT_X_MSB] << 8 | driver->registers[eHMC5883L_REG_DATA_OUTPUT_X_LSB];
}


uint16_t hmc_readY_raw(hmc_5883l_driver_t* driver)
{
	uint8_t hmc_command = eHMC5883L_REG_DATA_OUTPUT_Y_MSB;
	driver->write(driver, &hmc_command, sizeof(hmc_command));

	driver->read(driver, &driver->registers[eHMC5883L_REG_DATA_OUTPUT_Y_MSB], 2);

	return (uint16_t) driver->registers[eHMC5883L_REG_DATA_OUTPUT_Y_MSB] << 8 | driver->registers[eHMC5883L_REG_DATA_OUTPUT_Y_LSB];
}


uint16_t hmc_readZ_raw(hmc_5883l_driver_t* driver)
{
	uint8_t hmc_command = eHMC5883L_REG_DATA_OUTPUT_Z_MSB;
	driver->write(driver, &hmc_command, sizeof(hmc_command));

	driver->read(driver, &driver->registers[eHMC5883L_REG_DATA_OUTPUT_Z_MSB], 2);

	return (uint16_t) driver->registers[eHMC5883L_REG_DATA_OUTPUT_Z_MSB] << 8 | driver->registers[eHMC5883L_REG_DATA_OUTPUT_Z_LSB];
}


int hmc_readX(hmc_5883l_driver_t* driver)
{
	int x;
	uint16_t rawx = hmc_readX_raw(driver);

	x = (int) rawx;

	return x;
}


int hmc_readY(hmc_5883l_driver_t* driver)
{
	int y;
	uint16_t rawy = hmc_readY_raw(driver);

	y = (int) rawy;

	return y;
}


int hmc_readZ(hmc_5883l_driver_t* driver)
{
	int z;
	uint16_t rawz = hmc_readZ_raw(driver);

	z = (int) rawz;

	return z;
}