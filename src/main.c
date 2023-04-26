/*
 * Copyright (c) 2015 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

/**
 * @file Sample app using the Fujitsu MB85RC256V FRAM through I2C.
 */

/* Macros ========================================================*/
/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

/* I2C device address */
#define HMC5883L_I2C_ADDR	0x1E
#define MAX_HMC_FIELDS_ 	13

/* Threads =======================================================*/
static void hmc_configureContinuosMode(struct device *const i2c_dev)
{
	uint8_t hmc_byte[2], ret;

	// Write CRA (00) – send 0x3C 0x00 0x70 (8-average, 15 Hz default, normal measurement)
	hmc_byte[0] = 0;
	hmc_byte[1] = 0x70;
	ret = i2c_write(i2c_dev, hmc_byte, 2, HMC5883L_I2C_ADDR);
	if (ret)
		printk("Error on write CRA.");

	// Write CRB (01) – send 0x3C 0x01 0xA0 (Gain=5, or any other desired gain)
	hmc_byte[0] = 0x01;
	//hmc_byte[1] = 0xA0;
	hmc_byte[1] = 0xE0;
	ret = i2c_write(i2c_dev, hmc_byte, 2, HMC5883L_I2C_ADDR);
	if (ret)
		printk("Error on write CRB.");

	// Write Mode (02) – send 0x3C 0x02 0x00 (Continuous-measurement mode)
	hmc_byte[0] = 0x02;
	hmc_byte[1] = 0x00;
	ret = i2c_write(i2c_dev, hmc_byte, 2, HMC5883L_I2C_ADDR);
	if (ret)
		printk("Error on write Mode.");
}

static void hmc_thread(void)
{
	const struct device *const i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	uint8_t hmc_buf[MAX_HMC_FIELDS_];
	uint8_t ret;
	uint8_t command[2] = {0x06, 0x03};

	hmc_configureContinuosMode(i2c_dev);

	for(;;)
	{
		int16_t xx, yy, zz;
		uint8_t cmd;
		k_msleep(100);


		cmd=3;
		ret = i2c_write(i2c_dev, &cmd, 1, HMC5883L_I2C_ADDR);
		if (ret)
			printk("Error on write Mode.");
		ret = i2c_read(i2c_dev, &hmc_buf[3], 3, HMC5883L_I2C_ADDR);
		if (ret)
			printk("Error on write 1st command.\r\n");
		cmd=6;
		ret = i2c_write(i2c_dev, &cmd, 1, HMC5883L_I2C_ADDR);
		if (ret)
			printk("Error on write Mode.");
		ret = i2c_read(i2c_dev, &hmc_buf[6], 3, HMC5883L_I2C_ADDR);
		if (ret)
			printk("Error on write 1st command.\r\n");
		cmd=9;
		ret = i2c_write(i2c_dev, &cmd, 1, HMC5883L_I2C_ADDR);
		if (ret)
			printk("Error on write Mode.");
		ret = i2c_read(i2c_dev, &hmc_buf[9], 4, HMC5883L_I2C_ADDR);
		if (ret)
			printk("Error on write 1st command.\r\n");

		xx = hmc_buf[3] << 8 | hmc_buf[4];
		zz = hmc_buf[5] << 8 | hmc_buf[6];
		yy = hmc_buf[7] << 8 | hmc_buf[8];

		printk("x:(%i) y:(%i) z:(%i) status:(%x)\r\n", xx, yy, zz, hmc_buf[9]);
		printk("idA:(%x) idB:(%x) idC:(%x)", hmc_buf[10], hmc_buf[11], hmc_buf[12]);
	}

#if 0
	hmc_buf[0] = 9;
	ret = i2c_write(i2c_dev, hmc_buf, 1, HMC5883L_I2C_ADDR);
	if (ret)
	{
		printk("Error on 1st write.");
	}

	ret = i2c_read(i2c_dev, hmc_buf, 4, HMC5883L_I2C_ADDR);
	if (ret)
	{
		printk("Error on 1st write.");
	}
	else
	{
		printk("Successfully read all bytes:\r\n");
		for( uint8_t i=0; i<4; ++i)
		{
			printk("field[%d]: (0x%x)\r\n", i, hmc_buf[i]);
		}
	}
#endif
}


void main(void)
{
}

K_THREAD_DEFINE(hmc_id, STACKSIZE, hmc_thread, NULL, NULL, NULL,
		PRIORITY, 0, 0);