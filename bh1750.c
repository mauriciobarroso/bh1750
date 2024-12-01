/**
  ******************************************************************************
  * @file           : bh1750.c
  * @author         : Mauricio Barroso Benavides
  * @date           : Oct 1, 2024
  * @brief          : todo: write brief
  ******************************************************************************
  * @attention
  *
  * MIT License
  *
  * Copyright (c) 2024 Mauricio Barroso Benavides
  *
  * Permission is hereby granted, free of charge, to any person obtaining a copy
  * of this software and associated documentation files (the "Software"), to
  * deal in the Software without restriction, including without limitation the
  * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
  * sell copies of the Software, and to permit persons to whom the Software is
  * furnished to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in
  * all copies or substantial portions of the Software.
  * 
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
  * IN THE SOFTWARE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bh1750.h"

#ifdef ESP32_TARGET
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#endif /* ESP32_TARGET */

/* Private macros ------------------------------------------------------------*/
#define NOP() asm volatile ("nop")

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief Function that implements the default I2C read transaction
 *
 * @param data     : Pointer to the data to be read from addr
 * @param data_len : Length of the data transfer
 * @param intf     : Pointer to the interface descriptor
 *
 * @return 0 if successful, non-zero otherwise
 */
static int8_t bh1750_reg_read(uint8_t *data, uint32_t data_len, void *intf);

/**
 * @brief Function that implements the default I2C write transaction
 *
 * @param data     : Pointer to the data to be written to addr
 * @param data_len : Length of the data transfer
 * @param intf     : Pointer to the interface descriptor
 *
 * @return 0 if successful, non-zero otherwise
 */
static int8_t bh1750_reg_write(const uint8_t *data, uint8_t data_len, void *intf);

/**
 * @brief Function that implements a mili seconds delay
 *
 * @param time_ms: Time in us to delay
 */
static void delay_ms(uint32_t time_ms);

/* Exported functions definitions --------------------------------------------*/
/**
 * @brief Function to initialize a SGP41 instance
 */
int bh1750_init(bh1750_t *const me, void *i2c_handle, uint8_t dev_addr)
{
	/* Variable to return error code */
	int ret = 0;
	
	/**/
	#ifdef ESP32_TARGET
	/* Add device to I2C bus */
	i2c_device_config_t i2c_dev_conf = {
			.scl_speed_hz = 400000,
			.device_address = dev_addr
	};

	if (i2c_master_bus_add_device((i2c_master_bus_handle_t)i2c_handle,
			&i2c_dev_conf, &me->i2c_dev.handle) != 0) {
		return ret;
	}
#else
	me->i2c_dev.handle = (I2C_HandleTypeDef *)i2c_handle;
	me->i2c_dev.addr = dev_addr;
#endif /* ESP32_TARGET */
	
	/* Return 0 for success */	
	return ret;
}

/**
 * @brief Function to set BH170 as power down mode (low current)
 */
int bh1750_power_down(bh1750_t *const me)
{
	/* Variable to return error code */
	int ret = 0;
	
	/**/
	uint8_t cmd = (uint8_t)BH1750_CMD_POWER_DOWN;
	ret = bh1750_reg_write(&cmd, 1, &me->i2c_dev);
	
	if (ret != 0) {
		return ret;
	}
	
	/* Return 0 for success */	
	return ret;
}

/**
 * @brief Function to set BH1750 as power on mode
 */
int bh1750_power_on(bh1750_t *const me)
{
	/* Variable to return error code */
	int ret = 0;
	
	/**/
	uint8_t cmd = (uint8_t)BH1750_CMD_POWER_ON;
	ret = bh1750_reg_write(&cmd, 1, &me->i2c_dev);
	
	if (ret != 0) {
		return ret;
	}
	
	/* Return 0 for success */	
	return ret;
}

/**
 * @brief Function to reset BH1750 data register
 */
int bh1750_reset_data_reg(bh1750_t *const me)
{
	/* Variable to return error code */
	int ret = 0;
	
	/**/
	ret = bh1750_power_on(me);
	
	if (ret != 0) {
		return ret;
	}
	
	uint8_t cmd = BH1750_CMD_DATA_REG_RESET;
	ret = bh1750_reg_write(&cmd, 1, &me->i2c_dev);

	if (ret != 0) {
		return ret;
	}	
	
	/* Return 0 for success */	
	return ret;
}

/**
 * @brief Function to set BH1750 measurement mode
 */
int bh1750_set_meas_mode(bh1750_t *const me, bh1750_meas_mode_t meas_mode)
{
	/* Variable to return error code */
	int ret = 0;
	
	/**/
	uint8_t cmd = (uint8_t)meas_mode;
	ret = bh1750_reg_write(&cmd, 1, &me->i2c_dev);

	if (ret != 0) {
		return ret;
	}	 
	
	/* Return 0 for success */	
	return ret;
}

/**
 * @brief Function to get BH1750 light intensity data
 */
int bh1750_get_data(bh1750_t *const me, float *data)
{
	/* Variable to return error code */
	int ret = 0;
	
	/**/
	uint8_t buf[2] = {0};
	
	ret = bh1750_reg_read(buf, 2, &me->i2c_dev);

	if (ret != 0) {
		return ret;
	}	 	
	
    *data = ((buf[0] << 8 | buf[1]) / BH1750_MEAS_ACCURACY);

	
	/* Return 0 for success */	
	return ret;
}

/**
 * @brief Function to set BH1750 measurement mode and get the light intesity data
 */
int bh1750_get_light_intensity(bh1750_t *const me, bh1750_meas_mode_t meas_mode, float *data)
{
	/* Variable to return error code */
	int ret = 0;
	
	/**/
	ret = bh1750_set_meas_mode(me, meas_mode);
	
	if (ret != 0) {
		return ret;
	}
	
    if ((meas_mode == BH1750_MEAS_MODE_CONTINUE_4LX_RES) || (meas_mode == BH1750_MEAS_MODE_ONETIME_4LX_RES)) {
        delay_ms(24);
    } else {
        delay_ms(180);
    }

    ret = bh1750_get_data(me, data);
    
	if (ret != 0) {
		return ret;
	}
	
	/* Return 0 for success */	
	return ret;
}

/**
 * @brief Function to set BH1750 measurement time
 */
int bh1750_set_meas_time(bh1750_t *const me, uint8_t meas_time)
{
	/* Variable to return error code */
	int ret = 0;
	
	/**/
    uint8_t buf[2] = {0x40, 0x60};
    buf[0] |= meas_time >> 5;
    buf[1] |= meas_time & 0x1F;
    
    bh1750_reg_write(buf, 2, &me->i2c_dev);
	
	/* Return 0 for success */	
	return ret;
}

/* Private function definitions ----------------------------------------------*/
/**
 * @brief Function that implements the default I2C read transaction
 */
static int8_t bh1750_reg_read(uint8_t *data, uint32_t data_len, void *intf)
{
	bh1750_i2c_t *i2c_dev = (bh1750_i2c_t *)intf;

#ifdef ESP32_TARGET
	if (i2c_master_receive(i2c_dev->handle, data, data_len, -1)
			!= 0) {
		return -1;
	}
#else
	if (HAL_I2C_Master_Receive(i2c_dev->handle, (i2c_dev->addr << 1) | 0x01,
			data, data_len, 100) > 0) {
		return -1;
	}
#endif

	return 0;
}


/**
 * @brief Function that implements the default I2C write transaction
 */
static int8_t bh1750_reg_write(const uint8_t *data, uint8_t data_len, void *intf)
{
	uint8_t buf[32];
	
	for (uint8_t i = 0; i < data_len; i++) {
		buf[i] = data[i];
	}

	/* Transmit buffer */
	bh1750_i2c_t *i2c_dev = (bh1750_i2c_t *)intf;

#ifdef ESP32_TARGET
	if (i2c_master_transmit(i2c_dev->handle, buf, data_len, -1)
			!= 0) {
		return -1;
	}
#else
	if (HAL_I2C_Master_Transmit(i2c_dev->handle, i2c_dev->addr << 1, buf, data_len,	100)) {
		return -1;
	}
#endif /* ESP32_TARGET */
	return 0;
}

/**
 * @brief Function that implements a mili seconds delay
 */
static void delay_ms(uint32_t time_ms)
{
#ifdef ESP32_TARGET
	uint64_t m = (uint64_t)esp_timer_get_time();

	uint32_t period_us = time_ms * 1000;
	if (period_us) {
		uint64_t e = (m + period_us);

		if (m > e) { /* overflow */
			while ((uint64_t)esp_timer_get_time() > e) {
				NOP();
			}
		}

		while ((uint64_t)esp_timer_get_time() < e) {
			NOP();
		}
	}
#else
  HAL_Delay(time_ms);
#endif /* ESP32_TARGET */
}

/***************************** END OF FILE ************************************/
