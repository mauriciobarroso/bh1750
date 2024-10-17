/**
  ******************************************************************************
  * @file           : bh1750.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BH1750_H_
#define BH1750_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C2 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32P4
#define ESP32_TARGET
#endif

#ifdef ESP32_TARGET
#include "driver/i2c_master.h"
#else
#include "main.h"
#endif /* ESP32_TARGET */

/* Exported Macros -----------------------------------------------------------*/
#define BH1750_CMD_POWER_DOWN			0x00 /*!< Command to set Power Down */
#define BH1750_CMD_POWER_ON				0x01 /*!< Command to set Power On */
#define BH1750_CMD_DATA_REG_RESET		0x07 /*!< Command to reset data register, not acceptable in power down mode */

#define BH1750_I2C_ADDR	0x23 /*!< I2C slave address */

#define BH1750_MEAS_ACCURACY	1.2 /*!< the typical measurement accuracy of  BH1750 sensor */

/* Exported typedef ----------------------------------------------------------*/
typedef enum {
	BH1750_MEAS_MODE_CONTINUE_1LX_RES = 0x10, /*!< Command to set measure mode as Continuously H-Resolution mode */
	BH1750_MEAS_MODE_CONTINUE_HALFLX_RES = 0x11, /*!< Command to set measure mode as Continuously H-Resolution mode2 */
	BH1750_MEAS_MODE_CONTINUE_4LX_RES = 0x13, /*!< Command to set measure mode as Continuously L-Resolution mode */
	BH1750_MEAS_MODE_ONETIME_1LX_RES = 0x20, /*!< Command to set measure mode as One Time H-Resolution mode */
	BH1750_MEAS_MODE_ONETIME_HALFLX_RES = 0x21, /*!< Command to set measure mode as One Time H-Resolution mode2 */
	BH1750_MEAS_MODE_ONETIME_4LX_RES = 0x23 /*!< Command to set measure mode as One Time L-Resolution mode */	
} bh1750_meas_mode_t;

typedef struct {
#ifdef ESP32_TARGET
	i2c_master_dev_handle_t handle;
#else
	uint8_t addr;
	I2C_HandleTypeDef *handle;
#endif /* ESP32_TARGET */
} bh1750_i2c_t;

/*
 * @brief BH1750 device structure
 */
typedef struct {
	bh1750_i2c_t i2c_dev;
} bh1750_t;

/* Exported variables --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief Function to initialize a SGP41 instance
 *
 * @param me         : Pointer to a bh1750_t instance
 * @param i2c_handle : Pointer to I2C handle to configure the device 
 * @param dev_addr   : I2C device address
 *
 * @return 0 on success
 */
int bh1750_init(bh1750_t *const me, void *i2c_handle, uint8_t dev_addr);

/**
 * @brief Function to set BH170 as power down mode (low current)
 *
 * @param me : Pointer to a bh1750_t instance
 *
 * @return 0 on success
 */
int bh1750_power_down(bh1750_t *const me);

/**
 * @brief Function to set BH1750 as power on mode
 *
 * @param me : Pointer to a bh1750_t instance
 *
 * @return 0 on success
 */
int bh1750_power_on(bh1750_t *const me);

/**
 * @brief Function to reset BH1750 data register
 *
 * @param me : Pointer to a bh1750_t instance
 *
 * @return 0 on success
 */
int bh1750_reset_data_reg(bh1750_t *const me);

/**
 * @brief Function to set BH1750 measurement mode
 *
 * @param me        : Pointer to a bh1750_t instance
 * @param meas_mode : Measurement mode (continue or one-time)
 *
 * @return 0 on success
 */
int bh1750_set_meas_mode(bh1750_t *const me, bh1750_meas_mode_t meas_mode);

/**
 * @brief Function to get BH1750 light intensity data
 *
 * @param me   : Pointer to a bh1750_t instance
 * @param data : Light intensity value
 *
 * @return 0 on success
 */
int bh1750_get_data(bh1750_t *const me, float *data);

/**
 * @brief Function to set BH1750 measurement mode and get the light intesity data
 *
 * @param me        : Pointer to a bh1750_t instance
 * @param meas_mode : Measurement mode (continue or one-time)
 * @param data      : Light intensity value
 *
 * @return 0 on success
 */
int bh1750_get_light_intensity(bh1750_t *const me, bh1750_meas_mode_t meas_mode, float *data);

/**
 * @brief Function to set BH1750 measurement time
 *
 * @param me        : Pointer to a bh1750_t instance
 * @param meas_time : Measurement time
 *
 * @return 0 on success
 */
int bh1750_set_meas_time(bh1750_t *const me, uint8_t meas_time);

#ifdef __cplusplus
}
#endif

#endif /* BH1750_H_ */

/***************************** END OF FILE ************************************/
