/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "esp_err.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_eth.h" // for esp_eth_handle_t

#ifdef __cplusplus
extern "C" {
#endif

#define KSZ8863_I2C_DEV_ADDR 0xBE

typedef enum
{
    KSZ8863_I2C_MODE,
    KSZ8863_SPI_MODE,
    KSZ8863_SMI_MODE
} ksz8863_intf_mode_t;

typedef struct
{
    uint8_t dev_addr;
    i2c_port_t i2c_master_port;
} ksz8863_ctrl_i2c_config_t;

typedef struct
{
    spi_host_device_t host_id;
    int32_t clock_speed_hz;
    int32_t spics_io_num;
} ksz8863_ctrl_spi_config_t;

typedef struct
{
    ksz8863_intf_mode_t host_mode;
    union
    {
        ksz8863_ctrl_i2c_config_t *i2c_dev_config;
        ksz8863_ctrl_spi_config_t *spi_dev_config;
    };
} ksz8863_ctrl_intf_config_t;

esp_err_t ksz8863_ctrl_intf_init(ksz8863_ctrl_intf_config_t *config);

esp_err_t ksz8863_phy_reg_read(esp_eth_handle_t eth_handle, uint32_t phy_addr, uint32_t phy_reg, uint32_t *reg_value);
esp_err_t ksz8863_phy_reg_write(esp_eth_handle_t eth_handle, uint32_t phy_addr, uint32_t phy_reg, uint32_t reg_value);

#ifdef __cplusplus
}
#endif
