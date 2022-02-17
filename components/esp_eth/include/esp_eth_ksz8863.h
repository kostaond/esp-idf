/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "esp_eth.h"
#include "ksz8863_ctrl.h"

#define KSZ8863_PORT_1 (0)
#define KSZ8863_PORT_2 (1)

/**
 * @brief Default configuration for KSZ8863 Ethernet driver
 *
 */
#define ETH_KSZ8863_DEFAULT_CONFIG(emac, ephy)      \
    {                                               \
        .mac = emac,                                \
        .phy = ephy,                                \
        .check_link_period_ms = 2000,               \
        .stack_input = NULL,                        \
        .on_lowlevel_init_done = NULL,              \
        .on_lowlevel_deinit_done = NULL,            \
        .read_phy_reg = ksz8863_phy_reg_read,    \
        .write_phy_reg = ksz8863_phy_reg_write,  \
    }

typedef enum {
    KSZ8863_ETH_CMD_S_MAC_STA_TBL = ETH_CMD_S_PHY_LOOPBACK + 1,
    KSZ8863_ETH_CMD_G_MAC_STA_TBL,
    KSZ8863_ETH_CMD_G_MAC_DYN_TBL,
    KSZ8863_ETH_CMD_S_TAIL_TAG,
    KSZ8863_ETH_CMD_G_TAIL_TAG,
    KSZ8863_ETH_CMD_G_PORT_NUM,
} ksz8863_eth_io_cmd_t;

#include "../src/ksz8863.h" // TODO: just for demonstration will be probably removed
typedef struct
{
    uint16_t start_entry;
    uint16_t etries_num;
    union
    {
        ksz8863_dyn_mac_table_t *dyn_tbls;
        ksz8863_sta_mac_table_t *sta_tbls;
    };
} ksz8863_mac_tbl_info_t;

esp_err_t ksz8863_sw_reset(esp_eth_handle_t port_eth_handle);
esp_err_t ksz8863_hw_reset(int reset_gpio_num);
esp_err_t ksz8863_p3_rmii_internal_clk(esp_eth_handle_t port_eth_handle, bool rmii_internal_clk);
esp_err_t ksz8863_p3_rmii_clk_invert(esp_eth_handle_t port_eth_handle, bool rmii_clk_invert);

esp_err_t ksz8863_register_tail_tag_port(esp_eth_handle_t port_eth_handle, int32_t port_num);
esp_err_t ksz8863_tail_tag_port_forward(esp_eth_handle_t host_eth_handle, uint8_t *buffer, uint32_t length, void *priv);

esp_err_t ksz8863_eth_transmit_normal_lookup(esp_eth_handle_t host_eth_handle, void *buf, size_t length);

// TODO: consider making it static somehow
esp_err_t ksz8863_eth_transmit_tag(esp_eth_handle_t host_eth_handle, void *buf, size_t length, uint8_t tail_tag);
