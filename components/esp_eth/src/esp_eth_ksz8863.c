/*
 * SPDX-FileCopyrightText: 2019-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// TODO: clear
#include <string.h>
#include <stdlib.h>
#include <sys/cdefs.h>
#include <sys/queue.h>

#include "esp_check.h"

#include "esp_eth_ksz8863.h"
#include "ksz8863.h" // registers

static const char *TAG = "ksz8863_eth";


struct ksz8863_port_tbl_s {
    esp_eth_handle_t eth_handle;
    int32_t port_num;
    SLIST_ENTRY(ksz8863_port_tbl_s) next;
};

static SLIST_HEAD(slisthead, ksz8863_port_tbl_s) s_port_tbls_head;

esp_err_t ksz8863_register_port_hndl(esp_eth_handle_t port_eth_handle, int32_t port_num)
{
    struct ksz8863_port_tbl_s *item = malloc(sizeof(struct ksz8863_port_tbl_s));
    item->eth_handle = port_eth_handle;
    item->port_num = port_num;
    SLIST_INSERT_HEAD(&s_port_tbls_head, item, next);

    return ESP_OK;
}

esp_err_t ksz8863_unregister_port_hndl(esp_eth_handle_t eth_handle)
{
    struct ksz8863_port_tbl_s *item;
    while (!SLIST_EMPTY(&s_port_tbls_head)) {
        item = SLIST_FIRST(&s_port_tbls_head);
        SLIST_REMOVE_HEAD(&s_port_tbls_head, next);
        free(item);
    }
    return ESP_OK;
}

esp_err_t ksz8863_port_forward(esp_eth_handle_t host_eth_handle, uint8_t *buffer, uint32_t length, void *priv)
{
    struct ksz8863_port_tbl_s *item;
    //printf("len %u\n", length);
    SLIST_FOREACH(item, &s_port_tbls_head, next) {
        if (item->port_num == buffer[length - 1]) {
            esp_eth_mediator_t *eth = item->eth_handle;
            eth->stack_input(eth, buffer, length - 1);
            return ESP_OK;
        }
    }
    return ESP_OK;
}

esp_err_t ksz8863_sw_reset(esp_eth_handle_t eth_handle)
{
    esp_err_t ret = ESP_OK;
    esp_eth_mediator_t *eth = (esp_eth_mediator_t *)eth_handle;

    ESP_LOGW(TAG, "SW reset resets all Global, MAC and PHY registers!"); // TODO: discuss make it as debug message??
    ksz8863_reset_reg_t reset;
    reset.sw_reset = 1;
    reset.pcs_reset = 1; // Physical coding sublayer

    ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, 0, KSZ8863_RESET_ADDR, reset.val), err, TAG, "write reset failed");
    return ESP_OK;
err:
    return ret;
}

esp_err_t ksz8863_hw_reset(int reset_gpio_num)
{
    if (reset_gpio_num >= 0) {
        esp_rom_gpio_pad_select_gpio(reset_gpio_num);
        gpio_set_direction(reset_gpio_num, GPIO_MODE_OUTPUT);
        gpio_set_level(reset_gpio_num, 0);
        esp_rom_delay_us(150); // insert little more than min input assert time
        gpio_set_level(reset_gpio_num, 1);
    }
    return ESP_OK;
}

esp_err_t ksz8863_p3_rmii_internal_clk(esp_eth_handle_t eth_handle, bool rmii_internal_clk)
{
    esp_err_t ret = ESP_OK;
    esp_eth_mediator_t *eth = (esp_eth_mediator_t *)eth_handle;

    // Set P3 RMII Clock as Internal
    ksz8863_fwdfrmhostm_reg_t mode;
    ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, 0, KSZ8863_FWDFRM_HOSTM_ADDR, &(mode.val)), err, TAG, "read FWDFRM_HOSTM failed");
    mode.p3_rmii_clk = rmii_internal_clk;
    ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, 0, KSZ8863_FWDFRM_HOSTM_ADDR, mode.val), err, TAG, "write FWDFRM_HOSTM failed");

    return ESP_OK;
err:
    return ret;
}

esp_err_t ksz8863_p3_rmii_clk_invert(esp_eth_handle_t eth_handle, bool rmii_clk_invert)
{
    esp_err_t ret = ESP_OK;
    esp_eth_mediator_t *eth = (esp_eth_mediator_t *)eth_handle;

    ksz8863_idrlq0_reg_t idrlq0;
    ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, 0, KSZ8863_P3IDRLQ0_ADDR, &(idrlq0.val)), err, TAG, "read P3IDRLQ0 failed");
    idrlq0.rmii_reflck_invert = rmii_clk_invert;
    ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, 0, KSZ8863_P3IDRLQ0_ADDR, idrlq0.val), err, TAG, "write P3IDRLQ0 failed");

    return ESP_OK;
err:
    return ret;
}
