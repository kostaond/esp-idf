/*
 * SPDX-FileCopyrightText: 2019-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// TODO: think more about file and variables names, the idea about pmac = Port MAC (or pseudo MAC)
// TODO: clear the headers!
#include <string.h>
#include <stdlib.h>
#include <sys/cdefs.h>
#include <sys/queue.h>
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_eth.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_intr_alloc.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "hal/cpu_hal.h"

//#include "sdkconfig.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"

#include "esp_eth_ksz8863.h"
#include "ksz8863_ctrl_internal.h" // indirect read/write
#include "ksz8863.h" // registers

#define KSZ8863_GLOBAL_INIT_DONE     (1 << 0)

static const char *TAG = "ksz8863_mac";

typedef struct {
    esp_eth_mac_t parent;
    esp_eth_mediator_t *eth;
    uint32_t sw_reset_timeout_ms;
    pmac_ksz8863_mode_t mode;
    bool flow_ctrl_enabled;
    int32_t port;
    uint8_t port_reg_offset;
    esp_eth_handle_t host_eth_handle;
    uint32_t status;
} pmac_ksz8863_t;


struct slist_mac_ksz8863_s {
    pmac_ksz8863_t *mac_ksz8863_info;
    SLIST_ENTRY(slist_mac_ksz8863_s) next;
};

static SLIST_HEAD(slisthead, slist_mac_ksz8863_s) s_mac_ksz8863_head;

/**
 * @brief verify ksz8863 chip ID
 */
static esp_err_t ksz8863_verify_id(pmac_ksz8863_t *emac)
{
    esp_err_t ret = ESP_OK;
    esp_eth_mediator_t *eth = emac->eth;

    /* Check PHY ID */
    ksz8863_chipid0_reg_t id0;
    ksz8863_chipid1_reg_t id1;
    ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, 0, KSZ8863_CHIPID0_REG_ADDR, &(id0.val)), err, TAG, "read ID0 failed");
    ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, 0, KSZ8863_CHIPID1_REG_ADDR, &(id1.val)), err, TAG, "read ID1 failed");
    ESP_GOTO_ON_FALSE(id0.family_id == 0x88 && id1.chip_id == 0x03, ESP_FAIL, err, TAG, "wrong chip ID");
    return ESP_OK;
err:
    return ret;
}

/**
 * @brief
 */
static esp_err_t ksz8863_setup_port_defaults(pmac_ksz8863_t *emac)
{
    esp_err_t ret = ESP_OK;
    esp_eth_mediator_t *eth = emac->eth;

    if (emac->mode == KSZ8863_PORT_MODE) {
        // Filter frames with MAC addresses originating from us (typically broadcast frames "looped" back by other switch)
        ksz8863_pcr5_reg_t pcr5;
        ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, 0, KSZ8863_PCR5_BASE_ADDR + emac->port_reg_offset,
                            &(pcr5.val)), err, TAG, "read Port Control 5 failed");
        pcr5.filter_maca1_en = 1;
        pcr5.filter_maca2_en = 1;
        ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, 0, KSZ8863_PCR5_BASE_ADDR + emac->port_reg_offset,
                            pcr5.val), err, TAG, "write Port Control 5 failed");
    }

    return ESP_OK;
err:
    return ret;
}

static esp_err_t ksz8863_setup_global_defaults(pmac_ksz8863_t *emac)
{
    esp_err_t ret = ESP_OK;
    esp_eth_mediator_t *eth = emac->eth;

    // Initialize Global registers only once
    struct slist_mac_ksz8863_s *pmac_instance;
    bool global_init_done = false;
    SLIST_FOREACH(pmac_instance, &s_mac_ksz8863_head, next) {
        if ((pmac_instance->mac_ksz8863_info->status & KSZ8863_GLOBAL_INIT_DONE) == KSZ8863_GLOBAL_INIT_DONE) {
            global_init_done = true;
            break;
        }
    }
    if (global_init_done != true) {
        emac->status |= KSZ8863_GLOBAL_INIT_DONE;
        // TODO: test if below is true!! It appears like that (base on KSZ status regs.) but should be tested
        // Disable Flow control globally to be able to force it locally on port basis
        ksz8863_gcr1_reg_t gcr1;
        ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, 0, KSZ8863_GCR1_ADDR, &(gcr1.val)), err, TAG, "read GC1 failed");
        gcr1.rx_flow_ctrl_en = 0;
        gcr1.tx_flow_ctrl_en = 0;
        ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, 0, KSZ8863_GCR1_ADDR, gcr1.val), err, TAG, "write GC1 failed");

        // Forward IGMP packets directly to P3 (Host) port
        ksz8863_gcr3_reg_t gcr3;
        ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, 0, KSZ8863_GCR3_ADDR, &(gcr3.val)), err, TAG, "read GC3 failed");
        gcr3.igmp_snoop_en = 1;
        ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, 0, KSZ8863_GCR3_ADDR, gcr3.val), err, TAG, "write GC3 failed");

        if (emac->mode == KSZ8863_PORT_MODE) {
            // Enable forwarding of frames with unknown DA but do NOT specify any port to forward (it can be set later by "set_promiscuous").
            // This ensures that multicast frames are not forwarded directly between P1 and P2 and so these ports act as endpoints. Otherwise,
            // multicast frames could be looped between P1 and P2 and flood the network if P1 and P2 were connected the same switch
            // (or in presence of redundant path in used network).
            ksz8863_gcr12_reg_t gcr12;
            ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, 0, KSZ8863_GCR12_ADDR, &(gcr12.val)), err, TAG, "read GC12 failed");
            gcr12.unknown_da_to_port_en = 1;
            gcr12.unknown_da_to_port = 0;
            ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, 0, KSZ8863_GCR12_ADDR, gcr12.val), err, TAG, "write GC12 failed");

            // Enable Tail tagging
            ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, 0, KSZ8863_GCR1_ADDR, &(gcr1.val)), err, TAG, "read GC1 failed");
            gcr1.tail_tag_en = 1;
            ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, 0, KSZ8863_GCR1_ADDR, gcr1.val), err, TAG, "write GC1 failed");

            // Broadcast needs to be forwared to P3 and so P1/P2 act as endpoints (no traffic exchanged between them directly)
            ksz8863_sta_mac_table_t stat_mac_table;
            memset(stat_mac_table.data, 0, sizeof(stat_mac_table));
            stat_mac_table.fwd_ports = KSZ8863_TO_PORT3;
            stat_mac_table.entry_val = 1;
            memset(stat_mac_table.mac_addr, 0xFF, ETH_ADDR_LEN);
            ksz8863_indirect_write(KSZ8863_STA_MAC_TABLE, 0x0, stat_mac_table.data, sizeof(stat_mac_table));
        }
    }

    return ESP_OK;
err:
    return ret;
}

// TODO: investigate if needed (currently seems it no needed, since it is started automaticaly)
static esp_err_t emac_ksz8863_start(esp_eth_mac_t *mac)
{
    esp_err_t ret = ESP_OK;
    //pmac_ksz8863_t *emac = __containerof(mac, pmac_ksz8863_t, parent);

    return ret;
}

/**
 * @brief stop ksz8863: disable interrupt and stop receive
 */
static esp_err_t emac_ksz8863_stop(esp_eth_mac_t *mac)
{
    esp_err_t ret = ESP_OK;
    //pmac_ksz8863_t *emac = __containerof(mac, pmac_ksz8863_t, parent);

    return ret;
}

static esp_err_t emac_ksz8863_set_mediator(esp_eth_mac_t *mac, esp_eth_mediator_t *eth)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_FALSE(eth, ESP_ERR_INVALID_ARG, err, TAG, "can't set mac's mediator to null");
    pmac_ksz8863_t *emac = __containerof(mac, pmac_ksz8863_t, parent);
    emac->eth = eth;
    return ESP_OK;
err:
    return ret;
}

static esp_err_t emac_ksz8863_set_addr(esp_eth_mac_t *mac, uint8_t *addr)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_FALSE(addr, ESP_ERR_INVALID_ARG, err, TAG, "can't set mac addr to null");
    pmac_ksz8863_t *emac = __containerof(mac, pmac_ksz8863_t, parent);
    esp_eth_mediator_t *eth = emac->eth;

    uint32_t base_reg_addr;
    if (emac->port == 0) {
        base_reg_addr = KSZ8863_MACA1_MSB_ADDR;
    } else {
        base_reg_addr = KSZ8863_MACA2_MSB_ADDR;
    }
    for (int i = 0; i < ETH_ADDR_LEN; i++) {
        // MAC MSB is stored at reg. 147/153, hence is written the first
        eth->phy_reg_write(eth, 0, base_reg_addr - i, addr[i]);
    }
    return ESP_OK;
err:
    return ret;
}

static esp_err_t emac_ksz8863_get_addr(esp_eth_mac_t *mac, uint8_t *addr)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_FALSE(addr, ESP_ERR_INVALID_ARG, err, TAG, "can't copy mac addr to null");
    pmac_ksz8863_t *emac = __containerof(mac, pmac_ksz8863_t, parent);
    esp_eth_mediator_t *eth = emac->eth;

    uint32_t base_reg_addr;
    if (emac->port == 0) {
        base_reg_addr = KSZ8863_MACA1_MSB_ADDR;
    } else {
        base_reg_addr = KSZ8863_MACA2_MSB_ADDR;
    }
    for (int i = 0; i < ETH_ADDR_LEN; i++) {
        eth->phy_reg_read(eth, 0, base_reg_addr - i, (uint32_t *)&addr[i]);
    }

    return ESP_OK;
err:
    return ret;
}

static esp_err_t emac_ksz8863_set_link(esp_eth_mac_t *mac, eth_link_t link)
{
    // Do nothing, KSZ8863 Port 1/2 MAC is started automatically when link is up
    return ESP_OK;
}

static esp_err_t emac_ksz8863_set_speed(esp_eth_mac_t *mac, eth_speed_t speed)
{
    // Do nothing, KSZ8863 Port 1/2 MAC speed is set automatically based on its associated PHY settings
    return ESP_OK;
}

static esp_err_t emac_ksz8863_set_duplex(esp_eth_mac_t *mac, eth_duplex_t duplex)
{
    // Do nothing, KSZ8863 Port 1/2 MAC duplex is set automatically based on its associated PHY settings
    return ESP_OK;
}

static esp_err_t emac_ksz8863_set_promiscuous(esp_eth_mac_t *mac, bool enable)
{
    esp_err_t ret = ESP_OK;
    pmac_ksz8863_t *emac = __containerof(mac, pmac_ksz8863_t, parent);
    esp_eth_mediator_t *eth = emac->eth;

    ESP_GOTO_ON_FALSE(emac->mode == KSZ8863_PORT_MODE, ESP_ERR_INVALID_STATE, err, TAG, "promiscuous is available only in Port Mode");

    // Forward frames with unknown DA to Port 3 ("promiscuous" as such is not mentioned in datasheet)
    ksz8863_gcr12_reg_t gcr12;
    ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, 0, KSZ8863_GCR12_ADDR, &(gcr12.val)), err, TAG, "read GC12 failed");
    if (enable == true) {
        gcr12.unknown_da_to_port_en = 1;
        gcr12.unknown_da_to_port = KSZ8863_TO_PORT3;
    } else {
        gcr12.unknown_da_to_port_en = 1;
        gcr12.unknown_da_to_port = 0;
    }
    ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, 0, KSZ8863_GCR12_ADDR, gcr12.val), err, TAG, "write GC12 failed");

    ESP_LOGW(TAG, "forwarding frames with unknown DA applies for both P1 and P2 ingress ports"); // TODO: consider better formulation
err:
    return ret;
}

static esp_err_t emac_ksz8863_enable_flow_ctrl(esp_eth_mac_t *mac, bool enable)
{
    pmac_ksz8863_t *emac = __containerof(mac, pmac_ksz8863_t, parent);
    emac->flow_ctrl_enabled = enable;
    return ESP_OK;
}

// TODO: it is not possible to enable Pause when autonegotiation is disabled (because PHY does not check for peer's
// ability). This is probably an issue for other EMAC drivers, investigate how it could be fixed.
static esp_err_t emac_ksz8863_set_peer_pause_ability(esp_eth_mac_t *mac, uint32_t ability)
{
    esp_err_t ret = ESP_OK;
    pmac_ksz8863_t *emac = __containerof(mac, pmac_ksz8863_t, parent);
    esp_eth_mediator_t *eth = emac->eth;
    ksz8863_pcr2_reg_t pcr2;

    if (emac->port > KSZ8863_PORT_2) {
        ESP_LOGE(TAG, "flow control configuration is not available for Port 3 at MAC");
        return ESP_ERR_INVALID_ARG;
    }

    // When user wants to enable flow control and peer does support the pause function
    // then configure the MAC layer to enable flow control feature
    if (emac->flow_ctrl_enabled && ability) {
        ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, 0, KSZ8863_PCR2_BASE_ADDR + emac->port_reg_offset, &(pcr2.val)), err, TAG, "read PCR 2 failed");
        pcr2.force_flow_ctrl = 1;
        ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, 0, KSZ8863_PCR2_BASE_ADDR + emac->port_reg_offset, pcr2.val), err, TAG, "write PCR 2 failed");
        ESP_LOGD(TAG, "flow control forced for the link");
    } else {
        ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, 0, KSZ8863_PCR2_BASE_ADDR + emac->port_reg_offset, &(pcr2.val)), err, TAG, "read PCR 2 failed");
        pcr2.force_flow_ctrl = 0;
        ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, 0, KSZ8863_PCR2_BASE_ADDR + emac->port_reg_offset, pcr2.val), err, TAG, "write PCR 2 failed");
        ESP_LOGD(TAG, "flow control disabled for the link");
    }
/*
    printf("en %d, ability %d\n", emac->flow_ctrl_enabled, ability);
    ksz8863_psr1_reg_t psr1;
    ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, 0, KSZ8863_PSR1_BASE_ADDR + emac->port_reg_offset, &(psr1.val)), err, TAG, "read PSR 1 failed");
    printf("P%d, rx %d, tx %d\n", emac->port+1, psr1.rx_flow_ctrl_en, psr1.tx_flow_ctrl_en);
    ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, 0, KSZ8863_PSR1_BASE_ADDR + KSZ8863_PORT3_ADDR_OFFSET, &(psr1.val)), err, TAG, "read PSR 1 failed");
    printf("P3, rx %d, tx %d\n", psr1.rx_flow_ctrl_en, psr1.tx_flow_ctrl_en);*/
    return ESP_OK;
err:
    return ret;
}

static esp_err_t emac_ksz8863_set_mac_tbl(pmac_ksz8863_t *emac, ksz8863_mac_tbl_info_t *tbls_info)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_FALSE(!(emac->mode == KSZ8863_PORT_MODE && tbls_info->start_entry == 0), ESP_ERR_INVALID_STATE, err, TAG,
                        "static MAC tbl entry 0 cannot be changed in Multi-port Mode");
    for (int i = 0; i < tbls_info->etries_num; i++) {
        ESP_GOTO_ON_ERROR(ksz8863_indirect_write(KSZ8863_STA_MAC_TABLE, tbls_info->start_entry + i, &tbls_info->sta_tbls[i],
                            sizeof(ksz8863_sta_mac_table_t)), err, TAG, "failed to write MAC table");
    }
err:
    return ret;
}

static esp_err_t emac_ksz8863_get_mac_tbl(pmac_ksz8863_t *emac, ksz8863_indir_access_tbls_t tbl, ksz8863_mac_tbl_info_t *tbls_info)
{
    esp_err_t ret = ESP_OK;
    for (int i = 0; i < tbls_info->etries_num; i++) {
        ESP_GOTO_ON_ERROR(ksz8863_indirect_read(tbl, tbls_info->start_entry + i,
                            tbl == KSZ8863_STA_MAC_TABLE ? (void *)&tbls_info->sta_tbls[i] : (void *)&tbls_info->dyn_tbls[i],
                            tbl == KSZ8863_STA_MAC_TABLE ? sizeof(ksz8863_sta_mac_table_t) : sizeof(ksz8863_dyn_mac_table_t)),
                            err, TAG, "failed to read MAC table");
    }
err:
    return ret;
}

static esp_err_t emac_ksz8863_custom_ioctl(esp_eth_mac_t *mac, int32_t cmd, void *data)
{
    // TODO:
    // flush static/dynamic MAC
    // Tx/Rx/Learning (PCR2) for spanning tree support
    // get port number (from PMAC infostruct)

    esp_err_t ret = ESP_OK;
    pmac_ksz8863_t *emac = __containerof(mac, pmac_ksz8863_t, parent);
    esp_eth_mediator_t *eth = emac->eth;
    ksz8863_gcr1_reg_t gcr1;

    switch (cmd)
    {
    case KSZ8863_ETH_CMD_S_MAC_STA_TBL:
        ESP_GOTO_ON_FALSE(data, ESP_ERR_INVALID_ARG, err, TAG, "MAC tbl info can't be NULL");
        ESP_GOTO_ON_ERROR(emac_ksz8863_set_mac_tbl(emac, (ksz8863_mac_tbl_info_t *)data), err, TAG, "static MAC table write failed");
        break;
    case KSZ8863_ETH_CMD_G_MAC_STA_TBL:
        ESP_GOTO_ON_FALSE(data, ESP_ERR_INVALID_ARG, err, TAG, "no mem to store static MAC table");
        ESP_GOTO_ON_ERROR(emac_ksz8863_get_mac_tbl(emac, KSZ8863_STA_MAC_TABLE, (ksz8863_mac_tbl_info_t *)data),
                          err, TAG, "static MAC table read failed");
        break;
    case KSZ8863_ETH_CMD_G_MAC_DYN_TBL:
        ESP_GOTO_ON_FALSE(data, ESP_ERR_INVALID_ARG, err, TAG, "no mem to store dynamic MAC table");
        ESP_GOTO_ON_ERROR(emac_ksz8863_get_mac_tbl(emac, KSZ8863_DYN_MAC_TABLE, (ksz8863_mac_tbl_info_t *)data),
                          err, TAG, "dynamic MAC table read failed");
        break;
    case KSZ8863_ETH_CMD_S_TAIL_TAG:
        ESP_GOTO_ON_FALSE(data, ESP_ERR_INVALID_ARG, err, TAG, "can't set tail tag to null");
        ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, 0, KSZ8863_GCR1_ADDR, &(gcr1.val)), err, TAG, "read GC1 failed");
        gcr1.tail_tag_en = *(bool *)data;
        ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, 0, KSZ8863_GCR1_ADDR, gcr1.val), err, TAG, "write GC1 failed");
        break;
    case KSZ8863_ETH_CMD_G_TAIL_TAG:
        ESP_GOTO_ON_FALSE(data, ESP_ERR_INVALID_ARG, err, TAG, "no mem to store tail tag status");
        ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, 0, KSZ8863_GCR1_ADDR, &(gcr1.val)), err, TAG, "read GC1 failed");
        *(bool *)data = gcr1.tail_tag_en;
        break;
    case KSZ8863_ETH_CMD_G_PORT_NUM:
        ESP_GOTO_ON_FALSE(data, ESP_ERR_INVALID_ARG, err, TAG, "no mem to store port number");
        *(int32_t *)data = emac->port;
        break;
    default:
        ret = ESP_ERR_INVALID_ARG;
        break;
    }
err:
    return ret;
}

#include "hal/cpu_hal.h"
static esp_err_t pmac_ksz8863_transmit(esp_eth_mac_t *mac, uint8_t *buf, uint32_t length)
{
    esp_err_t ret = ESP_OK;
    pmac_ksz8863_t *emac = __containerof(mac, pmac_ksz8863_t, parent);

#ifdef TEST_W_MEMCPY
    uint8_t *buff_tail;
    uint32_t cycl1 = cpu_hal_get_cycle_count();
    if (length < ETH_HEADER_LEN + ETH_MIN_PAYLOAD_LEN) {
        length = ETH_HEADER_LEN + ETH_MIN_PAYLOAD_LEN;
    }
    // TODO: this is of course not optimal. The better solution is to allocate more memory in LwIP from the begining of packet processing.
    // Quick and dirty solution might be `p = (struct pbuf *)mem_malloc(alloc_len + 4);` of PBUF_RAM initialization in LwIP's pbuf.c
    // See #else for other solution at MAC layer (this will be preferred since it is transparent from L3 layer)
    buff_tail = malloc(length + 1);
    ESP_GOTO_ON_FALSE(buff_tail, ESP_ERR_NO_MEM, err, TAG, "no memory");
    memcpy(buff_tail, buf, length);
    buff_tail[length] = emac->port + 1;
    //printf("tx len %d, port %d\n", length, emac->port);
    emac->host_mac->transmit(emac->host_mac, buff_tail, length + 1);
    free(buff_tail);
    uint32_t cycl2 = cpu_hal_get_cycle_count();
    printf("tx cycles: %u\n", cycl2 - cycl1);
err:
    return ret;
#else
    //uint32_t cycl1 = cpu_hal_get_cycle_count();
    // ESP32 Ethernet Interface (host) is used to access KSZ8863
    ret = ksz8863_eth_transmit_tag(emac->host_eth_handle, buf, length, emac->port + 1);
    //uint32_t cycl2 = cpu_hal_get_cycle_count();
    //printf("tx cycles: %u\n", cycl2 - cycl1);
    return ret;
#endif
}

static esp_err_t emac_ksz8863_receive(esp_eth_mac_t *mac, uint8_t *buf, uint32_t *length)
{
    esp_err_t ret = ESP_OK;
    //pmac_ksz8863_t *emac = __containerof(mac, pmac_ksz8863_t, parent);
    return ret;
}

static esp_err_t emac_ksz8863_init(esp_eth_mac_t *mac)
{
    esp_err_t ret = ESP_OK;
    pmac_ksz8863_t *emac = __containerof(mac, pmac_ksz8863_t, parent);
    esp_eth_mediator_t *eth = emac->eth;

    ESP_GOTO_ON_ERROR(eth->on_state_changed(eth, ETH_STATE_LLINIT, NULL), err, TAG, "lowlevel init failed");
    /* verify chip id */
    ESP_GOTO_ON_ERROR(ksz8863_verify_id(emac), err, TAG, "vefiry chip ID failed");
    /* default setup of internal registers */
    ESP_GOTO_ON_ERROR(ksz8863_setup_port_defaults(emac), err, TAG, "ksz8863 default port specific setup failed");
    ESP_GOTO_ON_ERROR(ksz8863_setup_global_defaults(emac), err, TAG, "ksz8863 default global setup failed");

    return ESP_OK;
err:
    eth->on_state_changed(eth, ETH_STATE_DEINIT, NULL);
    return ret;
}

static esp_err_t emac_ksz8863_deinit(esp_eth_mac_t *mac)
{
    pmac_ksz8863_t *emac = __containerof(mac, pmac_ksz8863_t, parent);
    esp_eth_mediator_t *eth = emac->eth;
    mac->stop(mac);
    // TODO
    eth->on_state_changed(eth, ETH_STATE_DEINIT, NULL);
    return ESP_OK;
}

static esp_err_t emac_ksz8863_del(esp_eth_mac_t *mac)
{
    pmac_ksz8863_t *emac = __containerof(mac, pmac_ksz8863_t, parent);
    free(emac);

    struct slist_mac_ksz8863_s *pmac_instance;
    SLIST_FOREACH(pmac_instance, &s_mac_ksz8863_head, next) {
        if (pmac_instance->mac_ksz8863_info == emac) {
            SLIST_REMOVE(&s_mac_ksz8863_head, pmac_instance, slist_mac_ksz8863_s, next);
            free(pmac_instance);
        }
    }
    return ESP_OK;
}

esp_eth_mac_t *esp_eth_mac_new_ksz8863(const eth_mac_config_t *mac_config, const ksz8863_eth_mac_config_t *ksz8863_config)
{
    esp_eth_mac_t *ret = NULL;
    pmac_ksz8863_t *emac = NULL;
    //ESP_GOTO_ON_FALSE(ksz8863_config, NULL, err, TAG, "can't set ksz8863 specific config to null");
    ESP_GOTO_ON_FALSE(mac_config, NULL, err, TAG, "can't set mac config to null");
    emac = calloc(1, sizeof(pmac_ksz8863_t));
    ESP_GOTO_ON_FALSE(emac, NULL, err, TAG, "calloc emac failed");

    /* bind methods and attributes */
    emac->sw_reset_timeout_ms = mac_config->sw_reset_timeout_ms;
    emac->parent.set_mediator = emac_ksz8863_set_mediator;
    emac->parent.init = emac_ksz8863_init;
    emac->parent.deinit = emac_ksz8863_deinit;
    emac->parent.start = emac_ksz8863_start;
    emac->parent.stop = emac_ksz8863_stop;
    emac->parent.del = emac_ksz8863_del;
    emac->parent.write_phy_reg = NULL;
    emac->parent.read_phy_reg = NULL;
    emac->parent.set_addr = emac_ksz8863_set_addr;
    emac->parent.get_addr = emac_ksz8863_get_addr;
    emac->parent.set_speed = emac_ksz8863_set_speed;
    emac->parent.set_duplex = emac_ksz8863_set_duplex;
    emac->parent.set_link = emac_ksz8863_set_link;
    emac->parent.set_promiscuous = emac_ksz8863_set_promiscuous;
    emac->parent.set_peer_pause_ability = emac_ksz8863_set_peer_pause_ability;
    emac->parent.enable_flow_ctrl = emac_ksz8863_enable_flow_ctrl;
    emac->parent.custom_ioctl = emac_ksz8863_custom_ioctl;
    emac->parent.transmit = pmac_ksz8863_transmit;
    emac->parent.receive = emac_ksz8863_receive;

    emac->port = ksz8863_config->port_num;
    emac->host_eth_handle = ksz8863_config->host_eth_handle;
    emac->mode = ksz8863_config->pmac_mode;

    if (emac->port == KSZ8863_PORT_1) {
        emac->port_reg_offset = KSZ8863_PORT1_ADDR_OFFSET;
    } else if (emac->port == KSZ8863_PORT_2) {
        emac->port_reg_offset = KSZ8863_PORT2_ADDR_OFFSET;
    }

    struct slist_mac_ksz8863_s *pmac_instance = calloc(1, sizeof(struct slist_mac_ksz8863_s));
    ESP_GOTO_ON_FALSE(pmac_instance, NULL, err, TAG, "calloc pmac_instance failed");
    pmac_instance->mac_ksz8863_info = emac;
    SLIST_INSERT_HEAD(&s_mac_ksz8863_head, pmac_instance, next);

    return &(emac->parent);
err:
    if (emac) {
        free(emac);
    }
    return ret;
}
