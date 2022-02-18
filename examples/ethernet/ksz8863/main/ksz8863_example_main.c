/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* Ethernet Basic Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_check.h"

#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"

// needed for L2 TAP VFS
#include <stdio.h>
#include <unistd.h> // read/write
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include "esp_vfs_l2tap.h"
#include "lwip/prot/ethernet.h" // Ethernet headers
#include "errno.h"
#include "arpa/inet.h" // for ntohs, etc.

#include "esp_eth_ksz8863.h"
#include "esp_eth_netif_glue_ksz8863.h"

static const char *TAG = "ksz8863_eth_example";

/** Event handler for Ethernet events */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    int32_t port_num;
    esp_err_t ret;
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ret = esp_eth_ioctl(eth_handle, KSZ8863_ETH_CMD_G_PORT_NUM, &port_num);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Ethernet Link Up Port %d", port_num + 1);
        } else {
            ESP_LOGI(TAG, "Ethernet Link Up");
        }
        ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ret = esp_eth_ioctl(eth_handle, KSZ8863_ETH_CMD_G_PORT_NUM, &port_num);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Ethernet Link Down Port %d", port_num + 1);
        } else {
            ESP_LOGI(TAG, "Ethernet Link Down");
        }
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG, "~~~~~~~~~~~");
}


typedef struct {
    struct eth_hdr header;
    union {
        int cnt;
        char str[44];
    };
} test_vfs_eth_tap_msg_t;

// Global test message send by "send_task"
static test_vfs_eth_tap_msg_t s_test_msg = {
    .header = {
        .src.addr = {0x8c, 0x4b, 0x14, 0x0a, 0x14, 0x63},
        //.dest.addr = { 0x01, 0x00, 0x00, 0x00, 0xBE, 0xEF },
        .dest.addr = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
        .type = 0x7000,
    },
    .str = "This is ESP32 L2 TAP test msg from Port: "
};

esp_err_t i2c_init(i2c_port_t i2c_master_port, i2c_config_t *i2c_conf)
{
    esp_err_t ret;

    ESP_GOTO_ON_ERROR(i2c_param_config(i2c_master_port, i2c_conf), err, TAG, "I2C parameters configuration failed");
    ESP_GOTO_ON_ERROR(i2c_driver_install(i2c_master_port, i2c_conf->mode, 0, 0, 0), err, TAG, "I2C driver install failed");

    return ESP_OK;
err:
    return ret;
}

// board specific initialization routine, user to update per specific needs
esp_err_t ksz8863_board_specific_init(esp_eth_handle_t eth_handle)
{
    esp_err_t ret = ESP_OK;

#if CONFIG_EXAMPLE_CTRL_I2C
    // initialize I2C interface
    i2c_config_t i2c_bus_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CONFIG_EXAMPLE_I2C_SDA_GPIO,
        .scl_io_num = CONFIG_EXAMPLE_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = CONFIG_EXAMPLE_I2C_CLOCK_KHZ * 1000,
    };
    ESP_GOTO_ON_ERROR(i2c_init(CONFIG_EXAMPLE_I2C_MASTER_PORT, &i2c_bus_config), err, TAG, "I2C initialization failed");
    ksz8863_ctrl_i2c_config_t i2c_dev_config = {
        .dev_addr = KSZ8863_I2C_DEV_ADDR,
        .i2c_master_port = CONFIG_EXAMPLE_I2C_MASTER_PORT,
    };
    ksz8863_ctrl_intf_config_t ctrl_intf_cfg = {
        .host_mode = KSZ8863_I2C_MODE,
        .i2c_dev_config = &i2c_dev_config,
    };
#elif CONFIG_EXAMPLE_CTRL_SPI
    spi_bus_config_t buscfg = {
        .miso_io_num = CONFIG_EXAMPLE_ETH_SPI_MISO_GPIO,
        .mosi_io_num = CONFIG_EXAMPLE_ETH_SPI_MOSI_GPIO,
        .sclk_io_num = CONFIG_EXAMPLE_ETH_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(CONFIG_EXAMPLE_ETH_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ksz8863_ctrl_spi_config_t spi_dev_config = {
        .host_id = CONFIG_EXAMPLE_ETH_SPI_HOST,
        .clock_speed_hz = CONFIG_EXAMPLE_ETH_SPI_CLOCK_MHZ * 1000 * 1000,
        .spics_io_num = CONFIG_EXAMPLE_ETH_SPI_CS_GPIO,
    };
    ksz8863_ctrl_intf_config_t ctrl_intf_cfg = {
        .host_mode = KSZ8863_SPI_MODE,
        .spi_dev_config = &spi_dev_config,
    };
#endif
    ESP_GOTO_ON_ERROR(ksz8863_ctrl_intf_init(&ctrl_intf_cfg), err, TAG, "KSZ8863 control interface initialization failed");

#ifdef CONFIG_EXAMPLE_EXTERNAL_CLK_EN
    // Enable KSZ's external CLK
    esp_rom_gpio_pad_select_gpio(CONFIG_EXAMPLE_EXTERNAL_CLK_EN_GPIO);
    gpio_set_direction(CONFIG_EXAMPLE_EXTERNAL_CLK_EN_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_EXAMPLE_EXTERNAL_CLK_EN_GPIO, 1);
#endif

    ESP_GOTO_ON_ERROR(ksz8863_hw_reset(CONFIG_EXAMPLE_KSZ8863_RST_GPIO), err, TAG, "hardware reset failed");
    // it does not make much sense to execute SW reset right after HW reset but it is present here for demonstration purposes
    ESP_GOTO_ON_ERROR(ksz8863_sw_reset(eth_handle), err, TAG, "software reset failed");
#if CONFIG_EXAMPLE_P3_RMII_CLKI_INTERNAL
    ESP_GOTO_ON_ERROR(ksz8863_p3_rmii_internal_clk(eth_handle, true), err, TAG, "P3 internal clk config failed");
#endif

#if CONFIG_EXAMPLE_P3_RMII_CLKI_INVERT // TODO: add to Kconfig
    ESP_GOTO_ON_ERROR(ksz8863_p3_rmii_clk_invert(eth_handle, true), err, TAG, "P3 invert ckl failed");
#endif
err:
    return ret;
}

//TODO:
extern esp_eth_netif_glue_handle_t esp_eth_new_netif_glue_br(esp_eth_handle_t eth_hdl);

void app_main(void)
{
    // Initialize TCP/IP network interface (should be called only once in application)
    ESP_ERROR_CHECK(esp_netif_init());
    // Create default event loop that running in background
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Init MAC and PHY configs to default
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();

    phy_config.reset_gpio_num = -1; // KSZ8863 is reset by separate function call since multiple instances exist
    mac_config.smi_mdc_gpio_num = -1; // MIIM interface is not used since does not provide access to all registers
    mac_config.smi_mdio_gpio_num = -1;

#if CONFIG_EXAMPLE_PORT_MODE
    ESP_LOGW(TAG, "runs in Two Port endpoints mode...\n");

    // Init Host Ethernet Interface (Port 3)
    esp_eth_mac_t *host_mac = esp_eth_mac_new_esp32(&mac_config);
    phy_config.phy_addr = -1; // this PHY is entry point to host
    esp_eth_phy_t *host_phy = esp_eth_phy_new_ksz8863(&phy_config);

    esp_eth_config_t host_config = ETH_KSZ8863_DEFAULT_CONFIG(host_mac, host_phy);
    host_config.on_lowlevel_init_done = ksz8863_board_specific_init;
    esp_eth_handle_t host_eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&host_config, &host_eth_handle));

    // Init P1 Ethernet Interface
    ksz8863_eth_mac_config_t ksz8863_pmac_config = {
        .host_eth_handle = host_eth_handle,
        .pmac_mode = KSZ8863_PORT_MODE,
        .port_num = KSZ8863_PORT_1,
    };
    esp_eth_mac_t *p1_mac = esp_eth_mac_new_ksz8863(&mac_config, &ksz8863_pmac_config);
    phy_config.phy_addr = 0; // this PHY is Port 1
    esp_eth_phy_t *p1_phy = esp_eth_phy_new_ksz8863(&phy_config);

    esp_eth_config_t p1_config = ETH_KSZ8863_DEFAULT_CONFIG(p1_mac, p1_phy);
    esp_eth_handle_t p1_eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&p1_config, &p1_eth_handle));

    // Init P2 Ethernet Interface
    ksz8863_pmac_config.port_num = KSZ8863_PORT_2;
    esp_eth_mac_t *p2_mac = esp_eth_mac_new_ksz8863(&mac_config, &ksz8863_pmac_config);
    phy_config.phy_addr = 1; // this PHY is Port 2
    esp_eth_phy_t *p2_phy = esp_eth_phy_new_ksz8863(&phy_config);

    esp_eth_config_t p2_config = ETH_KSZ8863_DEFAULT_CONFIG(p2_mac, p2_phy);
    esp_eth_handle_t p2_eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&p2_config, &p2_eth_handle));

    // KSZ8863 Ports 1/2 does not have any default MAC
    ESP_ERROR_CHECK(esp_eth_ioctl(p1_eth_handle, ETH_CMD_S_MAC_ADDR, (uint8_t[]) {
            0x8c, 0x4b, 0x14, 0x0a, 0x14, 0x00
    }));
    ESP_ERROR_CHECK(esp_eth_ioctl(p2_eth_handle, ETH_CMD_S_MAC_ADDR, (uint8_t[]) {
            0x8c, 0x4b, 0x14, 0x0a, 0x14, 0x01
    }));

    bool enable = true;
    esp_eth_ioctl(p1_eth_handle, ETH_CMD_S_AUTONEGO, &enable); // specific to our board (boot strap issue on GPIO0)

    // Internal EMAC needs to receive frames from other KSZ8863 ports => do not perform any filterring
    esp_eth_ioctl(host_eth_handle, ETH_CMD_S_PROMISCUOUS, &enable);

    ksz8863_register_tail_tag_port(p1_eth_handle, 0);
    ksz8863_register_tail_tag_port(p2_eth_handle, 1);
    // Make "host eth" decide where to forward traffic
    esp_eth_update_input_path(host_eth_handle, ksz8863_tail_tag_port_forward, NULL);

    // Create instance(s) of esp-netif for Port1 & Port 2 Ethernets
    esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_ETH();
    esp_netif_config_t netif_cfg = {
        .base = &esp_netif_config,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH
    };
    esp_netif_t *eth_netif_port[2] = { NULL };
    char if_key_str[10];
    char if_desc_str[10];
    char num_str[3];
    for (int i = 0; i < 2; i++) {
        itoa(i, num_str, 10);
        strcat(strcpy(if_key_str, "ETH_"), num_str);
        strcat(strcpy(if_desc_str, "eth"), num_str);
        esp_netif_config.if_key = if_key_str;
        esp_netif_config.if_desc = if_desc_str;
        esp_netif_config.route_prio = 30 - i;
        eth_netif_port[i] = esp_netif_new(&netif_cfg);
    }
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif_port[0], esp_eth_new_netif_glue(p1_eth_handle)));
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif_port[1], esp_eth_new_netif_glue(p2_eth_handle)));

    // Register user defined event handers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    // start Ethernet driver state machines
    ESP_ERROR_CHECK(esp_eth_start(host_eth_handle));
    ESP_ERROR_CHECK(esp_eth_start(p1_eth_handle));
    ESP_ERROR_CHECK(esp_eth_start(p2_eth_handle));
#elif CONFIG_EXAMPLE_SWITCH_SIMPLE
// TODO add description
// very efficient & simple, no tail tagging
    ESP_LOGW(TAG, "runs in Simple Switch mode...\n");

    // Init Host Ethernet Interface (Port 3)
    esp_eth_mac_t *host_mac = esp_eth_mac_new_esp32(&mac_config);
    phy_config.phy_addr = -1; // this PHY is entry point to host
    esp_eth_phy_t *host_phy = esp_eth_phy_new_ksz8863(&phy_config);

    esp_eth_config_t host_config = ETH_KSZ8863_DEFAULT_CONFIG(host_mac, host_phy);
    host_config.on_lowlevel_init_done = ksz8863_board_specific_init;
    esp_eth_handle_t host_eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&host_config, &host_eth_handle));

    // Init P1 Ethernet Interface
    ksz8863_eth_mac_config_t ksz8863_pmac_config = {
        .host_eth_handle = host_eth_handle,
        .pmac_mode = KSZ8863_SWITCH_MODE,
        .port_num = KSZ8863_PORT_1,
    };
    esp_eth_mac_t *p1_mac = esp_eth_mac_new_ksz8863(&mac_config, &ksz8863_pmac_config);
    phy_config.phy_addr = 0; // this PHY is Port 1
    esp_eth_phy_t *p1_phy = esp_eth_phy_new_ksz8863(&phy_config);

    esp_eth_config_t p1_config = ETH_KSZ8863_DEFAULT_CONFIG(p1_mac, p1_phy);
    esp_eth_handle_t p1_eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&p1_config, &p1_eth_handle));

    // Init P2 Ethernet Interface
    ksz8863_pmac_config.port_num = KSZ8863_PORT_2;
    esp_eth_mac_t *p2_mac = esp_eth_mac_new_ksz8863(&mac_config, &ksz8863_pmac_config);
    phy_config.phy_addr = 1; // this PHY is Port 2
    esp_eth_phy_t *p2_phy = esp_eth_phy_new_ksz8863(&phy_config);

    esp_eth_config_t p2_config = ETH_KSZ8863_DEFAULT_CONFIG(p2_mac, p2_phy);
    esp_eth_handle_t p2_eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&p2_config, &p2_eth_handle));

    // Create new default instance of esp-netif for Host Ethernet Port (P3)
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(host_eth_handle)));

    // Register user defined event handers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    bool enable = true;
    esp_eth_ioctl(p1_eth_handle, ETH_CMD_S_AUTONEGO, &enable); // specific to our board (boot strap issue on GPIO0)

    // start Ethernet driver state machines
    ESP_ERROR_CHECK(esp_eth_start(host_eth_handle));
    ESP_ERROR_CHECK(esp_eth_start(p1_eth_handle)); // p1/2_eth_handle are going to be used basically only for Link Status indication and for configuration access
    ESP_ERROR_CHECK(esp_eth_start(p2_eth_handle));
#elif CONFIG_EXAMPLE_SWITCH_BRIDGE
// TODO: description
// problems with broadcast/multicast, lwip br is not efficient
    ESP_LOGW(TAG, "runs in Simple lwip bridge mode...\n");

    // Init Host Ethernet Interface (Port 3)
    esp_eth_mac_t *host_mac = esp_eth_mac_new_esp32(&mac_config);
    phy_config.phy_addr = -1; // this PHY is entry point to host
    esp_eth_phy_t *host_phy = esp_eth_phy_new_ksz8863(&phy_config);

    esp_eth_config_t host_config = ETH_KSZ8863_DEFAULT_CONFIG(host_mac, host_phy);
    host_config.on_lowlevel_init_done = ksz8863_board_specific_init;
    esp_eth_handle_t host_eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&host_config, &host_eth_handle));

    // Init P1 Ethernet Interface
    ksz8863_eth_mac_config_t ksz8863_pmac_config = {
        .host_eth_handle = host_eth_handle,
        .pmac_mode = KSZ8863_SWITCH_MODE,
        .port_num = KSZ8863_PORT_1,
    };
    esp_eth_mac_t *p1_mac = esp_eth_mac_new_ksz8863(&mac_config, &ksz8863_pmac_config);
    phy_config.phy_addr = 0; // this PHY is Port 1
    esp_eth_phy_t *p1_phy = esp_eth_phy_new_ksz8863(&phy_config);

    esp_eth_config_t p1_config = ETH_KSZ8863_DEFAULT_CONFIG(p1_mac, p1_phy);
    esp_eth_handle_t p1_eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&p1_config, &p1_eth_handle));

    // Init P2 Ethernet Interface
    ksz8863_pmac_config.port_num = KSZ8863_PORT_2;
    esp_eth_mac_t *p2_mac = esp_eth_mac_new_ksz8863(&mac_config, &ksz8863_pmac_config);
    phy_config.phy_addr = 1; // this PHY is Port 2
    esp_eth_phy_t *p2_phy = esp_eth_phy_new_ksz8863(&phy_config);

    esp_eth_config_t p2_config = ETH_KSZ8863_DEFAULT_CONFIG(p2_mac, p2_phy);
    esp_eth_handle_t p2_eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&p2_config, &p2_eth_handle));

    // KSZ8863 Ports 1/2 does not have any default MAC
    ESP_ERROR_CHECK(esp_eth_ioctl(p1_eth_handle, ETH_CMD_S_MAC_ADDR, (uint8_t[]) {
            0x8c, 0x4b, 0x14, 0x0a, 0x14, 0x00
    }));
    ESP_ERROR_CHECK(esp_eth_ioctl(p2_eth_handle, ETH_CMD_S_MAC_ADDR, (uint8_t[]) {
            0x8c, 0x4b, 0x14, 0x0a, 0x14, 0x01
    }));

    // Create instance(s) of esp-netif for Port1 & Port 2 Ethernets
    esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_ETH();
    esp_netif_config_t netif_cfg = {
        .base = &esp_netif_config,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH
    };
    esp_netif_t *eth_netif_port[2] = { NULL };
    char if_key_str[10];
    char if_desc_str[10];
    char num_str[3];
    for (int i = 0; i < 2; i++) {
        itoa(i, num_str, 10);
        strcat(strcpy(if_key_str, "ETH_"), num_str);
        strcat(strcpy(if_desc_str, "eth"), num_str);
        esp_netif_config.flags = ESP_NETIF_FLAG_PORT_BE_BRIDGED;
        esp_netif_config.if_key = if_key_str;
        esp_netif_config.if_desc = if_desc_str;
        esp_netif_config.route_prio = 30 - i;
        eth_netif_port[i] = esp_netif_new(&netif_cfg);
    }
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif_port[0], esp_eth_new_netif_glue(p1_eth_handle)));
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif_port[1], esp_eth_new_netif_glue(p2_eth_handle)));

    // Create instance of esp-netif for bridge interface
    esp_netif_inherent_config_t esp_netif_br_config = ESP_NETIF_INHERENT_DEFAULT_ETH();
    esp_netif_config_t netif_br_cfg = {
        .base = &esp_netif_br_config,
        .stack = ESP_NETIF_NETSTACK_BR_DEFAULT_ETH
    };
    esp_netif_br_config.flags |= ESP_NETIF_FLAG_BRIDGE;
    esp_netif_br_config.if_key = "BR_0";
    esp_netif_br_config.if_desc = "br0";
    esp_netif_t *br_netif = esp_netif_new(&netif_br_cfg);

    /*esp_netif_dhcpc_stop(br_netif);
    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 192, 168, 20, 105);
   	IP4_ADDR(&ip_info.gw, 192, 168, 20, 1);
   	IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
    esp_netif_set_ip_info(br_netif, &ip_info);
    */
    ESP_ERROR_CHECK(esp_netif_attach(br_netif, esp_eth_new_netif_glue_br(host_eth_handle)));

    // Make "host eth" decide where to forward traffic
    ksz8863_register_tail_tag_port(p1_eth_handle, 0);
    ksz8863_register_tail_tag_port(p2_eth_handle, 1);
    esp_eth_update_input_path(host_eth_handle, ksz8863_tail_tag_port_forward, NULL);

    // Register user defined event handers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    bool enable = true;
    esp_eth_ioctl(p1_eth_handle, ETH_CMD_S_AUTONEGO, &enable); // specific to our board (boot strap issue on GPIO0)

    esp_eth_ioctl(p1_eth_handle, KSZ8863_ETH_CMD_S_TAIL_TAG, &enable);

    // start Ethernet driver state machines
    ESP_ERROR_CHECK(esp_eth_start(host_eth_handle));
    ESP_ERROR_CHECK(esp_eth_start(p1_eth_handle));
    ESP_ERROR_CHECK(esp_eth_start(p2_eth_handle));

    // add KSZ8863 ports to the bridge
    esp_netif_add_port_bridge(br_netif, eth_netif_port[0]);
    esp_netif_add_port_bridge(br_netif, eth_netif_port[1]);

#elif CONFIG_EXAMPLE_SWITCH_TAIL_TAG
    ESP_LOGW(TAG, "runs in Switch with Tail Tagging mode...\n");

    // Init Host Ethernet Interface (Port 3)
    esp_eth_mac_t *host_mac = esp_eth_mac_new_esp32(&mac_config);
    phy_config.phy_addr = -1; // this PHY is entry point to host
    esp_eth_phy_t *host_phy = esp_eth_phy_new_ksz8863(&phy_config);

    esp_eth_config_t host_config = ETH_KSZ8863_DEFAULT_CONFIG(host_mac, host_phy);
    host_config.on_lowlevel_init_done = ksz8863_board_specific_init;
    esp_eth_handle_t host_eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&host_config, &host_eth_handle));

    // Init P1 Ethernet Interface
    ksz8863_eth_mac_config_t ksz8863_pmac_config = {
        .host_eth_handle = host_eth_handle,
        .pmac_mode = KSZ8863_SWITCH_MODE,
        .port_num = KSZ8863_PORT_1,
    };
    esp_eth_mac_t *p1_mac = esp_eth_mac_new_ksz8863(&mac_config, &ksz8863_pmac_config);
    phy_config.phy_addr = 0; // this PHY is Port 1
    esp_eth_phy_t *p1_phy = esp_eth_phy_new_ksz8863(&phy_config);

    esp_eth_config_t p1_config = ETH_KSZ8863_DEFAULT_CONFIG(p1_mac, p1_phy);
    esp_eth_handle_t p1_eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&p1_config, &p1_eth_handle));

    // Init P2 Ethernet Interface
    ksz8863_pmac_config.port_num = KSZ8863_PORT_2;
    esp_eth_mac_t *p2_mac = esp_eth_mac_new_ksz8863(&mac_config, &ksz8863_pmac_config);
    phy_config.phy_addr = 1; // this PHY is Port 2
    esp_eth_phy_t *p2_phy = esp_eth_phy_new_ksz8863(&phy_config);

    esp_eth_config_t p2_config = ETH_KSZ8863_DEFAULT_CONFIG(p2_mac, p2_phy);
    esp_eth_handle_t p2_eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&p2_config, &p2_eth_handle));

    bool enable = true;
    esp_eth_ioctl(p1_eth_handle, ETH_CMD_S_AUTONEGO, &enable); // specific to our board (boot strap issue on GPIO0)

    esp_eth_ioctl(p1_eth_handle, KSZ8863_ETH_CMD_S_TAIL_TAG, &enable);

    // KSZ8863 Ports 1/2 does not have any default MAC and not necessary need any TODO
    /*ESP_ERROR_CHECK(esp_eth_ioctl(p1_eth_handle, ETH_CMD_S_MAC_ADDR, (uint8_t[]) {
            0x8c, 0x4b, 0x14, 0x0a, 0x14, 0x00
    }));
    ESP_ERROR_CHECK(esp_eth_ioctl(p2_eth_handle, ETH_CMD_S_MAC_ADDR, (uint8_t[]) {
            0x8c, 0x4b, 0x14, 0x0a, 0x14, 0x01
    }));*/

    // Create instance(s) of esp-netif for Port1 & Port 2 Ethernets without association to IP stack.
    // This is done to be able to access the Port 1/2 interfaces using L2 TAP and so make KSZ Tail Tagging available for user.
    // Note that normal address lookup is used for IP traffic (associated with default eth_netif)
    esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_ETH();
    esp_netif_config_t netif_cfg = {
        .base = &esp_netif_config,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH // TODO: for this type of netif, it will not be used but now it is needed for init
    };
    esp_netif_t *eth_netif_port[2] = { NULL };
    char if_key_str[10];
    char if_desc_str[10];
    char num_str[3];
    for (int i = 0; i < 3; i++) {
        itoa(i, num_str, 10);
        strcat(strcpy(if_key_str, "ETH_"), num_str);
        strcat(strcpy(if_desc_str, "eth"), num_str);
        esp_netif_config.flags = 0;
        esp_netif_config.if_key = if_key_str;
        esp_netif_config.if_desc = if_desc_str;
        esp_netif_config.route_prio = 30 - i;
        eth_netif_port[i] = esp_netif_new(&netif_cfg);
    }
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif_port[0], esp_eth_new_netif_glue(p1_eth_handle)));
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif_port[1], esp_eth_new_netif_glue(p2_eth_handle)));

    // Create new default instance of esp-netif which is associated with IP stack
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif,
                                     ksz8863_esp_eth_new_netif_glue_switch(
                                                    host_eth_handle,
                                                    p1_eth_handle,
                                                    eth_netif_port[0],
                                                    p2_eth_handle,
                                                    eth_netif_port[1])));

    // Register user defined event handers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    // start Ethernet driver state machines
    ESP_ERROR_CHECK(esp_eth_start(host_eth_handle));
    ESP_ERROR_CHECK(esp_eth_start(p1_eth_handle));
    ESP_ERROR_CHECK(esp_eth_start(p2_eth_handle));
#endif

    // TODO: For Switch with TT. When Host Ethernet interface is stopped, P1/P2 needs to be stopped and started too to work properly again. It does not
    // apply when P1 or P2 is stopped though (no need to restart Host Eth). This needs to be either resolved or documented.
    /*vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_ERROR_CHECK(esp_eth_stop(host_eth_handle));
    ESP_ERROR_CHECK(esp_eth_start(host_eth_handle));
    //ESP_ERROR_CHECK(esp_eth_stop(p2_eth_handle));
    //ESP_ERROR_CHECK(esp_eth_start(p2_eth_handle)); */

#ifdef TEST_MAC_ACCESS
    ksz8863_dyn_mac_table_t dyn_mac_tbls[5];
    ksz8863_mac_tbl_info_t get_tbl_info ={
        .start_entry = 0,
        .etries_num = 5,
        .dyn_tbls = dyn_mac_tbls,
    };
    ksz8863_sta_mac_table_t sta_mac_tbls[2];
    ksz8863_mac_tbl_info_t get_sta_tbl_info ={
        .start_entry = 0,
        .etries_num = 2,
        .sta_tbls = sta_mac_tbls,
    };

    ksz8863_sta_mac_table_t sta_mac_tbl;
    ksz8863_mac_tbl_info_t set_sta_tbl_info ={
        .start_entry = 1,
        .etries_num = 1,
        .sta_tbls = &sta_mac_tbl,
    };
    sta_mac_tbl.fwd_ports = 1 << 2;
    sta_mac_tbl.entry_val = 1;
    memset(sta_mac_tbl.mac_addr, 0xff, 6);
    sta_mac_tbl.mac_addr[0] = 0x01;
    esp_eth_ioctl(p1_eth_handle, KSZ8863_ETH_CMD_S_MAC_STA_TBL, &set_sta_tbl_info);

    while (1) {
        esp_eth_ioctl(p1_eth_handle, KSZ8863_ETH_CMD_G_MAC_DYN_TBL, &get_tbl_info);
        ESP_LOGI(TAG, "valid entries %d", dyn_mac_tbls[0].val_entries + 1);
        for (int i = 0; i < (dyn_mac_tbls[0].val_entries + 1) && i < 5; i++) {
            ESP_LOGI(TAG, "port %d", dyn_mac_tbls[i].src_port + 1);
            ESP_LOG_BUFFER_HEX(TAG, dyn_mac_tbls[i].mac_addr, 6);
        }
        printf("\n");

        ESP_LOGI(TAG, "static MAC");
        esp_eth_ioctl(p1_eth_handle, KSZ8863_ETH_CMD_G_MAC_STA_TBL, &get_sta_tbl_info);
        ESP_LOGI(TAG, "fwd port %d", sta_mac_tbls[0].fwd_ports);
        ESP_LOG_BUFFER_HEX(TAG, sta_mac_tbls[0].mac_addr, 6);

        ESP_LOGI(TAG, "fwd port %d", sta_mac_tbls[1].fwd_ports);
        ESP_LOG_BUFFER_HEX(TAG, sta_mac_tbls[1].mac_addr, 6);
        printf("\n");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
#endif

#ifdef TEST_L2_TAIL_TAG
    esp_vfs_l2tap_intf_register(NULL);
    int ret;
    int eth_tap_fd_p1 = open("/dev/net/tap", O_NONBLOCK);
    int eth_tap_fd_p2 = open("/dev/net/tap", O_NONBLOCK);
    uint16_t eth_type_filter = 0x7000;
    if ((ret = ioctl(eth_tap_fd_p1, L2TAP_S_RCV_FILTER, &eth_type_filter)) == -1) {
        ESP_LOGE(TAG, "filter1 error %d", errno);
    }
    if ((ret = ioctl(eth_tap_fd_p2, L2TAP_S_RCV_FILTER, &eth_type_filter)) == -1) {
        ESP_LOGE(TAG, "filter2 error %d", errno);
    }
    // Set Ethernet interface on which to get raw frames
    if ((ret = ioctl(eth_tap_fd_p1, L2TAP_S_INTF_DEVICE, "ETH_0")) == -1) {
        ESP_LOGE(TAG, "dev set1 error %d", errno);
    }
    if ((ret = ioctl(eth_tap_fd_p2, L2TAP_S_INTF_DEVICE, "ETH_1")) == -1) {
        ESP_LOGE(TAG, "dev set2 error %d", errno);
    }

    uint8_t dev_mac_addr[6];
    if ((ret = esp_eth_ioctl(p1_eth_handle, ETH_CMD_G_MAC_ADDR, dev_mac_addr)) == -1) {
        ESP_LOGE(TAG, "get MAC addr error");
    }
    s_test_msg.header.type = htons(eth_type_filter);
    memcpy(s_test_msg.header.src.addr, dev_mac_addr, 6);
    int len = strlen(s_test_msg.str);
    while (1) {
        s_test_msg.str[len] = '1';
        ret = write(eth_tap_fd_p1, &s_test_msg, sizeof(s_test_msg));
        if (ret == -1) {
            printf("P1 tap errno: %d\n", errno);
        }
        s_test_msg.str[len] = '2';
        ret = write(eth_tap_fd_p2, &s_test_msg, sizeof(s_test_msg));
        if (ret == -1) {
            printf("P2 tap errno: %d\n", errno);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
#endif
    //i2c_driver_delete(I2C_MASTER_PORT);
}
