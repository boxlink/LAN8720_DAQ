#include "esp_log.h"
#include "esp_check.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "stdio.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include <ads111x.h>
#include "mbcontroller.h"



#define I2C_PORT 0
#define APP_CPU_NUM PRO_CPU_NUM
#define GAIN ADS111X_GAIN_0V256 // +-6.144V


#define SPI_ETHERNETS_NUM           0
#define INTERNAL_ETHERNETS_NUM      1




typedef struct{
    uint16_t sensor_a0;
    uint16_t sensor_a1;
}input_reg_parameters_t;

input_reg_parameters_t input_reg_parameters;
void* slave_ctx = NULL;
static const char *TAG_MB = "MODBUS_SLAVE";


static void setup_modbus_slave(void){
    mb_communication_info_t comm_info = {
            .tcp_opts={
                .mode = MB_TCP,
                .port = 5020,
                .uid = 1,
                .response_tout_ms = 1000,
                .addr_type = MB_IPV4,
                .ip_addr_table = NULL,
                .ip_netif_ptr = NULL,
            }
    };
    ESP_ERROR_CHECK(mbc_slave_create_tcp(&comm_info, &slave_ctx));
    if (slave_ctx == NULL) {
        ESP_LOGE(TAG_MB, "Fallo al crear el controlador Modbus");
        return;
    }
    mb_register_area_descriptor_t reg_area_desc = {
        .type = MB_PARAM_INPUT,
        .start_offset = 0,
        .address= (void*)&input_reg_parameters,
        .size=sizeof(input_reg_parameters_t),
        .access = MB_ACCESS_RO,
    };
    ESP_ERROR_CHECK(mbc_slave_set_descriptor(slave_ctx, reg_area_desc));
    esp_err_t err = mbc_slave_start(slave_ctx);
    ESP_LOGI(TAG_MB, "mbc_slave_start returned: %d", err);
    ESP_LOGI(TAG_MB, "Modbus Slave TCP OK.");
}

static const char *TAG = "example_eth_init";



/**
 * @brief Internal ESP32 Ethernet initialization
 *
 * @param[out] mac_out optionally returns Ethernet MAC object
 * @param[out] phy_out optionally returns Ethernet PHY object
 * @return
 *          - esp_eth_handle_t if init succeeded
 *          - NULL if init failed
 */
static esp_eth_handle_t eth_init_internal(esp_eth_mac_t **mac_out, esp_eth_phy_t **phy_out)
{
    esp_eth_handle_t ret = NULL;

    // Init common MAC and PHY configs to default
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();

    // Update PHY config based on board specific configuration
    phy_config.phy_addr = 0;
    phy_config.reset_gpio_num = -1;
    // Init vendor specific MAC config to default
    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    // Update vendor specific MAC config based on board configuration
    esp32_emac_config.smi_gpio.mdc_num = 23;
    esp32_emac_config.smi_gpio.mdio_num = 18;

    esp32_emac_config.clock_config.rmii.clock_mode = EMAC_CLK_OUT;
    esp32_emac_config.clock_config.rmii.clock_gpio = EMAC_CLK_OUT_180_GPIO;

    // Create new ESP32 Ethernet MAC instance
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
    // Create new PHY instance based on board configuration

    esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_config);

    // Init Ethernet driver to default and install it
    esp_eth_handle_t eth_handle = NULL;
    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    ESP_GOTO_ON_FALSE(esp_eth_driver_install(&config, &eth_handle) == ESP_OK, NULL,
                        err, TAG, "Ethernet driver install failed");

    if (mac_out != NULL) {
        *mac_out = mac;
    }
    if (phy_out != NULL) {
        *phy_out = phy;
    }
    return eth_handle;
err:
    if (eth_handle != NULL) {
        esp_eth_driver_uninstall(eth_handle);
    }
    if (mac != NULL) {
        mac->del(mac);
    }
    if (phy != NULL) {
        phy->del(phy);
    }
    return ret;
}


esp_err_t example_eth_init(esp_eth_handle_t *eth_handles_out[], uint8_t *eth_cnt_out)
{
    esp_err_t ret = ESP_OK;
    esp_eth_handle_t *eth_handles = NULL;
    uint8_t eth_cnt = 0;

    ESP_GOTO_ON_FALSE(eth_handles_out != NULL && eth_cnt_out != NULL, ESP_ERR_INVALID_ARG,
                        err, TAG, "invalid arguments: initialized handles array or number of interfaces");
    eth_handles = calloc(SPI_ETHERNETS_NUM + INTERNAL_ETHERNETS_NUM, sizeof(esp_eth_handle_t));
    ESP_GOTO_ON_FALSE(eth_handles != NULL, ESP_ERR_NO_MEM, err, TAG, "no memory");

    eth_handles[eth_cnt] = eth_init_internal(NULL, NULL);
    ESP_GOTO_ON_FALSE(eth_handles[eth_cnt], ESP_FAIL, err, TAG, "internal Ethernet init failed");
    eth_cnt++;





    *eth_handles_out = eth_handles;
    *eth_cnt_out = eth_cnt;

    return ret;

err:
    free(eth_handles);
    return ret;

}

esp_err_t example_eth_deinit(esp_eth_handle_t *eth_handles, uint8_t eth_cnt)
{
    ESP_RETURN_ON_FALSE(eth_handles != NULL, ESP_ERR_INVALID_ARG, TAG, "array of Ethernet handles cannot be NULL");
    for (int i = 0; i < eth_cnt; i++) {
        esp_eth_mac_t *mac = NULL;
        esp_eth_phy_t *phy = NULL;
        if (eth_handles[i] != NULL) {
            esp_eth_get_mac_instance(eth_handles[i], &mac);
            esp_eth_get_phy_instance(eth_handles[i], &phy);
            ESP_RETURN_ON_ERROR(esp_eth_driver_uninstall(eth_handles[i]), TAG, "Ethernet %p uninstall failed", eth_handles[i]);
        }
        if (mac != NULL) {
            mac->del(mac);
        }
        if (phy != NULL) {
            phy->del(phy);
        }
    }

    free(eth_handles);
    return ESP_OK;
}

static char* TAG1=" ETH EVENTS";

/** Event handler for Ethernet events */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG1, "Ethernet Link Up");
        ESP_LOGI(TAG1, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                    mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG1, "Ethernet Link Down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG1, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG1, "Ethernet Stopped");
        break;
    default:
        break;
    }
}
static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG1, "Ethernet Got IP Address");
    ESP_LOGI(TAG1, "~~~~~~~~~~~");
    ESP_LOGI(TAG1, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG1, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG1, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG1, "~~~~~~~~~~~");
}

//ads1115

static const uint8_t addr[1] =
{
    ADS111X_ADDR_GND,
    ADS111X_ADDR_VCC
};

// Descriptors
static i2c_dev_t devices[1];

// Gain value
static float gain_val;

SemaphoreHandle_t mutex;

static void measure(size_t n, size_t channel)
{
    int16_t raw = 0;
    if (ads111x_get_value(&devices[n], &raw) == ESP_OK)
    {
        if (channel == 0) {
            float voltage = gain_val / ADS111X_MAX_VALUE * raw;
            
            printf("[%u] Raw ADC value: %d, voltage: %.4f volts\n", channel, raw, voltage);
            input_reg_parameters.sensor_a0 = (uint16_t)(voltage * 1000);
        } else if (channel == 1) {
            float voltage = gain_val / ADS111X_MAX_VALUE * raw;
            
            printf("[%u] Raw ADC value: %d, voltage: %.4f volts\n", channel, raw, voltage);
            input_reg_parameters.sensor_a1 = (uint16_t)(voltage * 1000);
        }
    }
    else
        printf("[%u] Cannot read ADC value\n", channel);
}

void AN(void *pvParameters)
{
    gain_val = ads111x_gain_values[GAIN];

    for (size_t i = 0; i < 1; i++)
    {
        ESP_ERROR_CHECK(ads111x_init_desc(&devices[i], addr[i], I2C_PORT, 4, 5));
        ESP_ERROR_CHECK(ads111x_set_mode(&devices[i], ADS111X_MODE_SINGLE_SHOT));
        ESP_ERROR_CHECK(ads111x_set_data_rate(&devices[i], ADS111X_DATA_RATE_32));
        ESP_ERROR_CHECK(ads111x_set_gain(&devices[i], GAIN));
    }

    while (1)
    {
        
            // Canal 0 (AIN0)
            ads111x_set_input_mux(&devices[0], ADS111X_MUX_0_GND);
            ads111x_start_conversion(&devices[0]);
            bool busy = true;
            while (busy) {
                vTaskDelay(pdMS_TO_TICKS(2));
                ads111x_is_busy(&devices[0], &busy);
            }
            measure(0, 0);

            // Canal 1 (AIN3)
            ads111x_set_input_mux(&devices[0], ADS111X_MUX_3_GND);
            ads111x_start_conversion(&devices[0]);
            busy = true;
            while (busy) {
                vTaskDelay(pdMS_TO_TICKS(2));
                ads111x_is_busy(&devices[0], &busy);
            }
            measure(0, 1);

        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    // Initialize Ethernet



    // Initialize Modbus TCP slave
    uint8_t edad = 0;

    // prueba de lectura de registros de entrada antes de iniciar el slave
    
    uint8_t eth_port_cnt = 0;
    esp_eth_handle_t *eth_handles;
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_eth_init(&eth_handles,&eth_port_cnt));

    esp_netif_t *eth_netifs[eth_port_cnt];
    esp_eth_netif_glue_handle_t eth_netif_glues[eth_port_cnt];

    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    eth_netifs[0] = esp_netif_new(&cfg);
    eth_netif_glues[0] = esp_eth_new_netif_glue(eth_handles[0]);
    ESP_ERROR_CHECK(esp_netif_attach(eth_netifs[0], eth_netif_glues[0]));

    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    ESP_ERROR_CHECK(esp_eth_start(eth_handles[0]));
    mutex = xSemaphoreCreateMutex();
    if (mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }
    setup_modbus_slave();
    // Init library
    ESP_ERROR_CHECK(i2cdev_init());

    // Clear device descriptors
    memset(devices, 0, sizeof(devices));
   
    // Start task
    xTaskCreatePinnedToCore(AN, "ads111x_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    
    
}