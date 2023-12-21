/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* PPPoS Client Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_netif.h"
#include "esp_netif_ppp.h"
#include "mqtt_client.h"
#include "esp_modem_api.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "driver/gpio.h"

#define MODEM_RST 5
#define MODEM_PWKEY 4
#define MODEM_DTR 25
#define MODEM_TX 27
#define MODEM_RX 26

static const char *TAG = "pppos_example";
static EventGroupHandle_t event_group = NULL;
static const int CONNECT_BIT = BIT0;
static const int GOT_DATA_BIT = BIT2;
static const int USB_DISCONNECTED_BIT = BIT3; // Used only with USB DTE but we define it unconditionally, to avoid too many #ifdefs in the code
#define CHECK_USB_DISCONNECTION(event_group)

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIu32, base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id = 0;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_subscribe(client, CONFIG_EXAMPLE_MQTT_TEST_TOPIC, 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, CONFIG_EXAMPLE_MQTT_TEST_TOPIC, "Hello world", 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        xEventGroupSetBits(event_group, GOT_DATA_BIT);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        break;
    default:
        ESP_LOGI(TAG, "MQTT other event id: %d", event->event_id);
        break;
    }
}

static void on_ppp_changed(void *arg, esp_event_base_t event_base,
                           int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "PPP state changed event %" PRIu32, event_id);
    if (event_id == NETIF_PPP_ERRORUSER) {
        /* User interrupted event from esp-netif */
        esp_netif_t **p_netif = event_data;
        ESP_LOGI(TAG, "User interrupted event from netif:%p", *p_netif);
    }
}


static void on_ip_event(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "IP event! %" PRIu32, event_id);
    if (event_id == IP_EVENT_PPP_GOT_IP) {
        esp_netif_dns_info_t dns_info;

        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        esp_netif_t *netif = event->esp_netif;

        ESP_LOGI(TAG, "Modem Connect to PPP Server");
        ESP_LOGI(TAG, "~~~~~~~~~~~~~~");
        ESP_LOGI(TAG, "IP          : " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Netmask     : " IPSTR, IP2STR(&event->ip_info.netmask));
        ESP_LOGI(TAG, "Gateway     : " IPSTR, IP2STR(&event->ip_info.gw));
        esp_netif_get_dns_info(netif, 0, &dns_info);
        ESP_LOGI(TAG, "Name Server1: " IPSTR, IP2STR(&dns_info.ip.u_addr.ip4));
        esp_netif_get_dns_info(netif, 1, &dns_info);
        ESP_LOGI(TAG, "Name Server2: " IPSTR, IP2STR(&dns_info.ip.u_addr.ip4));
        ESP_LOGI(TAG, "~~~~~~~~~~~~~~");
        xEventGroupSetBits(event_group, CONNECT_BIT);

        ESP_LOGI(TAG, "GOT ip event!!!");
    } else if (event_id == IP_EVENT_PPP_LOST_IP) {
        ESP_LOGI(TAG, "Modem Disconnect from PPP Server");
    } else if (event_id == IP_EVENT_GOT_IP6) {
        ESP_LOGI(TAG, "GOT IPv6 event!");

        ip_event_got_ip6_t *event = (ip_event_got_ip6_t *)event_data;
        ESP_LOGI(TAG, "Got IPv6 address " IPV6STR, IPV62STR(event->ip6_info.ip));
    }
}

void Modem_Start(){
    // Set-up modem  power pin
    gpio_set_direction(MODEM_PWKEY, GPIO_MODE_OUTPUT);
    gpio_set_level(MODEM_PWKEY, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(MODEM_PWKEY, 1);
    // Starting the machine requires at least 1 second of low level, and with a level conversion, the levels are opposite
    vTaskDelay(2500 / portTICK_PERIOD_MS);
    gpio_set_level(MODEM_PWKEY, 0);
    ESP_LOGI(TAG, "Waiting till modem ready...");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
}

void app_main(void)
{
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    /* Init and register system/core components */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, &on_ip_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(NETIF_PPP_STATUS, ESP_EVENT_ANY_ID, &on_ppp_changed, NULL));

    initialize_sntp();

    /* Configure the PPP netif */
    esp_modem_dce_config_t dce_config = ESP_MODEM_DCE_DEFAULT_CONFIG("live.ANP.com");
    esp_netif_config_t netif_ppp_config = ESP_NETIF_DEFAULT_PPP();
    esp_netif_t *esp_netif = esp_netif_new(&netif_ppp_config);
    assert(esp_netif);

    event_group = xEventGroupCreate();

    esp_modem_dte_config_t dte_config = ESP_MODEM_DTE_DEFAULT_CONFIG();
    /* setup UART specific configuration based on kconfig options */
    dte_config.uart_config.flow_control = ESP_MODEM_FLOW_CONTROL_SW;
    dte_config.uart_config.tx_io_num = MODEM_TX;
    dte_config.uart_config.rx_io_num = MODEM_RX;
    dte_config.uart_config.rts_io_num = 25;

    ESP_LOGI(TAG, "Initializing esp_modem for the SIM7000 module...");
    
    esp_modem_dce_t *dce = esp_modem_new_dev(ESP_MODEM_DCE_SIM7000, &dte_config, &dce_config, esp_netif);

    assert(dce);

    //  clears the CONNECT_BIT, GOT_DATA_BIT, and USB_DISCONNECTED_BIT in the event_group event group. This might be done to reset the state of these bits before starting a new operation, for example.
    xEventGroupClearBits(event_group, CONNECT_BIT | GOT_DATA_BIT | USB_DISCONNECTED_BIT);

    // Send commands till the commandline is active. For later purpuse to activate GNSS
    char resp[32];
    esp_err_t err = esp_modem_at(dce, "ATI\r", resp, 1000);
    int X = 0;
    while (err != ESP_OK) {
       ESP_LOGE(TAG, "ATI failed with %d", err);
       vTaskDelay(1010 / portTICK_PERIOD_MS);    
       err = esp_modem_at(dce, "ATI\r", resp, 1000);
       if (err == ESP_ERR_TIMEOUT && X == 5) {
          // command timeout, modem may not be active
          Modem_Start(); 
          err = esp_modem_at(dce, "ATI\r", resp, 1000);
          X = 0;
       }
       X++;
    }
    if (err == ESP_OK) {
       ESP_LOGI(TAG, "ATI response=%s", resp);
    }

    int rssi, ber;
    esp_err_t err1 = esp_modem_get_signal_quality(dce, &rssi, &ber);
    if (err1 != ESP_OK) {
        ESP_LOGE(TAG, "esp_modem_get_signal_quality failed with %d %s", err, esp_err_to_name(err));
        return;
    }

    err1 = esp_modem_set_mode(dce, ESP_MODEM_MODE_DATA);
    if (err1 != ESP_OK) {
        ESP_LOGE(TAG, "esp_modem_set_mode(ESP_MODEM_MODE_DATA) failed with %d", err);
        return;
    }


    ESP_LOGI(TAG, "Signal quality: rssi=%d, ber=%d", rssi, ber);

    esp_netif_ip_info_t ip_info;
    esp_err_t err2 = esp_netif_get_ip_info(esp_netif, &ip_info);
    if (err2 != ESP_OK) {
        ESP_LOGE(TAG, "esp_netif_get_ip_info failed with %d", err2);
        return;
    } else {
        ESP_LOGI(TAG, "Connection IP: " IPSTR, IP2STR(&ip_info.ip));
    }

    /* Wait for IP address */
    ESP_LOGI(TAG, "Waiting for IP address");
    xEventGroupWaitBits(event_group, CONNECT_BIT | USB_DISCONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    CHECK_USB_DISCONNECTION(event_group);

    /* Config MQTT */
    ESP_LOGI(TAG, "Config MQTT");
    esp_mqtt_client_config_t mqtt_config = {
        .broker.address.uri = "mqtt://mqtts://io.adafruit.com",
        .broker.address.port = 1883,
        .credentials.username = "username",
        .credentials.authentication.password = "password",
    };

    esp_mqtt_client_handle_t mqtt_client = esp_mqtt_client_init(&mqtt_config);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
    xEventGroupWaitBits(event_group, GOT_DATA_BIT | USB_DISCONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    CHECK_USB_DISCONNECTION(event_group);

    esp_mqtt_client_destroy(mqtt_client);
    err = esp_modem_set_mode(dce, ESP_MODEM_MODE_COMMAND);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_modem_set_mode(ESP_MODEM_MODE_COMMAND) failed with %d", err);
        return;
    }
    char imsi[32];
    err = esp_modem_get_imsi(dce, imsi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_modem_get_imsi failed with %d", err);
        return;
    }
    ESP_LOGI(TAG, "IMSI=%s", imsi);

    #if defined(CONFIG_EXAMPLE_SERIAL_CONFIG_USB)
        // USB example runs in a loop to demonstrate hot-plugging and sudden disconnection features.
        ESP_LOGI(TAG, "USB demo finished. Disconnect and connect the modem to run it again");
        xEventGroupWaitBits(event_group, USB_DISCONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
        CHECK_USB_DISCONNECTION(event_group); // dce will be destroyed here
        // while (1)
    #else
        // UART DTE clean-up
        esp_modem_destroy(dce);
        esp_netif_destroy(esp_netif);
    #endif
}