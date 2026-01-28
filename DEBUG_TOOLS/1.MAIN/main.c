#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"

#include "mbcontroller.h" // esp-modbus API

static const char *TAG = "modbus_tcp_example";

/* Wi‑Fi 설정 */
#define WIFI_SSID "YOUR_SSID"
#define WIFI_PASS "YOUR_PASS"

/* Modbus 설정 */
#define SLAVE_IP   "192.168.0.16"
#define SLAVE_PORT 502
#define SLAVE_ID   1

/* 이벤트 그룹 비트 */
static EventGroupHandle_t s_wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

/* Wi‑Fi 이벤트 핸들러 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected, reconnecting...");
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished. Connecting to SSID:%s", WIFI_SSID);
}

/* Modbus 태스크: TCP 마스터로 주기적 읽기 수행 */
static void modbus_task(void *arg)
{
    void *master_handle = NULL;
    mb_communication_info_t comm = {
        .mode = MB_MODE_TCP,
        .port = SLAVE_PORT,
        .ip_addr = SLAVE_IP
    };

    esp_err_t err;

    /* 생성자 및 연결 */
    err = mbc_master_create_tcp(&comm, &master_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mbc_master_create_tcp failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    err = mbc_master_connect(master_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mbc_master_connect failed: %s", esp_err_to_name(err));
        mbc_master_destroy(master_handle);
        vTaskDelete(NULL);
        return;
    }

    /* 시작 */
    err = mbc_master_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mbc_master_start failed: %s", esp_err_to_name(err));
        mbc_master_destroy(master_handle);
        vTaskDelete(NULL);
        return;
    }

    uint16_t regs[10];

    while (1) {
        memset(regs, 0, sizeof(regs));
        err = mbc_master_read_holding_registers(master_handle, SLAVE_ID, 0, 10, regs);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Read regs: %u, %u, %u, ...", regs[0], regs[1], regs[2]);
        } else {
            ESP_LOGW(TAG, "Read failed: %s", esp_err_to_name(err));
            /* 필요하면 재연결 로직 추가 가능 */
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    /* 정리 (도달하지 않음) */  
    mbc_master_destroy(master_handle);
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_sta();

    /* IP 획득 대기 */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT,
                                           pdFALSE,
                                           pdTRUE,
                                           pdMS_TO_TICKS(10000));
    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGW(TAG, "WiFi not connected after timeout");
        /* 필요하면 재시도 로직 추가 */
    } else {
        /* Wi‑Fi 연결되면 Modbus 태스크 시작 */
        xTaskCreate(modbus_task, "modbus_task", 8192, NULL, 5, NULL);
    }

    /* 기존 앱 루프(선택) */
    while (1) {
        ESP_LOGI(TAG, "APP alive");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}