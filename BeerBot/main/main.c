#include <stdio.h>
#include "esp_system.h"
#include "string.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"



static const char *TAG = "ESP_NOW";
uint8_t broadcast_addr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 


typedef struct{
    float xdir;
    float ydir;
} bot_dir;



void init_wifi(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE)); // wifi channel set to 1
}

void init_espnow(void) {
    ESP_ERROR_CHECK(esp_now_init());

    esp_now_peer_info_t peer = {
        .peer_addr = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
        .channel = 1,
        .ifidx = ESP_IF_WIFI_STA,
        .encrypt = false
    };
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}


void transmit_data(void *pvParameter){
    bot_dir joystick;
    while(1){
        joystick.xdir = 0;
        joystick.ydir = 0;

        esp_err_t result = esp_now_send(broadcast_addr, (uint8_t *)&joystick, sizeof(joystick));
        if ( result == ESP_OK){
            ESP_LOGI(TAG, "Sent Successfully \n");

        }else if (result != ESP_OK) {
            ESP_LOGE(TAG, "Send error: %s", esp_err_to_name(result));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));

    }
}

void app_main(void)
{
    init_wifi();
    init_espnow();


    xTaskCreate(transmit_data, "Broadcast Loop", 2048, NULL, 1, NULL); // RTOS Task
    
}