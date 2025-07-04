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
#include "driver/uart.h"

const int uart_buffer_size = (1024 * 2);
const uart_port_t uart_num = UART_NUM_1;
static const char *TAG = "ESP_NOW";

QueueHandle_t uart_queue;

uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 122,
};

uint8_t broadcast_addr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 


typedef struct{
    float x,y,z;
} bot_dir;

void init_uart(void){
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, GPIO_NUM_17,GPIO_NUM_16,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE));


}

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

void uart_rx(void *pvParameter){
    uint8_t data[128];

    while(1){
        bot_dir direction= {0};
        

        int length =0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&length));

        if (length>0){
            if (length>sizeof(data)){length=sizeof(data)-1;}
            int read = uart_read_bytes(uart_num, data, length, 100 /portTICK_PERIOD_MS);
        
            if (read>0){
                data[read]='\0';

                if (sscanf((char*)data, "%f,%f,%f",&direction.x,&direction.y,&direction.z)==3){
                    ESP_LOGI(TAG, "Received direction: x=%f, y=%f, z=%f", direction.x, direction.y, direction.z);
                    xQueueSend(uart_queue, &direction, portMAX_DELAY);
                }else{
                    ESP_LOGE(TAG, "Something went Wrong");
                }
                
            }
        }
    vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void transmit_data(void *pvParameter){
    bot_dir dir_recv;
    while(1){
        xQueueReceive(uart_queue, &dir_recv, portMAX_DELAY);

        ESP_LOGI(TAG, "X: %f, Y: %f, Z: %f", dir_recv.x,dir_recv.y,dir_recv.z);
        esp_err_t result = esp_now_send(broadcast_addr, (uint8_t *)&dir_recv, sizeof(dir_recv));
        if ( result == ESP_OK){
            ESP_LOGI(TAG, "Sent Successfully ");

        }else if (result != ESP_OK) {
            ESP_LOGE(TAG, "Send error: %s", esp_err_to_name(result));
        }
        vTaskDelay(pdMS_TO_TICKS(100));

    }
}

void app_main(void)
{
    uart_queue = xQueueCreate(10, sizeof(bot_dir));
    init_uart();
    init_wifi();
    init_espnow();

    
    xTaskCreate(transmit_data, "Broadcast Loop", 2048, NULL, 1, NULL); // RTOS Task
    xTaskCreate(uart_rx, "Receive UART", 2048, NULL, 1, NULL);
    
}