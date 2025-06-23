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
#include "esp_timer.h"
#include "driver/ledc.h"


#define LOGIC_INPUT_1 GPIO_NUM_16
#define LOGIC_INPUT_2 GPIO_NUM_18

QueueHandle_t data_queue;

int64_t timeCheck;




static const char *TAG = "ESP_NOW Rx";



typedef struct {
    float xdir;
    float ydir;
} beerbot_test;

static void pwm_init(void){
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t channel = {
        .gpio_num = 17,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel));
    
}



void OnDataRecv(const uint8_t *mac_addr, const uint8_t *msg, int len) {
    if (len != sizeof(beerbot_test)) {
        ESP_LOGW(TAG, "Received unexpected data length: %d", len);
        return;
    }

    beerbot_test in_dir; // directions from the input
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac_addr[0], mac_addr[1], mac_addr[2],
             mac_addr[3], mac_addr[4], mac_addr[5]);


    memcpy(&in_dir, msg, sizeof(beerbot_test));  // Safe copy

    xQueueSend(data_queue, &in_dir, portMAX_DELAY);

}




void init_wifi(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void receiveMsg(void *pvParameter){
    while(1){
        beerbot_test data_recv; // struct of data received from queue
        
        if (xQueueReceive(data_queue, &data_recv, portMAX_DELAY) == pdTRUE){
            ESP_LOGI(TAG, "%f\n%f", data_recv.xdir, data_recv.ydir);
            if (data_recv.ydir > 1.0f) data_recv.ydir = 1.0f;
            if (data_recv.ydir < -1.0f) data_recv.ydir = -1.0f;

            if (data_recv.ydir == 0.0){           //Input for L298N motor controller
                gpio_set_level(LOGIC_INPUT_1,0);  //Logic input 1 and 2 connect to input 1 and 2 on motor controller
                gpio_set_level(LOGIC_INPUT_2,0);  //PWM GPIO connects to Enable on motor controller
            }else if(data_recv.ydir<0){
                gpio_set_level(LOGIC_INPUT_1,1);
                gpio_set_level(LOGIC_INPUT_1,0);
            }else if(data_recv.ydir>0){
                gpio_set_level(LOGIC_INPUT_1,0);
                gpio_set_level(LOGIC_INPUT_1,1);
            }
            uint32_t duty = (uint32_t)(data_recv.ydir * ((1 << LEDC_TIMER_8_BIT) - 1));
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        }
    }
}
void status_check(void *pvParameter){
    bool lost = false;
    while(1){
        if (esp_timer_get_time()-timeCheck>3000000){
            if(!lost){
            ESP_LOGI(TAG, "Connection Lost\n");
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

            lost = true;
            

            }
        } else{
            lost = false;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    timeCheck = esp_timer_get_time();
    ESP_ERROR_CHECK(nvs_flash_init());
    data_queue = xQueueCreate(10, sizeof(beerbot_test));
    gpio_set_direction(GPIO_NUM_17, GPIO_MODE_OUTPUT);

    pwm_init();
    init_wifi();

    if (esp_now_init() != ESP_OK){
        ESP_LOGE(TAG, "Error initializing ESP NOW");
        return;
    }
    esp_now_register_recv_cb(OnDataRecv);

    ESP_LOGI(TAG, "ESP-NOW Receiver Initialized");

    xTaskCreate(receiveMsg, "Message",2048, NULL, 5, NULL);
    xTaskCreate(status_check, "Check Connection", 2048, NULL, 4, NULL);
}