#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lcd_display_task.h"
#include "esp_http_client_example.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting application...");
    
    // Initialize and start LCD display task
    lcd_display_init();
    lcd_display_start();
    
    // Initialize network
    esp_err_t ret = init_network();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize network");
        return;
    }

    // Create GET request task
    xTaskCreate(&get_request_task, "get_request_task", 8192, NULL, 5, NULL);
    // Create POST request task
    xTaskCreate(&post_request_task, "post_request_task", 8192, NULL, 5, NULL);
    
    // Main loop - can be used for other tasks in the future
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
} 