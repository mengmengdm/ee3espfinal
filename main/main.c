#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lcd_display_task.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting application...");
    
    // Initialize and start LCD display task
    lcd_display_init();
    lcd_display_start();
    
    // Main loop - can be used for other tasks in the future
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
} 