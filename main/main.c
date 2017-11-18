#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

void emu_user_init(void);

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

void flic_led(){
    nvs_flash_init();

    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    int level = 0;
    for(int i=0; i<3; i++){
        gpio_set_level(GPIO_NUM_2, level);
        level = !level;
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    flic_led();      // For Hard Test
    emu_user_init(); // SMS Emu Main
}

