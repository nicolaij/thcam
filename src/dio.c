#include "main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "DIO";

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    // uint32_t gpio_num = (uint32_t)arg;
    // xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
    BaseType_t xHigherPriorityTaskWoken;

    // We have not woken a task at the start of the ISR.
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(arg, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

void dio_init()
{

    ESP_ERROR_CHECK(gpio_hold_dis(PIN_WATER1));

    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins that you want to set
    io_conf.pin_bit_mask = BIT64(PIN_WATER1) || BIT64(PIN_BATT_CONTROL);
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    // disable pull-up mode
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    gpio_config(&io_conf);

    gpio_set_level(PIN_WATER1, 1);
    gpio_set_level(PIN_BATT_CONTROL, 0);
    vTaskDelay(1);

    // ADC
    static int adc_raw;
    const float kV = 1147.0 / 5.01;

    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, PIN_BATT, &config));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, PIN_BATT, &adc_raw));
    result.measure.battery = adc_raw / kV;
    ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d, %.2f V", ADC_UNIT_1 + 1, PIN_BATT, adc_raw, result.measure.battery);

    gpio_reset_pin(PIN_BATT_CONTROL);

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, PIN_LIGHT, &config));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, PIN_LIGHT, &adc_raw));
    result.measure.light = adc_raw;
    ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d, %.2f V", ADC_UNIT_1 + 1, PIN_LIGHT, adc_raw, result.measure.light);

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, PIN_WATER2, &config));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, PIN_WATER2, &adc_raw));
    result.measure.water = adc_raw;
    ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d, %.2f V, mode1", ADC_UNIT_1 + 1, PIN_WATER2, adc_raw, result.measure.water);

    // если кз по датчику воды - переключаем вход на поддяжку и измеряем заново
    if (adc_raw > 000) // !!!  ВСЕГДА
    {
        gpio_reset_pin(PIN_WATER1);

        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = BIT64(PIN_WATER1);
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en = 1;
        gpio_config(&io_conf);
        vTaskDelay(1);

        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &config));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &adc_raw));
        result.measure.water2 = adc_raw;
        ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d, %.2f V, mode2", ADC_UNIT_1 + 1, ADC_CHANNEL_4, adc_raw, result.measure.water2);
    }

    // interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = BIT64(PIN_LIGHT) | BIT64(PIN_WATER2);
    // set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(PIN_LIGHT, gpio_isr_handler, (void *)PIN_LIGHT);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(PIN_WATER2, gpio_isr_handler, (void *)PIN_WATER2);
}

void dio_sleep()
{
    ESP_LOGI("main", "Light: %d; Water: %d", gpio_get_level(PIN_LIGHT), gpio_get_level(PIN_WATER2));

    uint64_t wake_mask = 0;
    if (gpio_get_level(PIN_LIGHT) == 0)
        wake_mask |= BIT64(PIN_LIGHT);
    if (gpio_get_level(PIN_WATER2) == 0)
        wake_mask |= BIT64(PIN_WATER2);

    // gpio_set_pull_mode(GPIO_NUM_2, GPIO_PULLDOWN_ONLY);
    // gpio_set_pull_mode(GPIO_NUM_4, GPIO_PULLDOWN_ONLY);
    // vTaskDelay(1000);

    ESP_ERROR_CHECK(gpio_hold_en(PIN_WATER1));
    //  ESP_ERROR_CHECK(gpio_sleep_set_pull_mode(GPIO_NUM_5, GPIO_PULLUP_ONLY));
    //  ESP_ERROR_CHECK(gpio_sleep_set_direction(GPIO_NUM_5, GPIO_MODE_INPUT));

    // esp_deep_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_ON);

    ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(wake_mask, ESP_GPIO_WAKEUP_GPIO_HIGH));

    // ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(BIT64(GPIO_NUM_2) | BIT64(GPIO_NUM_3), ESP_GPIO_WAKEUP_GPIO_HIGH));
    // ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(BIT64(GPIO_NUM_2), ESP_GPIO_WAKEUP_GPIO_LOW));
}
/*
static void gpio_task(void *arg)
{
    uint32_t io_num;
    for (;;)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
            if (io_num == GPIO_NUM_2)
            {
                gpio_set_level(LED4, gpio_get_level(io_num));
            }
            if (io_num == GPIO_NUM_3)
            {
                gpio_set_level(LED5, gpio_get_level(io_num));
            }

            ESP_LOGI(TAG, "GPIO[%" PRIu32 "] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}
*/