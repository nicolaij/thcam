#include "main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

#include "freertos/queue.h"
#include "driver/gptimer.h"

#include "led_strip.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "DIO";

gptimer_handle_t gptimer = NULL;

#define QUEUE_LENGTH 1
#define ITEM_SIZE sizeof(uint64_t)

QueueHandle_t xQueue = NULL;
uint8_t ucQueueStorageArea[QUEUE_LENGTH * ITEM_SIZE];

QueueHandle_t xQueueLed = NULL;

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken;

    // We have not woken a task at the start of the ISR.
    xHigherPriorityTaskWoken = pdFALSE;
    // xSemaphoreGiveFromISR(arg, &xHigherPriorityTaskWoken);
    uint64_t t;
    gptimer_get_raw_count(gptimer, &t);

    xQueueSendFromISR(xQueue, &t, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

void dio_init()
{
    static StaticQueue_t xStaticQueue;
    xQueue = xQueueCreateStatic(QUEUE_LENGTH, ITEM_SIZE, ucQueueStorageArea, &xStaticQueue);
    configASSERT(xQueue);

    xQueueLed = xQueueCreate(2, sizeof(led_task_data_t));

    ESP_LOGI(TAG, "Create timer handle");
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 40000000, // 40MHz
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));

    ESP_ERROR_CHECK(gpio_hold_dis(PIN_WATER1));

    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins that you want to set
    io_conf.pin_bit_mask = BIT64(PIN_WATER1) | BIT64(PIN_BATT_CONTROL);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    // configure GPIO with the given settings
    gpio_config(&io_conf);

    gpio_set_level(PIN_WATER1, 1);
    gpio_set_level(PIN_BATT_CONTROL, 0);

    io_conf.pin_bit_mask = BIT64(PIN_LIGHT) | BIT64(PIN_WATER2);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    led_task_data_t led = {.set = 0, .xTicksToDelay = pdMS_TO_TICKS(2000)};
    led.red = 40 * gpio_get_level(PIN_LIGHT);
    led.blue = 40 * gpio_get_level(PIN_WATER2);
    xQueueSend(xQueueLed, &led, 0);

    // ADC
    static int adc_raw;
    const float kV = 815.0 / 3.37;

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

    const int count = 5;
    unsigned int sum = 0;

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, PIN_BATT, &config));
    sum = 0;
    for (int i = 0; i < count; i++)
    {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, PIN_BATT, &adc_raw));
        sum = sum + adc_raw;
        ESP_LOGD("Battery", "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, PIN_BATT, adc_raw);
        vTaskDelay(1);
    }
    result.measure.battery = sum / count / kV;
    ESP_LOGI("Battery", "ADC%d Channel[%d] AVG: %d, %.2f V", ADC_UNIT_1 + 1, PIN_BATT, sum / count, result.measure.battery);

    gpio_reset_pin(PIN_BATT_CONTROL);

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, PIN_LIGHT, &config));

    sum = 0;
    for (int i = 0; i < count; i++)
    {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, PIN_LIGHT, &adc_raw));
        sum = sum + adc_raw;
        ESP_LOGD("Light", "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, PIN_LIGHT, adc_raw);
        vTaskDelay(1);
    }
    result.measure.light = sum / count;
    ESP_LOGI("Light", "ADC%d Channel[%d] AVG: %d", ADC_UNIT_1 + 1, PIN_LIGHT, sum / count);

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, PIN_WATER2, &config));
    sum = 0;
    for (int i = 0; i < count; i++)
    {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, PIN_WATER2, &adc_raw));
        sum = sum + adc_raw;
        ESP_LOGD("Water", "ADC%d Channel[%d] Raw Data: %d, mode1 +3.3", ADC_UNIT_1 + 1, PIN_WATER2, adc_raw);
        vTaskDelay(1);
    }
    result.measure.water = sum / count;
    ESP_LOGI("Water", "ADC%d Channel[%d] AVG: %d, mode1 +3.3", ADC_UNIT_1 + 1, PIN_WATER2, sum / count);

    // install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);

    // измерение dt0, dt1 воды
    /*
        uint64_t t;
        io_conf.intr_type = GPIO_INTR_NEGEDGE;
        io_conf.pin_bit_mask = BIT64(PIN_WATER2);
        // set as input mode
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf);

        xQueueReset(xQueue);
        vTaskDelay(pdMS_TO_TICKS(500));

        ESP_ERROR_CHECK(gptimer_set_raw_count(gptimer, 0));
        ESP_ERROR_CHECK(gptimer_start(gptimer));
        gpio_set_level(PIN_WATER1, 0);
        if (xQueueReceive(xQueue, &t, pdMS_TO_TICKS(500)))
        {
            ESP_LOGW(TAG, "Down. Timer stopped, count=%llu", t);

            vTaskDelay(pdMS_TO_TICKS(500));
        }
        else
        {
            gptimer_get_raw_count(gptimer, &t);
            ESP_LOGW(TAG, "Down. Missed one count event, count=%llu", t);
        }

        ESP_ERROR_CHECK(gptimer_stop(gptimer));

        io_conf.intr_type = GPIO_INTR_POSEDGE;
        io_conf.pin_bit_mask = BIT64(PIN_WATER2);
        // set as input mode
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf);

        xQueueReset(xQueue);
        ESP_ERROR_CHECK(gptimer_set_raw_count(gptimer, 0));
        ESP_ERROR_CHECK(gptimer_start(gptimer));
        gpio_set_level(PIN_WATER1, 1);
        if (xQueueReceive(xQueue, &t, pdMS_TO_TICKS(500)))
        {
            ESP_LOGW(TAG, "Up. Timer stopped, count=%llu", t);
        }
        else
        {
            gptimer_get_raw_count(gptimer, &t);
            ESP_LOGW(TAG, "Down. Missed one count event, count=%llu", t);
        }
        ESP_ERROR_CHECK(gptimer_stop(gptimer));
        ESP_ERROR_CHECK(gptimer_disable(gptimer));
    */

    // если кз по датчику воды - переключаем вход на поддяжку и измеряем заново
    if (adc_raw >= 000) // !!!  ВСЕГДА
    {
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = BIT64(PIN_WATER1);
        io_conf.pull_down_en = 1;
        io_conf.pull_up_en = 1;
        gpio_config(&io_conf);

        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, PIN_WATER2, &config));

        gpio_pulldown_en(PIN_WATER2);

        vTaskDelay(pdMS_TO_TICKS(50));

        sum = 0;
        for (int i = 0; i < count; i++)
        {
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, PIN_WATER2, &adc_raw));
            sum = sum + adc_raw;
            ESP_LOGD("Water", "ADC%d Channel[%d] Raw Data: %d, mode2 Pullup/Pulldown", ADC_UNIT_1 + 1, PIN_WATER2, adc_raw);
            vTaskDelay(1);
        }
        result.measure.water2 = sum / count;
        ESP_LOGI("Water", "ADC%d Channel[%d] AVG: %d, mode2 Pullup/Pulldown", ADC_UNIT_1 + 1, PIN_WATER2, sum / count);

        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = BIT64(PIN_WATER1);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf);
        gpio_set_level(PIN_WATER1, 1);
    }

    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = BIT64(PIN_LIGHT) | BIT64(PIN_WATER2);
    // set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    gpio_isr_handler_add(PIN_LIGHT, gpio_isr_handler, (void *)PIN_LIGHT);
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

    // ESP_ERROR_CHECK(gpio_hold_en(PIN_WATER1));
    //   ESP_ERROR_CHECK(gpio_sleep_set_pull_mode(GPIO_NUM_5, GPIO_PULLUP_ONLY));
    //   ESP_ERROR_CHECK(gpio_sleep_set_direction(GPIO_NUM_5, GPIO_MODE_INPUT));

    // esp_deep_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_ON);

    ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(wake_mask, ESP_GPIO_WAKEUP_GPIO_HIGH));

    // ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(BIT64(GPIO_NUM_2) | BIT64(GPIO_NUM_3), ESP_GPIO_WAKEUP_GPIO_HIGH));
    // ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(BIT64(GPIO_NUM_2), ESP_GPIO_WAKEUP_GPIO_LOW));
}

void led_task(void *arg)
{
    led_task_data_t data;
    TickType_t delay_time = portMAX_DELAY;

    led_strip_handle_t led_strip = NULL;

    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = GPIO_NUM_8,             // The GPIO that connected to the LED strip's data line
        .max_leds = 1,                            // The number of LEDs in the strip,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
        .led_model = LED_MODEL_SK6812,            // LED strip model
        .flags.invert_out = false,                // whether to invert the output signal
    };

    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };

    while (1)
    {
        if (xQueueReceive(xQueueLed, &(data), delay_time) == pdPASS)
        {
            if (led_strip == NULL)
            {
                ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
            }

            if (data.xTicksToDelay > 0)
            {
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, data.red, data.green, data.blue));
                ESP_ERROR_CHECK(led_strip_refresh(led_strip));
            }
            delay_time = data.xTicksToDelay;
        }
        else
        {
            ESP_ERROR_CHECK(led_strip_clear(led_strip));
            delay_time = portMAX_DELAY;
            if (data.xTicksToDelay > 0)
            {
                ESP_ERROR_CHECK(led_strip_del(led_strip));
            }
        }
    }
}
