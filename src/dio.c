#include "main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "freertos/queue.h"
#include "driver/gptimer.h"

#include "led_strip.h"

#include "esp_adc/adc_continuous.h"

static const char *TAG = "DIO";

#define QUEUE_LENGTH 1
#define ITEM_SIZE sizeof(uint64_t)

QueueHandle_t xQueue = NULL;
uint8_t ucQueueStorageArea[QUEUE_LENGTH * ITEM_SIZE];

QueueHandle_t xQueueLed = NULL;

adc_continuous_handle_t handle;

// R - внешний 10к
// r - внешний 10к + внутр. нижн. 45к.
// P - питание через верхний ключ
char water1_mode = 'R';

// r - внутр. нижн. 45к.
// 0 - свободный
char water2_mode = '0';

#define points 400

uint8_t adcresult[points * 4 * 2] = {0};

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    /* xHigherPriorityTaskWoken must be set to pdFALSE before it is used. */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    uint32_t gpio_num = (uint32_t)arg;

    if (gpio_num == PIN_BATT)
    {
        xEventGroupSetBitsFromISR(ready_event_group, NEED_WIFI, &xHigherPriorityTaskWoken);
        xEventGroupSetBitsFromISR(ready_event_group, NOW_CHARGE, &xHigherPriorityTaskWoken);
    }

    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        /* Writing to the queue caused a task to unblock and the unblocked task
           has a priority higher than or equal to the priority of the currently
           executing task (the task this interrupt interrupted). Perform a
           context switch so this interrupt returns directly to the unblocked
           task. */
        portYIELD_FROM_ISR(); /* or portEND_SWITCHING_ISR() depending on the
                                 port.*/
    }
}

static void continuous_adc_init()
{
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = points * 2 * 4,
        .conv_frame_size = points * 2 * 4,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 50 * 2 * points / 2,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    int num = 0;

    adc_pattern[num].atten = ADC_ATTEN_DB_12;
    adc_pattern[num].channel = PIN_LIGHT;
    adc_pattern[num].unit = ADC_UNIT_1;
    adc_pattern[num].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    num++;
    adc_pattern[num].atten = ADC_ATTEN_DB_12;
    adc_pattern[num].channel = PIN_WATER2;
    adc_pattern[num].unit = ADC_UNIT_1;
    adc_pattern[num].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    num++;
    dig_cfg.pattern_num = num;

    // ESP_LOGI(TAG, "adc_pattern[%d].atten is :%" PRIx8, i, adc_pattern[i].atten);
    // ESP_LOGI(TAG, "adc_pattern[%d].channel is :%" PRIx8, i, adc_pattern[i].channel);
    // ESP_LOGI(TAG, "adc_pattern[%d].unit is :%" PRIx8, i, adc_pattern[i].unit);

    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));
}

void dio_init()
{
    static StaticQueue_t xStaticQueue;
    xQueue = xQueueCreateStatic(QUEUE_LENGTH, ITEM_SIZE, ucQueueStorageArea, &xStaticQueue);
    configASSERT(xQueue);

    xQueueLed = xQueueCreate(2, sizeof(led_task_data_t));

    ESP_ERROR_CHECK(gpio_hold_dis(PIN_WATER1));
    ESP_ERROR_CHECK(gpio_hold_dis(PIN_WATER2));

    // install gpio isr service
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));

    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins that you want to set
    io_conf.pin_bit_mask = BIT64(PIN_WATER1) | BIT64(PIN_CHARGE_CONTROL);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    // configure GPIO with the given settings
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    water1_mode = 'P';
    gpio_set_level(PIN_WATER1, 1);
    gpio_set_level(PIN_CHARGE_CONTROL, 0);

    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = BIT64(PIN_BATT);
    // set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_BATT, gpio_isr_handler, (void *)PIN_BATT));

    continuous_adc_init();

    ESP_ERROR_CHECK(adc_continuous_start(handle));

    uint32_t ret_num = 0;
    uint32_t light = 0;
    uint32_t light_cnt = 0;
    uint32_t water = 0;
    uint32_t water_cnt = 0;
    esp_err_t ret = adc_continuous_read(handle, adcresult, sizeof(adcresult), &ret_num, ADC_MAX_DELAY);
    if (ret == ESP_OK)
    {
        ESP_ERROR_CHECK(adc_continuous_stop(handle));

        ESP_LOGI(TAG, "ret is %x, ret_num is %" PRIu32 " bytes", ret, ret_num);
        for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES)
        {
            adc_digi_output_data_t *p = (adc_digi_output_data_t *)&adcresult[i];
            uint32_t chan_num = p->type2.channel;
            uint32_t data = p->type2.data;
            if (chan_num == PIN_LIGHT)
            {
                light += data;
                light_cnt++;
            }
            if (chan_num == PIN_WATER2)
            {
                water += data;
                water_cnt++;
            }
        }

        result.measure.light = light / light_cnt;
        result.measure.water = water / water_cnt;

        ESP_LOGI("Light", "ADC chan %d: %4.0f", PIN_LIGHT, result.measure.light);
        ESP_LOGI("Water", "ADC chan %d: %4.0f", PIN_WATER2, result.measure.water);

#if LOG_LOCAL_LEVEL == ESP_LOG_DEBUG
        for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES)
        {
            adc_digi_output_data_t *p = (adc_digi_output_data_t *)&adcresult[i];
            uint32_t chan_num = p->type2.channel;
            uint32_t data = p->type2.data;
            if (chan_num == 3)
                printf("%3lu ", data);
        }
        printf("\n\n");
        for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES)
        {
            adc_digi_output_data_t *p = (adc_digi_output_data_t *)&adcresult[i];
            uint32_t chan_num = p->type2.channel;
            uint32_t data = p->type2.data;

            if (chan_num == 4)
                printf("%3lu ", data);
        }
        printf("\n\n");
#endif
    }
    else if (ret == ESP_ERR_TIMEOUT)
    {
        ESP_ERROR_CHECK(adc_continuous_stop(handle));
    }

    // PIN_WATER1 подтянут 10кОм, с pulldown будет около 2,5В
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = BIT64(PIN_WATER1);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    water1_mode = 'r';

    // включам подтяжку входа ADC, при кз будет около 1,8 в
    ESP_ERROR_CHECK(gpio_pulldown_en(PIN_WATER2));
    water2_mode = 'r';

    // ESP_ERROR_CHECK(gpio_pulldown_en(PIN_LIGHT));

    ESP_ERROR_CHECK(adc_continuous_start(handle));

    ret_num = 0;
    light = 0;
    light_cnt = 0;
    water = 0;
    water_cnt = 0;
    ret = adc_continuous_read(handle, adcresult, sizeof(adcresult), &ret_num, ADC_MAX_DELAY);
    if (ret == ESP_OK)
    {
        ESP_ERROR_CHECK(adc_continuous_stop(handle));

        ESP_LOGI(TAG, "ret is %x, ret_num is %" PRIu32 " bytes", ret, ret_num);
        for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES)
        {
            adc_digi_output_data_t *p = (adc_digi_output_data_t *)&adcresult[i];
            uint32_t chan_num = p->type2.channel;
            uint32_t data = p->type2.data;
            if (chan_num == PIN_LIGHT)
            {
                light += data;
                light_cnt++;
            }
            if (chan_num == PIN_WATER2)
            {
                water += data;
                water_cnt++;
            }
        }

        // result.measure.light = light / light_cnt;
        result.measure.water2 = water / water_cnt;
        ESP_LOGI("Water", "ADC chan %d: %4.0f (pulldown)", PIN_WATER2, result.measure.water2);

#if LOG_LOCAL_LEVEL == ESP_LOG_DEBUG
        for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES)
        {
            adc_digi_output_data_t *p = (adc_digi_output_data_t *)&adcresult[i];
            uint32_t chan_num = p->type2.channel;
            uint32_t data = p->type2.data;
            if (chan_num == 3)
                printf("%3lu ", data);
        }
        printf("\n\n");
        for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES)
        {
            adc_digi_output_data_t *p = (adc_digi_output_data_t *)&adcresult[i];
            uint32_t chan_num = p->type2.channel;
            uint32_t data = p->type2.data;

            if (chan_num == 4)
                printf("%3lu ", data);
        }
        printf("\n\n");
#endif
    }
    else if (ret == ESP_ERR_TIMEOUT)
    {
        ESP_ERROR_CHECK(adc_continuous_stop(handle));
    }

    ESP_ERROR_CHECK(adc_continuous_deinit(handle));

    /*
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.pin_bit_mask = BIT64(PIN_LIGHT) | BIT64(PIN_WATER2);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf);

        // gpio_set_level(PIN_CHARGE_CONTROL, 1);
        // vTaskDelay(1);
        // ESP_LOGI("Battery", "Charge 1 Pin[%d] Input: %d", PIN_BATT, gpio_get_level(PIN_BATT));
        // gpio_set_level(PIN_CHARGE_CONTROL, 0);

        led_task_data_t led = {.set = 0, .xTicksToDelay = pdMS_TO_TICKS(2000)};
        led.red = 40 * gpio_get_level(PIN_LIGHT);
        led.blue = 40 * gpio_get_level(PIN_WATER2);
        xQueueSend(xQueueLed, &led, 0);
    */

    // ADC
    /*
        static int adc_raw;
        // const float kV = get_menu_id("kbatt");

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

        const int count = 10;
        const int delay = 2;
        unsigned int sum = 0;

        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, PIN_LIGHT, &config));

        sum = 0;
        for (int i = 0; i < count; i++)
        {
            vTaskDelay(delay);
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, PIN_LIGHT, &adc_raw));
            sum = sum + adc_raw;
            ESP_LOGD("Light", "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, PIN_LIGHT, adc_raw);
        }
        result.measure.light = sum / count;
        ESP_LOGI("Light", "ADC%d Channel[%d] AVG: %d", ADC_UNIT_1 + 1, PIN_LIGHT, sum / count);

        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, PIN_WATER2, &config));

        sum = 0;
        for (int i = 0; i < count; i++)
        {
            vTaskDelay(delay);
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, PIN_WATER2, &adc_raw));
            sum = sum + adc_raw;
            ESP_LOGD("Water", "ADC%d Channel[%d] Raw Data: %d, mode1 +3.3", ADC_UNIT_1 + 1, PIN_WATER2, adc_raw);
        }
        result.measure.water = sum / count;
        ESP_LOGI("Water", "ADC%d Channel[%d] AVG: %d, mode1 +3.3", ADC_UNIT_1 + 1, PIN_WATER2, sum / count);

        // если кз по датчику воды - переключаем вход на поддяжку и измеряем заново
        // PIN_WATER1 подтянут 10кОм, с pulldown будет около 2,5В
        if (adc_raw >= 000) // !!!  ВСЕГДА
        {
            io_conf.intr_type = GPIO_INTR_DISABLE;
            io_conf.mode = GPIO_MODE_INPUT;
            io_conf.pin_bit_mask = BIT64(PIN_WATER1);
            io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
            io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
            gpio_config(&io_conf);

            // ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, PIN_WATER2, &config));

            // включам подтяжку входа ADC, при кз будет около 1,8 в
            gpio_pulldown_en(PIN_WATER2);

            sum = 0;
            for (int i = 0; i < count; i++)
            {
                vTaskDelay(delay);
                ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, PIN_WATER2, &adc_raw));
                sum = sum + adc_raw;
                ESP_LOGD("Water", "ADC%d Channel[%d] Raw Data: %d, mode2 Pullup/Pulldown", ADC_UNIT_1 + 1, PIN_WATER2, adc_raw);
            }
            result.measure.water2 = sum / count;
            ESP_LOGI("Water", "ADC%d Channel[%d] AVG: %d, mode2 Pulldown", ADC_UNIT_1 + 1, PIN_WATER2, sum / count);

        }
    */

    // Water1 - питание
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = BIT64(PIN_WATER1);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    water1_mode = 'P';

    // Water2 - без подтяжек
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = BIT64(PIN_LIGHT) | BIT64(PIN_WATER2);
    io_conf.mode = GPIO_MODE_INPUT;
    // io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    water2_mode = '0';

    result.measure.d_wet_mode = 0;

    vTaskDelay(1);

    // Water2 - с подтяжкой
    if (gpio_get_level(PIN_WATER2) == 1)
    {
        ESP_LOGI(TAG, "Water: %d (%c%c)", gpio_get_level(PIN_WATER2), water1_mode, water2_mode);
        io_conf.pin_bit_mask = BIT64(PIN_WATER2);
        io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        ESP_ERROR_CHECK(gpio_config(&io_conf));
        water2_mode = 'r';
        result.measure.d_wet_mode = 1;
    }

    if (gpio_get_level(PIN_LIGHT) == 1)
    {
        ESP_LOGI(TAG, "Light: %d", gpio_get_level(PIN_LIGHT));
        io_conf.pin_bit_mask = BIT64(PIN_LIGHT);
        io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        ESP_ERROR_CHECK(gpio_config(&io_conf));
    }

    vTaskDelay(1);

    ESP_ERROR_CHECK(gpio_hold_en(PIN_WATER1));
    ESP_ERROR_CHECK(gpio_hold_en(PIN_WATER2));

    ESP_LOGI(TAG, "Light: %d; Water: %d (%c%c); Charge: %d", gpio_get_level(PIN_LIGHT), gpio_get_level(PIN_WATER2), water1_mode, water2_mode, gpio_get_level(PIN_BATT));
}

void stop_charge()
{
    gpio_set_level(PIN_CHARGE_CONTROL, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
};

int get_charge()
{
    return gpio_get_level(PIN_BATT);
};

uint64_t dio_sleep()
{

    // ESP_ERROR_CHECK(gpio_hold_en(PIN_WATER2));

    ESP_LOGI(TAG, "Light: %d; Water: %d (%c%c); Charge: %d", gpio_get_level(PIN_LIGHT), gpio_get_level(PIN_WATER2), water1_mode, water2_mode, gpio_get_level(PIN_BATT));

    uint64_t wake_mask = 0;
    if (gpio_get_level(PIN_LIGHT) == 0)
        wake_mask |= BIT64(PIN_LIGHT);

    if (gpio_get_level(PIN_WATER2) == 0)
        wake_mask |= BIT64(PIN_WATER2);

    if (gpio_get_level(PIN_BATT) == 0)
        wake_mask |= BIT64(PIN_BATT);

    ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(wake_mask, ESP_GPIO_WAKEUP_GPIO_HIGH));

    return wake_mask;
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

void btn_task(void *arg)
{
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    // bit mask of the pins that you want to set
    io_conf.pin_bit_mask = BIT64(PIN_BUTTON_BOOT);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    // configure GPIO with the given settings
    gpio_config(&io_conf);

    int debounce = 0;

    int output = 0;
    const int output_count = 3;

    const int short_count = 4;
    const int long_count = 50;

    vTaskDelay(pdMS_TO_TICKS(500));

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(20));

        if (gpio_get_level(PIN_BUTTON_BOOT) == 0)
        {
            debounce++;
        }
        else
        {
            if (debounce > 0)
            {
                debounce--;
            }

            if (debounce > long_count) // долгое нажатие
            {
                ESP_LOGI("IO", "Button long press!");
                debounce = 0;
            }
            else if (debounce > short_count) // короткое нажатие
            {
                ESP_LOGI("IO", "Button short press!");
                debounce = 0;

                // xEventGroupSetBits(ready_event_group, NEED_WIFI);

                if (++output > output_count)
                {
                    output = 1;
                }

                gpio_config_t io_conf = {};

                switch (output)
                {
                case 1:
                    io_conf.intr_type = GPIO_INTR_DISABLE;
                    io_conf.mode = GPIO_MODE_INPUT;
                    io_conf.pin_bit_mask = BIT64(PIN_WATER2);
                    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
                    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
                    gpio_config(&io_conf);
                    break;
                case 2:
                    io_conf.intr_type = GPIO_INTR_DISABLE;
                    io_conf.mode = GPIO_MODE_INPUT;
                    io_conf.pin_bit_mask = BIT64(PIN_WATER1);
                    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
                    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
                    gpio_config(&io_conf);
                    break;
                case 3:
                    io_conf.intr_type = GPIO_INTR_DISABLE;
                    io_conf.mode = GPIO_MODE_INPUT;
                    io_conf.pin_bit_mask = BIT64(PIN_WATER1);
                    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
                    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
                    gpio_config(&io_conf);
                    break;

                default:
                    break;
                }
            };
        }
    }
};
