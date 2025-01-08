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

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/ledc.h"

static const char *TAG = "DIO";

#define QUEUE_LENGTH 1
#define ITEM_SIZE sizeof(uint64_t)

extern TaskHandle_t xTaskDallas;

QueueHandle_t xQueue = NULL;
uint8_t ucQueueStorageArea[QUEUE_LENGTH * ITEM_SIZE];

QueueHandle_t xQueueLed = NULL;

adc_cali_handle_t cal_handle1;
adc_cali_handle_t cal_handle2;
adc_cali_handle_t l_cal_handle;

adc_continuous_handle_t cont_handle;

adc_oneshot_unit_handle_t l_adc_handle;

// R - внешний 10к
// r - внешний 10к + внутр. нижн. 45к.
// P - питание через верхний ключ
char water1_mode = 'R';

// r - внутр. нижн. 45к.
// 0 - свободный
char water2_mode = '0';

#define CHAN 2
#define FREQ 1000
#define TIME 1000
#define BLOCKSIZE (50 * 4 * CHAN)
#define points (TIME * FREQ / 1000 * CHAN)

uint8_t adcresult[points * sizeof(int)] = {0};
int *adcresult32_1 = (int *)adcresult;
int *adcresult32_2 = (int *)(adcresult + points * sizeof(int));

/*

float u1 = 3.3;
const float uref = 2.5;
float calc1(int adc)
{
    int v = 0;
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cal_handle1, adc, &v));

    float u3 = v / 1000.0;
    float r4 = get_menu_id("range1") / 1000.0;
    if (result.measure.nbbattery > 2)
        u1 = result.measure.nbbattery;
    else if (old_result.measure.nbbattery > 2)
        u1 = old_result.measure.nbbattery;

    u1 = u1 - 0.1;

    float i = u3 / r4;

    return u1 / i - r4;
}

float calc2(int adc)
{
    int v = 0;
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cal_handle1, adc, &v));

    float u3 = v / 1000.0;
    float r1 = get_menu_id("range2.1") / 1000.0;
    float r2 = get_menu_id("range2.2") / 1000.0;
    float r3 = get_menu_id("range2.2") / 1000.0;
    float r4 = get_menu_id("range1") / 1000.0;

    if (result.measure.nbbattery > 2)
        u1 = result.measure.nbbattery;
    else if (old_result.measure.nbbattery > 2)
        u1 = old_result.measure.nbbattery;

    // x = -(R2 (R1 R3 U3 + R1 R4 U3 + R3 R4 (U3 - U1)) + R1 R3 R4 U3)/(U3 (R1 + R2) (R3 + R4)) and U2 = (-R1 R2 R3 U3 - R1 R2 R4 U3 + R2 R3 R4 U1)/(R1 R3 R4 + R2 R3 R4) and U3 (R1 + R2) (R3 + R4)!=0 and R2 R3 R4 U1!=R3 R4 U3 (R1 + R2) and R1 R2 (R1 R3 U3 + R1 R4 U3 - R3 R4 U1)!=0

    // R3=R2
    // U2 = (-R1 R3 U3 - R1 R4 U3 + R3 R4 U1)/(R1 R4 + R3 R4) and x = -(R3 (R1 R3 U3 + 2 R1 R4 U3 + R3 R4 (U3 - U1)))/(U3 (R1 + R3) (R3 + R4)) and R2 = R3 and R4 (R1 + R3)!=0 and R1 U3 (R3 + R4)!=0 and R1 R3 (R3 + R4) (R1 R3 U3 + R1 R4 U3 - R3 R4 U1)!=0
    return -(r2 * (r1 * r3 * u3 + r1 * r4 * u3 + r3 * r4 * (u3 - u1)) + r1 * r3 * r4 * u3) / (u3 * (r1 + r2) * (r3 + r4));
}

float calc3(int adc)
{
    int v = 0;
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cal_handle1, adc, &v));

    float u3 = v / 1000.0;
    float r1 = get_menu_id("range2.1") / 1000.0;
    float r3 = get_menu_id("range2.2") / 1000.0;
    float r4 = get_menu_id("range1") / 1000.0;

    if (result.measure.nbbattery > 2)
        u1 = result.measure.nbbattery;
    else if (old_result.measure.nbbattery > 2)
        u1 = old_result.measure.nbbattery;

    float i = u3 / (1 / (1 / r3 + 1 / r4));

    // x = (R3^2 (U1 - R4 U3 (R1 + R4)) + R3 R4 (2 U1 - R1 R4 U3) + R4^2 U1)/(R3 R4 U3 (R3 + R4)) and R3 R4 U3 (R3 + R4)!=0 and U1!=0

    return u1 / i - r1 - (1 / (1 / r3 + 1 / r4));
}
*/

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    /* xHigherPriorityTaskWoken must be set to pdFALSE before it is used. */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    uint32_t gpio_num = (uint32_t)arg;

    if (gpio_num == PIN_BATT)
    {
        xEventGroupSetBitsFromISR(status_event_group, NOW_CHARGE, &xHigherPriorityTaskWoken);
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

static void oneshot_adc_init()
{
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &l_adc_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(l_adc_handle, PIN_LIGHT, &config));

    adc_cali_curve_fitting_config_t l_cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = PIN_LIGHT,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&l_cali_config, &l_cal_handle));
}

static void continuous_adc_init()
{
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = BLOCKSIZE,
        .conv_frame_size = BLOCKSIZE,
    };

    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &cont_handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = FREQ * CHAN,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    int num = 0;

    adc_pattern[num].atten = ADC_ATTEN_DB_12;
    adc_pattern[num].channel = PIN_WATER1;
    adc_pattern[num].unit = ADC_UNIT_1;
    adc_pattern[num].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    num++;
    adc_pattern[num].atten = ADC_ATTEN_DB_12;
    adc_pattern[num].channel = PIN_WATER2;
    adc_pattern[num].unit = ADC_UNIT_1;
    adc_pattern[num].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    num++;

    dig_cfg.pattern_num = CHAN;

    // ESP_LOGI(TAG, "adc_pattern[%d].atten is :%" PRIx8, 0, adc_pattern[0].atten);
    // ESP_LOGI(TAG, "adc_pattern[%d].channel is :%" PRIx8, i, adc_pattern[i].channel);
    // ESP_LOGI(TAG, "adc_pattern[%d].unit is :%" PRIx8, i, adc_pattern[i].unit);

    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(cont_handle, &dig_cfg));
}

static portMUX_TYPE my_spinlock = portMUX_INITIALIZER_UNLOCKED;

void measure1()
{

    oneshot_adc_init();

    // переключаем каналы измерения попеременно
    uint32_t pulse_time = 10;
    int64_t start_time = esp_timer_get_time();
    int64_t lt = start_time;

    ESP_ERROR_CHECK(gpio_set_direction(PIN_WATER1, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(PIN_WATER2, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(PIN_WATER1, 0));
    ESP_ERROR_CHECK(gpio_set_level(PIN_WATER2, 1));
    vTaskDelay(1);

    for (int i = 0; i < points; i++)
    {
        // taskENTER_CRITICAL(&my_spinlock);

        ESP_ERROR_CHECK(gpio_set_direction(PIN_WATER1, GPIO_MODE_INPUT));
        ESP_ERROR_CHECK(gpio_set_direction(PIN_WATER2, GPIO_MODE_OUTPUT));
        ESP_ERROR_CHECK(gpio_set_level(PIN_WATER2, 0));
        // ESP_ERROR_CHECK(gpio_set_level(PIN_WATER1, 1));
        ESP_ERROR_CHECK(adc_oneshot_read(l_adc_handle, PIN_WATER1, &adcresult32_1[i]));
        // ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, PIN_WATER2, &raw_value));
        // ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan1_handle, raw_value, &adcresult[1][i]));
        // ESP_ERROR_CHECK(gpio_set_level(PIN_WATER, 0));
        // ESP_LOGI(TAG, "Water2: %d", raw_value);
        // vTaskDelay(1000 / portTICK_PERIOD_MS);

        /*
                ESP_ERROR_CHECK(gpio_set_direction(PIN_WATER1, GPIO_MODE_OUTPUT));
                ESP_ERROR_CHECK(gpio_set_direction(PIN_WATER2, GPIO_MODE_OUTPUT));
                ESP_ERROR_CHECK(gpio_set_level(PIN_WATER1, 0));
                ESP_ERROR_CHECK(gpio_set_level(PIN_WATER2, 1));
                esp_rom_delay_us(pulse_time);
                ESP_ERROR_CHECK(gpio_set_direction(PIN_WATER1, GPIO_MODE_OUTPUT));
                ESP_ERROR_CHECK(gpio_set_direction(PIN_WATER2, GPIO_MODE_OUTPUT));
                ESP_ERROR_CHECK(gpio_set_level(PIN_WATER1, 1));
                ESP_ERROR_CHECK(gpio_set_level(PIN_WATER2, 0));
                esp_rom_delay_us(pulse_time);


                        ESP_ERROR_CHECK(gpio_set_direction(PIN_WATER1, GPIO_MODE_OUTPUT));
                        ESP_ERROR_CHECK(gpio_set_direction(PIN_WATER2, GPIO_MODE_OUTPUT));
                        ESP_ERROR_CHECK(gpio_set_level(PIN_WATER1, 1));
                        ESP_ERROR_CHECK(gpio_set_level(PIN_WATER2, 0));
                        esp_rom_delay_us(pulse_time);

                        ESP_ERROR_CHECK(gpio_set_direction(PIN_WATER1, GPIO_MODE_OUTPUT));
                        ESP_ERROR_CHECK(gpio_set_direction(PIN_WATER2, GPIO_MODE_INPUT));
                        ESP_ERROR_CHECK(gpio_set_level(PIN_WATER1, 0));
                        // ESP_ERROR_CHECK(adc_oneshot_get_calibrated_result(adc1_handle, adc1_cali_chan0_handle, PIN_WATER1, &adcresult[0][i]));
                        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, PIN_WATER2, &adcresult[1][i]));
                */
        // taskEXIT_CRITICAL(&my_spinlock);

        if (i == 0)
        {
            // pulse_time = (esp_timer_get_time() - start_time) / 2;

            // < 50 mV (~ > 20mA )
            if (adcresult32_1[i] < 50)
            {
                continue;
            }
        }
        else
        {
            // < 50 mV (~ > 20mA )
            if (adcresult32_1[i] < 50)
            {
                ESP_LOGW(TAG, "КЗ");
                break;
            }
        }
    }

    ESP_ERROR_CHECK(gpio_set_level(PIN_WATER1, 0));
    ESP_ERROR_CHECK(gpio_set_level(PIN_WATER2, 0));

    ESP_LOGI(TAG, "Time: %lld, Pulse: %lu ", (esp_timer_get_time() - start_time), pulse_time);

    int avg1 = 0;
    int avg2 = 0;
    for (int i = 0; i < points; i++)
    {
        // printf("%4d/%4d ", adcresult[0][i], adcresult[1][i]);
        avg1 += adcresult32_1[i];
        avg2 += adcresult32_2[i];
    }
    // printf("\n");

    ESP_LOGI(TAG, "RAW AVG Water1: %d", avg1 / points);
    ESP_LOGI(TAG, "RAW AVG Water2: %d", avg2 / points);

    avg1 = 0;
    avg2 = 0;
    int adc_raw;
    int v_power = 3300;
    int v_adc = 0;
    int r_ch1 = 10000;
    int r_ch2 = 100000;
    for (int i = 0; i < points; i++)
    {
        adc_raw = adcresult32_1[i];
        // ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw, &v_adc));
        adcresult32_1[i] = r_ch1 * v_adc / (v_power - v_adc);

        adc_raw = adcresult32_2[i];
        // ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan1_handle, adc_raw, &v_adc));
        adcresult32_2[i] = r_ch2 * v_adc / (v_power - v_adc);

        avg1 += adcresult32_1[i];
        avg2 += adcresult32_2[i];
    }

    for (int i = 0; i < points; i++)
    {
        printf("%d ", adcresult32_1[i]);
    }
    printf("\n");
    for (int i = 0; i < points; i++)
    {
        printf("%d ", adcresult32_2[i]);
    }
    printf("\n");

    ESP_LOGI(TAG, "Volt AVG Water1: %d", avg1 / points);
    ESP_LOGI(TAG, "Volt AVG Water2: %d", avg2 / points);
};

void measure2()
{
    int64_t start_time = esp_timer_get_time();
    for (int i = 0; i < points; i++)
    {
        ESP_ERROR_CHECK(gpio_set_direction(PIN_WATER1, GPIO_MODE_OUTPUT));
        ESP_ERROR_CHECK(gpio_set_direction(PIN_WATER2, GPIO_MODE_INPUT));
        ESP_ERROR_CHECK(gpio_set_level(PIN_WATER1, 1));

        ESP_ERROR_CHECK(adc_oneshot_read(l_adc_handle, PIN_WATER2, &adcresult32_2[i]));
    }

    ESP_LOGI(TAG, "Time: %lld", (esp_timer_get_time() - start_time));

    int avg1 = 0;
    int avg2 = 0;
    for (int i = 0; i < points; i++)
    {
        // printf("%4d/%4d ", adcresult[0][i], adcresult[1][i]);
        avg1 += adcresult32_1[i];
        avg2 += adcresult32_2[i];
    }
    // printf("\n");

    ESP_LOGI(TAG, "RAW AVG Water1: %d", avg1 / points);
    ESP_LOGI(TAG, "RAW AVG Water2: %d", avg2 / points);

    avg1 = 0;
    avg2 = 0;
    int adc_raw;
    for (int i = 0; i < points; i++)
    {
        adc_raw = adcresult32_1[i];
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(l_cal_handle, adc_raw, &adcresult32_1[i]));
        adc_raw = adcresult32_2[i];
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(l_cal_handle, adc_raw, &adcresult32_2[i]));
        printf("%4d/%4d ", adcresult32_1[i], adcresult32_2[i]);
        avg1 += adcresult32_1[i];
        avg2 += adcresult32_2[i];
    }
    printf("\n");

    ESP_LOGI(TAG, "Volt AVG Water1: %d", avg1 / points);
    ESP_LOGI(TAG, "Volt AVG Water2: %d", avg2 / points);
};

void measure3()
{
    int64_t start_time = esp_timer_get_time();
    for (int i = 0; i < points; i++)
    {
        ESP_ERROR_CHECK(gpio_set_direction(PIN_WATER1, GPIO_MODE_INPUT));
        ESP_ERROR_CHECK(adc_oneshot_read(l_adc_handle, PIN_WATER1, &adcresult32_1[i]));
        ESP_ERROR_CHECK(gpio_set_direction(PIN_WATER2, GPIO_MODE_INPUT));
        ESP_ERROR_CHECK(adc_oneshot_read(l_adc_handle, PIN_WATER2, &adcresult32_2[i]));
    }

    ESP_LOGI(TAG, "Time: %lld", (esp_timer_get_time() - start_time));

    int avg1 = 0;
    int avg2 = 0;
    for (int i = 0; i < points; i++)
    {
        // printf("%4d/%4d ", adcresult[0][i], adcresult[1][i]);
        avg1 += adcresult32_1[i];
        avg2 += adcresult32_2[i];
    }
    // printf("\n");

    ESP_LOGI(TAG, "RAW AVG Water1: %d", avg1 / points);
    ESP_LOGI(TAG, "RAW AVG Water2: %d", avg2 / points);

    avg1 = 0;
    avg2 = 0;
    int adc_raw;
    for (int i = 0; i < points; i++)
    {
        adc_raw = adcresult32_1[i];
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(l_cal_handle, adc_raw, &adcresult32_1[i]));
        adc_raw = adcresult32_2[i];
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(l_cal_handle, adc_raw, &adcresult32_2[i]));
        printf("%4d/%4d ", adcresult32_1[i], adcresult32_2[i]);
        avg1 += adcresult32_1[i];
        avg2 += adcresult32_2[i];
    }
    printf("\n");

    ESP_LOGI(TAG, "Volt AVG Water1: %d", avg1 / points);
    ESP_LOGI(TAG, "Volt AVG Water2: %d", avg2 / points);
};

void cont_prepare()
{
    continuous_adc_init();

    // ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
    adc_cali_curve_fitting_config_t cali_config1 = {
        .unit_id = ADC_UNIT_1,
        .chan = PIN_WATER1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config1, &cal_handle1));

    adc_cali_curve_fitting_config_t cali_config2 = {
        .unit_id = ADC_UNIT_1,
        .chan = PIN_WATER2,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config2, &cal_handle2));
}

void cont_measure1(bool printdata)
{

    uint32_t ret_num = 0;
    uint32_t light = 0;
    uint32_t light_cnt = 0;
    esp_err_t ret = 0;
    uint8_t *buf = adcresult;
    int len = 0;
    uint8_t *switch_pos = 0;

    int water_max = 0;
    int water_max_cnt = 0;

    int dir = 0;

    gpio_pulldown_dis(PIN_WATER1);
    gpio_pullup_dis(PIN_WATER1);
    gpio_pulldown_dis(PIN_WATER2);
    gpio_pullup_dis(PIN_WATER2);

    ESP_ERROR_CHECK(adc_continuous_start(cont_handle));

    while (len < sizeof(adcresult))
    {
        if (dir++ % 2 == 0)
        {
            ESP_ERROR_CHECK(gpio_set_direction(PIN_WATER1, GPIO_MODE_OUTPUT));
            ESP_ERROR_CHECK(gpio_set_direction(PIN_WATER2, GPIO_MODE_INPUT)); // 10k
            ESP_ERROR_CHECK(gpio_set_level(PIN_WATER1, 0));
        }
        else
        {
            ESP_ERROR_CHECK(gpio_set_direction(PIN_WATER1, GPIO_MODE_INPUT)); // 10k
            ESP_ERROR_CHECK(gpio_set_direction(PIN_WATER2, GPIO_MODE_OUTPUT));
            ESP_ERROR_CHECK(gpio_set_level(PIN_WATER2, 0));
        }

        ret = adc_continuous_read(cont_handle, buf, sizeof(adcresult), &ret_num, ADC_MAX_DELAY);
        // printf("adc ret: %lu\n", ret_num);
        len = len + ret_num;
        buf += ret_num;
    }

    /*

        ESP_ERROR_CHECK(gpio_set_direction(PIN_WATER1, GPIO_MODE_INPUT)); // 10k
        ESP_ERROR_CHECK(gpio_set_direction(PIN_WATER2, GPIO_MODE_OUTPUT));
        ESP_ERROR_CHECK(gpio_set_level(PIN_WATER2, 0));

        gpio_pulldown_dis(PIN_WATER1);
        gpio_pullup_dis(PIN_WATER1);
        gpio_pulldown_dis(PIN_WATER2);
        gpio_pullup_dis(PIN_WATER2);

        while (len < sizeof(adcresult))
        {
            ret = adc_continuous_read(cont_handle, buf, sizeof(adcresult), &ret_num, ADC_MAX_DELAY);
            // printf("adc ret: %lu\n", ret_num);
            len = len + ret_num;
            buf += ret_num;
        }
    */
    ESP_ERROR_CHECK(adc_continuous_stop(cont_handle));

    ESP_ERROR_CHECK(gpio_set_direction(PIN_WATER1, GPIO_MODE_INPUT)); // 10k
    ESP_ERROR_CHECK(gpio_set_direction(PIN_WATER2, GPIO_MODE_INPUT)); // 10k

    int v_power = 3300;
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "latest ret is %x, ret_num is %" PRIu32 " bytes", ret, ret_num);

        dir = 0;
        water_max = 0;
        water_max_cnt = 0;
        for (int i = 0; i < len; i += SOC_ADC_DIGI_RESULT_BYTES)
        {
            adc_digi_output_data_t *p = (adc_digi_output_data_t *)&adcresult[i];
            uint32_t chan_num = p->type2.channel;
            uint32_t data = p->type2.data;

            int v = 0;

            if (chan_num == PIN_WATER2)
            {
                ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cal_handle2, data, &v));
                if (printdata)
                    printf("%d ", v);

                if ((i / (SOC_ADC_DIGI_RESULT_BYTES * 2)) % 50 == 49)
                {
                    if (dir++ % 2 == 0 && dir >= 10)
                    {
                        water_max += v;
                        water_max_cnt++;
                    }
                }
            }
        }
        if (printdata)
            printf("\n");

        int w2_max = 0;
        float w2 = 0;
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cal_handle2, 4095, &w2_max));
        w2 = (w2_max - (water_max / water_max_cnt)) * 100.0 / w2_max;
        // result.measure.water2_last = get_menu_id("r1.2") * water_max / water_max_cnt / (v_power - water_max / water_max_cnt);
        ESP_LOGI("Water2", "ADC chan %d: max: %.1f%% (%d мВ) - %d Ом", PIN_WATER2, w2, water_max / water_max_cnt, get_menu_id("r1.2") * water_max / water_max_cnt / (v_power - water_max / water_max_cnt));

        vTaskDelay(1);

        dir = 0;
        water_max = 0;
        water_max_cnt = 0;
        for (int i = 0; i < len; i += SOC_ADC_DIGI_RESULT_BYTES)
        {
            adc_digi_output_data_t *p = (adc_digi_output_data_t *)&adcresult[i];
            uint32_t chan_num = p->type2.channel;
            uint32_t data = p->type2.data;

            int v = 0;

            if (chan_num == PIN_WATER1)
            {
                ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cal_handle1, data, &v));

                if (printdata)
                    printf("%d ", v);

                if ((i / (SOC_ADC_DIGI_RESULT_BYTES * 2)) % 50 == 49)
                {
                    if (dir++ % 2 == 1 && dir >= 10)
                    {
                        water_max += v;
                        water_max_cnt++;
                    }
                }
            }
        }
        if (printdata)
            printf("\n");

        int w1_max = 0;
        float w1 = 0;
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cal_handle1, 4095, &w1_max));
        w1 = (w1_max - (water_max / water_max_cnt)) * 100.0 / w1_max;

        // result.measure.water1_last = get_menu_id("r1.1") * water_max / water_max_cnt / (v_power - water_max / water_max_cnt);
        ESP_LOGI("Water1", "ADC chan %d: max: %.1f%% (%d мВ) - %d Ом", PIN_WATER1, w1, water_max / water_max_cnt, get_menu_id("r1.1") * water_max / water_max_cnt / (v_power - water_max / water_max_cnt));
        result.measure.water = (w1 + w2) / 2.0;

        /*
                for (int i = 0; i < points * 2; i++)
                {
                    adc_digi_output_data_t *p = (adc_digi_output_data_t *)&adcresult32_1[i];
                    uint32_t chan_num = p->type2.channel;
                    uint32_t data = p->type2.data;

                    int v = 0;
                    int v_power = 3300;
                    int r_ch1 = 10000;
                    int r_ch2 = 100000;

                    if (chan_num == PIN_WATER2)
                    {
                        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cal_handle2, data, &v));

                        printf("%d ", r_ch1 * v / (v_power - v));

                        water += v;
                        water_cnt++;
                    }
                }
                printf("\n");

                ESP_LOGI("Water2", "ADC chan %d: %ld мВ - %.0f кОм", PIN_WATER2, water / water_cnt, result.measure.water);
        */
        /*
                for (int i = 0; i < points; i++)
                {
                    printf("%d ", adcresult[0][i]);
                }
                printf("\n");
                for (int i = 0; i < points; i++)
                {
                    //printf("%d ", adcresult[1][i]);
                }
                printf("\n");
        */
    }
}

void dio_init()
{
    static StaticQueue_t xStaticQueue;
    xQueue = xQueueCreateStatic(QUEUE_LENGTH, ITEM_SIZE, ucQueueStorageArea, &xStaticQueue);
    configASSERT(xQueue);

    xQueueLed = xQueueCreate(2, sizeof(led_task_data_t));

    ESP_ERROR_CHECK(gpio_hold_dis(PIN_LIGHT));
    ESP_ERROR_CHECK(gpio_hold_dis(PIN_WATER3));

    // install gpio isr service
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));

    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = BIT64(PIN_BATT);
    // set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_BATT, gpio_isr_handler, (void *)PIN_BATT));

    cont_prepare();
    cont_measure1(true);

    oneshot_adc_init();

    int l = 0;
    int l_max = 0;
    ESP_ERROR_CHECK(adc_oneshot_get_calibrated_result(l_adc_handle, l_cal_handle, PIN_LIGHT, &l));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(l_cal_handle, 4095, &l_max));

    result.measure.light = l * 100.0 / l_max;

    ESP_LOGI(TAG, "Light: %.1f%% (%d mV)", result.measure.light, l);

    /*
        ESP_ERROR_CHECK(gpio_pulldown_dis(PIN_WATER1));
        ESP_ERROR_CHECK(gpio_pullup_dis(PIN_WATER1));
        ESP_ERROR_CHECK(gpio_pulldown_dis(PIN_WATER2));
        ESP_ERROR_CHECK(gpio_pullup_dis(PIN_WATER2));
    */
    // measure1();

    // measure1();

    // Water1, Water2 - питание через резистор 10к
    water1_mode = 'R';

    // Water3 - без подтяжек
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = BIT64(PIN_LIGHT) | BIT64(PIN_WATER3);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    water2_mode = '0';
    result.measure.d_wet_mode = 0;

    vTaskDelay(1);

    // Water2 - с подтяжкой
    if (gpio_get_level(PIN_WATER3) == 1)
    {
        gpio_pulldown_en(PIN_WATER3);
        water2_mode = 'r';
        result.measure.d_wet_mode = 1;
        vTaskDelay(1);
        if (gpio_get_level(PIN_WATER3) == 1)
        {
            // нет смысла держать подтяжку. Экономим энергию
            gpio_pulldown_dis(PIN_WATER3);
            water2_mode = '0';
            result.measure.d_wet_mode = 0;
        }
    }
    // Light с подтяжкой
    /*
    if (gpio_get_level(PIN_LIGHT) == 1)
    {
        gpio_pulldown_en(PIN_LIGHT);
        vTaskDelay(1);
        if (gpio_get_level(PIN_LIGHT) == 1)
        {
            // нет смысла держать подтяжку. Экономим энергию
            gpio_pulldown_dis(PIN_LIGHT);
        }
    }
    */
    vTaskDelay(1);

    ESP_LOGI(TAG, "Light: %d; Water: %d (%c%c); Charge: %d", gpio_get_level(PIN_LIGHT), gpio_get_level(PIN_WATER3), water1_mode, water2_mode, gpio_get_level(PIN_BATT));
}

int get_charge()
{
    if (gpio_get_level(PIN_BATT))
    {
        return 1;
    }
    else
    {
        return 0;
    }
};

uint64_t dio_sleep()
{
    ESP_LOGI(TAG, "Light: %d; Water: %d (%c%c); Charge: %d", gpio_get_level(PIN_LIGHT), gpio_get_level(PIN_WATER3), water1_mode, water2_mode, gpio_get_level(PIN_BATT));

    uint64_t wake_mask = 0;
    if (gpio_get_level(PIN_LIGHT) == 0)
    {
        wake_mask |= BIT64(PIN_LIGHT);
        ESP_ERROR_CHECK(gpio_hold_en(PIN_LIGHT));
    }

    if (gpio_get_level(PIN_WATER3) == 0)
    {
        wake_mask |= BIT64(PIN_WATER3);
        ESP_ERROR_CHECK(gpio_hold_en(PIN_WATER3));
    }

    if (gpio_get_level(PIN_BATT) == 0)
    {
        wake_mask |= BIT64(PIN_BATT);
    }

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
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    vTaskDelay(pdMS_TO_TICKS(20));

    int debounce = 0;

    int output = 0;
    int output_count = 100;

    const int short_count = 4;
    const int long_count = 50;

    // vTaskDelay(pdMS_TO_TICKS(500));

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(20));

        if (gpio_get_level(PIN_BUTTON_BOOT) == 0)
        {
            debounce++;
            // printf("%d ", debounce);
        }
        else
        {
            if (debounce > 0)
            {
                debounce--;
            }

            if (debounce > long_count) // долгое нажатие
            {
                ESP_LOGI("IO", "Button long press! %d", output + 1);
                debounce = 0;

                // gpio_config_t io_conf = {};
                /*
                                if (++output > output_count)
                                {
                                    output = 1;
                                }

                                switch (output)
                                {
                                case 1:
                                    io_conf.intr_type = GPIO_INTR_DISABLE;
                                    io_conf.mode = GPIO_MODE_INPUT;
                                    io_conf.pin_bit_mask = BIT64(PIN_WATER2_LOAD);
                                    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
                                    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
                                    gpio_config(&io_conf);
                                    gpio_pulldown_en(PIN_WATER2_LOAD);
                                    gpio_pullup_dis(PIN_WATER2_LOAD);
                                    // gpio_set_level(PIN_WATER2_LOAD, 0);
                                    measure2();
                                    break;
                                case 2:
                                    io_conf.intr_type = GPIO_INTR_DISABLE;
                                    io_conf.mode = GPIO_MODE_INPUT;
                                    io_conf.pin_bit_mask = BIT64(PIN_WATER2_LOAD);
                                    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
                                    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
                                    gpio_config(&io_conf);
                                    gpio_pulldown_dis(PIN_WATER2_LOAD);
                                    gpio_pullup_en(PIN_WATER2_LOAD);
                                    // gpio_set_level(PIN_WATER2_LOAD, 1);
                                    measure2();
                                    break;
                                case 3:
                                    gpio_reset_pin(PIN_WATER2_LOAD);
                                    gpio_pulldown_dis(PIN_WATER2_LOAD);
                                    gpio_pullup_dis(PIN_WATER2_LOAD);
                                    measure2();
                                    break;
                                case 4:
                                    io_conf.intr_type = GPIO_INTR_DISABLE;
                                    io_conf.mode = GPIO_MODE_INPUT;
                                    io_conf.pin_bit_mask = BIT64(PIN_WATER2_LOAD);
                                    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
                                    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
                                    gpio_config(&io_conf);
                                    gpio_pulldown_dis(PIN_WATER2);
                                    gpio_pullup_dis(PIN_WATER2);
                                    gpio_pulldown_en(PIN_WATER2_LOAD);
                                    gpio_pullup_dis(PIN_WATER2_LOAD);
                                    measure2();
                                    break;
                                default:
                                    output_count = output - 1;
                                    break;
                                }
                                */
            }
            else if (debounce >= short_count) // короткое нажатие
            {
                ESP_LOGI("IO", "Button short press!");
                debounce = 0;

                xTaskNotifyGive(xTaskDallas);
                xTaskNotify(xTaskI2C, (NOTYFY_SENSOR_TH) | (NOTYFY_SENSOR_MAGACC), eSetBits);
                cont_measure1(false);

                xTaskNotifyGive(xHandleWifi); // включаем WiFi;
            };
        }
    }
};

int getResult_Data(char *line, int data_pos)
{
    const char *header = {"id,U1,U2\n"};
    char *pos = line;
    int l = 0;

    int v1 = 0;
    int v2 = 0;

    if (data_pos == 0)
    {
        strcpy(line, header);
        l = strlen(header);
        pos = line + l;
    }

    int i = data_pos * CHAN * SOC_ADC_DIGI_RESULT_BYTES;

    if (i >= sizeof(adcresult))
        return 0;

    adc_digi_output_data_t *p = (adc_digi_output_data_t *)&adcresult[i];
    uint32_t chan_num = p->type2.channel;
    uint32_t data = p->type2.data;

    if (chan_num == PIN_WATER1)
    {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cal_handle1, data, &v1));
    }
    else
        ESP_LOGE(TAG, "channel != PIN_WATER1");

    p++;
    chan_num = p->type2.channel;
    data = p->type2.data;

    if (chan_num == PIN_WATER2)
    {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cal_handle2, data, &v2));
    }
    else
        ESP_LOGE(TAG, "channel != PIN_WATER2");

    l += sprintf(pos, "%d,%d,%d\n", data_pos, v1, v2);

    return l;
}
