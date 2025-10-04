#include "main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "freertos/queue.h"
#include "driver/gptimer.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/ledc.h"

#include <math.h>

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

#define CHAN 2
#define FREQ 1000 // Hz
#define TIME 1000 // total measure time (ms)
#define BLOCKSIZE (50 * SOC_ADC_DIGI_RESULT_BYTES * CHAN)
#define points (TIME * FREQ / 1000 * CHAN)

DMA_ATTR uint8_t adcresult[points * sizeof(int)] = {0};
int *adcresult32_1 = (int *)adcresult;
int *adcresult32_2 = (int *)(adcresult + points * sizeof(int));

const TickType_t pulldown_switch_time = 10;

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
    else if (gpio_num == PIN_INT_ACC)
    {
        xTaskNotifyFromISR(xTaskI2C, NOTYFY_SENSOR_SET_MAGACC_INT, eSetBits, &xHigherPriorityTaskWoken);
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

static void light_adc_init()
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

static void water_continuous_adc_init()
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

    dig_cfg.pattern_num = CHAN;

    // ESP_LOGI(TAG, "adc_pattern[%d].atten is :%" PRIx8, 0, adc_pattern[0].atten);
    // ESP_LOGI(TAG, "adc_pattern[%d].channel is :%" PRIx8, i, adc_pattern[i].channel);
    // ESP_LOGI(TAG, "adc_pattern[%d].unit is :%" PRIx8, i, adc_pattern[i].unit);

    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(cont_handle, &dig_cfg));
}

void water_cont_prepare()
{
    water_continuous_adc_init();

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

void water_cont_measure(bool printdata)
{
    uint32_t ret_num = 0;
    esp_err_t ret = 0;
    uint8_t *buf = adcresult;
    int len = 0;

    int water_max = 0;
    int water_max_cnt = 0;
    int water_corr = 0;
    int water_corr_cnt = 0;

    int dir = 0;

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
        if (ret != ESP_OK)
            break;

        len = len + ret_num;
        buf += ret_num;
    }

    ESP_ERROR_CHECK(adc_continuous_stop(cont_handle));

    ESP_ERROR_CHECK(gpio_set_direction(PIN_WATER1, GPIO_MODE_INPUT)); // 10k
    ESP_ERROR_CHECK(gpio_set_direction(PIN_WATER2, GPIO_MODE_INPUT)); // 10k

    if (len > 0)
    {
        ESP_LOGI(TAG, "latest ret is %x, ret_num is %" PRIu32 " bytes", ret, ret_num);

        water_max = 0;
        water_max_cnt = 0;
        water_corr = 0;
        water_corr_cnt = 0;
        int cnt = 0; // счетчик пар (1000 всего)
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

                cnt++;

                if (cnt % 100 == 49) // берем последние данные перед реверсом измерения
                {
                    if (cnt > (points / CHAN) / 2) // берем данные только со второй половины измерений
                    {
                        water_max += v;
                        water_max_cnt++;

                        // Коррекция на сопротивление открытого нижнего ключа
                        p--; // 1-ый канал
                        v = 0;
                        if (p->type2.channel == PIN_WATER1)
                        {
                            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cal_handle1, p->type2.data, &v));
                            water_corr += v;
                            water_corr_cnt++;
                        }
                    }
                }
            }
        }
        if (printdata)
            printf("\n");

        int w2_max = 0;
        float w2 = 0;
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cal_handle2, 4095, &w2_max));
        w2 = (w2_max - water_max / water_max_cnt) * 100.0 / (w2_max - water_corr / water_corr_cnt);

        int batt = result.measure.nbbattery;
        if (batt == 0)
            batt = history[(history_pos - 1) % HISTORY_SIZE].measure.nbbattery; // old_result.measure.nbbattery;
        if (batt == 0)
            batt = 3300;

        // result.measure.water2_last = get_menu_id("r1.2") * water_max / water_max_cnt / (v_power - water_max / water_max_cnt);
        ESP_LOGI("Water2", "ADC chan %d: max: %.1f%% (%d мВ), corr %d, ~ %d Ом", PIN_WATER2, w2, water_max / water_max_cnt, water_corr / water_corr_cnt, get_menu_val_by_id("r1.2") * (water_max / water_max_cnt - water_corr / water_corr_cnt) / (batt - water_max / water_max_cnt));

        vTaskDelay(1);

        cnt = 0;
        water_corr = 0;
        water_corr_cnt = 0;
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

                cnt++;

                if (cnt % 100 == 99) // берем последние данные перед реверсом измерения
                {
                    if (cnt > (points / CHAN) / 2) // берем данные только со второй половины измерений
                    {
                        water_max += v;
                        water_max_cnt++;

                        // Коррекция на сопротивление открытого нижнего ключа
                        p++; // 2-ой канал
                        v = 0;
                        if (p->type2.channel == PIN_WATER2)
                        {
                            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cal_handle2, p->type2.data, &v));
                            water_corr += v;
                            water_corr_cnt++;
                        }
                    }
                }
            }
        }
        if (printdata)
            printf("\n");

        int w1_max = 0;
        float w1 = 0;
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cal_handle1, 4095, &w1_max));
        w1 = (w1_max - (water_max / water_max_cnt)) * 100.0 / (w1_max - (water_corr / water_corr_cnt));

        // result.measure.water1_last = get_menu_id("r1.1") * water_max / water_max_cnt / (v_power - water_max / water_max_cnt);
        ESP_LOGI("Water1", "ADC chan %d: max: %.1f%% (%d мВ), corr %d, ~ %d Ом", PIN_WATER1, w1, water_max / water_max_cnt, water_corr / water_corr_cnt, get_menu_val_by_id("r1.1") * (water_max / water_max_cnt - water_corr / water_corr_cnt) / (batt - water_max / water_max_cnt));

        result.measure.water = (w1 + w2) / 2.0;

        // если большая разбежка в измерениях - верим только PIN_WATER1
        if (fabs(w1 - w2) > 25.0)
        {
            result.measure.water = w1;
        }
    }

    result.ttime = time(0);
}

void light_measure(int test_count)
{
    const int reads = 3;
    float top = get_menu_val_by_id("lightrang");

    int l_max = 0;
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(l_cal_handle, 4095, &l_max));

    int light2 = 0;
    gpio_pulldown_en(PIN_LIGHT);

    for (int i = 0; i < test_count; i++)
    {
        // ESP_ERROR_CHECK(adc_oneshot_get_calibrated_result(l_adc_handle, l_cal_handle, PIN_LIGHT, &l));
        adc_oneshot_get_calibrated_result(l_adc_handle, l_cal_handle, PIN_LIGHT, &light2);
        ESP_LOGI(TAG, "%2d Light1 R: %d mV", i, light2);
    }

    vTaskDelay(1);

    int sum = 0;
    int cnt = 0;
    for (int i = 0; i < reads; i++)
    {
        esp_err_t e = adc_oneshot_get_calibrated_result(l_adc_handle, l_cal_handle, PIN_LIGHT, &light2);
        if (e == ESP_OK && i > 0) // пропускаем первое измерение
        {
            sum += light2;
            cnt++;
        }
    };

    result.measure.light = sum / cnt * (100.0 - top) / l_max;

    int l = 0;
    gpio_pulldown_dis(PIN_LIGHT);

    for (int i = 0; i < test_count; i++)
    {
        // ESP_ERROR_CHECK(adc_oneshot_get_calibrated_result(l_adc_handle, l_cal_handle, PIN_LIGHT, &l));
        adc_oneshot_get_calibrated_result(l_adc_handle, l_cal_handle, PIN_LIGHT, &l);
        ESP_LOGI(TAG, "%2d Light0: %d mV", i, l);
        // vTaskDelay(1);
    }

    vTaskDelay(pulldown_switch_time);

    sum = 0;
    cnt = 0;
    for (int i = 0; i < reads; i++)
    {
        esp_err_t e = adc_oneshot_get_calibrated_result(l_adc_handle, l_cal_handle, PIN_LIGHT, &l);
        if (e == ESP_OK && i > 0) // пропускаем первое измерение
        {
            sum += l;
            cnt++;
        }
    };

    result.measure.light += sum / cnt * top / l_max;

    ESP_LOGI(TAG, "Light: %.1f%% (%d / %d mV, max: %d mV)", result.measure.light, l, light2, l_max);
}

uint64_t dio_init()
{
    static StaticQueue_t xStaticQueue;
    xQueue = xQueueCreateStatic(QUEUE_LENGTH, ITEM_SIZE, ucQueueStorageArea, &xStaticQueue);
    configASSERT(xQueue);

    // install gpio isr service
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));

    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = BIT64(PIN_BATT) | BIT64(PIN_INT_ACC);
    // set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_BATT, gpio_isr_handler, (void *)PIN_BATT));

    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_INT_ACC, gpio_isr_handler, (void *)PIN_INT_ACC));

    gpio_hold_dis(PIN_LIGHT);
    gpio_hold_dis(PIN_WATER3);

    water_cont_prepare();
    water_cont_measure(false);

    light_adc_init();
    light_measure(0);

    // Water3 - без подтяжек
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = BIT64(PIN_LIGHT) | BIT64(PIN_WATER3);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    result.measure.d_wet_mode = 0;
    result.measure.d_light_mode = 0;

    vTaskDelay(1);
    uint64_t wake_mask = 0;

    // Water - с подтяжкой
    if (gpio_get_level(PIN_WATER3) == 1)
    {
        gpio_pulldown_en(PIN_WATER3);
        result.measure.d_wet_mode = 1;
        vTaskDelay(1);
        if (gpio_get_level(PIN_WATER3) == 1)
        {
            // нет смысла держать нижнюю подтяжку.
            gpio_pulldown_dis(PIN_WATER3);
            // включаем верхнюю
            gpio_pullup_en(PIN_WATER3);
            result.measure.d_wet_mode = 0;
        }
        else
        {
            wake_mask |= BIT64(PIN_WATER3);
        }
    }
    else
    {
        wake_mask |= BIT64(PIN_WATER3);
    }

    // Light - с подтяжкой
    if (gpio_get_level(PIN_LIGHT) == 1)
    {
        gpio_pulldown_en(PIN_LIGHT);
        result.measure.d_light_mode = 1;
        vTaskDelay(1);
        if (gpio_get_level(PIN_LIGHT) == 1)
        {
            // нет смысла держать подтяжку. Экономим энергию
            gpio_pulldown_dis(PIN_LIGHT);
            result.measure.d_light_mode = 0;
        }
        else
        {
            wake_mask |= BIT64(PIN_LIGHT);
        }
    }
    else
    {
        wake_mask |= BIT64(PIN_LIGHT);
    }

    vTaskDelay(1);

    ESP_LOGI(TAG, "Light: %d%c(%d); Water: %d%c(%d); Charge: %d(%d); INT ACC: %d(%d)", gpio_get_level(PIN_LIGHT), (result.measure.d_light_mode == 1) ? '+' : ' ', ((wake_mask & BIT64(PIN_LIGHT)) != 0), gpio_get_level(PIN_WATER3), (result.measure.d_wet_mode == 1) ? '+' : ' ', ((wake_mask & BIT64(PIN_WATER3)) != 0), gpio_get_level(PIN_BATT), ((wake_mask & BIT64(PIN_BATT)) != 0), gpio_get_level(PIN_INT_ACC), ((wake_mask & BIT64(PIN_INT_ACC)) != 0));

    return wake_mask;
}

int get_charge()
{
    return gpio_get_level(PIN_BATT);
};

uint64_t dio_sleep(uint64_t wake_mask)
{
    ESP_LOGI(TAG, "Light: %d%c(%d); Water: %d%c(%d); Charge: %d(%d); INT ACC: %d(%d)", gpio_get_level(PIN_LIGHT), (result.measure.d_light_mode == 1) ? '+' : ' ', ((wake_mask & BIT64(PIN_LIGHT)) != 0), gpio_get_level(PIN_WATER3), (result.measure.d_wet_mode == 1) ? '+' : ' ', ((wake_mask & BIT64(PIN_WATER3)) != 0), gpio_get_level(PIN_BATT), ((wake_mask & BIT64(PIN_BATT)) != 0), gpio_get_level(PIN_INT_ACC), ((wake_mask & BIT64(PIN_INT_ACC)) != 0));

    if (wake_mask & BIT64(PIN_LIGHT))
    {
        ESP_ERROR_CHECK(gpio_hold_en(PIN_LIGHT));
    }

    if (wake_mask & BIT64(PIN_WATER3))
    {
        ESP_ERROR_CHECK(gpio_hold_en(PIN_WATER3));
    }

    if (gpio_get_level(PIN_BATT) == 0)
    {
        wake_mask |= BIT64(PIN_BATT);
        ESP_ERROR_CHECK(gpio_hold_en(PIN_BATT));
    }

    if (gpio_get_level(PIN_INT_ACC) == 0)
    {
        wake_mask |= BIT64(PIN_INT_ACC);
        ESP_ERROR_CHECK(gpio_hold_en(PIN_INT_ACC));
    }
    gpio_deep_sleep_hold_en();
    ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(wake_mask, ESP_GPIO_WAKEUP_GPIO_HIGH));

    return wake_mask;
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

    const int short_count = 4;
    const int long_count = 50;

    // время сна в мин
    int sleeptime = get_menu_val_by_id("time");

    // время ожидания
    int wait = get_menu_val_by_id("waitnb");

    int64_t start_time = esp_timer_get_time();

    // while ((esp_timer_get_time() - start_time) < ticks_to_wait * portTICK_PERIOD_MS * 1000)

    // vTaskDelay(pdMS_TO_TICKS(500));

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(20));

        if (wait == 1000) // демонстрационный режим, без сна
        {
            if ((esp_timer_get_time() - start_time) > (sleeptime * 60LL * 1000000LL))
            {
                if (xTaskDallas)
                    xTaskNotifyGive(xTaskDallas);
                if (xTaskI2C)
                    xTaskNotify(xTaskI2C, NOTYFY_SENSOR_TH | NOTYFY_SENSOR_SET_MAGACC | NOTYFY_SENSOR_MAGACC | NOTYFY_SENSOR_SET_MAGACC_INT, eSetValueWithOverwrite);
                water_cont_measure(false);
                light_measure(0);

                start_time = esp_timer_get_time();

                xEventGroupSetBits(status_event_group, TEST_MODE_UPDATED);
            }
        }

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

                if (xTaskDallas)
                    xTaskNotifyGive(xTaskDallas);
                if (xTaskI2C)
                    xTaskNotify(xTaskI2C, NOTYFY_SENSOR_TH | NOTYFY_SENSOR_SET_MAGACC | NOTYFY_SENSOR_MAGACC | NOTYFY_SENSOR_SET_MAGACC_INT, eSetValueWithOverwrite);
                water_cont_measure(false);
                light_measure(10);

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
