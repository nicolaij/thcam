#include "main.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"

#include "esp_spiffs.h"
#include "sys/stat.h"

#include "freertos/ringbuf.h"

result_data_t result;

RTC_DATA_ATTR unsigned int bootCount = 0;
RTC_DATA_ATTR uint8_t history_pos = 0;

RTC_DATA_ATTR result_data_t history[HISTORY_SIZE];

EventGroupHandle_t status_event_group;

int wait_max_counter = 1;

TaskHandle_t xHandleNB = NULL;
TaskHandle_t xTaskI2C = NULL;
TaskHandle_t xTaskDallas = NULL;
TaskHandle_t xHandleWifi = NULL;

void app_main(void)
{

    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

    switch (wakeup_reason)
    {
    case ESP_SLEEP_WAKEUP_EXT0:
        ESP_LOGI("main", "Wakeup caused by external signal using RTC_IO");
        break;
#if SOC_PM_SUPPORT_EXT_WAKEUP
    case ESP_SLEEP_WAKEUP_EXT1:
    {
        uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
        if (wakeup_pin_mask != 0)
        {
            int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
            ESP_LOGI("main", "Wake up from GPIO %d", pin);
        }
        else
        {
            ESP_LOGI("main", "Wake up from GPIO");
        }
        break;
    }
#endif
#if SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
    case ESP_SLEEP_WAKEUP_GPIO:
    {
        uint64_t wakeup_pin_mask = esp_sleep_get_gpio_wakeup_status();
        if (wakeup_pin_mask != 0)
        {
            int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
            ESP_LOGI("main", "Wake up from GPIO %d", pin);

            if (pin == PIN_BATT)
                result.measure.d_charge = true;

            if (pin == PIN_LIGHT)
                result.measure.d_light = true;

            if (pin == PIN_WATER2)
                result.measure.d_water = true;

            if (pin == PIN_INT_ACC)
                result.measure.d_acc_int = true;
        }
        else
        {
            ESP_LOGI("main", "Wake up from GPIO");
        }
        break;
    }
#endif // SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
    case ESP_SLEEP_WAKEUP_TIMER:
        ESP_LOGI("main", "Wakeup caused by timer");
        break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
        ESP_LOGI("main", "Wakeup caused by touchpad");
        break;
    case ESP_SLEEP_WAKEUP_ULP:
        ESP_LOGI("main", "Wakeup caused by ULP program");
        break;
    default:
        ESP_LOGI("main", "Wakeup was not caused by deep sleep: %d", wakeup_reason);
        break;
    }

    bootCount++;
    //  "Количество загрузок: "
    ESP_LOGI("main", "Boot number: %d", bootCount);
    ESP_LOGI("main", "Free Heap: %u bytes", xPortGetFreeHeapSize());

    status_event_group = xEventGroupCreate();

    init_nvs();
    read_nvs_menu();

    result.measure.internal_temp = get_temperature_sensor();
    result.measure.temp = result.measure.internal_temp;
    result.measure.bootcount = bootCount;

    xTaskCreate(dallas_task, "dallas_task", 1024 * 6, NULL, configMAX_PRIORITIES - 10, &xTaskDallas);
    xTaskNotifyGive(xTaskDallas);

    xTaskCreate(i2c_task, "i2c_task", 1024 * 6, NULL, configMAX_PRIORITIES - 10, &xTaskI2C);
    xTaskNotify(xTaskI2C, NOTYFY_SENSOR_TH | NOTYFY_SENSOR_SET_MAGACC | NOTYFY_SENSOR_MAGACC | NOTYFY_SENSOR_SET_MAGACC_INT, eSetValueWithOverwrite);
    // xTaskNotify(xTaskI2C, NOTYFY_SENSOR_TH | NOTYFY_SENSOR_SET_MAGACC | NOTYFY_SENSOR_MAGACC, eSetBits);

    // Light, Water
    uint64_t wake_mask = dio_init() | (BIT64(PIN_BATT) | BIT64(PIN_INT_ACC));

    ESP_LOGI("main", "Current date/time: %s", get_datetime(time(0)));

    xTaskCreate(wifi_task, "wifi_task", 1024 * 4, NULL, configMAX_PRIORITIES - 5, &xHandleWifi);

    xTaskCreate(modem_task, "modem_task", 1024 * 6, NULL, configMAX_PRIORITIES - 10, &xHandleNB);
    xTaskNotifyGive(xHandleNB); // включаем NBIoT модуль

    xTaskCreate(btn_task, "btn_task", 1024 * 4, NULL, configMAX_PRIORITIES - 15, NULL);

    xTaskCreate(console_task, "console_task", 1024 * 10, NULL, configMAX_PRIORITIES - 15, NULL);

    if (result.measure.d_charge || get_charge()) // проснулись от зарядки
    {
        xEventGroupSetBits(status_event_group, NOW_CHARGE);
    }

    EventBits_t uxBits;

    // время ожидания
    int wait = get_menu_val_by_id("waitnb");

    if (wait == 1000) // демонстрационный режим, без сна
        if (xHandleWifi)
            xTaskNotifyGive(xHandleWifi); // включаем WiFi

    const EventBits_t nowake = SERIAL_TERMINAL_ACTIVE | WIFI_ACTIVE | NOW_CHARGE | TEST_MODE_UPDATED;

    uxBits = xEventGroupWaitBits(
        status_event_group, // The event group being tested.
        nowake | END_RADIO, // The bits within the event group to wait for.
        pdFALSE,            // BIT_0 & BIT_1 should be cleared before returning.
        pdFALSE,            // ОБА
        wait * 60000 / portTICK_PERIOD_MS);

    history[history_pos] = result;

    if (get_charge()) // идет зарядка
    {
        if (xHandleWifi)
            xTaskNotifyGive(xHandleWifi); // включаем WiFi
    };

    if (!((uxBits & END_RADIO) != 0 && (uxBits & (nowake)) == 0))
    {
        do // Ждем истечения таймаута
        {
            wait = get_menu_val_by_id("waitnb");

            uxBits = xEventGroupWaitBits(
                status_event_group, // The event group being tested.
                nowake,             // The bits within the event group to wait for.
                pdTRUE,             // BIT_0 & BIT_1 should be cleared before returning.
                pdFALSE,            // ОБА
                wait * 60000 / portTICK_PERIOD_MS);

            if ((uxBits & (NB_TERMINAL)) == 0)
                ESP_LOGD("main", "Wait end. uxBits: 0x%lx", uxBits);

            history[history_pos] = result;

            vTaskDelay(10000 / portTICK_PERIOD_MS);

            // не засыпаем совсем, если на зарядке
            if (get_charge())
            {
                xEventGroupSetBits(status_event_group, NOW_CHARGE);
                continue;
            }

            if (uxBits & TEST_MODE_UPDATED)
            {
                history_pos = (history_pos + 1) % HISTORY_SIZE;
            }

        } while (((uxBits & (nowake)) != 0));
    }

    // принудительно заканчиваем работу NBIoT и WiFi
    xEventGroupSetBits(status_event_group, END_WORK_NBIOT | END_WORK_WIFI);
    vTaskDelay(1);
    
    // время сна в мин
    int sleeptime = get_menu_val_by_id("time");

    // если затопление или засвет - сон короче в 2 раза.
    if ((wake_mask & BIT64(PIN_WATER3)) == 0 || (wake_mask & BIT64(PIN_LIGHT)) == 0)
    {
        if (sleeptime > 15)
            sleeptime = get_menu_val_by_id("time") / 2;
    }

    // если проснулись от затопления или засвета или изменения положения - следующий сон 5 мин.
    if (result.measure.d_light || result.measure.d_water || result.measure.d_acc_int)
    {
        sleeptime = 5;
    }

    // только если предыдущее и текущее ниже 3-х в
    if (result.measure.nbbattery > 0 && result.measure.nbbattery < 3.0 && history[(history_pos - 1) % HISTORY_SIZE].measure.nbbattery < 3.0)
    {
        sleeptime = get_menu_val_by_id("time") * 10;
    }

    // транспортное положение вверх ногами
    if (check_range(result.measure.acc[0] * 1000.0, result.measure.acc[1] * 1000.0, result.measure.acc[2] * 1000.0, 0, 0, 1000, 300))
    {
        sleeptime = 24 * 60;                                  // сутки
        wake_mask = ((BIT64(PIN_BATT) | BIT64(PIN_INT_ACC))); // только зарядка и положение!
        ESP_LOGW("main", "Storage mode");
    }

    if (result.measure.nbbattery > 0 && result.measure.nbbattery < 2.8)
    {
        sleeptime = get_menu_val_by_id("time") * 1000;
        wake_mask = (BIT64(PIN_BATT)); // только зарядка!
    }

    // если есть сигнал от датчика ACC и висит ошибка - отключаем датчик ACC ( он неисправен/отсутствует)
    if (gpio_get_level(PIN_INT_ACC) == 1 && result.measure.d_mag_sensor_error)
        wake_mask &= ~BIT64(PIN_INT_ACC);

    dio_sleep(wake_mask);

    ESP_LOGI("result", OUT_JSON, get_menu_val_by_id("idn"), result.measure.bootcount, get_datetime(result.ttime), OUT_MEASURE_VARS(result.measure));

    // store only changes
    if (history[history_pos].measure.flags != history[(history_pos - 1) % HISTORY_SIZE].measure.flags || history[history_pos].measure.flags != history[(history_pos - 2) % HISTORY_SIZE].measure.flags)
        history_pos = (history_pos + 1) % HISTORY_SIZE;

    // если зарядка - сон 5 мин.
    // if (result.measure.d_charge || get_charge())
    //    sleeptime = 5;

    ESP_LOGW("main", "Go sleep: %d min", sleeptime);

    uint64_t time_in_us = sleeptime * 60ULL * 1000000ULL;
    esp_sleep_enable_timer_wakeup(time_in_us);

    // ESP_ERROR_CHECK(gpio_dump_io_configuration(stdout, 0xffff));

    fflush(stdout);
    esp_deep_sleep_start();
}
