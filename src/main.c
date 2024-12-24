#include "main.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "esp_mac.h"

#include "esp_spiffs.h"
#include "sys/stat.h"

#include "freertos/ringbuf.h"

RTC_DATA_ATTR int bootCount = 0;

uint8_t mac[6];

result_data_t result;

RTC_DATA_ATTR result_data_t old_result;

RTC_DATA_ATTR measure_data_t history[HISTORY_SIZE];
RTC_DATA_ATTR int history_pos = 0;

EventGroupHandle_t status_event_group;

char buf[40];

int wait_max_counter = 1;

TaskHandle_t xHandleNB = NULL;
TaskHandle_t xTaskI2C = NULL;
TaskHandle_t xTaskDallas = NULL;
TaskHandle_t xHandleWifi = NULL;

void app_main(void)
{

    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

    status_event_group = xEventGroupCreate();

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
            {
                result.measure.d_charge = true;
            }

            if (pin == PIN_LIGHT)
                result.measure.d_light = true;

            if (pin == PIN_WATER2)
                result.measure.d_water = true;
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

    init_nvs();
    read_nvs_menu();

    result.measure.internal_temp = get_temperature_sensor();
    result.measure.temp = result.measure.internal_temp;
    result.measure.bootcount = bootCount;

    xTaskCreate(dallas_task, "dallas_task", 1024 * 6, NULL, configMAX_PRIORITIES - 10, &xTaskDallas);
    xTaskNotifyGive(xTaskDallas);

    xTaskCreate(i2c_task, "i2c_task", 1024 * 6, NULL, configMAX_PRIORITIES - 10, &xTaskI2C);
    xTaskNotify(xTaskI2C, NOTYFY_SENSOR_TH | NOTYFY_SENSOR_MAGACC, eSetBits);

    // Light, Water
    dio_init();

    time_t n = time(0);
    struct tm *localtm = localtime(&n);
    strftime((char *)buf, sizeof(buf), "%Y-%m-%d %T", localtm);

    ESP_LOGI("main", "Current date/time: %s", buf);

    esp_efuse_mac_get_default(mac);
    ESP_LOGI("main", "mac: %02x-%02x-%02x-%02x-%02x-%02x", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);

    ESP_LOGI("SPIFFS", "Initializing SPIFFS");
    esp_vfs_spiffs_conf_t spiffsconf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true};

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&spiffsconf);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE("SPIFFS", "Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE("SPIFFS", "Failed to find SPIFFS partition");
        }
        else
        {
            ESP_LOGE("SPIFFS", "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(spiffsconf.partition_label, &total, &used);
    if (ret != ESP_OK)
    {
        ESP_LOGE("SPIFFS", "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI("SPIFFS", "Partition size: total: %d, used: %d", total, used);
    }

    xTaskCreate(modem_task, "modem_task", 1024 * 10, NULL, configMAX_PRIORITIES - 10, &xHandleNB);
    xTaskNotifyGive(xHandleNB); // включаем NBIoT модуль

    xTaskCreate(btn_task, "btn_task", 1024 * 4, NULL, configMAX_PRIORITIES - 15, NULL);

    xTaskCreate(console_task, "console_task", 1024 * 10, NULL, configMAX_PRIORITIES - 15, NULL);

    xTaskCreate(wifi_task, "wifi_task", 1024 * 4, NULL, configMAX_PRIORITIES - 5, &xHandleWifi);
    if (result.measure.d_charge || get_charge()) // проснулись от зарядки
    {
        xEventGroupSetBits(status_event_group, NOW_CHARGE);
    }

    history[history_pos] = result.measure;

    EventBits_t uxBits;

    int wait = get_menu_id("waitnb");

    // время сна в мин
    int sleeptime = get_menu_id("time");

    const EventBits_t nowake = SERIAL_TERMINAL_ACTIVE | WIFI_ACTIVE | NOW_CHARGE;
    uxBits = xEventGroupWaitBits(
        status_event_group, /* The event group being tested. */
        nowake | END_RADIO, /* The bits within the event group to wait for. */
        pdFALSE,            /* BIT_0 & BIT_1 should be cleared before returning. */
        pdFALSE,            /* ОБА */
        wait * 60000 / portTICK_PERIOD_MS);

    history[history_pos] = result.measure;

    if (get_charge()) // идет зарядка
    {
        xTaskNotifyGive(xHandleWifi); // включаем WiFi
    };

    if ((uxBits & END_RADIO) == 0 && ((uxBits & (nowake)) != 0))
    {
        do // Ждем истечения таймаута
        {
            uxBits = xEventGroupWaitBits(
                status_event_group, /* The event group being tested. */
                nowake,             /* The bits within the event group to wait for. */
                pdTRUE,             /* BIT_0 & BIT_1 should be cleared before returning. */
                pdFALSE,            /* ОБА */
                wait * 60000 / portTICK_PERIOD_MS);

        } while (((uxBits & (nowake)) != 0));
    }

    old_result = result;

    history[history_pos] = result.measure;
    history_pos = (history_pos + 1) % HISTORY_SIZE;

    xEventGroupSetBits(status_event_group, END_WORK);

    const char *filepath = "/spiffs/" DATAFILE;
    FILE *fd = NULL;
    struct stat file_stat = {.st_size = 0};

    int maxfilesize = get_menu_id("filesize");
    // Сохраняем файл
    if (maxfilesize > 0)
    {
        if (stat(filepath, &file_stat) == -1)
        {
            fd = fopen(filepath, "w");
            fprintf(fd, "BootCounter, ttime, " OUT_MEASURE_HEADERS "\n");
            fclose(fd);
        }

        fd = fopen(filepath, "a+");
        if (fprintf(fd, "%4i, %10lli, " OUT_MEASURE_FORMATS "\n", bootCount, result.ttime, OUT_MEASURE_VARS(result.measure)) > 0)
        {
            ESP_LOGI("main", "Save \"%s\" successful", filepath);
        }
        else
        {
            ESP_LOGW("main", "Save \"%s\" error!", filepath);
        }
        fclose(fd);
    }

    // если модуль nbiot не выключился - то выключаем принудительно
    if ((uxBits & END_RADIO) == 0)
    {
        nbiot_power_off();
    };

    if (maxfilesize > 0 && file_stat.st_size > (maxfilesize * 1024))
    {
        remove("/spiffs/old" DATAFILE);
        rename(filepath, "/spiffs/old" DATAFILE);
    };

    // Light, Water
    uint64_t wake_mask = dio_sleep();

    // если затопление или засвет - сон 15 мин.
    if ((wake_mask & BIT64(PIN_WATER2)) == 0 || (wake_mask & BIT64(PIN_LIGHT)) == 0)
    {
        if (sleeptime > 15)
            sleeptime = 15;
    };

    ESP_LOGI("result", OUT_JSON, get_menu_id("id"), result.measure.bootcount, "", OUT_MEASURE_VARS(result.measure));

    // если зарядка - сон 1 мин.
    if (xEventGroupGetBits(status_event_group) & NOW_CHARGE || get_charge() == 1)
        sleeptime = 1;

    uint64_t time_in_us = sleeptime * 60ULL * 1000000ULL;

    ESP_LOGW("main", "Go sleep: %lld min", time_in_us / 60ULL / 1000000ULL);
    // ESP_ERROR_CHECK(gpio_dump_io_configuration(stdout,0xffff));
    fflush(stdout);

    esp_sleep_enable_timer_wakeup(time_in_us);
    esp_deep_sleep_start();
}
