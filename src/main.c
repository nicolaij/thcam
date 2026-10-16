#include "main.h"

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

// int wait_max_counter = 1;

TaskHandle_t xHandleNB = NULL;
TaskHandle_t xTaskI2C = NULL;
TaskHandle_t xTaskDallas = NULL;
TaskHandle_t xTaskDIO = NULL;
TaskHandle_t xHandleWifi = NULL;

void app_main(void)
{
    char datetime[DATETIME_STR_SIZE]; // общий буфер строки даты/времени

    // защита от мусора в RTC-памяти: history_pos вне диапазона -> сброс в начало
    if (history_pos >= HISTORY_SIZE)
        history_pos = 0;

    uint32_t causes = esp_sleep_get_wakeup_causes();

    ESP_LOGD("Wake up", "causes: 0x%x", causes);

    if (causes & BIT(ESP_SLEEP_WAKEUP_TIMER))
    {
        ESP_LOGI("Wake up", "timer");
    }

    if (causes & BIT(ESP_SLEEP_WAKEUP_GPIO))
    {
        uint64_t wakeup_pin_mask = esp_sleep_get_gpio_wakeup_status();
        if (wakeup_pin_mask != 0)
        {
            int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
            ESP_LOGI("Wake up", "GPIO %d", pin);

            // причины пробуждения по пинам (d_* флаги)
            result.measure.d_light = (wakeup_pin_mask & BIT64(PIN_LIGHT)) != 0;
            result.measure.d_water = ((wakeup_pin_mask & BIT64(PIN_WATER1)) != 0) ||
                                     ((wakeup_pin_mask & BIT64(PIN_WATER2)) != 0) ||
                                     (((PIN_WATER3 != GPIO_NUM_NC) ? (wakeup_pin_mask & BIT64(PIN_WATER3)) : 0) != 0);
            result.measure.d_acc_int = (wakeup_pin_mask & BIT64(PIN_INT_ACC)) != 0;
            result.measure.d_charge = (wakeup_pin_mask & BIT64(PIN_BATT)) != 0;
        }
    }

    esp_reset_reason_t reason = esp_reset_reason();
    ESP_LOGD("main", "After restart! %d", reason);
    if (reason == ESP_RST_BROWNOUT)
    {
        result.measure.d_rst_brownout = true;
    }
    else if (reason == ESP_RST_DEEPSLEEP)
    {
        result.measure.d_rst_deepsleep = true;
    }

    bootCount++;
    //  "Количество загрузок: "
    ESP_LOGI("main", "Boot number: %d", bootCount);
    get_datetime_str(time(0), datetime, sizeof(datetime));
    ESP_LOGI("main", "Current date/time: %s", datetime);

    ESP_LOGI("main", "Free Heap: %u bytes", xPortGetFreeHeapSize());

    status_event_group = xEventGroupCreate();

    // Initialize NVS
    esp_err_t err = init_nvs();
    if (err != ESP_OK)
    {
        ESP_LOGE("main", "NVS init error: %s", esp_err_to_name(err));
    }
    read_nvs_menu();

    result.measure.internal_temp = get_temperature_sensor();
    // result.measure.temp = result.measure.internal_temp;
    result.measure.bootcount = bootCount;

    xTaskCreate(dallas_task, "dallas_task", 1024 * 4, NULL, configMAX_PRIORITIES - 10, &xTaskDallas);
    xTaskNotifyGive(xTaskDallas);

    xTaskCreate(i2c_task, "i2c_task", 1024 * 4, NULL, configMAX_PRIORITIES - 2, &xTaskI2C);
    xTaskNotify(xTaskI2C, NOTYFY_SENSOR_TH | NOTYFY_SENSOR_SET_MAGACC | NOTYFY_SENSOR_MAGACC, eSetBits);

    // Light, Water
    xTaskCreate(dio_task, "dio_task", 1024 * 4, NULL, configMAX_PRIORITIES - 3, &xTaskDIO);

    xTaskCreate(wifi_task, "wifi_task", 1024 * 2, NULL, configMAX_PRIORITIES - 4, &xHandleWifi);

    xTaskCreate(modem_task, "modem_task", 1024 * 5, NULL, configMAX_PRIORITIES - 8, &xHandleNB);
    xTaskNotifyGive(xHandleNB); // включаем NBIoT модуль

    xTaskCreate(btn_task, "btn_task", 1024 * 4, NULL, configMAX_PRIORITIES - 20, NULL);

    xTaskCreate(console_task, "console_task", 1024 * 2, NULL, configMAX_PRIORITIES - 12, NULL);

    //taskYIELD();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    xTaskNotifyGive(xTaskDIO); // Notify from xTaskI2C

    if (result.measure.d_charge || gpio_get_level(PIN_BATT)) // проснулись от зарядки
    {
        xEventGroupSetBits(status_event_group, NOW_CHARGE);
    }

    // время ожидания в мин
    int wait = get_menu_val_by_id("waitnb");

    // время сна в мин
    int sleeptime = get_menu_val_by_id("time");

    if (gpio_get_level(PIN_BATT) || wait == 1000 || sleeptime <= wait) // идет зарядка или демонстрационный режим, без сна
    {
        if (xHandleWifi)
            xTaskNotifyGive(xHandleWifi); // включаем WiFi
    };

    const EventBits_t nowake = SERIAL_TERMINAL_ACTIVE | WIFI_ACTIVE | NOW_CHARGE | START_SENSORS_UPDATE;
    EventBits_t uxBits = 0;
    // Была ли активность (терминал/WiFi/зарядка/обновление датчиков) за время работы.
    // Если была - END_RADIO больше не прерывает работу: ждём истечения таймаута,
    // иначе сессия WiFi/терминала обрывается сразу после ответа модуля, т.к. бит
    // активности уже сброшен (xEventGroupWaitBits очищает найденные биты).
    bool activity = false;
    bool sleep_now = false;

    history[history_pos] = result;

    do // Ждем условий для засыпания
    {
        wait = get_menu_val_by_id("waitnb");
        sleeptime = get_menu_val_by_id("time");

        // Единственное ожидание: END_RADIO - радио закончило, либо активное событие
        // (терминал/WiFi/зарядка/тест), которое сбрасывает таймер wait минут.
        // Истечение таймера без событий - выход из цикла и сон.
        uxBits = xEventGroupWaitBits(
            status_event_group,
            END_RADIO | nowake,
            pdTRUE,  // очищаем найденные биты
            pdFALSE, // любой из битов
            wait * 60000 / portTICK_PERIOD_MS);

        ESP_LOGD("wait", "status_event_group=0x%x", uxBits);

        // активность запоминаем: сам бит уже очищен, а решение о сне принимается
        // ниже именно по факту активности за время работы
        if (uxBits & nowake)
            activity = true;

        if (uxBits & START_SENSORS_UPDATE)
        {
            history_pos = (history_pos + 1) % HISTORY_SIZE;
            vTaskDelay(3000 / portTICK_PERIOD_MS); // пауза для завершения измерений
        }

        history[history_pos] = result;

        // на зарядке не спим: даем поработать другим задачам и продолжаем цикл
        if (gpio_get_level(PIN_BATT))
        {
            get_batt_mv(); // напряжение на PIN_BATT в лог

            if (xHandleWifi)
                xTaskNotifyGive(xHandleWifi); // включаем WiFi

            vTaskDelay(30000 / portTICK_PERIOD_MS); // пауза для остальных задач

            if (xHandleNB)
                xTaskNotifyGive(xHandleNB); // включаем NBIoT модуль для контроля напряжения

            xEventGroupSetBits(status_event_group, NOW_CHARGE);
            uxBits = nowake; // не выходим из цикла на зарядке
        }

        // Спим, если: (1) истёк таймаут - в uxBits нет ни одного бита из маски;
        // (2) радио закончило работу (END_RADIO), а активности за это время не было.
        sleep_now = ((uxBits & (END_RADIO | nowake)) == 0) || ((uxBits & END_RADIO) && !activity);
    } while (!sleep_now);

    // принудительно заканчиваем работу NBIoT и WiFi
    xEventGroupSetBits(status_event_group, END_WORK_NBIOT | END_WORK_WIFI);
    // настраиваем INT
    xTaskNotify(xTaskI2C, NOTYFY_SENSOR_SET_MAGACC_INT, eSetValueWithOverwrite);
    vTaskDelay(20 / portTICK_PERIOD_MS); // 10 мало

    uint64_t wake_mask = dio_check((BIT64(PIN_BATT) | BIT64(PIN_INT_ACC)));

    /*
    // если затопление или засвет - сон короче в 2 раза.
        if ((wake_mask & BIT64(PIN_WATER1)) == 0 || (wake_mask & BIT64(PIN_LIGHT)) == 0)
        {
            if (sleeptime > 15)
                sleeptime = get_menu_val_by_id("time") / 2;
        }
    */
    // если проснулись от затопления или засвета или изменения положения - следующий сон 5 мин.
    if (result.measure.d_light || result.measure.d_water || result.measure.d_acc_int)
    {
        sleeptime = 5;
    }

    // только если предыдущее и текущее ниже 3B
    if (result.measure.nbbattery > 0 && result.measure.nbbattery < 3.0 && history_pos > 0 && history[(history_pos - 1) % HISTORY_SIZE].measure.nbbattery > 0.0 && history[(history_pos - 1) % HISTORY_SIZE].measure.nbbattery < 3.0)
    {
        sleeptime = get_menu_val_by_id("time") * 10;
    }

    // транспортное положение вверх ногами
    if (check_range(result.measure.acc[0] * 1000.0, result.measure.acc[1] * 1000.0, result.measure.acc[2] * 1000.0, 0, 0, 1000, 300))
    {
        // sleeptime = 24 * 60;                                  // сутки
        // wake_mask = ((BIT64(PIN_BATT) | BIT64(PIN_INT_ACC))); // только зарядка и положение!
        ESP_LOGW("main", "Storage mode");
    }

    if (result.measure.nbbattery > 0 && result.measure.nbbattery < 2.8)
    {
        sleeptime = get_menu_val_by_id("time") * 24;
        wake_mask = (BIT64(PIN_BATT)); // только зарядка!
    }

    // если есть сигнал от датчика ACC и висит ошибка - отключаем датчик ACC ( он неисправен/отсутствует)
    if (gpio_get_level(PIN_INT_ACC) == 1 && result.measure.d_mag_sensor_error)
        wake_mask &= ~BIT64(PIN_INT_ACC);

    dio_sleep(wake_mask);

    get_datetime_str(result.ttime, datetime, sizeof(datetime));
    ESP_LOGI("result", OUT_JSON, get_menu_val_by_id("idn"), result.measure.bootcount, datetime, OUT_MEASURE_VARS(result.measure));

    // store only changes: flags, ci
    if (history_pos < 2 || history[history_pos].measure.flags != history[(history_pos - 1) % HISTORY_SIZE].measure.flags || history[history_pos].measure.flags != history[(history_pos - 2) % HISTORY_SIZE].measure.flags ||
        history[history_pos].measure.ci != history[(history_pos - 1) % HISTORY_SIZE].measure.ci)
        history_pos = (history_pos + 1) % HISTORY_SIZE;

    // если зарядка - сон 5 мин.
    // if (result.measure.d_charge || gpio_get_level(PIN_BATT))
    //    sleeptime = 5;

    ESP_LOGW("main", "Go sleep: %d min", sleeptime);

    uint64_t time_in_us = sleeptime * 60ULL * 1000000ULL;

    ESP_ERROR_CHECK(gpio_dump_io_configuration(stdout, wake_mask | BIT64(PIN_WATER1) | BIT64(PIN_WATER2)));
    fflush(stdout);

    esp_sleep_enable_timer_wakeup(time_in_us);
    esp_deep_sleep_start();

    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    };
}
