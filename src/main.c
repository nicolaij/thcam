#include "main.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "esp_mac.h"

#include "driver/i2c_master.h"
i2c_master_dev_handle_t dev_handle;

#include "driver/temperature_sensor.h"

#include "bme280.h"
s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void BME280_delay_msek(u32 msek);
#define TAG_BME280 "BME280"

static const char *TAG = "THCAM";

RTC_DATA_ATTR int bootCount = 0;

uint8_t mac[6];

result_data_t result;

RTC_DATA_ATTR measure_data_t old_measure;

EventGroupHandle_t ready_event_group;

char buf[40];

void app_main(void)
{
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

    ready_event_group = xEventGroupCreate();

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
                xEventGroupSetBits(ready_event_group, NEED_WIFI);
            }
            if (pin == PIN_LIGHT)
                result.measure.d_light = true;
            if (pin == PIN_WATER1)
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
    result.bootCount = bootCount;

    init_nvs();
    read_nvs_menu();

    // Light, Water
    dio_init();

    time_t n = time(0);
    struct tm *localtm = localtime(&n);
    strftime((char *)buf, sizeof(buf), "%Y-%m-%d %T", localtm);

    ESP_LOGI("main", "Current date/time: %s", buf);

    esp_efuse_mac_get_default(mac);
    ESP_LOGI("main", "mac: %02x-%02x-%02x-%02x-%02x-%02x", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);

    ESP_LOGD("main", "Initializing Temperature sensor");

    temperature_sensor_handle_t temp_sensor = NULL;
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);

    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));

    ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, &result.measure.internal_temp));

    ESP_ERROR_CHECK(temperature_sensor_disable(temp_sensor));
    ESP_ERROR_CHECK(temperature_sensor_uninstall(temp_sensor));

    int l = snprintf((char *)buf, sizeof(buf), "Temperature:  %.01f°C", result.measure.internal_temp);
    ESP_LOGI("temperature_sensor", "%s", buf);

    // i2c
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = GPIO_NUM_7,
        .sda_io_num = GPIO_NUM_6,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err_rc;

    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x40, // HTU21
        .scl_speed_hz = 400000,
    };

    uint8_t buffer[4];

    int try = 2;
    do
    {
        try--;
        ESP_LOGD("THCAM", "try %d", dev_cfg.device_address);
        err_rc = i2c_master_probe(bus_handle, dev_cfg.device_address, 10);
        if (err_rc != ESP_OK)
        {
            if (try == 0)
            {
                if (dev_cfg.device_address != BME280_I2C_ADDRESS1)
                {
                    dev_cfg.device_address = BME280_I2C_ADDRESS1;
                    // dev_cfg.scl_speed_hz = 100000;
                    try = 2;
                }
                else
                {
                    break;
                }
            }

            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    } while (err_rc != ESP_OK);

    if (err_rc == ESP_OK)
    {

        ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

        if (dev_cfg.device_address == 0x40) // HTU21
        {
            uint8_t cmd = 0xfe; // Soft Reset
            ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit(dev_handle, &cmd, 1, 10));

            vTaskDelay(20 / portTICK_PERIOD_MS); // The soft reset takes less than 15ms.

            cmd = 0xe3; // Trigger Temperature Measurement
            ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit(dev_handle, &cmd, 1, 10));
            vTaskDelay(50 / portTICK_PERIOD_MS);
            err_rc = i2c_master_receive(dev_handle, buffer, 3, 10);

            if (err_rc == ESP_OK && (buffer[1] & 0b10) == 0) // Status (‘0’: temperature, ‘1’: humidity)
            {
                result.measure.temp = -46.85 + 175.72 * (int)((buffer[0] << 8) | (buffer[1] & 0b11111100)) / 65536.0;
            }

            cmd = 0xe5; // Trigger Humidity Measurement
            ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit(dev_handle, &cmd, 1, 10));
            vTaskDelay(50 / portTICK_PERIOD_MS);
            err_rc = i2c_master_receive(dev_handle, buffer, 3, 100);

            if (err_rc == ESP_OK && (buffer[1] & 0b10) != 0) // Status (‘0’: temperature, ‘1’: humidity)
            {
                result.measure.humidity = -6.0 + 125.0 * (int)((buffer[0] << 8) | (buffer[1] & 0b11111100)) / 65536.0;
            }

            ESP_LOGI(TAG, "Read from I2C: T=%.01f°C, H=%.01f%%", result.measure.temp, result.measure.humidity);
        }

        if (dev_cfg.device_address == BME280_I2C_ADDRESS1) // BME280
        {

            struct bme280_t bme280 = {
                .bus_write = BME280_I2C_bus_write,
                .bus_read = BME280_I2C_bus_read,
                .dev_addr = BME280_I2C_ADDRESS1,
                .delay_msec = BME280_delay_msek};

            s32 com_rslt;
            s32 v_uncomp_pressure_s32;
            s32 v_uncomp_temperature_s32;
            s32 v_uncomp_humidity_s32;

            com_rslt = bme280_init(&bme280);

            com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_1X);
            com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_1X);
            com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);

            com_rslt += bme280_set_filter(BME280_FILTER_COEFF_OFF);
            if (com_rslt == SUCCESS)
            {
                com_rslt = bme280_get_forced_uncomp_pressure_temperature_humidity(
                    &v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

                if (com_rslt == SUCCESS)
                {
                    result.measure.temp = bme280_compensate_temperature_double(v_uncomp_temperature_s32);
                    result.measure.pressure = bme280_compensate_pressure_double(v_uncomp_pressure_s32) / 100; // Pa -> hPa
                    result.measure.humidity = bme280_compensate_humidity_double(v_uncomp_humidity_s32);
                    ESP_LOGI(TAG_BME280, "%.2f degC / %.3f hPa / %.3f %%",
                             result.measure.temp,
                             result.measure.pressure,
                             result.measure.humidity);
                }
                else
                {
                    ESP_LOGE(TAG_BME280, "measure error. code: %d", com_rslt);
                }
            }
            else
            {
                ESP_LOGE(TAG_BME280, "init or setting error. code: %d", com_rslt);
            }
        }
    }
    else
    {
        ESP_LOGE(TAG, "I2C sensor error!");
    }

    // ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));

    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));

    // время сна
    int sleeptime = get_menu_id("time");

    if (get_charge() == 0)
    {
        xEventGroupSetBits(ready_event_group, NEED_TRANSMIT);
    }
    else
    {
        xEventGroupSetBits(ready_event_group, NOW_CHARGE);
        // Будем просыпаться раз в минуту, для контроля зарядки
        sleeptime = 1;
    }

    xTaskCreate(modem_task, "modem_task", 1024 * 10, NULL, configMAX_PRIORITIES - 10, NULL);

    // xTaskCreate(led_task, "led_task", 1024 * 5, NULL, configMAX_PRIORITIES - 15, NULL);

    xTaskCreate(btn_task, "btn_task", 1024 * 4, NULL, configMAX_PRIORITIES - 15, NULL);

    xTaskCreate(console_task, "console_task", 1024 * 10, NULL, configMAX_PRIORITIES - 15, NULL);

    TaskHandle_t xHandleWifi = NULL;
    xTaskCreate(wifi_task, "wifi_task", 1024 * 4, NULL, 5, &xHandleWifi);

    EventBits_t uxBits;

    int wait = get_menu_id("waitnb");

    uxBits = xEventGroupWaitBits(
        ready_event_group,           /* The event group being tested. */
        END_RADIO_SLEEP | WIFI_STOP, /* The bits within the event group to wait for. */
        pdFALSE,                     /* BIT_0 & BIT_1 should be cleared before returning. */
        pdTRUE,                      /* ОБА */
        wait * 60000 / portTICK_PERIOD_MS);

    xEventGroupSetBits(ready_event_group, END_WORK);

    // если модуль nbiot не выключился - то выключаем принудительно
    if ((uxBits & END_RADIO_SLEEP) == 0)
    {
        ESP_LOGW("main", "Force power off NB-IoT");
        nbiot_power_pin(2000 / portTICK_PERIOD_MS);
    }

    // Light, Water
    dio_sleep();

    if (get_charge() == 1) // идет зарядка
    {
        sleeptime = 1;
    }

    uint64_t time_in_us = sleeptime * 60ULL * 1000000ULL;

    ESP_LOGW("main", "Go sleep: %lld us", time_in_us);
    //ESP_ERROR_CHECK(gpio_dump_io_configuration(stdout,0xffff));
    fflush(stdout);

    esp_sleep_enable_timer_wakeup(time_in_us);
    esp_deep_sleep_start();

    while (1)
    {
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}
