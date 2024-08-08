#include "main.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "esp_mac.h"

#include "driver/i2c_master.h"

#include "driver/temperature_sensor.h"

#include "led_strip.h"

static const char *TAG = "THCAM";

RTC_DATA_ATTR int bootCount = 0;

uint8_t mac[6];

result_data_t result;

RTC_DATA_ATTR measure_data_t old_measure;

EventGroupHandle_t ready_event_group;

char buf[40];

led_strip_handle_t configure_led(void)
{
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = GPIO_NUM_8,             // The GPIO that connected to the LED strip's data line
        .max_leds = 1,                            // The number of LEDs in the strip,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
        .led_model = LED_MODEL_SK6812,            // LED strip model
        .flags.invert_out = false,                // whether to invert the output signal
    };

// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ (10 * 1000 * 1000)

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .flags.with_dma = false,               // DMA feature is available on ESP target like ESP32-S3
    };

    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
    return led_strip;
}

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

    // LED
    led_strip_handle_t led_strip = configure_led();
    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 127, 0, 0));
    /* Refresh the strip to send data */
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));

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
        .scl_speed_hz = 100000,
    };

    uint8_t buffer[4];

    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    vTaskDelay(100 / portTICK_PERIOD_MS);
    uint8_t cmd = 0xfe; // Soft Reset
    err_rc = i2c_master_transmit(dev_handle, &cmd, 1, 100);

    if (err_rc == ESP_OK)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        cmd = 0xe3; // Trigger Temperature Measurement
        ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit(dev_handle, &cmd, 1, 100));
        vTaskDelay(100 / portTICK_PERIOD_MS);
        err_rc = i2c_master_receive(dev_handle, buffer, 3, 100);

        if (err_rc == ESP_OK && (buffer[1] & 0b10) == 0) // Status (‘0’: temperature, ‘1’: humidity)
        {
            result.measure.temp = -46.85 + 175.72 * (int)((buffer[0] << 8) | (buffer[1] & 0b11111100)) / 65536.0;
        }

        cmd = 0xe5; // Trigger Humidity Measurement
        ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit(dev_handle, &cmd, 1, 100));
        vTaskDelay(100 / portTICK_PERIOD_MS);
        err_rc = i2c_master_receive(dev_handle, buffer, 3, 100);

        if ((buffer[1] & 0b10) != 0) // Status (‘0’: temperature, ‘1’: humidity)
        {
            result.measure.humidity = -6.0 + 125.0 * (int)((buffer[0] << 8) | (buffer[1] & 0b11111100)) / 65536.0;
        }

        ESP_LOGI(TAG, "Read from I2C: T=%.01f°C, H=%.01f%%", result.measure.temp, result.measure.humidity);

        ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    }

    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));

    ready_event_group = xEventGroupCreate();

    xTaskCreate(modem_task, "modem_task", 1024 * 4, NULL, configMAX_PRIORITIES - 10, NULL);

    xEventGroupWaitBits(
        ready_event_group, /* The event group being tested. */
        END_RADIO_SLEEP,   /* The bits within the event group to wait for. */
        pdFALSE,           /* BIT_0 & BIT_1 should be cleared before returning. */
        pdTRUE,
        1200000 / portTICK_PERIOD_MS);

    // Light, Water
    dio_sleep();

    uint64_t time_in_us = 5ULL * 60ULL * 1000000ULL;

    ESP_LOGW("main", "Go sleep: %lld us", time_in_us);
    fflush(stdout);

    esp_sleep_enable_timer_wakeup(time_in_us);
    esp_deep_sleep_start();

    while (1)
    {
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}
