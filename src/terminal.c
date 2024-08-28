#include "main.h"
#include "driver/uart.h"
#include <stdio.h>
#include <string.h>
#include "esp_event.h"

#include "nvs.h"
#include "nvs_flash.h"

uint8_t buffer[256];

static const char *TAG = "terminal";

nvs_handle_t my_handle;

menu_t menu[] = {
    {.id = "id", .name = "Номер датчика", .izm = "", .val = 1, .min = 1, .max = 100000},
    {.id = "time", .name = "Период пробуждений", .izm = "мин", .val = 60, .min = 10, .max = 100000},
};

esp_err_t init_nvs()
{
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    // Example of nvs_get_stats() to get the number of used entries and free entries:
    nvs_stats_t nvs_stats;
    nvs_get_stats(NULL, &nvs_stats);
    ESP_LOGD("NVS", "Count: UsedEntries = (%d), FreeEntries = (%d), AllEntries = (%d)", nvs_stats.used_entries, nvs_stats.free_entries, nvs_stats.total_entries);
    return err;
}

esp_err_t read_nvs_menu()
{
    // Open
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE("storage", "Error (%s) opening NVS handle!", esp_err_to_name(err));
    }
    else
    {
        for (int i = 0; i < sizeof(menu) / sizeof(menu_t); i++)
        {
            err = nvs_get_i32(my_handle, menu[i].id, &menu[i].val);
            switch (err)
            {
            case ESP_OK:
                ESP_LOGD("NVS", "Read \"%s\" = %ld", menu[i].name, menu[i].val);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                ESP_LOGD("NVS", "The value  \"%s\" is not initialized yet!", menu[i].name);
                break;
            default:
                ESP_LOGE("NVS", "Error (%s) reading!", esp_err_to_name(err));
            }
        }

        // Close
        nvs_close(my_handle);
    }
    return err;
}

int get_menu_id(const char *id)
{
    for (int i = 0; i < sizeof(menu) / sizeof(menu_t); i++)
    {
        int l = strlen(menu[i].id);
        if (strncmp(id, menu[i].id, l) == 0)
            return menu[i].val;
    }
    return 0;
}

void console_task(void *arg)
{

    uint8_t *data = buffer;

    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, sizeof(buffer), 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    uart_flush(UART_NUM_0);

    int enter_value = 0;

    while (1)
    {
        const int rxBytes = uart_read_bytes(UART_NUM_0, data, 128, 200 / portTICK_PERIOD_MS);
        if (rxBytes > 0)
        {
            if (data[rxBytes - 1] == '\n')
            {
                if (data[rxBytes - 2] == '\r')
                {
                    data[rxBytes - 2] = 0;
                };

                data[rxBytes - 1] = 0;
                ESP_LOGD(TAG, "Read bytes: '%s'", buffer);
                // ESP_LOG_BUFFER_HEXDUMP(TAG, data, rxBytes, ESP_LOG_INFO);
                data = buffer;
                int n = atoi((const char *)data);
                if (enter_value > 0)
                {
                    if (n >= menu[enter_value - 1].min && n <= menu[enter_value - 1].max)
                    {
                        menu[enter_value - 1].val = n;
                        esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
                        if (err != ESP_OK)
                        {
                            ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
                        }
                        else
                        {
                            err = nvs_set_i32(my_handle, menu[enter_value - 1].id, menu[enter_value - 1].val);
                            if (err != ESP_OK)
                            {
                                ESP_LOGE(TAG, "%s", esp_err_to_name(err));
                            }
                            else
                            {
                                ESP_LOGI("menu", "-------------------------------------------");
                                ESP_LOGI("menu", "%2i. %s: %li %s.", enter_value, menu[enter_value - 1].name, menu[enter_value - 1].val, menu[enter_value - 1].izm);
                                ESP_LOGI("menu", "-------------------------------------------");
                            }
                        }

                        // Commit written value.
                        // After setting any values, nvs_commit() must be called to ensure changes are written
                        // to flash storage. Implementations may write to storage at other times,
                        // but this is not guaranteed.
                        ESP_LOGD(TAG, "Committing updates in NVS ... ");
                        err = nvs_commit(my_handle);
                        if (err != ESP_OK)
                            ESP_LOGE(TAG, "Committing updates in NVS ... - Failed!");

                        // Close
                        nvs_close(my_handle);
                    }
                    enter_value = 0;
                }
                else
                {
                    if (n > 0 && n <= sizeof(menu) / sizeof(menu_t))
                    {
                        ESP_LOGI("menu", "-------------------------------------------");
                        ESP_LOGI("menu", "%2i. %s: %li %s. Введите новое значение: ", n, menu[n - 1].name, menu[n - 1].val, menu[n - 1].izm);
                        ESP_LOGI("menu", "-------------------------------------------");
                        enter_value = n;
                    }
                    else
                    {
                        ESP_LOGI("menu", "-------------------------------------------");
                        for (int i = 0; i < sizeof(menu) / sizeof(menu_t); i++)
                        {
                            ESP_LOGI("menu", "%2i. %s: %li %s", i + 1, menu[i].name, menu[i].val, menu[i].izm);
                        }
                        ESP_LOGI("menu", "-------------------------------------------");
                        enter_value = 0;
                    }
                }
            }
            else
            {
                data = data + rxBytes;
                if (data >= buffer + sizeof(buffer))
                    data = buffer;
            }
        }
    }
}
