#include "main.h"
#include "driver/uart.h"
#include <stdio.h>
#include <string.h>
#include "esp_event.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_now.h"
#include "esp_mac.h"

#include <arpa/inet.h>
#include "esp_netif.h"

static const char *TAG = "terminal";

nvs_handle_t my_handle;

int NB_terminal_mode = 0;

extern TaskHandle_t xHandleNB;
extern TaskHandle_t xTaskI2C;

menu_t menu[] = {
    {.id = "idn", .name = "Номер датчика", .izm = "", .val = 1, .min = 1, .max = 100000},
    {.id = "time", .name = "Период пробуждений", .izm = "мин", .val = 60, .min = 10, .max = 100000},
    {.id = "waitnb", .name = "Ожидание NB-IoT, WiFi", .izm = "мин", .val = 3, .min = 1, .max = 1000},
    //{.id = "ubatt", .name = "Окончание зарядки батареи", .izm = "мВ", .val = 3500, .min = 3000, .max = 3600},
    {.id = "ip", .name = "IP сервера", .izm = "", .val = ((10 << 24) | (179 << 16) | (40 << 8) | (20)), .min = INT32_MIN, .max = INT32_MAX},
    {.id = "tcpport", .name = "TCP порт сервера (0: не исп.)", .izm = "", .val = 48885, .min = 0, .max = 65535},
    {.id = "udpport", .name = "UDP порт сервера (0: не исп.)", .izm = "", .val = 0, .min = 0, .max = 65535},
    {.id = "MAC1", .name = "ESPNOW! Target MAC", .izm = "", .val = 0, .min = 0, .max = INT32_MAX},
    {.id = "MAC2", .name = "", .izm = "", .val = 0, .min = 0, .max = INT32_MAX},
    //{.id = "r1.1", .name = "Резистор ADC1", .izm = "Ом", .val = 10000, .min = 1, .max = 20000000},
    //{.id = "r1.2", .name = "Резистор ADC2", .izm = "Ом", .val = 10000, .min = 1, .max = 20000000},
    {.id = "openaccX", .name = "ACC Положение Открыто", .izm = "", .val = 0, .min = -9999, .max = 9999},
    {.id = "openaccY", .name = "", .izm = "", .val = 0, .min = -9999, .max = 9999},
    {.id = "openaccZ", .name = "", .izm = "", .val = 0, .min = -9999, .max = 9999},
    {.id = "openacce", .name = "", .izm = "", .val = 0, .min = 0, .max = 1},
    {.id = "closeaccX", .name = "ACC Положение Закрыто", .izm = "", .val = 0, .min = -9999, .max = 9999},
    {.id = "closeaccY", .name = "", .izm = "", .val = 0, .min = -9999, .max = 9999},
    {.id = "closeaccZ", .name = "", .izm = "", .val = 0, .min = -9999, .max = 9999},
    {.id = "closeacce", .name = "", .izm = "", .val = 0, .min = 0, .max = 1},
    {.id = "openmagX", .name = "MAG Положение Открыто", .izm = "", .val = 0, .min = -9999, .max = 9999},
    {.id = "openmagY", .name = "", .izm = "", .val = 0, .min = -9999, .max = 9999},
    {.id = "openmagZ", .name = "", .izm = "", .val = 0, .min = -9999, .max = 9999},
    {.id = "openmage", .name = "", .izm = "", .val = 0, .min = 0, .max = 1},
    {.id = "closemagX", .name = "MAG Положение Закрыто", .izm = "", .val = 0, .min = -9999, .max = 9999},
    {.id = "closemagY", .name = "", .izm = "", .val = 0, .min = -9999, .max = 9999},
    {.id = "closemagZ", .name = "", .izm = "", .val = 0, .min = -9999, .max = 9999},
    {.id = "closemage", .name = "", .izm = "", .val = 0, .min = 0, .max = 1},
    {.id = "deviation", .name = "Допустимое отклонение +-", .izm = "", .val = 500, .min = 0, .max = 1000},
    {.id = "lightrang", .name = "Переключение диапазона освещен.", .izm = "%", .val = 50, .min = 0, .max = 100},
    //{.id = "kbatt", .name = "Калибровка напр. батареи (ADC0)", .izm = "", .val = 448, .min = 1, .max = 10000},
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
    esp_err_t erro = nvs_open("storage", NVS_READONLY, &my_handle);
    if (erro != ESP_OK)
    {
        ESP_LOGE("storage", "Error (%s) opening NVS handle!", esp_err_to_name(erro));
    }
    else
    {
        for (int i = 0; i < sizeof(menu) / sizeof(menu_t); i++)
        {
            esp_err_t err = nvs_get_i32(my_handle, menu[i].id, (int32_t *)&menu[i].val);
            switch (err)
            {
            case ESP_OK:
                ESP_LOGD("NVS", "Read \"%s\" = %i", menu[i].name, menu[i].val);
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
    return erro;
}

esp_err_t read_nvs_id(const char *key, uint64_t *out_value)
{
    // Open
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE("storage", "Error (%s) opening NVS handle!", esp_err_to_name(err));
    }
    else
    {
        err = nvs_get_u64(my_handle, key, out_value);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGD("NVS", "Read \"%s\" = %016llX", key, *out_value);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGD("NVS", "The value  \"%s\" is not initialized yet!", key);
            break;
        default:
            ESP_LOGE("NVS", "Error (%s) reading!", esp_err_to_name(err));
        }

        // Close
        nvs_close(my_handle);
    }
    return err;
}

int get_menu_pos_by_id(const char *id)
{
    int ll = strlen(id);
    for (int i = 0; i < sizeof(menu) / sizeof(menu_t); i++)
    {
        int l = strlen(menu[i].id);
        if (ll == l && strncmp(id, menu[i].id, l) == 0)
            return i;
    }
    return -1;
}

int get_menu_val_by_id(const char *id)
{
    int ll = strlen(id);
    for (int i = 0; i < sizeof(menu) / sizeof(menu_t); i++)
    {
        int l = strlen(menu[i].id);
        if (ll == l && strncmp(id, menu[i].id, l) == 0)
            return menu[i].val;
    }
    return 0;
}

esp_err_t set_menu_val_by_id(const char *id, int value)
{
    esp_err_t err = ESP_FAIL;
    int ll = strlen(id);
    for (int i = 0; i < sizeof(menu) / sizeof(menu_t); i++)
    {
        int l = strlen(menu[i].id);
        if (ll == l && strncmp(id, menu[i].id, l) == 0)
        {
            if (menu[i].val != value)
            {
                err = nvs_open("storage", NVS_READWRITE, &my_handle);

                ESP_LOGD("NVS", "Write  \"%s\" : \"%i\"", menu[i].id, value);
                err = nvs_set_i32(my_handle, id, value);
                menu[i].val = value;
                nvs_close(my_handle);
            }
            break;
        }
    }

    return err;
}

int get_menu_json(char *buf)
{
    int pos = 0;
    buf[pos++] = '{';
    for (int i = 0; i < sizeof(menu) / sizeof(menu_t); i++)
    {
        pos += sprintf(&buf[pos], "\"%s\":[\"%s\",%i,\"%s\"]", menu[i].id, menu[i].name, menu[i].val, menu[i].izm);
        if (i < sizeof(menu) / sizeof(menu_t) - 1)
            buf[pos++] = ',';
        else
            buf[pos++] = '}';

        buf[pos] = '\0';
    }
    return pos;
}

int get_menu_html(char *buf)
{
    int pos = 0;
    static int index = 0;

    if (index == 0)
        pos = sprintf(buf, "<table>");

    while (index < sizeof(menu) / sizeof(menu_t))
    {
        if (pos > CONFIG_LWIP_TCP_MSS - 256)
        {
            return pos;
        }

        if (index == 3) // IP
        {
            esp_ip4_addr_t ip_addr;
            ip_addr.addr = (unsigned int)menu[index].val;
            pos += sprintf(&buf[pos], "<tr><td><label for=\"%s\">%s:</label></td><td><input type=\"text\" id=\"%s\" name=\"%s\" value=\"" IPSTR "\"/></td></tr>\n", menu[index].id, menu[index].name, menu[index].id, menu[index].id, IP2STR(&ip_addr));
        }
        else if (index == 6) // MAC
        {
            uint8_t mac_addr[6];
            mac_addr[0] = (menu[index].val >> 16) & 0xFF;
            mac_addr[1] = (menu[index].val >> 8) & 0xFF;
            mac_addr[2] = (menu[index].val >> 0) & 0xFF;
            mac_addr[3] = (menu[index + 1].val >> 16) & 0xFF;
            mac_addr[4] = (menu[index + 1].val >> 8) & 0xFF;
            mac_addr[5] = (menu[index + 1].val >> 0) & 0xFF;
            pos += sprintf(&buf[pos], "<tr><td><label for=\"%s\">%s:</label></td><td><input type=\"text\" id=\"%s\" name=\"%s\" value=\"" MACSTR "\"/></td></tr>\n", menu[index].id, menu[index].name, menu[index].id, menu[index].id, MAC2STR(mac_addr));
        }
        else if (index == 8 || index == 12 || index == 16 || index == 20) // Концевики
        {
            char e[8] = {0};
            if (menu[index + 3].val)
            {
                strcpy(e, "Вкл");
            };

            pos += sprintf(&buf[pos], "<tr><td><label for=\"%s\">%s:</label></td><td><input type=\"text\" id=\"%s\" name=\"%s\" value=\"%d %d %d\"/><b id=\"enable%s\">%s</b></td></tr>\n", menu[index].id, menu[index].name, menu[index].id, menu[index].id, menu[index].val, menu[index + 1].val, menu[index + 2].val, menu[index].id, e);
        }
        else if (strlen(menu[index].name) > 0)
        {
            pos += sprintf(&buf[pos], "<tr><td><label for=\"%s\">%s:</label></td><td><input type=\"text\" id=\"%s\" name=\"%s\" value=\"%d\"/>%s</td></tr>\n", menu[index].id, menu[index].name, menu[index].id, menu[index].id, menu[index].val, menu[index].izm);
        }
        else // hidden
        {
            pos += sprintf(&buf[pos], "<input type=\"hidden\" id=\"%s\" name=\"%s\" value=\"%d\">", menu[index].id, menu[index].id, menu[index].val);
        }

        index++;
    }

    if (pos > 0)
    {
        pos += sprintf(&buf[pos], "</table><br>");
    }
    else
    {
        index = 0;
    }

    return pos;
}

void console_task(void *arg)
{
    uint8_t serialbuffer[256];
    int selected_menu_id = 0;
    uint8_t *data = serialbuffer;

    uint8_t mac_addr[6];
    esp_ip4_addr_t ip_addr;

    int pos = 0;

    while (1)
    {
        const int c = fgetc(stdin);
        if (c > 0) // EOF = -1
        {
            if (c == '\n')
            {
                data[pos] = 0;

                const int nc = fgetc(stdin); // remove CRLF
                if (nc != '\n' && nc != '\r')
                    ungetc(nc, stdin);
            }
            else
            {
                if (pos < sizeof(serialbuffer))
                    data[pos++] = c;
            }
        }
        else
        {
            vTaskDelay(50 / portTICK_PERIOD_MS);
            continue;
        }

        xEventGroupSetBits(status_event_group, SERIAL_TERMINAL_ACTIVE);

        if (NB_terminal_mode)
        {
            if (c == '\n')
            {
                const char cl_return = '\r';
                uart_write_bytes(UART_NUM_1, &cl_return, 1);
            }
            else
            {
                uart_write_bytes(UART_NUM_1, &c, 1);
            }

            while (uart_read_bytes(UART_NUM_1, data, 1, 50 / portTICK_PERIOD_MS) > 0)
            {
                putchar(*data);
            }
            continue;
        }
        if (c == '\n')
        {
            xTaskNotify(xTaskI2C, NOTYFY_SENSOR_MAGACC_STOP, eSetBits);

            // ESP_LOG_BUFFER_HEXDUMP(TAG, data, pos + 1, ESP_LOG_INFO);
            // ESP_LOGD(TAG, "Read bytes: '%s'", data);
            int n = atoi((const char *)data);
            switch (selected_menu_id)
            {
            case 0:
                switch (n)
                {
                case 0: // Выводим меню

                    ESP_LOGI("result", OUT_JSON, get_menu_val_by_id("idn"), result.measure.bootcount, get_datetime(result.ttime), OUT_MEASURE_VARS(result.measure));

                    ESP_LOGI("menu", "-------------------------------------------");
                    int i = 0;
                    for (i = 0; i < sizeof(menu) / sizeof(menu_t); i++)
                    {
                        if (i == 3) // IP сервера
                        {
                            ip_addr.addr = (unsigned int)menu[i].val;
                            ESP_LOGI("menu", "%2i. %s: " IPSTR, i + 1, menu[i].name, IP2STR(&ip_addr));
                        }
                        else if (i == 6) // MAC
                        {
                            mac_addr[0] = (menu[6].val >> 16) & 0xFF;
                            mac_addr[1] = (menu[6].val >> 8) & 0xFF;
                            mac_addr[2] = (menu[6].val >> 0) & 0xFF;
                            mac_addr[3] = (menu[7].val >> 16) & 0xFF;
                            mac_addr[4] = (menu[7].val >> 8) & 0xFF;
                            mac_addr[5] = (menu[7].val >> 0) & 0xFF;
                            ESP_LOGI("menu", "%2i. %s: " MACSTR, i + 1, menu[i].name, MAC2STR(mac_addr));
                        }
                        else if (strlen(menu[i].name) > 0)
                            ESP_LOGI("menu", "%2i. %s: %i %s", i + 1, menu[i].name, menu[i].val, menu[i].izm);
                    }

                    ESP_LOGI("menu", "51. История: %u", bootCount);
                    ESP_LOGI("menu", "52. AT терминал NBIoT");
                    ESP_LOGI("menu", "53. Start WiFi");
                    ESP_LOGI("menu", "54. FreeRTOS INFO");
                    ESP_LOGI("menu", "60. Непрерывный опрос Mag/Acc");
                    ESP_LOGI("menu", "61. Непрерывный опрос Light");
                    ESP_LOGI("menu", "-------------------------------------------");
                    break;
                case 4: // IP сервера
                    ip_addr.addr = (unsigned int)menu[n - 1].val;
                    ESP_LOGI("menu", "-------------------------------------------");
                    ESP_LOGI("menu", "%2i. %s: " IPSTR ". Введите новое значение: ", n, menu[n - 1].name, IP2STR(&ip_addr));
                    ESP_LOGI("menu", "-------------------------------------------");
                    break;
                case 7: // MAC Address
                    mac_addr[0] = (menu[6].val >> 16) & 0xFF;
                    mac_addr[1] = (menu[6].val >> 8) & 0xFF;
                    mac_addr[2] = (menu[6].val >> 0) & 0xFF;
                    mac_addr[3] = (menu[7].val >> 16) & 0xFF;
                    mac_addr[4] = (menu[7].val >> 8) & 0xFF;
                    mac_addr[5] = (menu[7].val >> 0) & 0xFF;

                    ESP_LOGI("menu", "-------------------------------------------");
                    ESP_LOGI("menu", "%2i. %s: " MACSTR ". Введите новое значение: ", n, menu[n - 1].name, MAC2STR(mac_addr));
                    ESP_LOGI("menu", "-------------------------------------------");
                    break;
                case 51: // выводим историю
                    int cpos = history_pos + HISTORY_SIZE;
                    int cend = history_pos;
                    ESP_LOGI("menu", "-------------------------------------------");
                    ESP_LOGI("menu", "Datetime, Bootcount, " OUT_MEASURE_HEADERS);
                    while (cpos > cend)
                    {
                        int indx = cpos % HISTORY_SIZE;

                        ESP_LOGI("menu", "%s, %3u, " OUT_MEASURE_FORMATS, get_datetime(history[indx].ttime), history[indx].measure.bootcount, OUT_MEASURE_VARS(history[indx].measure));
                        cpos--;
                    }

                    ESP_LOGI("menu", "-------------------------------------------");
                    break;
                case 52: // AT терминал NBIoT
                    NB_terminal_mode = 1;
                    xEventGroupSetBits(status_event_group, NB_TERMINAL);
                    if (xHandleNB)
                        xTaskNotifyGive(xHandleNB); // если уже уснули
                    // vTaskSuspend(xHandleNB); // Suspend NBIot task
                    wait_max_counter = 3;
                    break;
                case 53: // WiFi
                    if (xHandleWifi)
                        xTaskNotifyGive(xHandleWifi); // включаем WiFi
                    break;
                case 54: // FreeRTOS INFO
                    ESP_LOGI("info", "Minimum free memory: %lu bytes", esp_get_minimum_free_heap_size());
                    ESP_LOGI("wifi_task", "Task watermark: %d bytes", uxTaskGetStackHighWaterMark(xHandleWifi));
                    // ESP_LOGI("adc_task", "Task watermark: %d bytes", uxTaskGetStackHighWaterMark(xHandleADC));
                    ESP_LOGI("modem_task", "Task watermark: %d bytes", uxTaskGetStackHighWaterMark(xHandleNB));
                    ESP_LOGI("console_task", "Task watermark: %d bytes", uxTaskGetStackHighWaterMark(NULL));
                    /*
                                        char statsbuf[600];
                                        vTaskGetRunTimeStats(statsbuf);
                                        printf(statsbuf);
                    */
                    break;
                case 60: // Непрерывный опрос MAG/ACC
                    xTaskNotify(xTaskI2C, NOTYFY_SENSOR_SET_MAGACC | NOTYFY_SENSOR_MAGACC_CONT, eSetValueWithOverwrite);
                    // заканчиваем работу NBIoT
                    xEventGroupSetBits(status_event_group, END_WORK_NBIOT);
                    // nbiot_power_off();
                    wait_max_counter = 3;
                    break;
                case 61:
                    xTaskNotify(xTaskI2C, NOTYFY_TEST, eSetValueWithOverwrite);
                    // заканчиваем работу NBIoT
                    xEventGroupSetBits(status_event_group, END_WORK_NBIOT);
                    // nbiot_power_off();
                    light_measure(10);
                    wait_max_counter = 3;
                    break;
                default:
                    if (n > 0 && n <= sizeof(menu) / sizeof(menu_t))
                    {
                        ESP_LOGI("menu", "-------------------------------------------");
                        ESP_LOGI("menu", "%2i. %s: %i %s. Введите новое значение: ", n, menu[n - 1].name, menu[n - 1].val, menu[n - 1].izm);
                        ESP_LOGI("menu", "-------------------------------------------");
                    }
                    break;
                }
                break;
            case 4: // IP сервера
                if (sscanf((const char *)serialbuffer, "%hhu.%hhu.%hhu.%hhu", &mac_addr[0], &mac_addr[1], &mac_addr[2], &mac_addr[3]) == 4)
                {
                    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
                    if (err == ESP_OK)
                    {
                        esp_ip4_addr_t ip_addr;
                        ip_addr.addr = (mac_addr[0] << 0) | (mac_addr[1] << 8) | (mac_addr[2] << 16) | (mac_addr[3] << 24);
                        ESP_LOGD("NVS", "Write  \"%s\" : \"" IPSTR "\"", menu[3].id, IP2STR(&ip_addr));
                        menu[3].val = (int)ip_addr.addr;
                        nvs_set_i32(my_handle, menu[3].id, menu[3].val);
                        nvs_close(my_handle);
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "Error IP address format");
                }
                break;
            case 7: // MAC ESPNOW!
                if (sscanf((const char *)serialbuffer, "%hhx%*[: -]%hhx%*[: -]%hhx%*[: -]%hhx%*[: -]%hhx%*[: -]%hhx",
                           &mac_addr[0], &mac_addr[1], &mac_addr[2], &mac_addr[3], &mac_addr[4], &mac_addr[5]) == 6)
                {
                    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
                    if (err == ESP_OK)
                    {
                        ESP_LOGD("NVS", "Write  \"%s\" : \"" MACSTR "\"", menu[6].id, MAC2STR(mac_addr));
                        menu[6].val = (mac_addr[0] << 16) | (mac_addr[1] << 8) | (mac_addr[2]);
                        nvs_set_i32(my_handle, menu[6].id, menu[6].val);
                        menu[7].val = (mac_addr[3] << 16) | (mac_addr[4] << 8) | (mac_addr[5]);
                        nvs_set_i32(my_handle, menu[7].id, menu[7].val);
                        nvs_close(my_handle);
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "Error MAC format");
                }
                break;
            default:
                if (selected_menu_id > 0 && selected_menu_id <= sizeof(menu) / sizeof(menu_t)) // selected_menu_id - номер пункта меню, n - value
                {
                    if (n >= menu[selected_menu_id - 1].min && n <= menu[selected_menu_id - 1].max)
                    {
                        menu[selected_menu_id - 1].val = n;
                        esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
                        if (err != ESP_OK)
                        {
                            ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
                        }
                        else
                        {
                            err = nvs_set_i32(my_handle, menu[selected_menu_id - 1].id, menu[selected_menu_id - 1].val);
                            if (err != ESP_OK)
                            {
                                ESP_LOGE(TAG, "%s", esp_err_to_name(err));
                            }
                            else
                            {
                                ESP_LOGI("menu", "-------------------------------------------");
                                ESP_LOGI("menu", "%2i. %s: %i %s.", selected_menu_id, menu[selected_menu_id - 1].name, menu[selected_menu_id - 1].val, menu[selected_menu_id - 1].izm);
                                ESP_LOGI("menu", "-------------------------------------------");
                            }
                        }

                        ESP_LOGD(TAG, "Committing updates in NVS ... ");
                        err = nvs_commit(my_handle);
                        if (err != ESP_OK)
                            ESP_LOGE(TAG, "Committing updates in NVS ... - Failed!");

                        // Close
                        nvs_close(my_handle);
                    }
                }
                break;
            }

            if (selected_menu_id == 0 && n > 0 && n <= sizeof(menu) / sizeof(menu_t))
                selected_menu_id = n;
            else
                selected_menu_id = 0;

            pos = 0;
        }
        vTaskDelay(1);
    }
}
