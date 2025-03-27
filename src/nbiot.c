#include "main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include <sys/time.h>

#include "esp_timer.h"

static const int RX_BUF_SIZE = 1024;
static const int TX_BUF_SIZE = 512;
static const char *TAG = "NBIoT";

char pdp_ip[20];
char net_status_current[32];

RTC_DATA_ATTR bool run_first = false;

unsigned int fromActiveTime(uint8_t val)
{
    uint8_t unit = val >> 5;
    uint8_t value = val & 0b11111;

    switch (unit)
    {
    case 0:
        return value * 2;
        break;
    case 1:
        return value * 60;
        break;
    case 2:
        return value * 60 * 6;
        break;
    }

    return 0;
}

unsigned int fromPeriodicTAU(uint8_t val)
{
    uint8_t unit = val >> 5;
    uint8_t value = val & 0b11111;

    switch (unit)
    {
    case 0:
        return value * 60 * 10;
        break;
    case 1:
        return value * 60 * 60;
        break;
    case 2:
        return value * 60 * 60 * 10;
        break;
    case 3:
        return value * 2;
        break;
    case 4:
        return value * 30;
        break;
    case 5:
        return value * 60;
        break;
    case 6:
        return value * 60 * 60 * 320;
        break;
    }

    return 0;
}

esp_err_t print_atcmd(const char *cmd, char *buffer)
{
    int txBytes = uart_write_bytes(UART_NUM_1, cmd, strlen(cmd));
    if (txBytes < 4)
    {
        return ESP_FAIL;
    }

    int len = uart_read_bytes(UART_NUM_1, buffer, (RX_BUF_SIZE - 1), 500 / portTICK_PERIOD_MS);
    if (len < 4)
    {
        return ESP_FAIL;
    };

    buffer[len] = '\0';
    ESP_LOGD(TAG, "Receive string:\"%s\"", (char *)buffer);

    return ESP_OK;
}

esp_err_t wait_string(char *buffer, const char *wait, TickType_t ticks_to_wait)
{

    const char *err = "ERROR\r\n";

    int64_t start_time = esp_timer_get_time();
    char *pb = buffer;
    *pb = '\0';
    esp_err_t res = ESP_ERR_TIMEOUT;
    do
    {
        int len = uart_read_bytes(UART_NUM_1, pb, (RX_BUF_SIZE - 1), 1);
        // ESP_LOGV(TAG, "len: %d", len);
        if (len > 0)
        {
            pb += len;
            *pb = '\0';

            if (strstr((const char *)buffer, wait) != NULL)
            {
                res = ESP_OK;
                break;
            }
            else if (strstr((const char *)buffer, err) != NULL)
            {
                res = ESP_ERR_INVALID_STATE;
                break;
            }
        }
        else if (len == -1)
        {
            return ESP_FAIL;
        }
    } while ((esp_timer_get_time() - start_time) < ticks_to_wait * portTICK_PERIOD_MS * 1000);

    return res;
}

esp_err_t at_reply_wait(const char *cmd, const char *wait, char *buffer, TickType_t ticks_to_wait)
{
    esp_err_t res = ESP_ERR_TIMEOUT;
    int txBytes = uart_write_bytes(UART_NUM_1, cmd, strlen(cmd));
    if (txBytes < 4)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    res = wait_string(buffer, wait, ticks_to_wait);

    ESP_LOGD(TAG, "Receive string:\"%s\"", (char *)buffer);

    return res;
}

esp_err_t wait_OK(char *buffer, TickType_t ticks_to_wait)
{
    const char *wait = "OK\r\n";
    esp_err_t res = wait_string(buffer, wait, ticks_to_wait);

    return res;
}

esp_err_t at_reply_wait_OK(const char *cmd, char *buffer, TickType_t ticks_to_wait)
{
    esp_err_t res = ESP_FAIL;

    ESP_LOGV(TAG, "Send string:\"%s\"", (char *)cmd);

    int txBytes = uart_write_bytes(UART_NUM_1, cmd, strlen(cmd));
    if (txBytes < 4)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    res = wait_OK(buffer, ticks_to_wait);

    ESP_LOGD(TAG, "Receive string:\"%s\"", (char *)buffer);
    return res;
}
/*
cmd "AT+CMUX?"
wait "+CMUX:"
получаем результат "+CMUX: 0,0,0,31,10,3,30,10,2"
дальше пробел " "
дальше цифры через запятую "0,0,0,31,10,3,30,10,2" (9 штук)
*/
esp_err_t at_reply_get(const char *cmd, const char *wait, char *buffer, int *resultdata, int resultcount, TickType_t ticks_to_wait)
{
    int txBytes = uart_write_bytes(UART_NUM_1, cmd, strlen(cmd));
    if (txBytes < 4)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    esp_err_t res = wait_OK(buffer, ticks_to_wait);

    ESP_LOGD(TAG, "Receive string:\"%s\"", (char *)buffer);

    char *s = strstr((const char *)buffer, wait);
    if (s == NULL)
    {
        return ESP_FAIL;
    }
    else
    {
        for (int i = 0; i < resultcount; i++)
        {
            if (i == 0)
                s = strchr(s + 1, ' ');
            else
                s = strchr(s + 1, ',');

            if (s == NULL)
                return ESP_OK;

            ESP_LOGV(TAG, "Found string:\"%s\"", s);

            if (*(s + 1) == '"') // HEX STRING (ex CREG?)
                resultdata[i] = strtol(s + 2, NULL, 16);
            else
                resultdata[i] = atoi(s + 1);
        }
    }

    return ESP_OK;
}

/*
 Send Data to Remote Via Socket With Data Mode
*/
esp_err_t at_csosend(int socket, char *data, char *buffer)
{
    esp_err_t res = ESP_FAIL;
    char buf[14];
    int len_data = strlen(data);

    snprintf(buf, sizeof(buf), "AT+CSOSEND=%d,", socket);
    int txBytes = uart_write_bytes(UART_NUM_1, buf, strlen(buf));
    if (txBytes < 4)
    {
        return ESP_ERR_INVALID_SIZE;
    }
    snprintf(buf, sizeof(buf), "%d,", len_data * 2);
    txBytes = uart_write_bytes(UART_NUM_1, buf, strlen(buf));
    if (txBytes < 3)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    for (int i = 0; i < len_data; i++)
    {
        snprintf(buf, 3, "%02x", data[i]);
        txBytes = uart_write_bytes(UART_NUM_1, buf, 2);
        if (txBytes < 2)
        {
            return ESP_ERR_INVALID_SIZE;
        }
    }
    txBytes = uart_write_bytes(UART_NUM_1, "\r", 1);

    res = wait_OK(buffer, 30000 / portTICK_PERIOD_MS);
    ESP_LOGD(TAG, "Receive string:\"%s\"", (char *)buffer);

    return res;
}

esp_err_t at_csosend_wait_SEND(int socket, char *data, char *buffer)
{
    esp_err_t res = ESP_FAIL;

    char buf[14];
    int len_data = strlen(data);

    snprintf(buf, sizeof(buf), "AT+CSOSEND=%d,", socket);
    int txBytes = uart_write_bytes(UART_NUM_1, buf, strlen(buf));
    if (txBytes < 4)
    {
        return ESP_ERR_INVALID_SIZE;
    }
    snprintf(buf, sizeof(buf), "%d,", len_data * 2);
    txBytes = uart_write_bytes(UART_NUM_1, buf, strlen(buf));
    if (txBytes < 3)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    for (int i = 0; i < len_data; i++)
    {
        snprintf(buf, 3, "%02x", data[i]);
        txBytes = uart_write_bytes(UART_NUM_1, buf, 2);
        if (txBytes < 2)
        {
            return ESP_ERR_INVALID_SIZE;
        }
    }
    txBytes = uart_write_bytes(UART_NUM_1, "\r", 1);

    res = wait_string(buffer, "SEND:", 30000 / portTICK_PERIOD_MS);

    ESP_LOGD(TAG, "Receive string:\"%s\"", (char *)buffer);

    return res;
}

void modem_task(void *arg)
{
    char data[RX_BUF_SIZE];

    char send_data[TX_BUF_SIZE];

    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT_OD;
    // bit mask of the pins that you want to set
    io_conf.pin_bit_mask = BIT64(MODEM_POWER);
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    // disable pull-up mode
    io_conf.pull_up_en = 1;
    // configure GPIO with the given settings
    gpio_config(&io_conf);
    gpio_set_level(MODEM_POWER, 1);

    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    int try_counter = 0;

    char datetime[24];

    strcpy(net_status_current, "OFF");
    result.measure.d_nbiot_error = false;
    int d_nbiot_error_counter = 5;

    int protocol = 1; // TCP = 1, UDP =2

    while (1)
    {
        /* Ждем необходимости запуска передачи, либо зарядка*/
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Ожидаем уведомления беcконечно, для повторного опроса

        while (1) // повторы опроса модуля
        {
            uart_flush(UART_NUM_1);

            esp_err_t ee = 0;

            // check modem
            ee = at_reply_wait_OK("AT\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
            if (ee != ESP_OK)
            {
                result.measure.d_nbiot_error = true;
                d_nbiot_error_counter--;

                ESP_LOGW(TAG, "Modem not reply");
                strcpy(net_status_current, "Modem not reply");

                if ((xEventGroupGetBits(status_event_group) & END_WORK_NBIOT) || d_nbiot_error_counter == 0)
                    break;

#ifdef NBIOT_PSM
                if (d_nbiot_error_counter == 1)
                {
                    // power on
                    nbiot_power_pin(1000 / portTICK_PERIOD_MS);
                    vTaskDelay(3000 / portTICK_PERIOD_MS);
                }
                else
                {
                    gpio_set_level(MODEM_POWER, 0);
                    vTaskDelay(10);
                    gpio_set_level(MODEM_POWER, 1);
                    vTaskDelay(1000 / portTICK_PERIOD_MS);

                    /*
                                        int64_t start_time = esp_timer_get_time();
                                        gpio_set_level(MODEM_POWER, 0);
                                        // wait 5ms
                                        while ((esp_timer_get_time() - start_time) < (5000LL))
                                        {
                                            portNOP();
                                        }
                                        ESP_LOGW(TAG, "time: %lli", (esp_timer_get_time() - start_time));
                                        gpio_set_level(MODEM_POWER, 1);
                    */
                }
#else
                // power on
                nbiot_power_pin(1000 / portTICK_PERIOD_MS);
                vTaskDelay(2000 / portTICK_PERIOD_MS);
#endif
                continue;
            }

            ee = at_reply_wait_OK("ATE1;+IPR=115200\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);

            strcpy(net_status_current, "ready");
            result.measure.d_nbiot_error = false;

            // если запускаем терминал - стоп работа с модулем
            if (xEventGroupGetBits(status_event_group) & NB_TERMINAL)
            {
                break;
            }

            // ee = at_reply_wait("ATE1\r\n", "OK", (char *)data, 1000 / portTICK_PERIOD_MS);

            // Battery Charge
            int cbc[2] = {-1, -1};

            do // loop when charge
            {
                ee = at_reply_get("AT+CBC\r\n", "CBC:", (char *)data, cbc, 2, 1000 / portTICK_PERIOD_MS);
                result.measure.nbbattery = cbc[1] / 1000.0;
                if (ee != ESP_OK)
                {
                    ESP_LOGW(TAG, "AT+CBC");
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
                else
                {
                    result.measure.d_nbiot_error = false;

                    // Зарядка окончена. Передаем информацию
                    if (result.measure.nbbattery > 3.5)
                    {
                        ESP_LOGI(TAG, "Charge complete");
                        xEventGroupSetBits(status_event_group, CHARGE_COMPLETE);
                        break;
                    };
                };

                if ((xEventGroupGetBits(status_event_group) & NOW_CHARGE) || get_charge())
                {
                    vTaskDelay(25000 / portTICK_PERIOD_MS);
                }

            } while (((xEventGroupGetBits(status_event_group) & NOW_CHARGE) || get_charge()) && (xEventGroupGetBits(status_event_group) & NB_TERMINAL) == 0);

            // если запускаем терминал - стоп работа с модулем
            if (xEventGroupGetBits(status_event_group) & NB_TERMINAL)
            {
                break;
            }

            if (strnstr(net_status_current, "Error", sizeof(net_status_current)) == NULL) // Если ошибка SIM - то не перезаписываем ее
                strcpy(net_status_current, "Check SIM...");

            // Enter PIN
            ee = at_reply_wait("AT+CPIN?\r\n", "CPIN: READY", (char *)data, 1000 / portTICK_PERIOD_MS);
            if (ee != ESP_OK)
            {
                strcpy(net_status_current, "SIM Error!");
                result.measure.d_nbiot_error = true;
                try_counter++;
                ESP_LOGW(TAG, "CPIN:\n%s", data);

                if ((xEventGroupGetBits(status_event_group) & END_WORK_NBIOT) || d_nbiot_error_counter-- == 0)
                    break;

                // Reset and Set Phone Functionality
                if ((try_counter % 3) == 0) // if fail restart sim
                {
                    ESP_LOGI(TAG, "Modem CFUN Reset");
                    at_reply_wait_OK("AT+CFUN=0\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
                    vTaskDelay(5000 / portTICK_PERIOD_MS);
                    at_reply_wait_OK("AT+CFUN=1\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
                    vTaskDelay(5000 / portTICK_PERIOD_MS);
                }

                vTaskDelay(1000 / portTICK_PERIOD_MS);
                continue;
            }
            else
            {
                ESP_LOGI(TAG, "PIN OK");
            }

            result.measure.d_nbiot_error = false;
            strcpy(net_status_current, "Network search...");

#ifdef NBIOT_PSM
            if (run_first == false)
            {
                at_reply_wait_OK("AT+CEREG=5\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
            }
            //+CEREG: 5,1,"00A0","0002920C",9,"00",0,0,"00100010","00010010"

            // Network Registration Status
            int try_network = 60;
            int tAT = 0;
            int tRT = 0;
            while (try_network > 0)
            {
                int creg[10] = {-1, -1};
                ee = at_reply_get("AT+CEREG?\r\n", "CEREG:", (char *)data, creg, 10, 1000 / portTICK_PERIOD_MS);
                if (ee != ESP_OK)
                {
                    ESP_LOGW(TAG, "AT+CREG?");
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
                else
                {
                    if (creg[1] == 1) // 1 Registered, home network.
                    {
                        char bf[10];
                        snprintf(bf, 9, "%8X", creg[8]);
                        tAT = fromActiveTime(strtol(bf, NULL, 2));
                        snprintf(bf, 9, "%8X", creg[9]);
                        tRT = fromPeriodicTAU(strtol(bf, NULL, 2));
                        ESP_LOGI(TAG, "Registered. Home network. TAC=%u, CI=%u Active-Time=%02d:%02d:%02d Periodic-TAU=%02d:%02d:%02d", creg[2], creg[3], (tAT / (60 * 60)), (tAT / 60) % 60, (tAT % 60), (tRT / (60 * 60)), (tRT / 60) % 60, (tRT % 60));
                        break;
                    }
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
                try_network--;

                // если запускаем терминал - стоп работа с модулем
                if (xEventGroupGetBits(status_event_group) & NB_TERMINAL)
                {
                    break;
                }
            }
#else
            at_reply_wait_OK("AT+CREG=2\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);

            // Network Registration Status
            int try_network = 500;
            while (try_network > 0)
            {
                int creg[7] = {-1, -1};
                ee = at_reply_get("AT+CREG?\r\n", "CREG:", (char *)data, creg, 5, 1000 / portTICK_PERIOD_MS);
                if (ee != ESP_OK)
                {
                    ESP_LOGW(TAG, "AT+CREG?");
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
                else
                {
                    if (creg[1] == 1) // 1 Registered, home network.
                    {
                        ESP_LOGI(TAG, "Registered. Home network. TAC=%u, CI=%u", creg[2], creg[3]);
                        break;
                    }
                }
                try_network--;

                // если запускаем терминал - стоп работа с модулем
                if (xEventGroupGetBits(status_event_group) & NB_TERMINAL)
                {
                    break;
                }
            }
#endif

            // Signal Quality Report
            int csq[2] = {-1, -1};
            ee = at_reply_get("AT+CSQ\r\n", "CSQ:", (char *)data, csq, 2, 1000 / portTICK_PERIOD_MS);
            if (ee != ESP_OK)
            {
                ESP_LOGW(TAG, "AT+CSQ");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            else
            {
                result.measure.rssi = csq[0] * 2.0 + -113.0;
                ESP_LOGI(TAG, "RSSI: %.0f", result.measure.rssi);
            }

            // Clock
            if (run_first == false)
            {
                // AT+CURTC? AT+CTZR?
                // ee = at_reply_wait_OK("AT+CTZR=?\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
                ee = at_reply_wait_OK("AT+CURTC=0\r\n", (char *)data, 1000 / portTICK_PERIOD_MS); // CCLK show UTC time after network time synchronization
                ee = at_reply_wait_OK("AT+CTZU=1\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);  // Automatic time update via NITZ

#ifdef NBIOT_PSM
                at_reply_wait_OK("AT+CPSMSTATUS=1\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
                // #TAU 30sec * 3 , ACC 8 sec
                at_reply_wait_OK("AT+CPSMS=1,,,\"10000011\",\"00000100\"\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
#endif

                run_first = true;
            }

            ee = at_reply_wait_OK("AT+CCLK?\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
            if (ee != ESP_OK)
            {
                ESP_LOGW(TAG, "AT+CCLK?");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            else
            {
                int dt[] = {0, 0, 0, 0, 0, 0, 0, 0};
                const char *pdata = strstr((const char *)data, "CCLK:");

                //+CCLK: 24/04/30,07:49:36+12
                char *s = strchr(pdata, ' ');
                if (s)
                {
                    // year
                    dt[0] = atoi(s + 1);
                    if (dt[0] >= 0 && dt[0] < 100)
                        dt[0] = dt[0] + 2000;
                    s = strchr(s + 1, '/');
                    if (s)
                    {
                        // month
                        dt[1] = atoi(s + 1);
                        s = strchr(s + 1, '/');
                        if (s)
                        {
                            // day
                            dt[2] = atoi(s + 1);
                            s = strchr(s + 1, ',');
                            if (s)
                            {
                                // hour
                                dt[3] = atoi(s + 1);
                                s = strchr(s + 1, ':');
                                if (s)
                                {
                                    // minute
                                    dt[4] = atoi(s + 1);
                                    s = strchr(s + 1, ':');
                                    if (s)
                                    {
                                        // second
                                        dt[5] = atoi(s + 1);

                                        // + - timezone
                                        dt[6] = 0;
                                        char *tzs = strchr(s + 1, '-');
                                        if (!tzs)
                                        {
                                            tzs = strchr(s + 1, '+');
                                        }

                                        if (tzs)
                                        {
                                            dt[6] = atoi(tzs);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                struct tm tm;
                tm.tm_year = dt[0] - 1900;
                tm.tm_mon = dt[1] - 1;
                tm.tm_mday = dt[2];
                tm.tm_hour = dt[3];
                tm.tm_min = dt[4];
                tm.tm_sec = dt[5];

                dt[6] = 3; // FORCE TIMEZONE

                time_t t = mktime(&tm) + dt[6] * 3600; // UNIX time + timezone offset
                struct timeval now = {.tv_sec = t};
                settimeofday(&now, NULL);
                strftime(datetime, sizeof(datetime), "%Y-%m-%d %T", &tm);
                ESP_LOGI(TAG, "Set date and time: %s", datetime);
            }

            // get current date time
            result.ttime = time(0);
            struct tm *localtm = localtime(&result.ttime);
            strftime(datetime, sizeof(datetime), "%Y-%m-%d %T", localtm);

            // ee = at_reply_wait_OK("AT+CTZU?\r\n", (char *)data, 10000 / portTICK_PERIOD_MS);

            // Show the Complete PDP Address
            ee = at_reply_wait_OK("AT+IPCONFIG\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
            if (ee != ESP_OK)
            {
                ESP_LOGW(TAG, "AT+IPCONFIG");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            else
            {
                const char *pdata = strstr((const char *)data, "IPCONFIG:");
                char *s = strchr(pdata, ' ');
                if (s)
                {
                    char *s_end = strchr(s, '\r');
                    if (s_end)
                        *s_end = 0;

                    strncpy(pdp_ip, s + 1, 18);
                };

                strncpy(pdp_ip, s + 1, 18);

                // если нет нормального IP - рестарт модуля
                if (atoi(pdp_ip) == 127)
                {
                    ESP_LOGE(TAG, "IP: %s", pdp_ip);
                    print_atcmd("AT+CPOWD=1\r\n", data);
                    continue;
                }
                else
                {
                    ESP_LOGI(TAG, "IP: %s", pdp_ip);
                }
            };

            // если запускаем терминал - стоп работа с модулем
            if (xEventGroupGetBits(status_event_group) & NB_TERMINAL)
            {
                break;
            }

            int ip = get_menu_val_by_id("ip");
            /*
                        // ping
                        // AT+CIPPING

                        snprintf(send_data, sizeof(send_data), "AT+CIPPING=\"%i.%i.%i.%i\"\r\n", (ip >> 24) & 0xff, (ip >> 16) & 0xff, (ip >> 8) & 0xff, (ip) & 0xff);
                        ee = at_reply_wait_OK(send_data, (char *)data, 60000 / portTICK_PERIOD_MS);
                        if (ee != ESP_OK)
                        {
                            ESP_LOGW(TAG, "AT+CIPPING:%s", data);
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                        }

                        vTaskDelay(5000 / portTICK_PERIOD_MS);

                        ee = at_reply_wait_OK("AT+CIPPING?\r\n", (char *)data, 60000 / portTICK_PERIOD_MS);
                        if (ee != ESP_OK)
                        {
                            ESP_LOGW(TAG, "AT+CIPPING?:%s", data);
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                        }
            */
            // TCP Connect
            /*
            AT+CSOC=1,1,1
            +CSOC: 0
            OK
            //Created one TCP socket, <socket_id>=0
            AT+CSOCON=0,5245,"116.247.119.165"
            OK
            //Connected remote TCP server
            AT+CSOSEND=0,0,”Hello World”
            OK
            //Send TCP data out
            AT+CSOCL=0 //Close socket
            */

            strcpy(net_status_current, "Send data...");

            int socket = 0;

            int tcpport = get_menu_val_by_id("tcpport");
            int udpport = get_menu_val_by_id("udpport");

            if (tcpport > 0)
            {
                protocol = 1;
                ee = at_reply_wait_OK("AT+CSOSENDFLAG=1\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
            }
            else if (udpport > 0)
            {
                protocol = 2;
                tcpport = udpport;
            }
            else
            {
                break;
            }

            snprintf(send_data, sizeof(send_data), "AT+CSOC=1,%i,1\r\n", protocol);
            at_reply_wait_OK(send_data, (char *)data, 1000 / portTICK_PERIOD_MS); // Create socket
            if (ee != ESP_OK)
            {
                ESP_LOGW(TAG, "AT+CSOC");
            }
            else
            {
                const char *pdata = strstr((const char *)data, "CSOC: ");
                socket = atoi(pdata + 6);

                ESP_LOGI(TAG, "Socket %i connect...", socket);

                try_counter = 3;
                while (try_counter)
                {
                    snprintf(send_data, sizeof(send_data), "AT+CSOCON=%i,%i,\"%i.%i.%i.%i\"\r\n", socket, tcpport, (ip >> 24) & 0xff, (ip >> 16) & 0xff, (ip >> 8) & 0xff, (ip) & 0xff);
                    ee = at_reply_wait_OK(send_data, (char *)data, 60000 / portTICK_PERIOD_MS);
                    if (ee != ESP_OK)
                    {
                        ESP_LOGW(TAG, "AT+CSOCON:%s", data);
                        result.measure.d_nbiot_error = true;
                        /* ping
                            // AT+CIPPING
                            snprintf(send_data, sizeof(send_data), "AT+CIPPING=\"%i.%i.%i.%i\"\r\n", (ip >> 24) & 0xff, (ip >> 16) & 0xff, (ip >> 8) & 0xff, (ip) & 0xff);
                            ee = at_reply_wait(send_data, "CIPPING", (char *)data, 40000 / portTICK_PERIOD_MS);
                            if (ee != ESP_OK)
                            {
                                ESP_LOGW(TAG, "AT+CIPPING:%s", data);
                                vTaskDelay(3000 / portTICK_PERIOD_MS);
                            }
                        */
                    }
                    else
                    {
                        result.measure.d_nbiot_error = false;
                        // ESP_LOGI(TAG, "AT+CSOCON:%s", data);
                        // snprintf(send_data, sizeof(send_data), "{\"id\":\"cam%d\",\"num\":%d,\"dt\":\"%s\",\"rssi\":%d,\"NBbatt\":%d,\"batt\":%.2f,\"adclight\":%.0f,\"adcwater\":%.0f,\"adcwater2\":%.0f,\"cputemp\":%.1f,\"temp\":%.1f,\"humidity\":%.1f,\"pressure\":%.3f}", get_menu_id("id"), result.bootCount, datetime, csq[0] * 2 + -113, cbc[1], result.measure.battery, result.measure.light, result.measure.water, result.measure.water2, result.measure.internal_temp, result.measure.temp, result.measure.humidity, result.measure.pressure);
                        snprintf(send_data, sizeof(send_data), OUT_JSON, get_menu_val_by_id("id"), result.measure.bootcount, datetime, OUT_MEASURE_VARS(result.measure));

                        ESP_LOGI(TAG, "Send...");

                        if (protocol == 1) // TCP
                            ee = at_csosend_wait_SEND(socket, send_data, (char *)data);

                        if (protocol == 2) // UDP
                            ee = at_csosend(socket, send_data, (char *)data);

                        if (ee == ESP_OK)
                        {
                            // print_atcmd("AT+CSOACK\r\n", (char *)data);
                            // vTaskDelay(2000 / portTICK_PERIOD_MS);

                            result.measure.d_nbiot_send_succes = true;
                            break;
                        }

                        ESP_LOGW(TAG, "AT+CSOSEND");
                    }
                    vTaskDelay(2000 / portTICK_PERIOD_MS);
                    try_counter--;
                }

                // wait to transmit
                // vTaskDelay(10000 / portTICK_PERIOD_MS);
            };

            // wait 10s for reply from server
            int64_t start_time = esp_timer_get_time();
            do
            {
                if (wait_string(data, "\r\n", 1000 / portTICK_PERIOD_MS) == ESP_OK)
                {
                    ESP_LOGI(TAG, "Modem: %s", data);
                    const char *pdata = strstr((const char *)data, "+CSONMI: ");
                    if (pdata)
                    { //+CSONMI: 0,20,5468616E6B20796F7521
                        char *s = strchr(pdata, ',');
                        if (s)
                        {
                            // len
                            int l = atoi(s + 1);
                            s = strchr(s + 1, ',');
                            if (s++)
                            {
                                // message
                                for (int i = 0; i < l; i = i + 2)
                                {
                                    char c[3] = {*(s++), *(s++), 0};
                                    send_data[i / 2] = (char)strtol(c, NULL, 16);
                                }
                                send_data[l / 2] = '\0';
                                ESP_LOGI(TAG, "Text: %s", send_data);
                                break;
                            }
                        }
                    }
                }
            } while ((esp_timer_get_time() - start_time) < 10 * 1000000);

            if (protocol == 1) // TCP
            {
                snprintf(send_data, sizeof(send_data), "AT+CSOCL=%i\r\n", socket);
                at_reply_wait_OK(send_data, (char *)data, 1000 / portTICK_PERIOD_MS); // CLOSE socket
            }
            break;
        };

        // clear notify
        ulTaskNotifyTake(pdTRUE, 0);

        // если есть бит END_WORK - то модуль уже выключили из main()
        // if ((xEventGroupGetBits(status_event_group) & END_WORK) == 0)
        //{
        // ВЫКЛЮЧАЕМ
#if !defined NBIOT_PSM
        // если запускаем терминал - стоп работа с модулем
        if (xEventGroupGetBits(status_event_group) & NB_TERMINAL)
        {
            continue;
        }

        if (print_atcmd("AT+CPOWD=1\r\n", data) == ESP_OK)
            strcpy(net_status_current, "Success OFF");
#endif
        // print_atcmd("AT+CFUN=0\r\n", data);
        //}
        // else
        //{
        //    strcpy(net_status_current, "Extern OFF");
        //}

        xEventGroupSetBits(status_event_group, END_RADIO);
    }
}

void nbiot_power_pin(const TickType_t xTicksToDelay)
{
    gpio_set_level(MODEM_POWER, 0);
    vTaskDelay(xTicksToDelay);
    gpio_set_level(MODEM_POWER, 1);
};

void nbiot_power_off()
{
    ESP_LOGW("main", "Force power off NB-IoT");
    nbiot_power_pin(2000 / portTICK_PERIOD_MS);
};
