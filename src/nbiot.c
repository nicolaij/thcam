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

RTC_DATA_ATTR bool run_once = false;

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

esp_err_t at_reply_wait(const char *cmd, const char *wait, char *buffer, TickType_t timeout)
{
    int txBytes = uart_write_bytes(UART_NUM_1, cmd, strlen(cmd));
    if (txBytes < 4)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    int len = uart_read_bytes(UART_NUM_1, buffer, (RX_BUF_SIZE - 1), timeout);
    if (len < 4)
    {
        return ESP_ERR_TIMEOUT;
    };

    buffer[len] = '\0';
    ESP_LOGD(TAG, "Receive string:\"%s\"", (char *)buffer);

    if (strstr((const char *)buffer, wait) == NULL)
    {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t at_reply_wait_OK(const char *cmd, char *buffer, TickType_t timeout)
{
    esp_err_t res = ESP_FAIL;

    const char *wait = "OK\r\n";
    const char *err = "ERROR\r\n";

    ESP_LOGV(TAG, "Send string:\"%s\"", (char *)cmd);

    int txBytes = uart_write_bytes(UART_NUM_1, cmd, strlen(cmd));
    if (txBytes < 4)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    int64_t start_time = esp_timer_get_time();
    char *pb = buffer;
    *pb = '\0';
    res = ESP_ERR_TIMEOUT;
    while ((esp_timer_get_time() - start_time) < timeout * portTICK_PERIOD_MS * 1000)
    {
        int len = uart_read_bytes(UART_NUM_1, pb, (RX_BUF_SIZE - 1), pdMS_TO_TICKS(500));
        // ESP_LOGV(TAG, "len: %d", len);
        if (len > 0)
        {
            pb += len;
            *pb = '\0';

            if (strstr((const char *)buffer, wait) != NULL)
            {
                // ESP_LOGV(TAG, "Compare OK");
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
    }

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
esp_err_t at_reply_get(const char *cmd, const char *wait, char *buffer, int *resultdata, int resultcount, TickType_t timeout)
{
    int txBytes = uart_write_bytes(UART_NUM_1, cmd, strlen(cmd));
    if (txBytes < 4)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    int len = uart_read_bytes(UART_NUM_1, buffer, (RX_BUF_SIZE - 1), timeout);
    if (len < 4)
    {
        return ESP_ERR_TIMEOUT;
    };

    buffer[len] = '\0';
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

            ESP_LOGV(TAG, "Found string:\"%s\"", s);

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

    int len = uart_read_bytes(UART_NUM_1, buffer, (RX_BUF_SIZE - 1), 30000 / portTICK_PERIOD_MS);
    if (len < 4)
    {
        return ESP_ERR_TIMEOUT;
    };

    buffer[len] = '\0';
    ESP_LOGD(TAG, "Receive string:\"%s\"", (char *)buffer);

    return ESP_OK;
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
        .baud_rate = 9600,
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

    int try_counter = 1;

    char datetime[24];

    EventBits_t uxBits;

    strcpy(net_status_current, "OFF");

    while (1)
    {
        /* Ждем необходимости запуска передачи, либо зарядка*/
        uxBits = xEventGroupWaitBits(
            ready_event_group,          /* The event group being tested. */
            NEED_TRANSMIT | NOW_CHARGE, /* The bits within the event group to wait for. */
            pdFALSE,                    /* BIT_0 & BIT_1 should be cleared before returning. */
            pdFALSE,                    /* Don't wait for both bits, either bit will do. */
            portMAX_DELAY);

        while (1) // повторы опроса модуля
        {
            uart_flush(UART_NUM_1);

            esp_err_t ee = 0;
            // check modem
            ee = at_reply_wait("AT\r\n", "OK", (char *)data, 1000 / portTICK_PERIOD_MS);
            if (ee != ESP_OK)
            {
                ESP_LOGW(TAG, "Modem not reply");
                // power on
                nbiot_power_pin(1000 / portTICK_PERIOD_MS);

                vTaskDelay(2000 / portTICK_PERIOD_MS);
                continue;
            }

            // если запускаем терминал - стоп работа с модулем
            while (xEventGroupGetBits(ready_event_group) & NB_STOP)
            {
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }

            if (strnstr(net_status_current, "Error", sizeof(net_status_current)) == NULL) // Если ошибка SIM - то не перезаписываем ее
                strcpy(net_status_current, "Check SIM...");

            ee = at_reply_wait("ATE1\r\n", "OK", (char *)data, 1000 / portTICK_PERIOD_MS);

            // Battery Charge
            int cbc[2] = {-1, -1};
            ee = at_reply_get("AT+CBC\r\n", "CBC:", (char *)data, cbc, 2, 1000 / portTICK_PERIOD_MS);
            result.measure.nbbattery = cbc[1] / 1000.0;
            if (ee != ESP_OK)
            {
                ESP_LOGW(TAG, "AT+CBC");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            else
            {
                // Зарядка окончена
                if (cbc[1] >= get_menu_id("ubatt"))
                {
                    ESP_LOGI(TAG, "Charge complete");
                    xEventGroupSetBits(ready_event_group, CHARGE_COMPLETE);
                    stop_charge();

                    if (get_charge() == 0)
                        break;
                }
            }

            // если была зарядка
            if (uxBits & NOW_CHARGE)
            {
                if (get_charge() == 0) // зарядка окончена
                {
                    // STOP
                    break;
                }

                ESP_LOGI(TAG, "Modem Minimum functionality");
                at_reply_wait("AT+CFUN=0\r\n", "OK", (char *)data, 1000 / portTICK_PERIOD_MS);
                vTaskDelay(30000 / portTICK_PERIOD_MS);
                continue;
            }

            // Reset and Set Phone Functionality
            if ((try_counter % 5) == 0) // if fail restart sim
            {
                if (try_counter == 10)
                {
                    // STOP
                    break;
                };

                ESP_LOGI(TAG, "Modem CFUN Reset");
                at_reply_wait("AT+CFUN=0\r\n", "OK", (char *)data, 1000 / portTICK_PERIOD_MS);
                vTaskDelay(5000 / portTICK_PERIOD_MS);
                at_reply_wait("AT+CFUN=1\r\n", "OK", (char *)data, 1000 / portTICK_PERIOD_MS);
                vTaskDelay(5000 / portTICK_PERIOD_MS);
            }

            // Enter PIN
            ee = at_reply_wait("AT+CPIN?\r\n", "CPIN: READY", (char *)data, 1000 / portTICK_PERIOD_MS);
            if (ee != ESP_OK)
            {
                strcpy(net_status_current, "SIM Error!");
                result.measure.d_nbiot_sim_error = true;
                try_counter++;
                ESP_LOGW(TAG, "CPIN:\n%s", data);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                continue;
            }
            else
            {
                ESP_LOGI(TAG, "PIN OK");
            }

            result.measure.d_nbiot_sim_error = false;
            strcpy(net_status_current, "Network search...");

            // Network Registration Status
            int try_network = 500;
            while (try_network > 0)
            {
                int cgreg[2] = {-1, -1};
                ee = at_reply_get("AT+CGREG?\r\n", "CGREG:", (char *)data, cgreg, 2, 1000 / portTICK_PERIOD_MS);
                if (ee != ESP_OK)
                {
                    ESP_LOGW(TAG, "AT+CGREG?");
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
                else
                {
                    if (cgreg[1] == 1) // 1 Registered, home network.
                    {
                        ESP_LOGI(TAG, "Registered. Home network.");
                        break;
                    }
                }
                try_network--;

                // если запускаем терминал - стоп работа с модулем
                while (xEventGroupGetBits(ready_event_group) & NB_STOP)
                {
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
            }

            // Signal Quality Report
            int csq[2] = {-1, -1};
            ee = at_reply_get("AT+CSQ\r\n", "CSQ:", (char *)data, csq, 2, 1000 / portTICK_PERIOD_MS);
            result.measure.rssi = csq[0] * 2.0 + -113.0;
            if (ee != ESP_OK)
            {
                ESP_LOGW(TAG, "AT+CSQ");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }

            // Clock
            if (run_once == false)
            {
                // AT+CURTC? AT+CTZR?
                // ee = at_reply_wait_OK("AT+CTZR=?\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
                ee = at_reply_wait_OK("AT+CURTC=0\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
                ee = at_reply_wait_OK("AT+CTZU=1\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
                run_once = true;
            }
            ee = at_reply_wait("AT+CCLK?\r\n", "CCLK:", (char *)data, 1000 / portTICK_PERIOD_MS);
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
            ee = at_reply_wait("AT+IPCONFIG\r\n", "IPCONFIG:", (char *)data, 1000 / portTICK_PERIOD_MS);
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
            while (xEventGroupGetBits(ready_event_group) & NB_STOP)
            {
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }

            /*
                    //ping
                    //AT+CIPPING
                    ee = at_reply_wait_OK("AT+CIPPING=\"10.179.40.20\"\r\n", (char *)data, 60000 / portTICK_PERIOD_MS);
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

            ee = at_reply_wait_OK("AT+CSOSENDFLAG=1\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);

            int socket = 0;

            ee = at_reply_wait_OK("AT+CSOC=1,1,1\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
            if (ee != ESP_OK)
            {
                ESP_LOGW(TAG, "AT+CSOC=1,1,1");
            }
            else
            {
                const char *pdata = strstr((const char *)data, "CSOC: ");
                socket = atoi(pdata + 6);

                ESP_LOGI(TAG, "Socket %i connect...", socket);

                int port = get_menu_id("tcpport");
                int ip = get_menu_id("ip");

                try_counter = 3;
                while (try_counter)
                {
                    snprintf(send_data, sizeof(send_data), "AT+CSOCON=%i,%i,\"%i.%i.%i.%i\"\r\n", socket, port, (ip >> 24) & 0xff, (ip >> 16) & 0xff, (ip >> 8) & 0xff, (ip) & 0xff);
                    ee = at_reply_wait_OK(send_data, (char *)data, 60000 / portTICK_PERIOD_MS);
                    if (ee != ESP_OK)
                    {
                        ESP_LOGW(TAG, "AT+CSOCON:%s", data);
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
                        // ESP_LOGI(TAG, "AT+CSOCON:%s", data);
                        // snprintf(send_data, sizeof(send_data), "{\"id\":\"cam%d\",\"num\":%d,\"dt\":\"%s\",\"rssi\":%d,\"NBbatt\":%d,\"batt\":%.2f,\"adclight\":%.0f,\"adcwater\":%.0f,\"adcwater2\":%.0f,\"cputemp\":%.1f,\"temp\":%.1f,\"humidity\":%.1f,\"pressure\":%.3f}", get_menu_id("id"), result.bootCount, datetime, csq[0] * 2 + -113, cbc[1], result.measure.battery, result.measure.light, result.measure.water, result.measure.water2, result.measure.internal_temp, result.measure.temp, result.measure.humidity, result.measure.pressure);
                        snprintf(send_data, sizeof(send_data), OUT_JSON, get_menu_id("id"), result.bootCount, datetime, result.measure.rssi, result.measure.nbbattery, result.measure.light, result.measure.water, result.measure.water2, result.measure.internal_temp, result.measure.temp, result.measure.humidity, result.measure.pressure, result.measure.discrete);

                        ESP_LOGI(TAG, "Send...");

                        ee = at_csosend(socket, send_data, (char *)data);
                        if (ee == ESP_OK)
                        {
                            result.measure.d_nbiot_send_succes = true;
                            break;
                        }

                        ESP_LOGW(TAG, "AT+CSOSEND");
                    }
                    vTaskDelay(2000 / portTICK_PERIOD_MS);
                    try_counter--;

                    // если запускаем терминал - стоп работа с модулем
                    while (xEventGroupGetBits(ready_event_group) & NB_STOP)
                    {
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                    }
                }

                // wait to transmit
                // vTaskDelay(10000 / portTICK_PERIOD_MS);
            };

            snprintf(send_data, sizeof(send_data), "AT+CSOCL=%i\r\n", socket);
            at_reply_wait_OK(send_data, (char *)data, 1000 / portTICK_PERIOD_MS); // CLOSE socket
            break;
        }

        // если запускаем терминал - стоп работа с модулем
        while (xEventGroupGetBits(ready_event_group) & NB_STOP)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        // если есть бит END_WORK - то модуль уже выключили из main()
        if ((xEventGroupGetBits(ready_event_group) & END_WORK) == 0)
        {
            print_atcmd("AT+CPOWD=1\r\n", data);
            strcpy(net_status_current, "Success OFF");
            // print_atcmd("AT+CFUN=0\r\n", data);
        }
        else
        {
            strcpy(net_status_current, "Extern OFF");
        }

        xEventGroupSetBits(ready_event_group, END_RADIO_SLEEP);
        xEventGroupClearBits(ready_event_group, NEED_TRANSMIT | NOW_CHARGE);
    }
}

void nbiot_power_pin(const TickType_t xTicksToDelay)
{
    gpio_set_level(MODEM_POWER, 0);
    vTaskDelay(xTicksToDelay);
    gpio_set_level(MODEM_POWER, 1);
}
