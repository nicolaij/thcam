#include "main.h"
#include "NBIoTlib.h"
#include "esp_wifi.h"

static const char *TAG = "NBIoT";
esp_ip4_addr_t pdp_ip;
char net_status_current[32];

RTC_DATA_ATTR bool first_run_completed = false;

extern int32_t timezone;

void modem_task(void *arg)
{
    char data[RX_BUF_SIZE];
    char send_data[TX_BUF_SIZE];

    bool cpsms0 = false;

    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT_OD;
    io_conf.pin_bit_mask = BIT64(MODEM_POWER);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(MODEM_POWER, 1));

    ESP_ERROR_CHECK(uart_init(UART_NUM_1, RXD_PIN, TXD_PIN));

    int try_counter = 0;

    strcpy(net_status_current, "OFF");
    // result.measure.d_nbiot_error = true;

    int protocol = 1; // ESPNOW = 0, TCP = 1, UDP =2

    const int mac1 = get_menu_val_by_id("MAC1");
    const int mac2 = get_menu_val_by_id("MAC2");
    uint8_t mac_addr[6] = {
        (mac1 >> 16) & 0xFF,
        (mac1 >> 8) & 0xFF,
        (mac1 >> 0) & 0xFF,
        (mac2 >> 16) & 0xFF,
        (mac2 >> 8) & 0xFF,
        (mac2 >> 0) & 0xFF,
    };

    bool espnow_need_send = false;

    if (mac1 > 0 || mac2 > 0)
    {
        espnow_need_send = true;
    }
    int chip = 0;

    while (1)
    {
        /* Ждем необходимости запуска передачи, либо зарядка*/
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Ожидаем уведомления беcконечно, для повторного опроса

        bool cpin = false;
        int d_nbiot_error_counter = 0;

        int stat = -1;
        int tac = -1;
        int ci = -1;
        int AcT = -1;
        int ActiveTime = -1;
        int PeriodicTAU = -1;

        while (1) // повторы опроса модуля
        {
            esp_err_t ee = 0;

            bool network_registration = false;

            int tAT = 0;
            int tRT = 0;

#if HW != 11
            switch (d_nbiot_error_counter++ % 4)
            {
            case 1:
            case 3:
                // power on
                ESP_LOGD(TAG, "Try %d. Power ON", d_nbiot_error_counter);
                nbiot_power_pin(1000 / portTICK_PERIOD_MS, MODEM_POWER);
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                break;
            case 2:
                // power off
                ESP_LOGD(TAG, "Try %d. Power OFF", d_nbiot_error_counter);
                nbiot_power_pin(1500 / portTICK_PERIOD_MS, MODEM_POWER);
                vTaskDelay(5000 / portTICK_PERIOD_MS);
                break;
            default:
                // sleep exit
                ESP_LOGD(TAG, "Try %d. Wakeup", d_nbiot_error_counter);
                nbiot_power_pin(100 / portTICK_PERIOD_MS, MODEM_POWER);
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                break;
            }

            ee = wait_string(data, "\"EXIT PSM\"", 1000 / portTICK_PERIOD_MS);

            ESP_LOGD(TAG, "Wait... %s", data);

            if (strstr((const char *)data, "CPIN: READY") != NULL)
            {
                cpin = true;
                ESP_LOGI(TAG, "CPIN: READY");
            }
            else
            {
                // check modem
                ee = at_reply_wait_OK("ATI\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);

                if (ee == ESP_OK)
                {
                    if (strnstr((const char *)data, "SIM7028", 20) != NULL)
                        chip = 7028;
                }
                else
                {
                    ESP_LOGW(TAG, "Modem not reply");
                    strcpy(net_status_current, "Modem not reply");

                    if (d_nbiot_error_counter > 8)
                        break;

                    continue;
                }

                /*                if (first_run_completed == false)
                                    at_reply_wait_OK("ATE0;+IPR=115200\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
                */
            };
#else
            chip = 7028;
            do
            {
                ESP_LOGD(TAG, "Try %d. Wakeup", ++d_nbiot_error_counter);
                gpio_set_level(MODEM_POWER, 0);
                ee = at_reply_wait_OK("AT\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
            } while (ee != ESP_OK && d_nbiot_error_counter < 5);

#endif
            ee = at_reply_wait_OK("ATE1;+IPR=115200\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
            ee = at_reply_wait_OK("ATI\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);

            if (check_cereg(data, chip, &stat, &tac, &ci, &AcT, &ActiveTime, &PeriodicTAU) == ESP_OK)
            {
                network_registration = true;
            };

            if ((xEventGroupGetBits(status_event_group) & END_WORK_NBIOT))
                break;

            strcpy(net_status_current, "ready");
            // result.measure.d_nbiot_error = false;

            // если запускаем терминал - стоп работа с модулем
            if (xEventGroupGetBits(status_event_group) & NB_TERMINAL)
            {
                break;
            }

            // Battery Charge
            int cbc[2] = {-1, -1};
            if (chip == 7028)
                ee = at_reply_get("AT+QCADC=Vbat\r\n", "VBAT, ", (char *)data, &cbc[1], 1, 1000 / portTICK_PERIOD_MS);
            else
                ee = at_reply_get("AT+CBC\r\n", "CBC:", (char *)data, cbc, 2, 1000 / portTICK_PERIOD_MS);
            result.measure.nbbattery = cbc[1] / 1000.0;
            if (ee != ESP_OK)
            {
                ESP_LOGE(TAG, "AT+CBC");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            else
            {
                d_nbiot_error_counter = 0;

                // Зарядка окончена. Передаем информацию
                if (result.measure.nbbattery > 3.5)
                {
                    ESP_LOGI(TAG, "Charge complete");
                    xEventGroupSetBits(status_event_group, CHARGE_COMPLETE);
                };
            };

            if (get_charge())
            {
                result.ttime = time(0); // ОБНОВЛЯЕМ ВРЕМЯ НА ЗАРЯДКЕ
                                        /*
                                                        if (!cpsms0)
                                                        {
                                                            at_reply_wait_OK("AT+CPSMS=0\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
                                                            cpsms0 = true;
                                                        }
                                        */
                break;                  // стоп работа с модулем
            }

            // если запускаем терминал - стоп работа с модулем
            if (xEventGroupGetBits(status_event_group) & NB_TERMINAL)
            {
                at_reply_wait_OK("AT+CPSMS=0\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
                cpsms0 = true;
                break; // стоп работа с модулем
            }

            if (cpin == false)
            {
                if (strnstr(net_status_current, "Error", sizeof(net_status_current)) == NULL) // Если ошибка SIM - то не перезаписываем ее
                    strcpy(net_status_current, "Check SIM...");

                // Enter PIN
                ee = at_reply_wait_OK("AT+CPIN?\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
                if (ee != ESP_OK || strstr(data, "CPIN: READY") == NULL)
                {
                    strcpy(net_status_current, "SIM Error!");
                    // result.measure.d_nbiot_error = true;
                    try_counter++;
                    ESP_LOGW(TAG, "CPIN:\n%s", data);

                    if ((xEventGroupGetBits(status_event_group) & END_WORK_NBIOT))
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
                    if (try_counter > 4)
                    {
                        if (chip == 7028)
                        {
                            gpio_set_level(MODEM_POWER, 1);
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                        }
                        else
                        {
                            // power off
                            if (print_atcmd("AT+CPOWD=1\r\n", data) == ESP_OK)
                                ESP_LOGI(TAG, "Power DOWN");
                        }
                        break;
                    }
                    else
                        continue;
                }
                else
                {
                    ESP_LOGI(TAG, "PIN OK");
                }
            }

            // result.measure.d_nbiot_error = false;
            strcpy(net_status_current, "Network search...");

            if (first_run_completed == false)
            {
                at_reply_wait_OK("AT+CEREG=5\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
            }
            //+CEREG: 5,1,"00A0","0002920C",9,"00",0,0,"00100010","00010010"

            // Network Registration Status
            int try_network = 120;
            if (network_registration == false)
            {
                while (try_network > 0)
                {
                    at_reply_wait_OK("AT+CEREG?\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);

                    // ee = at_reply_get("AT+CEREG?\r\n", "CEREG: 5", (char *)data, creg, 10, 1000 / portTICK_PERIOD_MS);
                    if (check_cereg(data, chip, &stat, &tac, &ci, &AcT, &ActiveTime, &PeriodicTAU) == ESP_OK)
                    {
                        /*                         if (creg[0] != 5) // Исправится следующий раз
                                                {
                                                    first_run_completed = false;
                                                } */

                        if (stat == 1) // 1 Registered, home network.
                        {
                            network_registration = true;
                            break;
                        }
                    }

                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    try_network--;

                    // если запускаем терминал - стоп работа с модулем
                    if (xEventGroupGetBits(status_event_group) & NB_TERMINAL)
                    {
                        // Disable the use of PSM and discard all parameters for PSM or, if available reset to the manufacturer specific default values.
                        print_atcmd("AT+CPSMS=2\r\n", data);
                        cpsms0 = true;
                        break;
                    }
                }
            }

            if (network_registration)
            {
                char bf[10];
                snprintf(bf, 9, "%8X", ActiveTime);
                tAT = fromActiveTime(strtol(bf, NULL, 2));
                snprintf(bf, 9, "%8X", PeriodicTAU);
                tRT = fromPeriodicTAU(strtol(bf, NULL, 2));
                ESP_LOGI(TAG, "Registered. Home network. TAC=%u, CI=%u Active-Time=%02d:%02d:%02d Periodic-TAU=%02d:%02d:%02d", tac, ci, (tAT / (60 * 60)), (tAT / 60) % 60, (tAT % 60), (tRT / (60 * 60)), (tRT / 60) % 60, (tRT % 60));

                result.measure.tac = tac;
                result.measure.ci = ci;
            }

            // Signal Quality Report
            int csq[2] = {-1, -1};
            ee = at_reply_get("AT+CSQ\r\n", "+CSQ: ", (char *)data, csq, 2, 1000 / portTICK_PERIOD_MS);
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
            if (first_run_completed == false || cpsms0)
            {
                // AT+CURTC? AT+CTZR?
                // ee = at_reply_wait_OK("AT+CTZR=?\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
                if (chip != 7028)
                    at_reply_wait_OK("AT+CURTC=0\r\n", (char *)data, 1000 / portTICK_PERIOD_MS); // CCLK show UTC time after network time synchronization

                at_reply_wait_OK("AT+CTZU=1\r\n", (char *)data, 1000 / portTICK_PERIOD_MS); // Automatic time update via NITZ

                if (chip != 7028)
                    at_reply_wait_OK("AT+CPSMSTATUS=1\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
                // Enable PSM mode
                //  #TAU 30sec * 3 , ACC 8 sec
                //  at_reply_wait_OK("AT+CPSMS=1,,,\"10000011\",\"00000100\"\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
                //  TAU 25h 1*25, ACC 0 sec
                at_reply_wait_OK("AT+CPSMS=1,,,\"00111001\",\"00000000\"\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
                cpsms0 = false;

                if (chip != 7028)
                    at_reply_wait_OK("AT+CSOSENDFLAG=1\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);

                first_run_completed = true;
            }

            ee = at_reply_wait_OK("AT+CCLK?\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
            if (ee != ESP_OK)
            {
                ESP_LOGW(TAG, "AT+CCLK?");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            else
            {
                //+CCLK: 24/04/30,07:49:36+12
                //+CCLK: "2026/03/09,08:41:39+12" - SIM7028
                int dt[6] = {0, 0, 0, 0, 0, 0};
                char *pdata = strstr((const char *)data, "CCLK: ");
                char *ss = pdata + 6;

                if (chip == 7028)
                {
                    ss++;
                }

                int parsed = sscanf((const char *)ss, "%d/%d/%d,%d:%d:%d", &dt[0], &dt[1], &dt[2], &dt[3], &dt[4], &dt[5]);
                if (parsed == 6)
                {
                    struct tm tm;
                    tm.tm_year = (dt[0] > 1900) ? dt[0] - 1900 : dt[0] + 100;
                    tm.tm_mon = dt[1] - 1;
                    tm.tm_mday = dt[2];
                    tm.tm_hour = dt[3];
                    tm.tm_min = dt[4];
                    tm.tm_sec = dt[5];

                    time_t t = mktime(&tm) + timezone * 3600; // UNIX time + timezone offset
                    struct timeval now = {.tv_sec = t};
                    settimeofday(&now, NULL);
                    ESP_LOGI(TAG, "Set date and time: %s", get_datetime(time(0)));
                }
            }

            // get current date time
            result.ttime = time(0);

            if (espnow_need_send)
            {
                snprintf((char *)send_data, TX_BUF_SIZE, OUT_JSON, get_menu_val_by_id("idn"), result.measure.bootcount, get_datetime(result.ttime), OUT_MEASURE_VARS(result.measure));
                send_by_espnow(mac_addr, send_data);
                espnow_need_send = false;
            }

            // ee = at_reply_wait_OK("AT+CTZU?\r\n", (char *)data, 10000 / portTICK_PERIOD_MS);

            // test PDP context
            ee = at_reply_wait_OK("AT+CGDCONT?\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);

            if (ee != ESP_OK)
            {
                ESP_LOGW(TAG, "AT+CGDCONT?");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            else
            {
                char *s = strstr((const char *)data, "+CGDCONT: ");
                if (s)
                {
                    for (int i = 0; i < 3; i++)
                    {
                        s = strstr((const char *)++s, ",");
                    }
                }
                int parsed = sscanf((const char *)++s, "\"%hhu.%hhu.%hhu.%hhu\"", &((uint8_t *)(&pdp_ip.addr))[0], &((uint8_t *)(&pdp_ip.addr))[1], &((uint8_t *)(&pdp_ip.addr))[2], &((uint8_t *)(&pdp_ip.addr))[3]);
                if (parsed == 4)
                {
                    // если нет нормального IP - рестарт модуля
                    if (esp_ip4_addr1(&pdp_ip) == 127)
                    {
                        ESP_LOGE(TAG, "IP: " IPSTR, IP2STR(&pdp_ip));
                        print_atcmd("AT+CPOWD=1\r\n", data);
                        first_run_completed = false;
                        vTaskDelay(2000 / portTICK_PERIOD_MS);
                        continue;
                    }
                    else
                    {
                        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&pdp_ip));
                    }
                }
            };

            // если запускаем терминал - стоп работа с модулем
            if (xEventGroupGetBits(status_event_group) & NB_TERMINAL)
            {
                break;
            }

            esp_ip4_addr_t ipaddr;
            ipaddr.addr = (unsigned int)get_menu_val_by_id("ipaddr");

            /*
                                    // ping
                                    // AT+CIPPING

                                    snprintf(send_data, sizeof(send_data), "AT+CIPPING=\"%i.%i.%i.%i\"\r\n", (ipaddr >> 24) & 0xff, (ipaddr >> 16) & 0xff, (ipaddr >> 8) & 0xff, (ipaddr) & 0xff);
                                    ee = at_reply_wait_OK(send_data, (char *)data, 60000 / portTICK_PERIOD_MS);
                                    if (ee != ESP_OK)
                                    {
                                        ESP_LOGW(TAG, "AT+CIPPING:%s", data);
                                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                                    }

                                    vTaskDelay(10000 / portTICK_PERIOD_MS);

                                    ee = at_reply_wait_OK("AT+CIPPING?\r\n", (char *)data, 60000 / portTICK_PERIOD_MS);
                                    if (ee != ESP_OK)
                                    {
                                        ESP_LOGW(TAG, "AT+CIPPING?:%s", data);
                                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                                    }
            */
            strcpy(net_status_current, "Отправка даных...");

            int socket = 0;
            const int tcpport = get_menu_val_by_id("tcpport");
            const int udpport = get_menu_val_by_id("udpport");

            int port = tcpport;

            if (tcpport > 0)
            {
                protocol = 1;
            }

            if (udpport > 0)
            {
                port = udpport;
                protocol = 2;
            }

            if (tcpport == 0 && udpport == 0)
            {
                break;
            }

            char send_cmd[64];

            // END_DS18B20
            xEventGroupWaitBits(status_event_group, END_DS18B20, pdTRUE, pdFALSE, 2000 / portTICK_PERIOD_MS);

            if (chip == 7028)
            {
                at_reply_wait_OK("AT+NETOPEN\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
                int l = snprintf(send_data, sizeof(send_data), OUT_JSON, get_menu_val_by_id("idn"), result.measure.bootcount, get_datetime(result.ttime), OUT_MEASURE_VARS(result.measure));

                at_reply_wait_OK("AT+CIPRXGET=1\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);

                try_counter = 3;
                while (try_counter--)
                {
                    if (protocol == 2) // UDP
                    {
                        snprintf(send_cmd, sizeof(send_cmd), "AT+CIPOPEN=%i,\"UDP\",,,%i\r\n", socket, port);
                        ee = at_reply_wait_OK(send_cmd, (char *)data, 1000 / portTICK_PERIOD_MS);
                    }
                    if (protocol == 1) // TCP
                    {
                        snprintf(send_cmd, sizeof(send_cmd), "AT+CIPOPEN=%i,\"TCP\",\"" IPSTR "\",%i\r\n", socket, IP2STR(&ipaddr), port);
                        // ee = at_reply_wait(send_cmd, "+CIPOPEN: ", (char *)data, 60000 / portTICK_PERIOD_MS);
                        ee = at_reply_wait_OK(send_cmd, (char *)data, 1000 / portTICK_PERIOD_MS);

                        const char *s1 = strstr((const char *)data, "+CIPOPEN: ");
                        if (s1 == NULL)
                        {
                            ee = wait_string(data, "+CIPOPEN: ", 90000 / portTICK_PERIOD_MS);
                        };
                    }

                    ee = at_reply_wait_OK("AT+CIPOPEN?\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);
                    if (ee == ESP_OK)
                    {
                        // AT+CIPOPEN?
                        //+CIPOPEN:0,"UDP","172.30.239.20",48885,-1
                        snprintf(send_cmd, sizeof(send_cmd), "+CIPOPEN:%i,\"%s\"", socket, (protocol == 2) ? "UDP" : "TCP");
                        if (strstr(data, send_cmd) == NULL)
                            ee = ESP_FAIL;
                    }

                    if (ee == ESP_OK)
                    {
                        if (protocol == 2) // UDP
                        {
                            // AT+CIPSEND=<link_num>,<length>,<serverIP>,<serverPort>
                            snprintf(send_cmd, sizeof(send_cmd), "AT+CIPSEND=%i,%i,\"" IPSTR "\",%i\r\n", socket, l, IP2STR(&ipaddr), port);
                        }
                        if (protocol == 1) // TCP
                        {
                            snprintf(send_cmd, sizeof(send_cmd), "AT+CIPSEND=%i,%i\r\n", socket, l);
                        }

                        ee = at_reply_wait(send_cmd, ">", (char *)data, 1000 / portTICK_PERIOD_MS);
                    }

                    esp_err_t send_ee = ESP_FAIL;
                    if (ee == ESP_OK)
                        send_ee = at_reply_wait(send_data, "+CIPSEND: ", (char *)data, 60000 / portTICK_PERIOD_MS);

                    // vTaskDelay(5000 / portTICK_PERIOD_MS);

                    // wait 10s for reply from server
                    while (wait_string(data, "\r\n", 10000 / portTICK_PERIOD_MS) == ESP_OK)
                    {
                        ESP_LOGI(TAG, "Modem: %s", data);
                        ee = check_received_message(data, send_data, chip);
                        if (ee == ESP_OK)
                            break;
                    }

                    snprintf(send_cmd, sizeof(send_cmd), "AT+CIPCLOSE=%i\r\n", socket);
                    at_reply_wait_OK(send_cmd, (char *)data, 1000 / portTICK_PERIOD_MS);

                    if (send_ee == ESP_OK)
                        break;
                }

                at_reply_wait_OK("AT+NETCLOSE\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);

                gpio_set_level(MODEM_POWER, 1);
            }
            else
            {
                at_reply_wait_OK("AT+CEREG?\r\n", (char *)data, 1000 / portTICK_PERIOD_MS);

                snprintf(send_data, sizeof(send_data), "AT+CSOC=1,%i,1\r\n", protocol);
                ee = at_reply_wait_OK(send_data, (char *)data, 1000 / portTICK_PERIOD_MS); // Create socket

                if (ee != ESP_OK)
                {
                    ESP_LOGW(TAG, "AT+CSOC");
                }
                else
                {
                    const char *pdata = strstr((const char *)data, "CSOC: ");
                    if (pdata)
                    {
                        socket = atoi(pdata + 6);

                        try_counter = 3;
                        while (try_counter--)
                        {
                            snprintf(send_data, sizeof(send_data), "AT+CSOCON=%i,%i,\"" IPSTR "\"\r\n", socket, port, IP2STR(&ipaddr));
                            ESP_LOGI(TAG, "%i Socket %i connect...", 3 - try_counter, socket);
                            ee = at_reply_wait_OK(send_data, (char *)data, 60000 / portTICK_PERIOD_MS);

                            if (ee != ESP_OK)
                            {
                                ESP_LOGW(TAG, "AT+CSOCON:%s", data);
                                // result.measure.d_nbiot_error = true;
                            }
                            else
                            {
                                // result.measure.d_nbiot_error = false;
                                int l = snprintf(send_data, sizeof(send_data), OUT_JSON, get_menu_val_by_id("idn"), result.measure.bootcount, get_datetime(result.ttime), OUT_MEASURE_VARS(result.measure));
                                ESP_LOGD(TAG, "Send... %s", send_data);
                                do
                                {
                                    if (protocol == 1) // TCP
                                        ee = at_csosend(socket, send_data, l, data, "SEND:");
                                    if (protocol == 2) // UDP
                                        ee = at_csosend(socket, send_data, l, data, "OK\r");

                                    if (ee == ESP_OK)
                                    {
                                        // обрабатываем если нет сети или приход команды
                                        ee = check_for_wait_cereg(data, send_data, chip, 20000 / portTICK_PERIOD_MS);
                                        if (ee != ESP_ERR_NOT_FINISHED)
                                        {
                                            // common_data_transmit = true;
                                            result.measure.d_nbiot_send_succes = true;
                                            strcpy(net_status_current, "Отправлено.");
                                            ESP_LOGI(TAG, "Send OK");
                                        }
                                    }
                                } while (ee == ESP_ERR_NOT_FINISHED);
                                break; // All OK
                            }
                            vTaskDelay(2000 / portTICK_PERIOD_MS);
                        }
                    }
                    else
                    {
                        // Скорее всего сокеты закончились...
                        socket = 4;
                    }
                };

                // wait 10s for reply from server
                while (wait_string(data, "\r\n", 10000 / portTICK_PERIOD_MS) == ESP_OK)
                {
                    ESP_LOGD(TAG, "Modem: %s", data);
                    ee = check_received_message(data, send_data, chip);
                }

                while (socket >= 0)
                {
                    snprintf(send_data, sizeof(send_data), "AT+CSOCL=%i\r\n", socket--);
                    at_reply_wait_OK(send_data, (char *)data, 1000 / portTICK_PERIOD_MS); // CLOSE socket
                }

                download_firmware(data, send_data);
            };

            // если запускаем терминал - стоп работа с модулем
            if (xEventGroupGetBits(status_event_group) & NB_TERMINAL)
            {
                // Disable the use of PSM and discard all parameters for PSM or, if available reset to the manufacturer specific default values.
                print_atcmd("AT+CPSMS=2\r\n", data);
                cpsms0 = true;
            }
            break;
        };

        // clear notify
        ulTaskNotifyTake(pdTRUE, 0);
        xEventGroupSetBits(status_event_group, END_RADIO);
    }
}
