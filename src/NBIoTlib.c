#include "NBIoTlib.h"
#include "main.h"

#include "driver/gpio.h"

#include "esp_timer.h"

#include "esp_partition.h"
#include <esp_ota_ops.h>

#include "esp_mac.h"
#include "esp_now.h"
#include "esp_wifi.h"

#include "cJSON.h"

#include "psa/crypto.h"
psa_status_t psa_status = -1;
#define IV_LEN 12
#define TAG_LEN 16

static const char *TAG = "NBIoTlib";

cJSON *firmware = NULL;

int32_t timezone = 3;

int cereg[10] = {-1, -1};

char cmdbuf[64];

esp_err_t uart_init(uart_port_t uart_num, int rx_pin, int tx_pin)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t res = uart_driver_install(uart_num, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    psa_status = psa_crypto_init();

    return res;
}

size_t encrypt_text(char *source, char *desc, uint8_t *key, char *aad)
{
    uint8_t iv[IV_LEN];
    size_t text_len = strnlen(source, 512 - 16 - 1);
    size_t ciphertext_len = text_len + TAG_LEN;

    // Аппаратно генерируем случайный уникальный IV
    psa_status_t status = psa_generate_random(iv, IV_LEN);

    // 2. Настройка атрибутов ключа
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, 128);
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT);
    // Задаем алгоритм AES-GCM с длиной тега 16 байт (128 бит)
    psa_set_key_algorithm(&attributes, PSA_ALG_GCM);

    // 3. Импорт ключа в RAM
    mbedtls_svc_key_id_t key_id;
    status = psa_import_key(&attributes, (uint8_t *)key, 16, &key_id);
    if (status != PSA_SUCCESS)
    {
        // ESP_LOGE(TAG, "Не удалось импортировать ключ: %d", status);
        return 0;
    }

    const size_t aad_len = 4;
    memcpy(desc, (uint8_t *)aad, aad_len);
    uint8_t *iv_ptr = (uint8_t *)desc + aad_len;
    memcpy(iv_ptr, iv, IV_LEN);
    uint8_t *ciphertext_ptr = iv_ptr + IV_LEN;

    size_t encrypted_len = 0;
    status = psa_aead_encrypt(key_id, PSA_ALG_GCM,
                              iv, IV_LEN,
                              (uint8_t *)aad, aad_len,
                              (uint8_t *)source, strlen(source),
                              (uint8_t *)ciphertext_ptr, ciphertext_len, &encrypted_len);

    if (status != PSA_SUCCESS)
    {
        return 0;
    }

    psa_destroy_key(key_id);

    return aad_len + IV_LEN + encrypted_len;
}

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

void nbiot_power_pin(const TickType_t xTicksToDelay, int pin)
{
#ifdef PCFNBPOWER
    pcf8575_set(pin);
    vTaskDelay(xTicksToDelay);
    pcf8575_set(pin);
#else
    gpio_set_level(pin, 0);
    vTaskDelay(xTicksToDelay);
    gpio_set_level(pin, 1);
#endif
};

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
    }

    buffer[len] = '\0';
    ESP_LOGD(TAG, "Receive string:\"%s\"", (char *)buffer);

    return ESP_OK;
}

esp_err_t wait_string(char *buffer, const char *swait, TickType_t ticks_to_wait)
{
    const char *err = "ERROR\r";

    int64_t start_time = esp_timer_get_time();
    char *pb = buffer;
    int total_len = 0;
    *pb = '\0';
    int lw = strlen(swait);
    int reqlen = (lw > 0) ? lw : 1;
    // BUG
    // reqlen = RX_BUF_SIZE - 1 - total_len;

    esp_err_t res = ESP_ERR_TIMEOUT;
    int len = -1;
    size_t ssize;
    // static size_t ssizemax = 0;
    do
    {
        if (len == 0) // если предыдущее чтение == 0
            len = uart_read_bytes(UART_NUM_1, pb, reqlen, 10 / portTICK_PERIOD_MS);
        else
            len = uart_read_bytes(UART_NUM_1, pb, reqlen, 0);

        // uart_get_buffered_data_len(UART_NUM_1, &ssize);
        // if (ssize > 1000) ESP_LOGD("wait_string", "RX len: %d (%d): %02x %02x %02x", len, (int)ssize, *pb, *(pb + 1), *(pb + 2));

        if (len > 0)
        {
            pb += len;
            total_len += len;
            *pb = '\0';

            if (strstr((const char *)buffer, swait) != NULL)
            {
                if (total_len > 2)
                {
                    ESP_LOGV("wait", "...%2x %2x %2x", buffer[total_len - 3], buffer[total_len - 2], buffer[total_len - 1]);
                }

                if (buffer[total_len - 1] == '\n')
                {
                    res = ESP_OK;
                    break;
                }
            }
            else if (strstr((const char *)buffer, err) != NULL)
            {
                res = ESP_ERR_INVALID_STATE;
                break;
            }

            reqlen = 1;

            start_time = esp_timer_get_time();
        }
        else if (len == -1)
        {
            return ESP_FAIL;
        }

    } while ((esp_timer_get_time() - start_time) < ticks_to_wait * portTICK_PERIOD_MS * 1000LL);

    ESP_LOGV("wait", "%s return: %s", swait, esp_err_to_name(res));

    return res;
}

esp_err_t at_reply_wait(const char *cmd, const char *wait, char *buffer, TickType_t ticks_to_wait)
{
    esp_err_t res = ESP_ERR_TIMEOUT;
    int l = strlen(cmd);
    int txBytes = 0;
    txBytes = uart_write_bytes(UART_NUM_1, cmd, l);
    if (txBytes < l)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    if (ticks_to_wait == 0)
        return res;

    res = wait_string(buffer, wait, ticks_to_wait);

    ESP_LOGD(TAG, "Reply wait...:\"%s\"", (char *)buffer);

    return res;
}

esp_err_t at_reply_wait_OK(const char *cmd, char *buffer, TickType_t ticks_to_wait)
{
    return at_reply_wait(cmd, "OK\r", buffer, ticks_to_wait);
}

// ищем пробел, затем параметры через запятую
esp_err_t at_result_get(const char *wait, const char *buffer, int *resultdata, int resultcount)
{
    char *s = strstr(buffer, wait);
    if (s)
    {
        for (int i = 0; i < resultcount; i++)
        {
            if (i == 0)
                s = strchr(s + 1, ' ');
            else
                s = strchr(s + 1, ',');

            if (s == NULL)
                return ESP_ERR_NOT_FOUND;

            ESP_LOGV(TAG, "Found string:\"%s\"", s);

            if (*(s + 1) == '"') // HEX STRING (ex CREG?)
                resultdata[i] = strtol(s + 2, NULL, 16);
            else
                resultdata[i] = atoi(s + 1);
        }
        return ESP_OK;
    }

    return ESP_ERR_NOT_FOUND;
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

    wait_string(buffer, "OK\r", ticks_to_wait);

    ESP_LOGD(TAG, "Receive string:\"%s\"", (char *)buffer);

    return at_result_get(wait, (const char *)buffer, resultdata, resultcount);
}

/*
 Send Data to Remote Via Socket With Data Mode
*/
esp_err_t at_csosend(const int socket, const char *data, const int len_data, char *buffer, const char *swait)
{
    esp_err_t res = ESP_FAIL;

    int l = snprintf(cmdbuf, sizeof(cmdbuf), "AT+CSOSEND=%d,%d,", socket, len_data * 2);
    int txBytes = uart_write_bytes(UART_NUM_1, cmdbuf, l);
    if (txBytes < 4)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    for (int i = 0; i < len_data; i++)
    {
        snprintf(cmdbuf, 3, "%02x", data[i]);
        txBytes = uart_write_bytes(UART_NUM_1, cmdbuf, 2);
        if (txBytes < 2)
        {
            return ESP_ERR_INVALID_SIZE;
        }
    }

    uart_write_bytes(UART_NUM_1, "\r\n", 2);

    res = wait_string(buffer, swait, 30000 / portTICK_PERIOD_MS);

    ESP_LOGD(TAG, "Receive string:\"%s\"", (char *)buffer);

    return res;
}

char *get_datetime(time_t ttime)
{
    static char datetime[24];
    struct tm *localtm = localtime(&ttime);
    strftime(datetime, sizeof(datetime), "%Y-%m-%d %T", localtm);
    return datetime;
}

esp_err_t apply_command(const char *cmd, size_t len)
{
    cJSON *json = cJSON_ParseWithLength(cmd, len);
    if (json)
    {
        cJSON *item = NULL;
        cJSON_ArrayForEach(item, json)
        {
            if (get_menu_pos_by_id(item->string) >= 0)
            {
                ESP_LOGI(TAG, "Setup %s: %d", item->string, item->valueint);
                if (get_menu_val_by_id(item->string) != item->valueint)
                {
                    set_menu_val_by_id(item->string, item->valueint);
                }
            }
        }

        item = cJSON_GetObjectItemCaseSensitive(json, "timestamp");
        if (cJSON_IsString(item) && (item->valuestring != NULL))
        {
            long long ts = atoll((const char *)item->valuestring);
            if (ts > 1715923962LL) // 17 May 2024 05:32:42
            {
                struct timeval now = {.tv_sec = ts + timezone * 3600}; // UNIX time + timezone offset
                settimeofday(&now, NULL);
                ESP_LOGI(TAG, "Set date and time: %s", get_datetime(time(0)));
            }
        }

        item = cJSON_GetObjectItemCaseSensitive(json, "firmware");
        if (cJSON_IsObject(item))
        {
            firmware = cJSON_Duplicate(item, 1);

            char *json_string = cJSON_PrintUnformatted(item);
            if (json_string)
            {
                ESP_LOGI(TAG, "Update firmware: %s", json_string);
                cJSON_free(json_string);
            }
        }
    }
    else
    {
        const char *er = cJSON_GetErrorPtr();
        ESP_LOGW(TAG, "Parse err: %c in \"%s\"", *er, cmd);
    }
    cJSON_Delete(json);

    return ESP_OK;
}

// data - hex символы, len - запрашиваемая длина результата, return - длина результата
int decode_fromHEX(const char *data, int len, char *result_data)
{
    char *s = (char *)data;
    int i = 0;
    while (i < len + 1) // строка должна заканчиваться \r
    {
        if (*s < '0' || *(s + 1) < '0' || *s > 'f' || *(s + 1) > 'f')
            break;
        char c[3] = {*(s++), *(s++), 0};
        result_data[i] = (char)strtol(c, NULL, 16);
        i++;
    }
    result_data[i] = '\0';
    return i;
}

// data - hex символы, return - длина результата
int encode_toHEX(const char *data, char *result_data)
{
    char *s = result_data;
    int i = 0;
    int len = strlen(data);
    while (i < len)
    {
        snprintf(s, 3, "%02x", data[i]);
        i++;
        s = s + 2;
    }

    return i;
}

esp_err_t check_for_wait_cereg(char *message_data, char *result_data, int chip, TickType_t timewait)
{
    esp_err_t ee = ESP_ERR_NOT_FOUND;

    check_received_message(message_data, result_data, chip);

    // обрабатываем если нет сети
    char *s = strstr((const char *)message_data, "+CEREG: 2");
    if (s != NULL)
    {
        ESP_LOGV("check_received_message", "Found: %s", s);
        // Ждем сообщения о наличии сети
        if (wait_string(message_data, "+CEREG: 1", timewait) == ESP_OK)
        {
            ESP_LOGD("check_received_message", "Found +CEREG: 1");
            // ESP_LOGD(TAG, "Wait CEREG:\"%s\"", (char *)message_data);
            vTaskDelay(2000 / portTICK_PERIOD_MS); // много пропущенных пакетов после выхода из PSM, так лучше
        };
        check_received_message(message_data, result_data, chip);
        // желательно повторить передачу
        return ESP_ERR_NOT_FINISHED;
    }
    return ee;
};

esp_err_t check_received_message(char *message_data, char *result_data, int chip)
{
    esp_err_t ee = ESP_ERR_NOT_FOUND;

    char *rdata = NULL;
    if (chip == 7028)
    {
        char *s = strstr((const char *)message_data, "+CIPRXGET: ");
        if (s)
        {
            char *ss = strchr(s, ',');
            int link_num = atoi(ss + 1);
            int l = snprintf(cmdbuf, sizeof(cmdbuf), "AT+CIPRXGET=2,%d\r\n", link_num);
            ee = at_reply_wait_OK(cmdbuf, result_data, 1000 / portTICK_PERIOD_MS);
            if (ee == ESP_OK)
            {
                //+CIPRXGET: 2,0,13,0
                ss = strstr((const char *)result_data, "+CIPRXGET: ");
                if (ss)
                {
                    ss = strchr(ss + 1, ',');
                    if (ss)
                    {
                        link_num = atoi(ss + 1);
                        ss = strchr(ss + 1, ',');
                        if (ss)
                        {
                            l = atoi(ss + 1);

                            ss = strchr(ss + 1, 0x0A);
                            if (ss)
                            {
                                *(ss + 1 + l) = '\0';
                                apply_command((const char *)(ss + 1), l);
                                return ESP_OK;
                            }
                        }
                    }
                }
            }
        }
    }
    else
    {
        //+CSONMI: 0,20,5468616E6B20796F7521
        rdata = strstr((const char *)message_data, "+CSONMI: ");

        if (rdata)
        {
            char *s = strchr(rdata, ',');

            // len
            int len = atoi(s + 1);
            s = strchr(s + 1, ',');
            if (decode_fromHEX(s + 1, len / 2, result_data) == len / 2)
            {
                apply_command((const char *)result_data, len / 2);
                return ESP_OK;
            }
        }
    }
    return ESP_ERR_NOT_FOUND;
}

esp_err_t check_cereg(const char *rx_buffer, const int chip, int *stat, int *tac, int *ci, int *AcT, int *ActiveTime, int *PeriodicTAU)
{
    esp_err_t res = ESP_ERR_NOT_FOUND;

    if (strstr(rx_buffer, "+CEREG: 2") != NULL)
    {
        return ESP_ERR_NOT_ALLOWED;
    }
    else if (strstr(rx_buffer, "+CEREG: 1") != NULL)
    {
        if (chip == 7028)
            res = at_result_get("+CEREG: 1,", rx_buffer, cereg + 1, 8);
        else
            res = at_result_get("+CEREG: 1,", rx_buffer, cereg + 1, 9);
    }
    else if (strstr(rx_buffer, "+CEREG: 5") != NULL)
    {
        if (chip == 7028)
            res = at_result_get("+CEREG: ", rx_buffer, cereg, 9);
        else
            res = at_result_get("+CEREG: ", rx_buffer, cereg, 10);
    }

    if (res == ESP_OK)
    {
        *stat = *(cereg + 1);
        *tac = *(cereg + 2);
        *ci = *(cereg + 3);
        *AcT = *(cereg + 4);

        if (chip == 7028)
        {
            *ActiveTime = *(cereg + 7);
            *PeriodicTAU = *(cereg + 8);
        }
        else
        {
            *ActiveTime = *(cereg + 8);
            *PeriodicTAU = *(cereg + 9);
        }
    }

    return res;
}

esp_err_t download_firmware(char *rx_buffer, char *tx_buffer)
{
    esp_err_t ee = ESP_ERR_NOT_FOUND;

    // качаем firmware
    if (cJSON_IsObject(firmware))
    {
        int file_id = -1;
        /* Пишем в next_ota и прошивку и spiffs.bin*/
        const esp_partition_t *ota_partition = esp_ota_get_next_update_partition(NULL);
        esp_ota_handle_t ota_handle;

        int block = ota_partition->erase_size * 5; // 4096 Erase Must be aligned to partition->erase_size.
        int contentstart = 0;
        int contenttotal = 0;
        int session_retry = 2;
        size_t ssize;

        const cJSON *cjserver = cJSON_GetObjectItemCaseSensitive(firmware, "server");
        const cJSON *cjpath = cJSON_GetObjectItemCaseSensitive(firmware, "path");
        // const cJSON *cjcrc = cJSON_GetObjectItemCaseSensitive(firmware, "crc");

        if (!cJSON_IsString(cjserver) || !cJSON_IsString(cjpath))
            return ESP_ERR_INVALID_ARG;

        at_reply_wait_OK("AT+CPSMS=0\r\n", rx_buffer, 1000 / portTICK_PERIOD_MS);

#ifdef LV_CMDON //sodk only
        vTaskSuspend(xHandleADC);
#endif

        do
        {
            int http_error_retry = 5;

            snprintf(tx_buffer, TX_BUF_SIZE, "AT+CHTTPCREATE=\"%s\"\r\n", cjserver->valuestring);
            ee = at_reply_wait_OK(tx_buffer, (char *)rx_buffer, 1000 / portTICK_PERIOD_MS);
            const char *pdata = strstr((const char *)rx_buffer, "+CHTTPCREATE: ");
            if (pdata)
            {
                int httpclient_id = atoi(pdata + 14);

                snprintf(tx_buffer, TX_BUF_SIZE, "AT+CHTTPCON=%i\r\n", httpclient_id);
                ESP_LOGI(TAG, "HTTP Socket %i connect...", httpclient_id);
                ee = at_reply_wait_OK(tx_buffer, (char *)rx_buffer, 30000 / portTICK_PERIOD_MS);

                while (http_error_retry > 0 && (ee == ESP_OK || ee == ESP_ERR_TIMEOUT))
                {
                    // ESP_LOGD(TAG, "try %i", http_error_retry);
                    snprintf(tx_buffer, TX_BUF_SIZE, "Range: bytes=%i-%i\n", contentstart, (file_id == -1) ? 500 - 1 : contentstart + block - 1);
                    encode_toHEX(tx_buffer, rx_buffer);
                    snprintf(tx_buffer, TX_BUF_SIZE, "AT+CHTTPSEND=%i,0,\"%s\",\"%s\"\r\n", httpclient_id, cjpath->valuestring, rx_buffer);
                    ee = at_reply_wait_OK(tx_buffer, (char *)rx_buffer, 1000 / portTICK_PERIOD_MS);
                    if (ee != ESP_OK)
                    {
                        vTaskDelay(5000 / portTICK_PERIOD_MS);
                        uart_flush(UART_NUM_1);
                        break;
                    };

                    int contentlen = 0;
                    int contentrange = 0;
                    int sum_getpart_length = 0;
                    char *s;
                    char *hs;
                    int64_t start_time = esp_timer_get_time();
                    do
                    {
                        uart_get_buffered_data_len(UART_NUM_1, &ssize);
                        ee = wait_string(rx_buffer, "\r\n", 10);
                        if (ee == ESP_OK)
                        {
                            s = strstr(rx_buffer, "+CHTTPERR:");
                            if (s)
                            {
                                ee = ESP_ERR_INVALID_STATE;
                                break;
                            };

                            ESP_LOGD(TAG, "(%i) PROCESS: \"%s\"", ssize, (char *)rx_buffer);
                            if (contentlen == 0)
                            {
                                //+CHTTPNMIH: 2,200,233,Content-Length: 22620
                                s = strstr(rx_buffer, "+CHTTPNMIH:");
                                if (s)
                                {
                                    s = strstr(s, "Content-Length: ");
                                    contentlen = atoi(s + 16);
                                }
                            }

                            if (contentrange == 0)
                            {
                                s = strstr(rx_buffer, "Content-Range: bytes ");
                                if (s)
                                {
                                    contentrange = atoi(s + 21);

                                    if (contentrange != contentstart)
                                    {
                                        ESP_LOGE(TAG, "ERROR Content-Range: %i", contentrange);
                                        break;
                                    }

                                    if (contenttotal == 0)
                                    {
                                        s = strstr(s, "/");
                                        contenttotal = atoi(s + 1);
                                    }
                                }
                            }

                            if (contentlen > 0)
                            {
                                //+CHTTPNMIC: 0,1,989328,500,637cd7040d476389e7040327090789461306d7ff63efc628a38fe43e85a00345090611e993...
                                s = strstr(rx_buffer, "+CHTTPNMIC:");
                                if (s)
                                {
                                    s = strchr(s + 1, ' ');
                                    if (!s || atoi(s + 1) != httpclient_id)
                                    {
                                        ESP_LOGE(TAG, "ERROR data ");
                                        break;
                                    }

                                    s = strchr(s + 1, ',');
                                    int flag = atoi(s + 1);
                                    if (!s)
                                    {
                                        ESP_LOGE(TAG, "ERROR data ");
                                        break;
                                    }

                                    s = strchr(s + 1, ',');
                                    if (!s || atoi(s + 1) != contentlen)
                                    {
                                        ESP_LOGE(TAG, "ERROR data ");
                                        break;
                                    }

                                    s = strchr(s + 1, ',');
                                    int hex_length = atoi(s + 1);
                                    s = strchr(s + 1, ',');
                                    hs = s + 1;

                                    int l = 0;
                                    if (hex_length > 0)
                                    {
                                        l = decode_fromHEX(hs, hex_length, tx_buffer);
                                        if (hex_length != l)
                                        {
                                            ESP_LOGE(TAG, "Bad data %i of %i", l, hex_length);
                                            ee = ESP_ERR_TIMEOUT;
                                            break;
                                        }
                                        else
                                        {
                                            if (file_id == -1) // first probe data block
                                            {
                                                file_id = tx_buffer[0];

                                                if (file_id == ESP_IMAGE_HEADER_MAGIC)
                                                {
                                                    ESP_ERROR_CHECK(esp_ota_begin(ota_partition, OTA_SIZE_UNKNOWN, &ota_handle));
                                                }
                                                else if (contenttotal == 0x50000) // SPIFFS Image
                                                {
                                                    file_id = contenttotal;
                                                    ESP_ERROR_CHECK(esp_partition_erase_range(ota_partition, 0, 0x50000));
                                                }
                                            }
                                            else if (file_id == ESP_IMAGE_HEADER_MAGIC) // firmware.bin
                                            {
                                                // Successful Upload: Flash firmware chunk
                                                if (esp_ota_write_with_offset(ota_handle, (const void *)tx_buffer, l, contentstart + sum_getpart_length) != ESP_OK)
                                                {
                                                    ESP_LOGE(TAG, "Flash write Error");
                                                    ee = ESP_FAIL;
                                                    break;
                                                }
                                            }
                                            else if (file_id == 0x50000) // spiffs.bin
                                            {
                                                ESP_ERROR_CHECK(esp_partition_write(ota_partition, contentstart + sum_getpart_length, (const void *)tx_buffer, l));
                                            }
                                            else
                                            {
                                                ESP_LOGE(TAG, "Data Error");
                                                ee = ESP_FAIL;
                                                break;
                                            }
                                        }
                                    }

                                    sum_getpart_length += l;

                                    if (flag == 0)
                                        break;

                                    start_time = esp_timer_get_time();
                                }
                            }
                        }
                    } while ((esp_timer_get_time() - start_time) < (60000 * 1000LL)); // 60s

                    if (sum_getpart_length > 0 && sum_getpart_length == contentlen)
                    {
                        if (contentstart == 0 && sum_getpart_length == 500) // проба файла
                        {
                            ESP_LOGI(TAG, "Type file: 0x%x, size %i", file_id, contenttotal);
                        }
                        else
                        {
                            xEventGroupSetBits(status_event_group, SERIAL_TERMINAL_ACTIVE); // предупреждаем засыпание

                            http_error_retry = 3;
                            session_retry = 1;
                            ESP_LOGI(TAG, "Download OK! Start: %i, bytes %i", contentstart, sum_getpart_length);
                            contentstart = contentstart + block;
                            if (block > contenttotal - contentstart)
                                block = contenttotal - contentstart;

                            // All download OK
                            if (contentstart == contenttotal)
                            {
                                ESP_LOGI(TAG, "Download All OK! bytes %i. Burn...", contenttotal);

                                if (file_id == ESP_IMAGE_HEADER_MAGIC)
                                {
                                    // Validate and switch to new OTA image and reboot
                                    if (esp_ota_end(ota_handle) != ESP_OK || esp_ota_set_boot_partition(ota_partition) != ESP_OK)
                                    {
                                        ESP_LOGE(TAG, "OTA Validation / Activation Error");
                                        ee = ESP_FAIL;
                                        break;
                                    }
                                    ESP_LOGW(TAG, "Firmware update complete, rebooting now!");
                                    if (xHandleWifi)
                                        xTaskNotify(xHandleWifi, REBOOT_NOW, eSetValueWithOverwrite);

                                    session_retry = 0;
                                    http_error_retry = 0; // All OK
                                }
                                else if (file_id == 0x50000) // spiffs.bin
                                {
                                    const esp_partition_t *storage_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
                                    ESP_ERROR_CHECK(esp_partition_erase_range(storage_partition, 0, 0x50000));

                                    int remaining = contenttotal;
                                    while (remaining > 0)
                                    {
                                        int recv_len = MIN(remaining, RX_BUF_SIZE);
                                        if (esp_partition_read(ota_partition, (contenttotal - remaining), (void *)rx_buffer, recv_len) == ESP_OK)
                                        {
                                            ESP_ERROR_CHECK(esp_partition_write(storage_partition, (contenttotal - remaining), (const void *)rx_buffer, recv_len));
                                            vTaskDelay(10 / portTICK_PERIOD_MS);
                                        }
                                        remaining -= recv_len;
                                    }
                                    ESP_LOGW(TAG, "SPIFFS update complete, rebooting now!");
                                    if (xHandleWifi)
                                        xTaskNotify(xHandleWifi, REBOOT_NOW, eSetValueWithOverwrite);

                                    session_retry = 0;
                                    http_error_retry = 0; // All OK
                                }
                                break;
                            }
                        }
                    }
                    else
                    {
                        ESP_LOGW(TAG, "Download Fail! Bytes %i of %i", sum_getpart_length, contentlen);

                        ESP_ERROR_CHECK(esp_partition_erase_range(ota_partition, contentstart, block));

                        vTaskDelay(10000 / portTICK_PERIOD_MS);

                        uart_flush(UART_NUM_1);

                        http_error_retry--;
                    }
                }

                while (httpclient_id >= 0)
                {
                    snprintf(tx_buffer, TX_BUF_SIZE, "AT+CHTTPDISCON=%i\r\n", httpclient_id);
                    at_reply_wait_OK(tx_buffer, (char *)rx_buffer, 1000 / portTICK_PERIOD_MS);
                    snprintf(tx_buffer, TX_BUF_SIZE, "AT+CHTTPDESTROY=%i\r\n", httpclient_id);
                    at_reply_wait_OK(tx_buffer, (char *)rx_buffer, 1000 / portTICK_PERIOD_MS);
                    httpclient_id--;
                }
            }
        } while (session_retry-- > 0);

        cJSON_Delete(firmware);

        print_atcmd("AT+CPSMS=1\r\n", rx_buffer);
    }

    return ee;
};

// Callback при отправке
// static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status) // espidf 5.5.0
{
    ESP_LOGI(TAG, "Packet to " MACSTR ", status: %s",
             MAC2STR(tx_info->des_addr),
             // MAC2STR(mac_addr),
             status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}

// Callback при получении
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    ESP_LOGI(TAG, "Received from " MACSTR ", len: %d", MAC2STR(recv_info->src_addr), len);
    apply_command((const char *)data, len);
}

esp_err_t init_espnow(uint8_t *peer_addr)
{
    // 1. Инициализация ESP-NOW
    esp_err_t err_rc = esp_now_init();

    // 2. Регистрация callback'ов
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    // 3. Добавьте peer-устройства (если нужно)
    esp_now_peer_info_t peer_info = {
        .peer_addr = {MAC2STR(peer_addr)}, //
        .channel = 0,                      // Должен совпадать с каналом WiFi
        .ifidx = WIFI_IF_AP,
        .encrypt = false};
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
    return err_rc;
}

esp_err_t send_by_espnow(uint8_t *mac_addr, char *send_data)
{
    if (xHandleWifi)
        xTaskNotify(xHandleWifi, NOTYFY_WIFI_ESPNOW, eSetValueWithOverwrite); // включаем WiFi для ESPNOW

    ESP_LOGI("ESPNOW", "WiFi start");

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    init_espnow(mac_addr);

    esp_err_t err_rc = esp_now_send(mac_addr, (const uint8_t *)send_data, strlen((const char *)send_data));

    if (err_rc == ESP_OK)
    {
        ESP_LOGI("ESPNOW", "Message sent successfully");
    }
    else
    {
        ESP_LOGE("ESPNOW", "Error sending message: %s to " MACSTR, esp_err_to_name(err_rc), MAC2STR(mac_addr));
    }

    // wait 1s for reply from server
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(esp_now_deinit());

    return err_rc;
}
