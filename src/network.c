/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "main.h"

#include "esp_partition.h"
#include <esp_ota_ops.h>

#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_netif.h"
#include "esp_timer.h"

#include <esp_http_server.h>

#include "esp_spiffs.h"

extern uint8_t mac[6];
extern char pdp_ip[20];
extern char net_status_current[32];

#define CLIENT_WIFI_SSID "ap1"
#define CLIENT_WIFI_PASS "123123123"
#define AP_WIFI_SSID "THCam"
#define AP_WIFI_PASS "123123123"

#define EXAMPLE_MAX_STA_CONN 2

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

extern uint16_t port;
extern char apn[32];
extern char serverip[17];
extern int32_t proto;
extern int32_t timezone;

extern int32_t id;

static const char *TAGW = "wifi";

static const char *TAGH = "httpd";

static int s_retry_num = 0;

#define TRANSFER_SIZE (CONFIG_LWIP_TCP_MSS - 14) //Correction for Chunked transfer encoding
static char network_buf[CONFIG_LWIP_TCP_MSS];

size_t buf_len;

int64_t timeout_begin;

bool need_ws_send = false;

bool restart = false;

typedef struct
{
    char filepath[32];
    char content[32];
} down_data_t;

void reset_sleep_timeout()
{
    timeout_begin = esp_timer_get_time();
    // ESP_LOGV(TAGW, "Timeout reset");
    xEventGroupSetBits(status_event_group, WIFI_ACTIVE);
}

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        reset_sleep_timeout();
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < 1)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAGW, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAGW, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAGW, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

        reset_sleep_timeout();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        reset_sleep_timeout();
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAGW, "station " MACSTR " join, AID=%d", MAC2STR(event->mac), event->aid);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAGW, "station " MACSTR " leave, AID=%d", MAC2STR(event->mac), event->aid);
    }
}

void wifi_init_softap()
{

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        NULL));
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = AP_WIFI_SSID,
            .ssid_len = strlen(AP_WIFI_SSID),
            .channel = 1,
            .password = AP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK},
    };

    if (strlen(AP_WIFI_PASS) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    static char wifi_name[sizeof(wifi_config.ap.ssid)] = AP_WIFI_SSID;
    int l = strlen(wifi_name);
    itoa(get_menu_val_by_id("id"), &wifi_name[l], 10);

    strlcpy((char *)wifi_config.ap.ssid, wifi_name, sizeof(wifi_config.ap.ssid));
    wifi_config.ap.ssid_len = strlen(wifi_name);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(44)); // влияние на измерения АЦП незначительные

    int8_t power = 0;
    ESP_ERROR_CHECK(esp_wifi_get_max_tx_power(&power));

    ESP_LOGI(TAGW, "wifi_init_softap finished. SSID:%s password:%s channel:%d power:%d", AP_WIFI_SSID, AP_WIFI_PASS, wifi_config.ap.channel, power);
}

int wifi_init_sta(void)
{

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CLIENT_WIFI_SSID,
            .password = CLIENT_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false},
        }};

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAGW, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAGW, "connected to ap SSID:%s password:%s", CLIENT_WIFI_SSID, CLIENT_WIFI_PASS);
        return 1;
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAGW, "Failed to connect to SSID:%s, password:%s", CLIENT_WIFI_SSID, CLIENT_WIFI_PASS);
    }
    else
    {
        ESP_LOGE(TAGW, "UNEXPECTED EVENT");
    }

    return 0;
}

static esp_err_t download_get_handler(httpd_req_t *req)
{
    reset_sleep_timeout();

    char *filepath = ((down_data_t *)(req->user_ctx))->filepath;
    FILE *fd = NULL;
    struct stat file_stat;

    if (stat(filepath, &file_stat) == -1)
    {
        ESP_LOGE(TAGH, "Failed to stat file : %s", filepath);
        /* Respond with 404 Not Found */
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File does not exist");
        return ESP_FAIL;
    }

    fd = fopen(filepath, "r");
    if (!fd)
    {
        ESP_LOGE(TAGH, "Failed to read existing file : %s", filepath);
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, ((down_data_t *)(req->user_ctx))->content);

    int l = strlen(filepath);
    if (filepath[l - 3] == '.' && filepath[l - 2] == 'g' && filepath[l - 1] == 'z')
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");

    ESP_LOGI(TAGH, "Sending file : %s (%ld bytes)...", filepath, file_stat.st_size);

    size_t chunksize;
    do
    {
        chunksize = fread(network_buf, 1, TRANSFER_SIZE, fd);

        if (chunksize > 0)
        {
            /* Send the buffer contents as HTTP response chunk */
            if (httpd_resp_send_chunk(req, network_buf, chunksize) != ESP_OK)
            {
                fclose(fd);
                ESP_LOGE(TAGH, "File %s sending failed!", filepath);
                /* Abort sending file */
                httpd_resp_sendstr_chunk(req, NULL);
                /* Respond with 500 Internal Server Error */
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
                return ESP_FAIL;
            }
        }
    } while (chunksize != 0);

    /* Close file after sending complete */
    fclose(fd);
    // ESP_LOGI(TAG, "File sending complete");

    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
};

static esp_err_t menu_get_handler(httpd_req_t *req)
{
    reset_sleep_timeout();

    int l = 0;
    char datetime[24];
    struct tm *localtm = localtime(&result.ttime);
    strftime(datetime, sizeof(datetime), "%Y-%m-%d %T", localtm);
    //l += snprintf(&buf[l], CONFIG_LWIP_TCP_MSS - l, " CURRENT DATA = ");
    l += snprintf(&network_buf[l], TRANSFER_SIZE - l, OUT_JSON, get_menu_val_by_id("id"), result.measure.bootcount, datetime, OUT_MEASURE_VARS(result.measure));

    const esp_app_desc_t *app_ver = esp_app_get_description();

    l += snprintf(&network_buf[l], TRANSFER_SIZE - l, "<br>Firmware: %s (%s)", app_ver->version, app_ver->date);

    l += snprintf(&network_buf[l], TRANSFER_SIZE - l, ", STATUS: ");

    if ((xEventGroupGetBits(status_event_group) & NOW_CHARGE) || get_charge())
    {
        l += snprintf(&network_buf[l], TRANSFER_SIZE - l, "<b>charge... </b>");
    }

    if (xEventGroupGetBits(status_event_group) & CHARGE_COMPLETE)
    {
        l += snprintf(&network_buf[l], TRANSFER_SIZE - l, "<b>Charge complete </b>");
    }

    l += snprintf(&network_buf[l], TRANSFER_SIZE - l, ", NB-IoT: <b>%s</b> ", net_status_current);

    if (strlen(pdp_ip) > 0)
        l += snprintf(&network_buf[l], TRANSFER_SIZE - l, ", IP: %s", pdp_ip);

    l += snprintf(&network_buf[l], TRANSFER_SIZE - l, ", Error: <b>%s %s %s %s</b>", (result.measure.d_thsensor_error == 1) ? "TH" : "", (result.measure.d_dallas_sensor_error == 1) ? "DS" : "", (result.measure.d_mag_sensor_error == 1) ? "ACC/MAG" : "", (result.measure.d_nbiot_error == 1) ? "NBIoT" : "");

    l += snprintf(&network_buf[l], TRANSFER_SIZE - l, ", OPEN: <b>%s</b> ", (result.measure.open == 1) ? "1" : "0");
    l += snprintf(&network_buf[l], TRANSFER_SIZE - l, ", CLOSE: <b>%s</b> ", (result.measure.close == 1) ? "1" : "0");


    httpd_resp_send_chunk(req, network_buf, l);

    do
    {
        l = get_menu_html(network_buf);
        if (l > 0)
            httpd_resp_send_chunk(req, network_buf, l);

    } while (l > 0);

    httpd_resp_sendstr_chunk(req, NULL);

    return ESP_OK;
}

static esp_err_t menu_post_handler(httpd_req_t *req)
{
    int ret, remaining = req->content_len;

    while (remaining > 0)
    {
        /* Read the data for the request */
        if ((ret = httpd_req_recv(req, network_buf, MIN(remaining, sizeof(network_buf)))) <= 0)
        {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT)
            {
                /* Retry receiving if timeout occurred */
                continue;
            }
            return ESP_FAIL;
        }

        remaining -= ret;

        /* Log data received */
        // ESP_LOGI(TAGH, "=========== RECEIVED DATA ==========");
        // ESP_LOGI(TAGH, "%.*s", ret, buf);
        // ESP_LOGI(TAGH, "====================================");
    }

    network_buf[req->content_len] = '\0';

    char *s = network_buf;
    char name[16];
    while (s && s < (network_buf + req->content_len))
    {
        char *e = strchr(s, '=');
        *e = '\0';
        strncpy(name, s, sizeof(name));

        int v = atoi(e + 1);
        // ESP_LOGD("menu_post_handler", "Name  \"%s\" : \"%i\"", name, v);
        set_menu_val_by_id(name, v);

        s = strchr(e + 1, '&');
        if (s)
            s = s + 1;
    }

    // End response
    return download_get_handler(req);
}

esp_err_t get_history(httpd_req_t *req)
{
    int l = 0;
    reset_sleep_timeout();

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Connection", "close");

    network_buf[0] = '\0';

    int hpos = history_pos + HISTORY_SIZE;
    int hend = history_pos;

    l = snprintf(&network_buf[l], (TRANSFER_SIZE - l), "bootcount, " OUT_MEASURE_HEADERS "\n");

    while (hpos > hend)
    {
        int indx = hpos % HISTORY_SIZE;
        // ESP_LOGI("menu", "%3i, " OUT_MEASURE_FORMATS, history[indx].bootcount, OUT_MEASURE_VARS(history[indx]));
        l += snprintf(&network_buf[l], (TRANSFER_SIZE - l), "%3i, " OUT_MEASURE_FORMATS "\n", history[indx].bootcount, OUT_MEASURE_VARS(history[indx]));
        hpos--;

        if ((TRANSFER_SIZE - l) < sizeof(OUT_MEASURE_FORMATS) * 2)
        {
            if (httpd_resp_send_chunk(req, network_buf, l) != ESP_OK)
            {
                ESP_LOGE("WWW", "History sending failed!");
                /* Abort sending file */
                ESP_ERROR_CHECK_WITHOUT_ABORT(httpd_resp_sendstr_chunk(req, NULL));
                /* Respond with 500 Internal Server Error */
                ESP_ERROR_CHECK_WITHOUT_ABORT(httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file"));
                return ESP_FAIL;
            }
            l = 0;
        }
    }

    if (l > 0)
    {
        if (httpd_resp_send_chunk(req, network_buf, l) != ESP_OK)
        {
            ESP_LOGE("WWW", "History sending failed!");
            /* Abort sending file */
            ESP_ERROR_CHECK_WITHOUT_ABORT(httpd_resp_sendstr_chunk(req, NULL));
            /* Respond with 500 Internal Server Error */
            ESP_ERROR_CHECK_WITHOUT_ABORT(httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file"));
            return ESP_FAIL;
        }
    }

    /* Respond with an empty chunk to signal HTTP response completion */
    ESP_ERROR_CHECK_WITHOUT_ABORT(httpd_resp_send_chunk(req, NULL, 0));
    return ESP_OK;
};

// Get measure data
esp_err_t d_get(httpd_req_t *req)
{
    reset_sleep_timeout();

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"data.txt\"");
    httpd_resp_set_hdr(req, "Connection", "close");

    int l = 0;
    int ll = 0;
    int n = 0;
    network_buf[0] = '\0';

    do
    {

        ll = getResult_Data(&network_buf[l], n);

        if (ll == 0) // data end
            break;

        l = l + ll;
        if (l > (sizeof(network_buf) - 128))
        {
            /* Send the buffer contents as HTTP response chunk */
            if (httpd_resp_send_chunk(req, network_buf, l) != ESP_OK)
            {
                ESP_LOGE("WWW", "File sending failed!");
                /* Abort sending file */
                ESP_ERROR_CHECK_WITHOUT_ABORT(httpd_resp_sendstr_chunk(req, NULL));
                /* Respond with 500 Internal Server Error */
                ESP_ERROR_CHECK_WITHOUT_ABORT(httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file"));
                return ESP_FAIL;
            }

            l = 0;
        }
        n++;
    } while (ll > 0);

    if (l > 0)
    {
        if (httpd_resp_send_chunk(req, network_buf, l) != ESP_OK)
        {
            ESP_LOGE("WWW", "File sending failed!");
            /* Abort sending file */
            ESP_ERROR_CHECK_WITHOUT_ABORT(httpd_resp_sendstr_chunk(req, NULL));
            /* Respond with 500 Internal Server Error */
            ESP_ERROR_CHECK_WITHOUT_ABORT(httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file"));
            return ESP_FAIL;
        }
    }

    /* Respond with an empty chunk to signal HTTP response completion */
    ESP_ERROR_CHECK_WITHOUT_ABORT(httpd_resp_send_chunk(req, NULL, 0));
    return ESP_OK;
};

#define ESP_IMAGE_HEADER_MAGIC 0xE9 /*!< The magic word for the esp_image_header_t structure. */

/*
 * Handle OTA file upload
 */
esp_err_t update_post_handler(httpd_req_t *req)
{
    int file_id = -1;
    esp_ota_handle_t ota_handle;
    int remaining = req->content_len;

    reset_sleep_timeout();

    /* Пишем в next_ota и прошивку и spiffs.bin*/
    const esp_partition_t *ota_partition = esp_ota_get_next_update_partition(NULL);

    while (remaining > 0)
    {
        int recv_len = httpd_req_recv(req, network_buf, MIN(remaining, sizeof(network_buf)));

        // Timeout Error: Just retry
        if (recv_len == HTTPD_SOCK_ERR_TIMEOUT)
        {
            continue;

            // Serious Error: Abort OTA
        }
        else if (recv_len <= 0)
        {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Protocol Error");
            return ESP_FAIL;
        }

        if (file_id == -1) // first data block
        {
            file_id = network_buf[0];

            if (file_id == ESP_IMAGE_HEADER_MAGIC)
            {
                ESP_ERROR_CHECK(esp_ota_begin(ota_partition, OTA_SIZE_UNKNOWN, &ota_handle));
            }
            else if (remaining == 0x80000)
            {
                file_id = remaining;
                ESP_ERROR_CHECK(esp_partition_erase_range(ota_partition, 0, 0x80000));
            }
            else
            {
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "File Error");
                return ESP_FAIL;
            }
        }

        // firmware.bin
        if (file_id == ESP_IMAGE_HEADER_MAGIC)
        {
            // Successful Upload: Flash firmware chunk
            if (esp_ota_write(ota_handle, (const void *)network_buf, recv_len) != ESP_OK)
            {
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Flash Error");
                return ESP_FAIL;
            }
            vTaskDelay(1);
        }
        else
            // spiffs.bin
            if (file_id == 0x80000)
            {
                ESP_ERROR_CHECK(esp_partition_write(ota_partition, (req->content_len - remaining), (const void *)network_buf, recv_len));
                vTaskDelay(1);
            }

        remaining -= recv_len;
    }

    if (file_id == ESP_IMAGE_HEADER_MAGIC)
    {
        // Validate and switch to new OTA image and reboot
        if (esp_ota_end(ota_handle) != ESP_OK || esp_ota_set_boot_partition(ota_partition) != ESP_OK)
        {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Validation / Activation Error");
            return ESP_FAIL;
        }

        httpd_resp_sendstr(req, "Firmware update complete, rebooting now!\n");
        ESP_LOGW(TAGH, "Firmware update complete, rebooting now!");
        vTaskDelay(500 / portTICK_PERIOD_MS);
        restart = true;
    }
    else if (file_id == 0x80000)
    {
        const esp_partition_t *storage_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
        ESP_ERROR_CHECK(esp_partition_erase_range(storage_partition, 0, 0x80000));

        remaining = req->content_len;
        int recv_len = sizeof(network_buf);
        while (remaining > 0)
        {
            recv_len = MIN(remaining, sizeof(network_buf));
            if (esp_partition_read(ota_partition, (req->content_len - remaining), (void *)network_buf, recv_len) == ESP_OK)
            {
                ESP_ERROR_CHECK(esp_partition_write(storage_partition, (req->content_len - remaining), (const void *)network_buf, recv_len));
                vTaskDelay(1);
            }
            remaining -= recv_len;
        }

        httpd_resp_sendstr(req, "SPIFFS update complete, rebooting now!\n");
        ESP_LOGW(TAGH, "SPIFFS update complete, rebooting now!");
        vTaskDelay(500 / portTICK_PERIOD_MS);
        restart = true;
    }

    return ESP_OK;
}

/*
 * Structure holding server handle
 * and internal socket fd in order
 * to use out of request send
 */
struct t_async_resp_arg
{
    httpd_handle_t hd;
    int fd;
    char *data;
} async_resp_arg;

esp_err_t ws_send_acc_data(httpd_req_t *req)
{
    static httpd_ws_frame_t ws_pkt;
    static char accbuf[48];
    // ws_pkt.len = sprintf(accbuf, "%.2f %.2f %.2f;%.2f %.2f %.2f;0x%04X", result.measure.acc[0], result.measure.acc[1], result.measure.acc[2], result.measure.mag[0], result.measure.mag[1], result.measure.mag[2], result.measure.flags);
    ws_pkt.len = sprintf(accbuf, OUT_MEASURE_ACC_FORMATS, OUT_MEASURE_ACC_VARS(result.measure));
    ws_pkt.payload = (uint8_t *)accbuf;
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    if (req)
        return httpd_ws_send_frame(req, &ws_pkt);
    else
        return httpd_ws_send_frame_async(async_resp_arg.hd, async_resp_arg.fd, &ws_pkt);
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET)
    {
        ESP_LOGI(TAGH, "Handshake done, the new connection was opened");
        return ESP_OK;
    }
    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    /* Set max_len = 0 to get the frame len */
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAGH, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ret;
    }
    ESP_LOGI(TAGH, "frame len is %d", ws_pkt.len);
    if (ws_pkt.len)
    {
        /* ws_pkt.len + 1 is for NULL termination as we are expecting a string */
        buf = calloc(1, ws_pkt.len + 1);
        if (buf == NULL)
        {
            ESP_LOGE(TAGH, "Failed to calloc memory for buf");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        /* Set max_len = ws_pkt.len to get the frame payload */
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAGH, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }
        ESP_LOGI(TAGH, "Got packet with message: %s", ws_pkt.payload);
    }

    ESP_LOGI(TAGH, "Packet type: %d", ws_pkt.type);
    if ((ws_pkt.type == HTTPD_WS_TYPE_TEXT) && (strncmp((const char *)ws_pkt.payload, "id=", 3) == 0))
    {
        if (strncmp((const char *)(&ws_pkt.payload[3]), "accmag", 6) == 0)
        {
            xTaskNotify(xTaskI2C, NOTYFY_SENSOR_SET_MAGACC | NOTYFY_SENSOR_MAGACC_SPEEDCONT, eSetValueWithOverwrite);
            // return trigger_async_send(req->handle, req);
            async_resp_arg.fd = httpd_req_to_sockfd(req);
            async_resp_arg.hd = req->handle;

            // ESP_LOGI(TAGH, "fd: %d", async_resp_arg.fd);
        }
        else
        {
            ws_send_acc_data(req);
            xTaskNotify(xTaskI2C, NOTYFY_SENSOR_SET_MAGACC_INT, eSetValueWithOverwrite);
            // async_resp_arg.fd = 0;
        }

        reset_sleep_timeout();
    }

    free(buf);
    return ret;
}

/*
esp_err_t sensor_get_handler(httpd_req_t *req)
{

    buf_len = httpd_req_get_url_query_len(req) + 1;

    if (buf_len < 15)
    {
        if (httpd_req_get_url_query_str(req, paramstring, buf_len) == ESP_OK)
        {
            if (httpd_query_key_value(paramstring, "id", param, sizeof(param)) == ESP_OK)
            {
                ESP_LOGI("http", "Found URL query parameter => id=%s", param);
            };
        };
    };

    if (strncmp("accmag", param) == 0)
    {
        xTaskNotify(xTaskI2C, NOTYFY_SENSOR_MAGACC | NOTYFY_SENSOR_MAGACC_SPEEDCONT, eSetBits);

        EventBits_t uxBits = xEventGroupWaitBits(
            status_event_group, // The event group being tested.
            END_MAG_SENSOR,     // The bits within the event group to wait for.
            pdTRUE,             // BIT_0 & BIT_1 should be cleared before returning.
            pdFALSE,            // ОБА
            1000 / portTICK_PERIOD_MS);

        sprintf(buf, "%.1f, %.1f, %.1f, %.1f, %.1f, %.1f", result.measure.acc[0], result.measure.acc[1], result.measure.acc[2], result.measure.mag[0], result.measure.mag[1], result.measure.mag[2]);
        httpd_resp_sendstr(req, buf);
    }
    else
    {
        // Set status
        httpd_resp_set_status(req, "302 Temporary Redirect");
        // Redirect to the "/" root directory
        httpd_resp_set_hdr(req, "Location", "/");
        // iOS requires content in the response to detect a captive portal, simply redirecting is not sufficient.
        httpd_resp_send(req, "Redirect to root", HTTPD_RESP_USE_STRLEN);
    }

    return ESP_OK;
}
*/
static const httpd_uri_t main_page = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = download_get_handler,
    .user_ctx = &((down_data_t){.filepath = "/spiffs/main.html", .content = "text/html"}),
};

static const httpd_uri_t menu_page = {
    .uri = "/menu",
    .method = HTTP_GET,
    .handler = menu_get_handler,
    .user_ctx = &((down_data_t){.filepath = "/spiffs/main.html", .content = "text/html"}),
};

static const httpd_uri_t data_page = {
    .uri = "/" DATAFILE,
    .method = HTTP_GET,
    .handler = download_get_handler,
    .user_ctx = &((down_data_t){.filepath = "/spiffs/" DATAFILE, .content = "text/csv"}),
};

static const httpd_uri_t olddata_page = {
    .uri = "/old" DATAFILE,
    .method = HTTP_GET,
    .handler = download_get_handler,
    .user_ctx = &((down_data_t){.filepath = "/spiffs/old" DATAFILE, .content = "text/csv"})};

static const httpd_uri_t menu_post = {
    .uri = "/",
    .method = HTTP_POST,
    .handler = menu_post_handler,
    .user_ctx = &((down_data_t){.filepath = "/spiffs/main.html", .content = "text/html"})};

static const httpd_uri_t update_get = {
    .uri = "/update",
    .method = HTTP_GET,
    .handler = download_get_handler,
    .user_ctx = &((down_data_t){.filepath = "/spiffs/update.html", .content = "text/html"})};

static const httpd_uri_t update_post = {
    .uri = "/update",
    .method = HTTP_POST,
    .handler = update_post_handler,
    .user_ctx = NULL};

static const httpd_uri_t data_csv = {
    .uri = "/d.csv",
    .method = HTTP_GET,
    .handler = d_get,
    .user_ctx = NULL};

static const httpd_uri_t history_url = {
    .uri = "/history",
    .method = HTTP_GET,
    .handler = get_history,
    .user_ctx = NULL};

static const httpd_uri_t d3 = {
    .uri = "/d3",
    .method = HTTP_GET,
    .handler = download_get_handler,
    .user_ctx = &((down_data_t){.filepath = "/spiffs/D3.html", .content = "text/html"})};

static const httpd_uri_t d3_get_gz = {
    .uri = "/d3.min.js",
    .method = HTTP_GET,
    .handler = download_get_handler,
    .user_ctx = &((down_data_t){.filepath = "/spiffs/d3.min.js.gz", .content = "application/javascript"})};

static const httpd_uri_t ws = {
    .uri = "/ws",
    .method = HTTP_GET,
    .handler = ws_handler,
    .user_ctx = NULL,
    .is_websocket = true};

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    // config.max_open_sockets = 5;
    // config.stack_size = 1024 * 10;
    config.lru_purge_enable = true;
    // config.send_wait_timeout = 30;
    // config.recv_wait_timeout = 30;
    // config.task_priority = 6;
    // config.close_fn = ws_close_fn;
    config.max_uri_handlers = 16;

    // Start the httpd server
    ESP_LOGI(TAGH, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK)
    {
        // Set URI handlers
        ESP_LOGI(TAGH, "Registering URI handlers");
        httpd_register_uri_handler(server, &main_page);
        httpd_register_uri_handler(server, &menu_page);
        httpd_register_uri_handler(server, &menu_post);

        httpd_register_uri_handler(server, &update_post);
        httpd_register_uri_handler(server, &update_get);

        httpd_register_uri_handler(server, &data_page);
        httpd_register_uri_handler(server, &olddata_page);

        httpd_register_uri_handler(server, &history_url);

        httpd_register_uri_handler(server, &data_csv);
        httpd_register_uri_handler(server, &d3);
        httpd_register_uri_handler(server, &d3_get_gz);

        httpd_register_uri_handler(server, &ws);

        return server;
    }

    ESP_LOGE(TAGH, "Error starting server!");
    return NULL;
}

void wifi_task(void *arg)
{
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Ожидаем уведомления безконечно, для запуска WiFi

    xEventGroupClearBits(status_event_group, END_WIFI);

    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_softap(); // WiFi channel

    /* Start the server for the first time */
    start_webserver();

    /* Mark current app as valid */
    const esp_partition_t *partition = esp_ota_get_running_partition();
    // printf("Currently running partition: %s\r\n", partition->label);
    ESP_LOGI(TAGW, "Currently running partition: %s", partition->label);

    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(partition, &ota_state) == ESP_OK)
    {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY)
        {
            esp_ota_mark_app_valid_cancel_rollback();
        }
    }

    reset_sleep_timeout();

    while (1)
    {
        // WiFi timeout
        // if (esp_timer_get_time() - timeout_begin > 6 * 60 * 1000000)
        //{
        //    xEventGroupSetBits(status_event_group, WIFI_STOP);
        //}

        EventBits_t uxBits = xEventGroupWaitBits(
            status_event_group, //* The event group being tested.
            READ_MAG_SENSOR,    //* The bits within the event group to wait for.
            pdTRUE,             //* BIT_0 & BIT_1 should be cleared before returning.
            pdFALSE,            //* ОБА
            100 / portTICK_PERIOD_MS);

        if ((uxBits & READ_MAG_SENSOR) && async_resp_arg.fd > 0)
        {
            if (ws_send_acc_data(0) != ESP_OK)
            {
                async_resp_arg.fd = 0;
            };
        }

        if (uxBits & END_WORK_NBIOT)
        {
            esp_wifi_stop();
            xEventGroupSetBits(status_event_group, END_WIFI);
        }

        if (restart == true)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            esp_wifi_stop();
            esp_restart();
        }

        // vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
