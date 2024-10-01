#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

#include "esp_log.h"

#include <sys/time.h>

#define MODEM_POWER GPIO_NUM_10
#define PIN_BATT GPIO_NUM_0
#define PIN_CHARGE_CONTROL GPIO_NUM_5
#define PIN_LIGHT GPIO_NUM_4
#define PIN_WATER1 GPIO_NUM_2
#define PIN_WATER2 GPIO_NUM_3
#define PIN_BUTTON_BOOT GPIO_NUM_9

#define TXD_PIN (GPIO_NUM_19)
#define RXD_PIN (GPIO_NUM_18)

#define END_WORK BIT1
#define WIFI_STOP BIT2
#define END_RADIO_SLEEP BIT3
#define NOW_CHARGE BIT4
#define NEED_TRANSMIT BIT5
#define NEED_WIFI BIT6
#define CHARGE_COMPLETE BIT7
#define NBTERMINAL_ACTIVE BIT8
#define NB_STOP BIT9

extern EventGroupHandle_t ready_event_group;

void modem_task(void *arg);
void led_task(void *arg);
void console_task(void *arg);
void btn_task(void *arg);
void wifi_task(void *arg);

esp_err_t read_nvs_menu();
esp_err_t init_nvs();
int get_menu_id(const char *id);
esp_err_t set_menu_id(const char *id, int value);
int get_menu_json(char *buf);
int get_menu_html(char *buf);

void dio_init();
uint64_t dio_sleep();
void stop_charge();
int get_charge();

void nbiot_power_pin(const TickType_t xTicksToDelay);

esp_err_t print_atcmd(const char *cmd, char *buffer);


typedef struct
{
    const char id[10];
    const char name[64];
    const char izm[8];
    int32_t val;
    const int32_t min;
    const int32_t max;
} menu_t;

typedef struct
{
    union
    {
        uint32_t set;
        struct
        {
            uint8_t bright;
            uint8_t red;
            uint8_t green;
            uint8_t blue;
        };
    };

    TickType_t xTicksToDelay;

} led_task_data_t;

typedef struct
{
    union
    {
        uint8_t discrete;
        struct
        {
            bool d_light : 1;
            bool d_water : 1;
            bool d_charge : 1;
            bool d_wet_mode : 1;
            bool d_nbiot_send_succes : 1;
            bool d_nbiot_sim_error : 1;
            bool d_thsensor_error : 1;
        };
    };
    float internal_temp;
    float temp;
    float humidity;
    float pressure;
    float light;
    float water;
    float water2;
    float battery;
    float nbbattery;
    float rssi;
} measure_data_t;

typedef struct
{
    int bootCount;
    measure_data_t measure;
    time_t ttime;
} result_data_t;

extern result_data_t result;

#define OUT_JSON "{\"id\":\"cam%d\",\"num\":%d,\"dt\":\"%s\",\"RSSI\":%3.0f,\"Battery\":%1.3f,\"Light\":%4.0f,\"Water\":%4.0f,\"Water2\":%4.0f,\"Temp\":%2.1f,\"Humidity\":%3.1f,\"Pressure\":%4.3f,\"Flags\":\"0x%02X\"}"
#define OUT_MEASURE_VARS(prefix) prefix.rssi, prefix.nbbattery, prefix.light, prefix.water, prefix.water2, prefix.temp, prefix.humidity, prefix.pressure, prefix.discrete
#define OUT_MEASURE_HEADERS "RSSI, Battery, Light, Water, Water2, Temp, Humidity, Pressure, Flags"
#define OUT_MEASURE_FORMATS "%3.0f, %1.3f, %4.0f, %4.0f, %4.0f, %2.1f, %3.1f, %4.3f, 0x%02X"

#define HISTORY_SIZE 150
extern measure_data_t history[HISTORY_SIZE];

extern int bootCount;

#define DATAFILE "data.csv"
