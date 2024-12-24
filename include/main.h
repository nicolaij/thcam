#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

#include "esp_log.h"

#include <sys/time.h>

#define MODEM_POWER GPIO_NUM_10
// пробуждение от зарядки
#define PIN_BATT GPIO_NUM_0
#define PIN_INT_MAG GPIO_NUM_5
#define PIN_WATER3 GPIO_NUM_1
#define PIN_LIGHT GPIO_NUM_4
#define PIN_WATER1 GPIO_NUM_2
#define PIN_WATER2 GPIO_NUM_3
#define PIN_ONEWARE GPIO_NUM_8
#define PIN_BUTTON_BOOT GPIO_NUM_9

#define SCL_PIN (GPIO_NUM_6)
#define SDA_PIN (GPIO_NUM_7)
#define TXD_PIN (GPIO_NUM_19)
#define RXD_PIN (GPIO_NUM_18)

#define END_WORK BIT1
#define END_WIFI BIT2
#define END_RADIO BIT3
#define NOW_CHARGE BIT4
#define WIFI_ACTIVE BIT5
// #define NEED_WIFI BIT6
#define CHARGE_COMPLETE BIT7
#define SERIAL_TERMINAL_ACTIVE BIT8
#define NB_TERMINAL BIT9
#define END_DS_SENSOR BIT10
#define END_TH_SENSOR BIT11
#define END_MAG_SENSOR BIT12

#define ONEWIRE_MAX_DS18B20 1

#define NOTYFY_SENSOR_TH BIT0
#define NOTYFY_SENSOR_MAG BIT1
#define NOTYFY_SENSOR_ACC BIT2
#define NOTYFY_SENSOR_MAGACC BIT3
#define NOTYFY_SENSOR_MAGACC_CONT BIT4
#define NOTYFY_SENSOR_MAGACC_SPEEDCONT BIT5
#define NOTYFY_SENSOR_MAGACC_STOP BIT6

extern EventGroupHandle_t status_event_group;

extern TaskHandle_t xHandleWifi;
extern TaskHandle_t xTaskI2C;

void modem_task(void *arg);
void led_task(void *arg);
void console_task(void *arg);
void btn_task(void *arg);
void wifi_task(void *arg);
void dallas_task(void *arg);
void i2c_task(void *arg);

esp_err_t read_nvs_menu();
esp_err_t init_nvs();
int get_menu_id(const char *id);
esp_err_t set_menu_id(const char *id, int value);
int get_menu_json(char *buf);
int get_menu_html(char *buf);

void dio_init();
uint64_t dio_sleep();
int get_charge();

void nbiot_power_pin(const TickType_t xTicksToDelay);
void nbiot_power_off();

esp_err_t print_atcmd(const char *cmd, char *buffer);

int getResult_Data(char *line, int data_pos);

esp_err_t read_nvs_id(const char *key, uint64_t *out_value);

float get_temperature_sensor();

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
        uint16_t discrete;
        struct
        {
            bool d_light : 1;
            bool d_water : 1;
            bool d_wet_mode : 1;
            bool d_charge : 1;

            bool d_nbiot_send_succes : 1;
            bool reserved6 : 1;
            bool reserved7 : 1;
            bool reserved8 : 1;
            
            bool d_thsensor_error : 1;
            bool d_dallas_sensor_error : 1;
            bool d_mag_sensor_error : 1;
            bool d_nbiot_error : 1;
            
            bool reserved13 : 1;
            bool reserved14 : 1;
            bool reserved15 : 1;
            bool reserved16 : 1;
        };
    };
    int bootcount;
    float internal_temp;
    float temp;
    float humidity;
    float pressure;
    float light;
    float water_temp;
    float water;
    float acc[3];
    float mag[3];
    float nbbattery;
    float rssi;
} measure_data_t;

typedef struct
{
    measure_data_t measure;
    time_t ttime;
} result_data_t;

extern result_data_t result;
extern result_data_t old_result;

#define OUT_JSON "{\"id\":\"cam%d\",\"num\":%d,\"dt\":\"%s\",\"RSSI\":%.0f,\"Battery\":%.3f,\"Light\":%.1f,\"Water\":%.1f,\"WaterTemp\":%.1f,\"Temp\":%.1f,\"Humidity\":%.1f,\"Pressure\":%.3f,\"Acc\":[%.1f,%.1f,%.1f],\"Mag\":[%.1f,%.1f,%.1f],\"Flags\":\"0x%04X\"}"
#define OUT_MEASURE_VARS(prefix) prefix.rssi, prefix.nbbattery, prefix.light, prefix.water, prefix.water_temp, prefix.temp, prefix.humidity, prefix.pressure, prefix.acc[0], prefix.acc[1], prefix.acc[2], prefix.mag[0], prefix.mag[1], prefix.mag[2], prefix.discrete
#define OUT_MEASURE_HEADERS "RSSI, Battery, Light, Water, WaterTemp, Temp, Humidity, Pressure, AccX, AccY, AccZ, MagX, MagY, MagZ, Flags"
#define OUT_MEASURE_FORMATS "%2.0f, %.3f, %3.1f, %3.1f, %2.1f, %2.1f, %2.1f, %3.3f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, 0x%04X"

#define HISTORY_SIZE 100
extern measure_data_t history[HISTORY_SIZE];
extern RTC_DATA_ATTR int history_pos;

extern int bootCount;
extern int wait_max_counter;

#define DATAFILE "data.csv"
