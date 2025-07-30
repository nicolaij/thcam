#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

#include "esp_log.h"

#include "cJSON.h"

#include <sys/time.h>

#define MODEM_POWER GPIO_NUM_10
// пробуждение от зарядки
#define PIN_BATT GPIO_NUM_0
#define PIN_INT_ACC GPIO_NUM_5
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

#define WIFI_CHANNEL 11
#define AP_WIFI_SSID "THCam"
#define AP_WIFI_PASS "123123123"

#define END_WORK_NBIOT BIT1
#define END_WIFI BIT2
#define END_RADIO BIT3
#define NOW_CHARGE BIT4
#define WIFI_ACTIVE BIT5
#define END_WORK_WIFI BIT6
#define CHARGE_COMPLETE BIT7
#define SERIAL_TERMINAL_ACTIVE BIT8
#define NB_TERMINAL BIT9
// #define END_DS_SENSOR BIT10
// #define END_TH_SENSOR BIT11
#define READ_MAG_SENSOR BIT12
#define TEST_MODE_UPDATED BIT13
#define REBOOT_NOW BIT14

#define ONEWIRE_MAX_DS18B20 1

#define NOTYFY_SENSOR_TH BIT0
#define NOTYFY_SENSOR_MAG BIT1
#define NOTYFY_SENSOR_ACC BIT2
#define NOTYFY_SENSOR_MAGACC BIT3
#define NOTYFY_SENSOR_MAGACC_CONT BIT4
#define NOTYFY_SENSOR_MAGACC_SPEEDCONT BIT5
#define NOTYFY_SENSOR_MAGACC_STOP BIT6
#define NOTYFY_SENSOR_SET_MAGACC_INT BIT7
#define NOTYFY_SENSOR_MAGACC_GET_INT BIT8
#define NOTYFY_SENSOR_SET_MAGACC BIT9
#define NOTYFY_TEST BIT10

#define NOTYFY_WIFI_ESPNOW BIT2
#define NOTYFY_WIFI BIT1

extern EventGroupHandle_t status_event_group;

extern TaskHandle_t xHandleWifi;
extern TaskHandle_t xTaskI2C;

void modem_task(void *arg);
void console_task(void *arg);
void btn_task(void *arg);
void wifi_task(void *arg);
void dallas_task(void *arg);
void i2c_task(void *arg);

esp_err_t read_nvs_menu();
esp_err_t init_nvs();
int get_menu_pos_by_id(const char *id);
int get_menu_val_by_id(const char *id);
esp_err_t set_menu_val_by_id(const char *id, int value);
int get_menu_json(char *buf);
int get_menu_html(char *buf);
void light_measure(int test_count);

uint64_t dio_init();
uint64_t dio_sleep(uint64_t wake_mask);
int get_charge();

void nbiot_power_pin(const TickType_t xTicksToDelay);

esp_err_t print_atcmd(const char *cmd, char *buffer);

int getResult_Data(char *line, int data_pos);

esp_err_t read_nvs_id(const char *key, uint64_t *out_value);

bool check_range(int x, int y, int z, int setx, int sety, int setz, int devi);

float get_temperature_sensor();

char *get_datetime(time_t ttime);

typedef struct
{
    const char id[10];
    const char name[64];
    const char izm[8];
    int val;
    const int min;
    const int max;
} menu_t;

typedef struct
{
    union
    {
        uint16_t flags;
        struct
        {
            bool d_light : 1;   // пробуждение от датчика света
            bool d_water : 1;   // пробуждение от повышения влажности
            bool d_acc_int : 1; // пробуждение от прерывания ACC
            bool d_charge : 1;  // пробуждение от зарядки

            bool d_light_mode : 1; // режим высокой освещенности (включена нижняя подтяжка)
            bool d_wet_mode : 1;   // режим высокой влажности (включена нижняя подтяжка)
            bool reserved7 : 1;
            bool d_nbiot_send_succes : 1; // признак успешной передачи NBIoT

            bool d_thsensor_error : 1;      // ошибка датчика
            bool d_dallas_sensor_error : 1; // ошибка датчика
            bool d_mag_sensor_error : 1;    // ошибка датчика
            bool d_nbiot_error : 1;         // ошибка модуля NBIoT

            bool open : 1;  // Дискретный сигнал открыто
            bool close : 1; // Дискретный сигнал закрыто
            bool reserved15 : 1;
            bool reserved16 : 1;
        };
    };
    unsigned int bootcount;
    float internal_temp;
    float temp;
    float humidity;
    float light;
    float water_temp;
    float water;
    float acc[3];
    float mag[3];
    float nbbattery;
    float rssi;
    unsigned int tac;
    unsigned int ci;
} measure_data_t;

typedef struct
{
    measure_data_t measure;
    time_t ttime;
} result_data_t;

extern result_data_t result;

#define OUT_JSON "{\"id\":\"cam%d\",\"num\":%u,\"dt\":\"%s\",\"Battery\":%.3f,\"RSSI\":%.0f,\"Light\":%.1f,\"Water\":%.1f,\"WaterTemp\":%.1f,\"Temp\":%.1f,\"Humidity\":%.1f,\"Flags\":\"0x%04X\",\"Acc\":[%.2f,%.2f,%.2f],\"Mag\":[%.2f,%.2f,%.2f],\"tac\":%u,\"ci\":%u}"
#define OUT_MEASURE_ACC_VARS(prefix) prefix.acc[0], prefix.acc[1], prefix.acc[2], prefix.mag[0], prefix.mag[1], prefix.mag[2]
#define OUT_MEASURE_VARS(prefix) prefix.nbbattery, prefix.rssi, prefix.light, prefix.water, prefix.water_temp, prefix.temp, prefix.humidity, prefix.flags, OUT_MEASURE_ACC_VARS(prefix), prefix.tac, prefix.ci
#define OUT_MEASURE_HEADERS "Battery, RSSI, Light, Water, WaterTemp, Temp, Humidity, Flags, AccX, AccY, AccZ, MagX, MagY, MagZ, tac, ci"
#define OUT_MEASURE_ACC_FORMATS "%.2f, %.2f, %.2f, %.2f, %.2f, %.2f"
#define OUT_MEASURE_FORMATS "%.3f, %2.0f, %3.1f, %3.1f, %2.1f, %2.1f, %2.1f, 0x%04X, " OUT_MEASURE_ACC_FORMATS ", %u, %u"

#define HISTORY_SIZE 80
extern result_data_t history[HISTORY_SIZE];
extern uint8_t history_pos;

extern unsigned int bootCount;
extern int wait_max_counter;

#define DATAFILE "data.csv"
