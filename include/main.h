#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

#include <sys/time.h>

#define MODEM_POWER GPIO_NUM_10
#define PIN_BATT GPIO_NUM_0
#define PIN_BATT_CONTROL GPIO_NUM_5
#define PIN_LIGHT GPIO_NUM_3
#define PIN_WATER1 GPIO_NUM_2
#define PIN_WATER2 GPIO_NUM_4

#define TXD_PIN (GPIO_NUM_19)
#define RXD_PIN (GPIO_NUM_18)

#define END_RADIO_SLEEP BIT3
#define END_WORK BIT1

extern EventGroupHandle_t ready_event_group;

void modem_task(void *arg);
void led_task(void *arg);

void dio_init();
void dio_sleep();

void nbiot_power_pin(const TickType_t xTicksToDelay);

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
        unsigned int discrete;
        struct
        {
            bool d_light;
            bool d_water;
        };
    };
    float internal_temp;
    float temp;
    float humidity;
    float light;
    float water;
    float water2;
    float battery;
} measure_data_t;

typedef struct
{
    int bootCount;
    measure_data_t measure;
    time_t ttime;
} result_data_t;

extern result_data_t result;
