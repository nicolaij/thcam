#include "main.h"

#include "onewire_bus.h"
#include "ds18b20.h"

RTC_DATA_ATTR uint64_t water_temp_id;

RTC_DATA_ATTR uint8_t th_sensor;

#include "driver/i2c_master.h"

i2c_master_dev_handle_t th_handle;

i2c_master_dev_handle_t lsm303A_handle;
i2c_master_dev_handle_t lsm303M_handle;

i2c_master_dev_handle_t exp_handle;

#include "driver/temperature_sensor.h"

#define I2C_TIMEOUT_VALUE_MS 20

#include "LSM303DLHC.h"

bool check_range(int x, int y, int z, int setx, int sety, int setz, int devi)
{
    if ((setx + devi) > x && (setx - devi) < x && (sety + devi) > y && (sety - devi) < y && (setz + devi) > z && (setz - devi) < z)
    {
        return true;
    }
    return false;
}

float get_temperature_sensor()
{
    float internal_temp = 0;
    ESP_LOGD("main", "Initializing Temperature sensor");

    temperature_sensor_handle_t temp_sensor = NULL;
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);

    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));

    ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, &internal_temp));

    ESP_ERROR_CHECK(temperature_sensor_disable(temp_sensor));
    ESP_ERROR_CHECK(temperature_sensor_uninstall(temp_sensor));

    ESP_LOGI("temperature_sensor", "Internal temperature:  %.01f°C", internal_temp);
    return internal_temp;
};

int compare_function(const void *a, const void *b)
{
    const int *x = a;
    const int *y = b;
    return *x - *y;
}

bool htu21_crc_check(uint16_t value, uint8_t crc)
{
    uint32_t polynom = 0x988000; // x^8 + x^5 + x^4 + 1
    uint32_t msb = 0x800000;
    uint32_t mask = 0xFF8000;
    uint32_t result = (uint32_t)value << 8; // Pad with zeros as specified in spec

    while (msb != 0x80)
    {

        // Check if msb of current value is 1 and apply XOR mask
        if (result & msb)
            result = ((result ^ polynom) & mask) | (result & ~mask);

        // Shift by one
        msb >>= 1;
        mask >>= 1;
        polynom >>= 1;
    }
    if (result == crc)
        return true;
    else
        return false;
}

void i2c_task(void *arg)
{

    // i2c
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = SCL_PIN,
        .sda_io_num = SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err_rc;

    i2c_master_bus_handle_t i2cbus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2cbus_handle));

    th_sensor = 0x40; // HTU21

    i2c_device_config_t dev_th_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = th_sensor,
        .scl_speed_hz = 400000,
    };

    // LSM303M
    i2c_device_config_t dev_m_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LSM303DLHC_ADDRESS_M,
        .scl_speed_hz = 400000,
    };
    i2c_device_config_t dev_a_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LSM303DLHC_ADDRESS_A,
        .scl_speed_hz = 400000,
    };

    // PI4IOE5V6408
    i2c_device_config_t dev_exp_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x43,
        .scl_speed_hz = 400000,
    };

    result.measure.d_mag_sensor_error = true;
    result.measure.d_thsensor_error = true;
    result.measure.d_exp_error = true;

    int try = 4;
    do
    {
        try--;
        err_rc = i2c_master_probe(i2cbus_handle, dev_th_cfg.device_address, I2C_TIMEOUT_VALUE_MS);
        if (err_rc != ESP_OK)
        {
            ESP_LOGE("HTU21", "try 0x%02x: %s", dev_th_cfg.device_address, esp_err_to_name(err_rc));
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        else
        {
            ESP_LOGI("HTU21", "try 0x%02x: OK", dev_th_cfg.device_address);
        }
    } while (err_rc != ESP_OK && try > 0);

    if (err_rc == ESP_OK) // Датчик TH найден!
    {
        th_sensor = dev_th_cfg.device_address;
        result.measure.d_thsensor_error = false;
        xTaskNotify(xTaskGetCurrentTaskHandle(), NOTYFY_SENSOR_TH, eSetBits);
    }

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2cbus_handle, &dev_th_cfg, &th_handle));

    try = 2;
    do
    {
        try--;
        err_rc = i2c_master_probe(i2cbus_handle, dev_m_cfg.device_address, I2C_TIMEOUT_VALUE_MS);
        if (err_rc != ESP_OK)
        {
            ESP_LOGE("LSM303", "try 0x%02x: %s", dev_m_cfg.device_address, esp_err_to_name(err_rc));
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        else
        {
            ESP_LOGI("LSM303", "try 0x%02x: OK", dev_m_cfg.device_address);
            xTaskNotify(xTaskGetCurrentTaskHandle(), NOTYFY_SENSOR_MAGACC, eSetBits);
            result.measure.d_mag_sensor_error = false;
        };
    } while (err_rc != ESP_OK && try > 0);

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2cbus_handle, &dev_m_cfg, &lsm303M_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2cbus_handle, &dev_a_cfg, &lsm303A_handle));

    try = 2;
    do
    {
        try--;
        err_rc = i2c_master_probe(i2cbus_handle, dev_exp_cfg.device_address, I2C_TIMEOUT_VALUE_MS);
        if (err_rc != ESP_OK)
        {
            ESP_LOGE("PI4IOE5V6408", "try 0x%02x: %s", dev_exp_cfg.device_address, esp_err_to_name(err_rc));
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        else
        {
            ESP_LOGI("PI4IOE5V6408", "try 0x%02x: OK", dev_exp_cfg.device_address);
            // xTaskNotify(xTaskGetCurrentTaskHandle(), NOTYFY_EXPANDER_RESET, eSetBits);
            // xTaskNotify(xTaskI2C, NOTYFY_EXPANDER_P1_UP | NOTYFY_EXPANDER_P0_UP, eSetBits);
            result.measure.d_exp_error = false;
        };
    } while (err_rc != ESP_OK && try > 0);

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2cbus_handle, &dev_exp_cfg, &exp_handle));

    // start ADC measure
    xTaskNotifyGive(xTaskDIO);
    vTaskDelay(10 / portTICK_PERIOD_MS); //start ADC

    // scan
    /*         for (int i = 0; i < 127; i++)
            {
                err_rc = i2c_master_probe(i2cbus_handle, i, 50);
                if (err_rc == ESP_OK)
                {
                    ESP_LOGI("i2c", "found addr 0x%02x", i);
                }
                vTaskDelay(1);
            } */

    TickType_t tm = 1000 / portTICK_PERIOD_MS;
    uint8_t cmd[2];
    uint8_t buffer[16];

    while (1)
    {
        uint32_t ulNotifiedValue;

        /* Ожидание оповещения. */
        xTaskNotifyWait(pdFALSE,                                                                                 /* Не очищать биты на входе. */
                        ULONG_MAX & ~(NOTYFY_SENSOR_MAGACC_CONT | NOTYFY_SENSOR_MAGACC_SPEEDCONT | NOTYFY_TEST), /* Очистка всех бит на выходе. кроме BIT_NOTYFY_SENSOR_MAGACC_CONT*/
                        &ulNotifiedValue,                                                                        /* Сохраняет значение оповещения. */
                        tm);

        if (result.measure.d_exp_error == false)
        {
            if (ulNotifiedValue & NOTYFY_EXPANDER_RESET)
            {
                cmd[0] = 0x01;
                cmd[1] = 0x01;
                ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit(exp_handle, cmd, 2, I2C_TIMEOUT_VALUE_MS));
                // err_rc = i2c_master_transmit_receive(exp_handle, cmd, 1, buffer, 1, I2C_TIMEOUT_VALUE_MS);
                // if (err_rc == ESP_OK)
                // {
                //     ESP_LOGI("PI4IOE5V6408", "Device ID and Control: 0x%02x", buffer[0]);
                // };
                vTaskDelay(1);
                cmd[0] = 0x00;
                err_rc = i2c_master_transmit_receive(exp_handle, cmd, 1, buffer, 16, I2C_TIMEOUT_VALUE_MS);
                if (err_rc == ESP_OK)
                {
                    ESP_LOG_BUFFER_HEX("PI4IOE5V6408", buffer, 16);
                };
            }

            if (ulNotifiedValue & (NOTYFY_EXPANDER_P1_UP | NOTYFY_EXPANDER_P0_UP | NOTYFY_EXPANDER_P1_DOWN | NOTYFY_EXPANDER_P0_DOWN | NOTYFY_EXPANDER_P0_PULLUP | NOTYFY_EXPANDER_P1_PULLUP | NOTYFY_EXPANDER_P0_PULLDIS | NOTYFY_EXPANDER_P1_PULLDIS))
            {
                // ESP_LOGI("PI4IOE5V6408", "Set output");
                //  Register 03h : I/O Direction
                cmd[0] = 0x03;
                cmd[1] = 0x00;
                if (ulNotifiedValue & (NOTYFY_EXPANDER_P0_UP | NOTYFY_EXPANDER_P0_DOWN))
                    cmd[1] |= BIT0;
                if (ulNotifiedValue & (NOTYFY_EXPANDER_P1_UP | NOTYFY_EXPANDER_P1_DOWN))
                    cmd[1] |= BIT1;

                ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit(exp_handle, cmd, 2, I2C_TIMEOUT_VALUE_MS));
                // vTaskDelay(1);

                // Register 05h : Output Port Register
                cmd[0] = 0x05;
                cmd[1] = 0x00;
                if (ulNotifiedValue & (NOTYFY_EXPANDER_P0_UP))
                    cmd[1] |= BIT0;
                if (ulNotifiedValue & (NOTYFY_EXPANDER_P1_UP))
                    cmd[1] |= BIT1;

                ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit(exp_handle, cmd, 2, I2C_TIMEOUT_VALUE_MS));
                // vTaskDelay(1);

                // Register 07h : Output High-Impedance
                cmd[0] = 0x07;
                cmd[1] = 0xff;
                if (ulNotifiedValue & (NOTYFY_EXPANDER_P0_UP | NOTYFY_EXPANDER_P0_DOWN))
                    cmd[1] &= ~BIT0;
                if (ulNotifiedValue & (NOTYFY_EXPANDER_P1_UP | NOTYFY_EXPANDER_P1_DOWN))
                    cmd[1] &= ~BIT1;

                ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit(exp_handle, cmd, 2, I2C_TIMEOUT_VALUE_MS));
                // vTaskDelay(1);

                // Register 0bh : Pull-Up/-Down Enable
                cmd[0] = 0x0b;
                cmd[1] = 0xff;
                if (ulNotifiedValue & (NOTYFY_EXPANDER_P0_UP | NOTYFY_EXPANDER_P0_PULLDIS))
                    cmd[1] &= ~BIT0;
                if (ulNotifiedValue & (NOTYFY_EXPANDER_P1_UP | NOTYFY_EXPANDER_P1_PULLDIS))
                    cmd[1] &= ~BIT1;
                ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit(exp_handle, cmd, 2, I2C_TIMEOUT_VALUE_MS));

                // Register 0Dh : Pull-Up/-Down Select
                cmd[0] = 0x0d;
                cmd[1] = 0x00;
                if (ulNotifiedValue & (NOTYFY_EXPANDER_P0_PULLUP))
                    cmd[1] |= BIT0;
                if (ulNotifiedValue & (NOTYFY_EXPANDER_P1_PULLUP))
                    cmd[1] |= BIT1;
                ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit(exp_handle, cmd, 2, I2C_TIMEOUT_VALUE_MS));

                // cmd[0] = 0x03;
                // err_rc = i2c_master_transmit_receive(exp_handle, cmd, 1, buffer, 1, I2C_TIMEOUT_VALUE_MS);
                // if (err_rc == ESP_OK)
                // {
                //     ESP_LOGI("PI4IOE5V6408", "Register 0x%02x: 0x%02x", cmd[0], buffer[0]);
                // };

                vTaskDelay(1);
                cmd[0] = 0x00;
                err_rc = i2c_master_transmit_receive(exp_handle, cmd, 1, buffer, 16, I2C_TIMEOUT_VALUE_MS);
                if (err_rc == ESP_OK)
                {
                    ESP_LOG_BUFFER_HEX("PI4IOE5V6408", buffer, 16);
                };
                /*
                                cmd[0] = 0x05;
                                err_rc = i2c_master_transmit_receive(exp_handle, cmd, 1, buffer, 1, I2C_TIMEOUT_VALUE_MS);
                                if (err_rc == ESP_OK)
                                {
                                    ESP_LOGD("PI4IOE5V6408", "Register 0x%02x: 0x%02x", cmd[0], buffer[0]);
                                };
                                cmd[0] = 0x07;
                                err_rc = i2c_master_transmit_receive(exp_handle, cmd, 1, buffer, 1, I2C_TIMEOUT_VALUE_MS);
                                if (err_rc == ESP_OK)
                                {
                                    ESP_LOGD("PI4IOE5V6408", "Register 0x%02x: 0x%02x", cmd[0], buffer[0]);
                                };
                                cmd[0] = 0x0b;
                                err_rc = i2c_master_transmit_receive(exp_handle, cmd, 1, buffer, 1, I2C_TIMEOUT_VALUE_MS);
                                if (err_rc == ESP_OK)
                                {
                                    ESP_LOGD("PI4IOE5V6408", "Register 0x%02x: 0x%02x", cmd[0], buffer[0]);
                                };

                                cmd[0] = 0x0d;
                                err_rc = i2c_master_transmit_receive(exp_handle, cmd, 1, buffer, 1, I2C_TIMEOUT_VALUE_MS);
                                if (err_rc == ESP_OK)
                                {
                                    ESP_LOGD("PI4IOE5V6408", "Register 0x%02x: 0x%02x", cmd[0], buffer[0]);
                                };

                                cmd[0] = 0x0f;
                                err_rc = i2c_master_transmit_receive(exp_handle, cmd, 1, buffer, 1, I2C_TIMEOUT_VALUE_MS);
                                if (err_rc == ESP_OK)
                                {
                                    ESP_LOGD("PI4IOE5V6408", "Register 0x%02x: 0x%02x", cmd[0], buffer[0]);
                                };
                */
            }
        }

        if ((ulNotifiedValue & (NOTYFY_SENSOR_TH | NOTYFY_SENSOR_TH_HEATER_ON | NOTYFY_SENSOR_TH_HEATER_OFF)) && result.measure.d_thsensor_error == false)
        {
            if (th_sensor == 0x40) // HTU21
            {
                bool ser_crc = true;

                // cmd = 0xfe; // Soft Reset
                //  ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit(th_handle, &cmd, 1, 100));
                // vTaskDelay(20 / portTICK_PERIOD_MS); // The soft reset takes less than 15ms.

                if (ulNotifiedValue & NOTYFY_SENSOR_TH_HEATER_ON)
                {
                    cmd[0] = 0xe6; // Write user register
                    cmd[1] = 0b110;
                    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit(th_handle, cmd, 2, I2C_TIMEOUT_VALUE_MS));
                }

                if (ulNotifiedValue & NOTYFY_SENSOR_TH_HEATER_OFF)
                {
                    cmd[0] = 0xe6; // Write user register
                    cmd[1] = 0b010;
                    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit(th_handle, cmd, 2, I2C_TIMEOUT_VALUE_MS));
                };

                cmd[0] = 0xf3; // Trigger Temperature Measurement
                ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit(th_handle, cmd, 1, I2C_TIMEOUT_VALUE_MS));
                vTaskDelay(50 / portTICK_PERIOD_MS); // 14 bit 44-50 ms
                err_rc = i2c_master_receive(th_handle, buffer, 3, I2C_TIMEOUT_VALUE_MS);
                if (err_rc == ESP_OK)
                {
                    if (!htu21_crc_check((buffer[0] << 8) | buffer[1], buffer[2]))
                    {
                        ESP_LOGW("HTU21", "T %x %x %x CRC BAD", buffer[0], buffer[1], buffer[2]);
                        ser_crc = false;
                    }

                    if ((buffer[1] & 0b11) == 0) // Status (‘0’: temperature, ‘1’: humidity)
                    {
                        result.measure.temp = -46.85 + 175.72 * (int)((buffer[0] << 8) | (buffer[1] & 0b11111100)) / 65536.0;
                    }
                }

                cmd[0] = 0xf5; // Trigger Humidity Measurement
                ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit(th_handle, cmd, 1, I2C_TIMEOUT_VALUE_MS));
                vTaskDelay(16 / portTICK_PERIOD_MS); // 12 bits 14-16 ms
                err_rc = i2c_master_receive(th_handle, buffer, 3, I2C_TIMEOUT_VALUE_MS);
                if (err_rc == ESP_OK)
                {
                    if (!htu21_crc_check((buffer[0] << 8) | buffer[1], buffer[2]))
                    {
                        ESP_LOGW("HTU21", "H %x %x %x CRC BAD", buffer[0], buffer[1], buffer[2]);
                        ser_crc = false;
                    }

                    if ((buffer[1] & 0b11) == 0b10) // Status (‘0’: temperature, ‘1’: humidity)
                    {
                        result.measure.humidity = -6.0 + 125.0 * (int)((buffer[0] << 8) | (buffer[1] & 0b11111100)) / 65536.0;
                    }
                }

                if (ser_crc)
                    ESP_LOGI("HTU21", "Read: T=%.01f°C, H=%.01f%%", result.measure.temp, result.measure.humidity);
                else
                    ESP_LOGW("HTU21", "Read: T=%.01f°C, H=%.01f%% CRC BAD", result.measure.temp, result.measure.humidity);

                // cmd[0] = 0xe7; // Read user register
                // ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit(th_handle, cmd, 1, I2C_TIMEOUT_VALUE_MS));
                // err_rc = i2c_master_receive(th_handle, buffer, 1, I2C_TIMEOUT_VALUE_MS);
                // if (err_rc == ESP_OK)
                // {
                //     ESP_LOGD("HTU21 user register", "0x%02x", buffer[0]);
                // }

                // /*SERIAL NUM*/
                // ser_crc = true;
                // cmd[0] = 0xfa; // Read HTU21_READ_SERIAL_FIRST_8BYTES_COMMAND
                // cmd[1] = 0x0f; // Return SNB_3,CRC,SNB_2,CRC,SNB_1,CRC,SNB_0,CRC
                // err_rc = i2c_master_transmit_receive(th_handle, cmd, 2, buffer, 8, I2C_TIMEOUT_VALUE_MS);
                // if (err_rc == ESP_OK)
                // {
                //     for (int i = 0; i < 8; i += 2)
                //     {
                //         if (!htu21_crc_check(buffer[i], buffer[i + 1]))
                //             ser_crc = false;
                //     }

                //     cmd[0] = 0xfc; // Read HTU21_READ_SERIAL_LAST_6BYTES_COMMAND
                //     cmd[1] = 0xc9; // Return SNC_1,SNC_0,CRC,SNA_1,SNA_0,CRC
                //     err_rc = i2c_master_transmit_receive(th_handle, cmd, 2, buffer + 8, 6, I2C_TIMEOUT_VALUE_MS);
                //     if (err_rc == ESP_OK)
                //     {
                //         for (int i = 8; i < 14; i += 3)
                //         {
                //             if (!htu21_crc_check((buffer[i] << 8) | buffer[i + 1], buffer[i + 2]))
                //                 ser_crc = false;
                //         }

                //         ESP_LOG_BUFFER_HEX_LEVEL("HTU21 Serial RAW", buffer, 14, ESP_LOG_DEBUG);

                //         // HTU2X Serial Number reading HPC207_0 October 2012
                //         uint64_t serial_number = ((uint64_t)buffer[11] << 56) | ((uint64_t)buffer[12] << 48) | ((uint64_t)buffer[0] << 40) | ((uint64_t)buffer[2] << 32) | ((uint64_t)buffer[4] << 24) | ((uint64_t)buffer[6] << 16) | ((uint64_t)buffer[8] << 8) | ((uint64_t)buffer[9] << 0);

                //         if (ser_crc)
                //             ESP_LOGI("HTU21", "Serial № %llu, CRC OK", serial_number);
                //         else
                //             ESP_LOGW("HTU21", "Serial № %llu, CRC BAD", serial_number);
                //     }
                // }
            }
        };

        if (ulNotifiedValue & NOTYFY_SENSOR_MAGACC_STOP)
        {
            xTaskNotify(xTaskGetCurrentTaskHandle(), 0, eSetValueWithOverwrite);
        };

        if (ulNotifiedValue & NOTYFY_TEST)
        {
            light_measure(10);
        }

        if (result.measure.d_mag_sensor_error)
            continue;

        if (ulNotifiedValue & NOTYFY_SENSOR_MAGACC_GET_INT)
        {
            // clear INT1
            uint8_t int1 = LSM303DLHC_getAccelInterrupt1Source();
            uint8_t click = LSM303DLHC_getAccelClickSource();

            int16_t ax, ay, az;
            LSM303DLHC_getAcceleration(&ax, &ay, &az);

            if (get_menu_val_by_id("openacce"))
            {
                result.measure.open = check_range((ax >> 4) * 2, (ay >> 4) * 2, (az >> 4) * 2, get_menu_val_by_id("openaccX"), get_menu_val_by_id("openaccY"), get_menu_val_by_id("openaccZ"), get_menu_val_by_id("deviation"));
            }
            if (get_menu_val_by_id("closeacce"))
            {
                result.measure.close = check_range((ax >> 4) * 2, (ay >> 4) * 2, (az >> 4) * 2, get_menu_val_by_id("closeaccX"), get_menu_val_by_id("closeaccY"), get_menu_val_by_id("closeaccZ"), get_menu_val_by_id("deviation"));
            }

            ESP_LOGD("LSM303", "INT1 src=%02x; CLICK src=%02x; ACC=%4d;%4d;%4d", int1, click, (ax >> 4) * 2, (ay >> 4) * 2, (az >> 4) * 2);

            // xTaskNotify(xTaskGetCurrentTaskHandle(), 0, eSetValueWithOverwrite);
        }

        if ((ulNotifiedValue & NOTYFY_SENSOR_SET_MAGACC))
        {
            tm = 1000 / portTICK_PERIOD_MS;

            LSM303DLHC_initialize();
            LSM303DLHC_setAccelFullScale(4); // 4G

            // set accel data rate to 1Hz
            LSM303DLHC_setAccelOutputDataRate(1);
            LSM303DLHC_setMagOutputDataRate(1);

            LSM303DLHC_setAccelLowPowerEnabled(false);
            LSM303DLHC_setAccelHighResOutputEnabled(true);

            LSM303DLHC_setAccelINT1AOI1Enabled(false);

            LSM303DLHC_setMagGain(670);

            if (ulNotifiedValue & NOTYFY_SENSOR_MAGACC_CONT)
            {
                LSM303DLHC_setMagMode(LSM303DLHC_MD_CONTINUOUS);
            }
            else if (ulNotifiedValue & NOTYFY_SENSOR_MAGACC_SPEEDCONT) // WiFi
            {
                // заканчиваем работу NBIoT
                // xEventGroupSetBits(status_event_group, END_WORK_NBIOT);

                LSM303DLHC_setAccelOutputDataRate(10);
                LSM303DLHC_setMagOutputDataRate(15);
                LSM303DLHC_setMagMode(LSM303DLHC_MD_CONTINUOUS);
                // nbiot_power_off();
                tm = 100 / portTICK_PERIOD_MS;
            }
            else
            {
                LSM303DLHC_setMagMode(LSM303DLHC_MD_SINGLE);
            }
        };

        if ((ulNotifiedValue & (NOTYFY_SENSOR_MAGACC | NOTYFY_SENSOR_MAGACC_CONT | NOTYFY_SENSOR_MAGACC_SPEEDCONT)))
        {
            esp_err_t ret;
            int16_t ax, ay, az;
            int16_t mx, my, mz;
            ret = LSM303DLHC_getAcceleration(&ax, &ay, &az);
            if (ret != ESP_OK)
            {
                result.measure.d_mag_sensor_error = true;
            }
            else
            {
                ret = LSM303DLHC_getMag(&mx, &my, &mz);
                //  Calculation by scale
                /*
                result.measure.acc[0] = (float)(ax >> _lsm303Acc_SHIFT) * _lsm303Acc_LSB * SENSORS_GRAVITY_STANDARD;
                result.measure.acc[1] = (float)(ay >> _lsm303Acc_SHIFT) * _lsm303Acc_LSB * SENSORS_GRAVITY_STANDARD;
                result.measure.acc[2] = (float)(az >> _lsm303Acc_SHIFT) * _lsm303Acc_LSB * SENSORS_GRAVITY_STANDARD;
                result.measure.mag[0] = (float)mx / _lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
                result.measure.mag[0] = (float)mx / _lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
                result.measure.mag[0] = (float)mx / _lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
                */
                result.measure.acc[0] = (float)((ax >> 4) * 2) / 1000.0; // + - 4g
                result.measure.acc[1] = (float)((ay >> 4) * 2) / 1000.0; // + - 4g
                result.measure.acc[2] = (float)((az >> 4) * 2) / 1000.0; // + - 4g

                result.measure.mag[0] = (float)(mx) / 670.0; //+ - 2.5 Gauss
                result.measure.mag[1] = (float)(my) / 670.0; //+ - 2.5 Gauss
                result.measure.mag[2] = (float)(mz) / 600.0; //+ - 2.5 Gauss

                ESP_LOGI("LSM303", "acc=%2.2f %2.2f %2.2f; mag=%3.2f %3.2f %3.2f (%04hx %04hx %04hx)", result.measure.acc[0], result.measure.acc[1], result.measure.acc[2], result.measure.mag[0], result.measure.mag[1], result.measure.mag[2], mx, my, mz);

                if (get_menu_val_by_id("openacce"))
                {
                    result.measure.open = check_range((ax >> 4) * 2, (ay >> 4) * 2, (az >> 4) * 2, get_menu_val_by_id("openaccX"), get_menu_val_by_id("openaccY"), get_menu_val_by_id("openaccZ"), get_menu_val_by_id("deviation"));
                }
                if (get_menu_val_by_id("openmage"))
                {
                    result.measure.open = check_range(mx * 1000 / 670, my * 1000 / 670, mz * 1000 / 600, get_menu_val_by_id("openmagX"), get_menu_val_by_id("openmagY"), get_menu_val_by_id("openmagZ"), get_menu_val_by_id("deviation"));
                }
                if (get_menu_val_by_id("closeacce"))
                {
                    result.measure.close = check_range((ax >> 4) * 2, (ay >> 4) * 2, (az >> 4) * 2, get_menu_val_by_id("closeaccX"), get_menu_val_by_id("closeaccY"), get_menu_val_by_id("closeaccZ"), get_menu_val_by_id("deviation"));
                }
                if (get_menu_val_by_id("closemage"))
                {
                    result.measure.close = check_range(mx * 1000 / 670, my * 1000 / 670, mz * 1000 / 600, get_menu_val_by_id("closemagX"), get_menu_val_by_id("closemagY"), get_menu_val_by_id("closemagZ"), get_menu_val_by_id("deviation"));
                }

                xEventGroupSetBits(status_event_group, READ_MAG_SENSOR);
            }
        }

        if ((ulNotifiedValue & NOTYFY_SENSOR_SET_MAGACC_INT))
        {
            LSM303DLHC_setAccelOutputDataRate(1);
            LSM303DLHC_setMagOutputDataRate(0);
            LSM303DLHC_setAccelLowPowerEnabled(true);
            LSM303DLHC_setAccelHighResOutputEnabled(false);

            LSM303DLHC_setAccelInterrupt1RequestLatched(true);
            LSM303DLHC_setAccelInterruptActiveLowEnabled(false);

            int16_t ax, ay, az;
            LSM303DLHC_getAcceleration(&ax, &ay, &az);
            int x = (ax >> 4) * 2;
            int y = (ay >> 4) * 2;
            int z = (az >> 4) * 2;
            ESP_LOGD("LSM303", "ACC=%4d;%4d;%4d", x, y, z);

            int data[] = {abs(x), abs(y), abs(z)};
            qsort(data, 3, sizeof(int), compare_function);

            // ESP_LOGD("LSM303", "sort=%d %d %d", data[0], data[1], data[2]);

            // берем больший промежуток
            int avg = 0;
            if ((data[2] - data[1]) > (data[1] - data[0]))
                avg = (data[2] + data[1]) / 2;
            else
                avg = (data[1] + data[0]) / 2;

            // ESP_LOGD("LSM303", "avg ACC=%d", avg);

            uint8_t int_mask = BIT(LSM303DLHC_INT1_XLIE_XDOWNE_BIT) | BIT(LSM303DLHC_INT1_XHIE_XUPE_BIT) | BIT(LSM303DLHC_INT1_YLIE_YDOWNE_BIT) | BIT(LSM303DLHC_INT1_YHIE_YUPE_BIT) | BIT(LSM303DLHC_INT1_ZLIE_ZDOWNE_BIT) | BIT(LSM303DLHC_INT1_ZHIE_ZUPE_BIT);

            if (x > avg)
                int_mask &= ~BIT(LSM303DLHC_INT1_XHIE_XUPE_BIT);
            if (x < (avg * -1))
                int_mask &= ~BIT(LSM303DLHC_INT1_XLIE_XDOWNE_BIT);
            if (y > avg)
                int_mask &= ~BIT(LSM303DLHC_INT1_YHIE_YUPE_BIT);
            if (y < (avg * -1))
                int_mask &= ~BIT(LSM303DLHC_INT1_YLIE_YDOWNE_BIT);
            if (z > avg)
                int_mask &= ~BIT(LSM303DLHC_INT1_ZHIE_ZUPE_BIT);
            if (z < (avg * -1))
                int_mask &= ~BIT(LSM303DLHC_INT1_ZLIE_ZDOWNE_BIT);

            // setup INT1 (all axis)
            writeByte(LSM303DLHC_DEFAULT_ADDRESS_A, LSM303DLHC_RA_INT1_CFG_A, int_mask | BIT(LSM303DLHC_INT1_6D_BIT));
            LSM303DLHC_setAccelInterrupt1Threshold(avg * 128 / (4 * 1000)); //(set 0.5g) 1 LSB = full-scale / 128
            LSM303DLHC_setAccelInterrupt1Duration(1);

            // writeByte(LSM303DLHC_DEFAULT_ADDRESS_A, LSM303DLHC_RA_CLICK_SRC_A, 0b01101100); // Double-click enable,  negative detection, Z

            // LSM303DLHC_setAcceLClickThreshold(128 / 8); // 1 LSB = full-scale / 128
            //   LSM303DLHC_setAcceLClickTimeLimit(2);       // 1 LSB = 1/ODR
            //   LSM303DLHC_setAcceLClickTimeLatency(1);     // 1 LSB = 1/ODR
            //   LSM303DLHC_setAcceLClickTimeWindow(3);      // 1 LSB = 1/ODR

            // LSM303DLHC_setAccelINT1ClickEnabled(true);
            LSM303DLHC_setAccelINT1AOI1Enabled(true);

            // LSM303DLHC_setAccelINT2ClickEnabled(true);
            // LSM303DLHC_setAccelINT2Interrupt1Enabled(true);

            xTaskNotify(xTaskGetCurrentTaskHandle(), NOTYFY_SENSOR_MAGACC_GET_INT, eSetBits);
        };
    }
}

void dallas_task(void *arg)
{
    // install new 1-wire bus
    onewire_bus_handle_t bus;
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = PIN_ONEWARE,
    };
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10, // 1byte ROM command + 8byte ROM number + 1byte device command
    };
    ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));
    ESP_LOGI("DS18B20", "1-Wire bus installed on GPIO%d", PIN_ONEWARE);

    int ds18b20_device_num = 0;
    ds18b20_device_handle_t ds18b20s[ONEWIRE_MAX_DS18B20];
    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_onewire_device;
    esp_err_t search_result = ESP_OK;
    result.measure.d_dallas_sensor_error = true;
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Ожидаем уведомления безконечно, для повторного поиска

        if (water_temp_id != 0)
        {
            ds18b20_config_t ds_cfg = {};
            next_onewire_device.bus = bus;
            next_onewire_device.address = water_temp_id;
            if (ds18b20_new_device_from_enumeration(&next_onewire_device, &ds_cfg, &ds18b20s[0]) == ESP_OK)
            {
                ESP_LOGI("DS18B20", "DS18B20[%d], address: %016llX", 0, water_temp_id);
            };
            ds18b20_device_num = 1;
        }
        else
        {
            // create 1-wire device iterator, which is used for device search
            ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));
            ESP_LOGI("DS18B20", "Device iterator created, start searching...");
            do
            {
                search_result = onewire_device_iter_get_next(iter, &next_onewire_device);
                if (search_result == ESP_OK)
                { // found a new device, let's check if we can upgrade it to a DS18B20
                    ds18b20_config_t ds_cfg = {};
                    if (ds18b20_new_device_from_enumeration(&next_onewire_device, &ds_cfg, &ds18b20s[0]) == ESP_OK)
                    {
                        water_temp_id = next_onewire_device.address;
                        ESP_LOGI("DS18B20", "Found a DS18B20[%d], address: %016llX", 0, next_onewire_device.address);
                        ds18b20_device_num = 1;
                        // if (ds18b20_device_num >= ONEWIRE_MAX_DS18B20)
                        //{
                        // ESP_LOGI("DS18B20", "Max DS18B20 number reached, stop searching...");
                        //}
                    }
                    else
                    {
                        ESP_LOGI("DS18B20", "Found an unknown device, address: %016llX", next_onewire_device.address);
                    }
                }
            } while (search_result != ESP_ERR_NOT_FOUND);
            ESP_ERROR_CHECK(onewire_del_device_iter(iter));
            ESP_LOGI("DS18B20", "Searching done, %d DS18B20 device(s) found", ds18b20_device_num);

            // set resolution for all DS18B20s
            for (int i = 0; i < ds18b20_device_num; i++)
            {
                // set resolution
                ESP_ERROR_CHECK(ds18b20_set_resolution(ds18b20s[i], DS18B20_RESOLUTION_12B));
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        // get temperature from sensors one by one
        float temperature;
        esp_err_t ret;
        while (water_temp_id != 0)
        {
            ESP_ERROR_CHECK(ds18b20_trigger_temperature_conversion(ds18b20s[0]));
            ret = ds18b20_get_temperature(ds18b20s[0], &temperature);
            if (ret == ESP_OK)
            {
                result.measure.water_temp = temperature;
                result.measure.d_dallas_sensor_error = false;

                xEventGroupSetBits(status_event_group, END_DS18B20);

                ESP_LOGI("DS18B20", "temperature read from DS18B20: %.2fC", temperature);

                vTaskDelay(pdMS_TO_TICKS(1000));
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Ожидаем уведомления безконечно, для повторного опроса
            }
            else
            {
                ESP_LOGW("DS18B20", "Error read from DS18B20");
                water_temp_id = 0;
                break;
            };
        }
    }
};