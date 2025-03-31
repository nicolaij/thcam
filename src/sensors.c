#include "main.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "onewire_bus.h"
#include "ds18b20.h"

RTC_DATA_ATTR uint64_t water_temp_id;

RTC_DATA_ATTR uint8_t th_sensor;

#include "driver/i2c_master.h"

i2c_master_dev_handle_t th_handle;

i2c_master_dev_handle_t lsm303A_handle;
i2c_master_dev_handle_t lsm303M_handle;

#include "driver/temperature_sensor.h"

#include "bme280.h"
s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void BME280_delay_msek(u32 msek);
#define TAG_BME280 "BME280"

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

    if (th_sensor != BME280_I2C_ADDRESS1)
    {
        th_sensor = 0x40; // HTU21
    }

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

    int try = 4;
    result.measure.d_thsensor_error = true;
    do
    {
        try--;
        ESP_LOGD("THCAM", "try %d", dev_th_cfg.device_address);
        err_rc = i2c_master_probe(i2cbus_handle, dev_th_cfg.device_address, 10);
        if (err_rc != ESP_OK)
        {
            if (try == 2)
            {
                if (th_sensor == dev_th_cfg.device_address)
                {
                    if (dev_th_cfg.device_address == BME280_I2C_ADDRESS1)
                    {
                        dev_th_cfg.device_address = 0x40;
                    }
                    else if (dev_th_cfg.device_address == 0x40)
                    {
                        dev_th_cfg.device_address = BME280_I2C_ADDRESS1;
                    }
                }
            }
            vTaskDelay(1);
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
    result.measure.d_mag_sensor_error = true;
    do
    {
        try--;
        ESP_LOGD("LSM303", "try %d", dev_m_cfg.device_address);
        err_rc = i2c_master_probe(i2cbus_handle, dev_m_cfg.device_address, 10);
        if (err_rc != ESP_OK)
        {
            vTaskDelay(1);
        }
        else
        {
            xTaskNotify(xTaskGetCurrentTaskHandle(), NOTYFY_SENSOR_MAGACC, eSetBits);
            result.measure.d_mag_sensor_error = false;
        };
    } while (err_rc != ESP_OK && try > 0);

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2cbus_handle, &dev_m_cfg, &lsm303M_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2cbus_handle, &dev_a_cfg, &lsm303A_handle));

    // scan
    /*    for (int i = 0; i < 127; i++)
        {
            err_rc = i2c_master_probe(i2cbus_handle, i, 50);
            if (err_rc == ESP_OK)
            {
                ESP_LOGI("i2c", "found addr %d", i);
            }
            vTaskDelay(1);
        }
    */

    TickType_t tm = 1000 / portTICK_PERIOD_MS;

    while (1)
    {
        uint32_t ulNotifiedValue;

        /* Ожидание оповещения. */
        xTaskNotifyWait(pdFALSE,                                                                                 /* Не очищать биты на входе. */
                        ULONG_MAX & ~(NOTYFY_SENSOR_MAGACC_CONT | NOTYFY_SENSOR_MAGACC_SPEEDCONT | NOTYFY_TEST), /* Очистка всех бит на выходе. кроме BIT_NOTYFY_SENSOR_MAGACC_CONT*/
                        &ulNotifiedValue,                                                                        /* Сохраняет значение оповещения. */
                        tm);

        if ((ulNotifiedValue & NOTYFY_SENSOR_TH) && result.measure.d_thsensor_error == false)
        {
            if (th_sensor == 0x40) // HTU21
            {
                uint8_t cmd = 0xfe; // Soft Reset
                uint8_t buffer[4];
                ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit(th_handle, &cmd, 1, 10));

                vTaskDelay(20 / portTICK_PERIOD_MS); // The soft reset takes less than 15ms.

                cmd = 0xe3; // Trigger Temperature Measurement
                ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit(th_handle, &cmd, 1, 10));
                vTaskDelay(50 / portTICK_PERIOD_MS); // 14 bit 44-50 ms
                err_rc = i2c_master_receive(th_handle, buffer, 3, 10);

                if (err_rc == ESP_OK)
                {
                    if ((buffer[1] & 0b10) == 0) // Status (‘0’: temperature, ‘1’: humidity)
                    {
                        result.measure.temp = -46.85 + 175.72 * (int)((buffer[0] << 8) | (buffer[1] & 0b11111100)) / 65536.0;
                    }
                }

                cmd = 0xe5; // Trigger Humidity Measurement
                ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit(th_handle, &cmd, 1, 10));
                vTaskDelay(20 / portTICK_PERIOD_MS); // 12 bits 14-16 ms
                err_rc = i2c_master_receive(th_handle, buffer, 3, 10);
                if (err_rc == ESP_OK)
                {
                    if ((buffer[1] & 0b10) != 0) // Status (‘0’: temperature, ‘1’: humidity)
                    {
                        result.measure.humidity = -6.0 + 125.0 * (int)((buffer[0] << 8) | (buffer[1] & 0b11111100)) / 65536.0;
                    }
                }

                ESP_LOGI("HTU21", "Read from I2C: T=%.01f°C, H=%.01f%%", result.measure.temp, result.measure.humidity);
            }

            if (th_sensor == BME280_I2C_ADDRESS1) // BME280
            {
                dev_th_cfg.device_address = BME280_I2C_ADDRESS1;

                struct bme280_t bme280 = {
                    .bus_write = BME280_I2C_bus_write,
                    .bus_read = BME280_I2C_bus_read,
                    .dev_addr = BME280_I2C_ADDRESS1,
                    .delay_msec = BME280_delay_msek};

                s32 com_rslt;
                s32 v_uncomp_pressure_s32;
                s32 v_uncomp_temperature_s32;
                s32 v_uncomp_humidity_s32;

                com_rslt = bme280_init(&bme280);

                com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_1X);
                com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_1X);
                com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);

                com_rslt += bme280_set_filter(BME280_FILTER_COEFF_OFF);
                if (com_rslt == SUCCESS)
                {
                    com_rslt = bme280_get_forced_uncomp_pressure_temperature_humidity(
                        &v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

                    if (com_rslt == SUCCESS)
                    {
                        result.measure.temp = bme280_compensate_temperature_double(v_uncomp_temperature_s32);
                        // result.measure.pressure = bme280_compensate_pressure_double(v_uncomp_pressure_s32) / 100; // Pa -> hPa
                        result.measure.humidity = bme280_compensate_humidity_double(v_uncomp_humidity_s32);
                        ESP_LOGI(TAG_BME280, "%.2f degC / %.3f hPa / %.3f %%",
                                 result.measure.temp,
                                 bme280_compensate_pressure_double(v_uncomp_pressure_s32) / 100,
                                 result.measure.humidity);

                        result.measure.d_thsensor_error = false;
                    }
                    else
                    {
                        result.measure.d_thsensor_error = true;
                        ESP_LOGE(TAG_BME280, "measure error. code: %d", com_rslt);
                    }
                }
                else
                {
                    result.measure.d_thsensor_error = true;
                    ESP_LOGE(TAG_BME280, "init or setting error. code: %d", com_rslt);
                }
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
                // заканчиваем работу NBIoT
                xEventGroupSetBits(status_event_group, END_WORK_NBIOT);

                LSM303DLHC_setMagMode(LSM303DLHC_MD_CONTINUOUS);
                // nbiot_power_off();
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

            // xTaskNotify(xTaskGetCurrentTaskHandle(), NOTYFY_SENSOR_MAGACC, eSetValueWithOverwrite);
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

                ESP_LOGI("LSM303", "acc=%2.1f %2.1f %2.1f; mag=%3.1f %3.1f %3.1f", result.measure.acc[0], result.measure.acc[1], result.measure.acc[2], result.measure.mag[0], result.measure.mag[1], result.measure.mag[2]);

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
            if (ds18b20_new_device(&next_onewire_device, &ds_cfg, &ds18b20s[0]) == ESP_OK)
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
                    if (ds18b20_new_device(&next_onewire_device, &ds_cfg, &ds18b20s[0]) == ESP_OK)
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
                ESP_LOGI("DS18B20", "temperature read from DS18B20: %.2fC", temperature);
                result.measure.water_temp = temperature;
                result.measure.d_dallas_sensor_error = false;

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