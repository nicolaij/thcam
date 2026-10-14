# AGENTS.md

Автономный датчик на ESP32-C3 (esp32-c3-devkitm-1) с глубоким сном ("THCam"): NBIoT (SIM7028) + WiFi/HTTP/OTA, датчики по I2C/LSM303 + HTU21 + DS18B20, пробуждение по свету/воде/акселерометру, веб-интерфейс отдаётся из SPIFFS. Сборка — PlatformIO с фреймворком ESP-IDF. Комментарии и строки логов — на русском.

## Сборка / прошивка

PlatformIO установлен на стороне Windows, а не в WSL. `pio` нет в PATH — вызывайте Windows-бинарник напрямую (инкрементальная сборка ~10 с):

```bash
PIO=/mnt/c/Users/nicolai/.platformio/penv/Scripts/pio.exe
$PIO run -e debug-v11              # сборка
$PIO run -e debug-v11 -t upload    # прошивка
$PIO run -e debug-v11 -t uploadfs  # записать data/ -> SPIFFS
$PIO run -e debug-v11 -t monitor   # монитор порта (115200)
```

Тестов нет: `test/` — только заглушка PlatformIO, к тому же в .gitignore. Не рассчитывайте, что `pio test` что-то сделает.

## Окружения и ловушка с версией платы (HW)

- `debug-v11` — основное окружение (совпадает с `.vscode/launch.json` и закоммиченным `sdkconfig.debug-v11`).
- `env:debug-v11` передаёт `-DHW=11`. В `include/main.h` для `HW==11` иначе назначаются пины I2C/`INT_ACC`/UART. `debug-c3-mini` и `release-c3-mini` **не** задают `HW`, поэтому используют карту пинов по умолчанию. Сборка/прошивка не того окружения молча задействует не те пины.
- PlatformIO генерирует `sdkconfig.<env>` из `sdkconfig.defaults`. `sdkconfig.debug-v11` закоммичен и имеет приоритет, поэтому правки `sdkconfig.defaults` могут не примениться к уже существующему окружению; чтобы перегенерировать, удалите `sdkconfig.debug-v11`.
- Прошивка занимает ~92% от app-раздела 0x130000. Добавление кода быстро приводит к переполнению.

## Карта исходников

- `include/main.h` — всё центральное: карта пинов (HW11 vs по умолчанию), биты event-group (`END_WORK_*`, `NOW_CHARGE`, ...), биты task-notify, `measure_data_t`/`result_data_t`, макросы вывода `OUT_JSON` / `OUT_MEASURE_*`, `HISTORY_SIZE`.
- `src/main.c` — `app_main`, вход/выход из глубокого сна. Состояние `RTC_DATA_ATTR` (`bootCount`, кольцевой `history[]`, `history_pos`) переживает сон. Создаёт все задачи, затем вызывает `dio_sleep()` + `esp_deep_sleep_start()`.
- `src/sensors.c` — `i2c_task` (HTU21 @0x40, LSM303 accel/mag, расширитель PI4IOE5V6408 @0x43, WPC NU1680 @0x60) и `dallas_task` (DS18B20, one-wire, температура воды).
- `src/dio.c` — `dio_task` (light/water через непрерывный DMA-ADC, ADC батареи, пробуждение по GPIO), `dio_check`/`dio_sleep`/`get_charge`, `btn_task`, `light_measure`, `water_cont_measure`.
- `src/nbiot.c` — `modem_task`: питание SIM7028, отправка по TCP/UDP/ESP-NOW.
- `src/NBIoTlib.c` — низкоуровневые AT/UART-хелперы, hex-кодирование/декодирование, прошивка модема, отправка по ESP-NOW.
- `src/network.c` — WiFi soft-AP (`THCam`/`123123123`, канал 11) + HTTP-сервер + WebSocket, OTA. `wifi_task` запускается по уведомлению.
- `src/terminal.c` — настройки в NVS (`menu[]`, `get/set_menu_val_by_id`) и консольное меню `console_task` (пункты 51-66 плюс нумерованные настройки).
- `data/` — веб-интерфейс, прошиваемый в SPIFFS (`main.html`, `update.html`, `D3.html`, `d3.min.js.gz`).
- `lib/` — приватные библиотеки (`LSM303DLHC` используется, `bme280` не используется/вендорный).
- `src/idf_component.yml` — managed-компоненты (`espressif/ds18b20`, `espressif/cjson`); версии зафиксированы в `dependencies.lock`; `managed_components/` в .gitignore.

## Соглашения и подводные камни

- Все `.c` в `src/` автоматически подхватываются `src/CMakeLists.txt` (`FILE(GLOB_RECURSE)`); регистрировать новые исходники не нужно.
- В `lib/LSM303DLHC/CMakeLists.txt` указан `LSM303DLHC.cpp`, но существует только `LSM303DLHC.c`. PlatformIO компилирует сам `.c`, так что устаревший CMake безвреден — не "чините" его вслепую.
- OTA `POST /update` (см. `update_post_handler`) сам определяет прошивку или SPIFFS: прошивка — по магическому первому байту `0xE9`, иначе образ SPIFFS принимается только при `content_len == 0x50000`. Не меняйте размер SPIFFS-раздела, не обновив эту константу.
- HTTP-эндпоинты: `/`, `/menu`, `/data.csv`, `/olddata.csv`, `/d.csv`, `/history`, `/d3`, `/d3.min.js`, `/update`, `/ws`.
- `monitor_filters = ... log2file` сохраняет сессии монитора в `logs/` (в .gitignore).
- Переводы строк: в коммитах LF, а в рабочем дереве CRLF (нет `.gitattributes`), поэтому `git status` показывает почти все файлы как изменённые. Реальные изменения смотрите через `git diff --ignore-cr-at-eol`; не делайте массовую перекодировку переводов строк.
