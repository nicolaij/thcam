#ifndef NBIOTLIB_H_
#define NBIOTLIB_H_

#include "driver/uart.h"

extern int creg[10];

#define RX_BUF_SIZE 1024 * 2
#define TX_BUF_SIZE 512

esp_err_t uart_init(uart_port_t uart_num, int rx_pin, int tx_pin);

unsigned int fromActiveTime(uint8_t val);
unsigned int fromPeriodicTAU(uint8_t val);
void nbiot_power_pin(const TickType_t xTicksToDelay, int pin);
esp_err_t print_atcmd(const char *cmd, char *buffer);
esp_err_t wait_string(char *buffer, const char *swait, TickType_t ticks_to_wait);
esp_err_t at_reply_wait(const char *cmd, const char *wait, char *buffer, TickType_t ticks_to_wait);
esp_err_t at_reply_wait_OK(const char *cmd, char *buffer, TickType_t ticks_to_wait);
esp_err_t at_result_get(const char *wait, const char *buffer, int *resultdata, int resultcount);
esp_err_t at_reply_get(const char *cmd, const char *wait, char *buffer, int *resultdata, int resultcount, TickType_t ticks_to_wait);
esp_err_t at_csosend(const int socket, const char *data, const int len_data, char *buffer, const char *wait);
esp_err_t apply_command(const char *cmd, size_t len);
int decode_fromHEX(const char *data, int len, char *result_data);
int encode_toHEX(const char *data, char *result_data);


/*  Если найден "+CEREG: 2" - нет сети, то ждем "+CEREG: 1" время timewait.
*   Если найден "+CSONMI:" - то обрабатываем через apply_command
*/
esp_err_t check_for_wait_cereg(char *message_data, char *result_data, int chip, TickType_t timewait);
esp_err_t check_received_message(char *message_data, char *result_data, int chip);

esp_err_t download_firmware(char *rx_buffer, char *tx_buffer);

esp_err_t send_by_espnow(uint8_t *mac_addr, char *send_data);
esp_err_t init_espnow(uint8_t *peer_addr);

esp_err_t check_cereg(const char *rx_buffer, const int chip, int *stat, int *tac, int *ci, int *AcT, int *ActiveTime, int *PeriodicTAU);

size_t encrypt_text(char *source, char *desc, uint8_t *key, char *aad);

#endif