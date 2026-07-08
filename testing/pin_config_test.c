/*
 * SCADAESP32 - Pin configuration and RS485 link test
 *
 * Purpose:
 * Validate that the ESP32 UART pin configuration and the TTL-RS485 modules are wired correctly
 * before testing against the FoxESS charger or another external Modbus RTU slave.
 *
 * Test topology:
 *
 *   ESP32 UART2  <->  RS485 module A  <==== A/B bus ====>  RS485 module B  <->  ESP32 UART1
 *
 * UART2 side, same pins used by the main firmware:
 *   GPIO17 TX2 -> RXD module A
 *   GPIO16 RX2 <- TXD module A
 *
 * UART1 side, test pins for the second RS485 module:
 *   GPIO26 TX  -> RXD module B
 *   GPIO27 RX  <- TXD module B
 *
 * Both modules:
 *   VCC -> 3V3 or 5V according to module specification
 *   GND -> ESP32 GND, common ground
 *
 * RS485 bus:
 *   A module A -> A module B
 *   B module A -> B module B
 *   If the test fails, swap only A/B on one module.
 *
 * How to use:
 *   1. Backup main/my_esp32_project.c.
 *   2. Temporarily copy this file over main/my_esp32_project.c.
 *   3. Run: idf.py build flash monitor
 *   4. Restore the original main/my_esp32_project.c after the test.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_err.h"

#define MASTER_UART       UART_NUM_2
#define MASTER_TX_PIN     17
#define MASTER_RX_PIN     16

#define TEST_UART         UART_NUM_1
#define TEST_TX_PIN       26
#define TEST_RX_PIN       27

#define TEST_BAUDRATE     9600
#define RX_BUF_SIZE       256
#define READ_TIMEOUT_MS   1000

static void print_hex(const char *label, const uint8_t *data, int len)
{
    printf("%s", label);
    for (int i = 0; i < len; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
}

static void configure_uart(uart_port_t uart_num, int tx_pin, int rx_pin, const char *name)
{
    const uart_config_t uart_config = {
        .baud_rate = TEST_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(uart_num, RX_BUF_SIZE * 2, RX_BUF_SIZE * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_flush_input(uart_num));

    printf("%s configurado: TX=GPIO%d RX=GPIO%d, %d 8N1\n", name, tx_pin, rx_pin, TEST_BAUDRATE);
}

static int send_and_receive(uart_port_t tx_uart,
                            uart_port_t rx_uart,
                            const uint8_t *tx_data,
                            int tx_len,
                            uint8_t *rx_data,
                            int rx_max_len)
{
    uart_flush_input(rx_uart);
    uart_write_bytes(tx_uart, (const char *)tx_data, tx_len);
    uart_wait_tx_done(tx_uart, pdMS_TO_TICKS(200));

    int rx_len = uart_read_bytes(rx_uart, rx_data, rx_max_len, pdMS_TO_TICKS(READ_TIMEOUT_MS));
    return rx_len;
}

static void check_result(const char *direction,
                         const uint8_t *expected,
                         int expected_len,
                         const uint8_t *received,
                         int received_len)
{
    printf("\n[%s]\n", direction);
    print_hex("TX esperado: ", expected, expected_len);

    if (received_len <= 0) {
        printf("Resultado: FAIL - timeout. No se recibio ningun byte.\n");
        printf("Revise VCC, GND comun, TX/RX cruzados y A/B entre modulos.\n");
        return;
    }

    print_hex("RX recibido: ", received, received_len);

    if (received_len == expected_len && memcmp(expected, received, expected_len) == 0) {
        printf("Resultado: PASS - bytes recibidos correctamente.\n");
    } else {
        printf("Resultado: FAIL - bytes recibidos no coinciden.\n");
        printf("Revise baudrate, A/B invertidos, ruido o cruce TX/RX.\n");
    }
}

void app_main(void)
{
    printf("\n=== SCADAESP32: PIN CONFIG + RS485 LOOP TEST ===\n");
    printf("Objetivo: validar GPIO17/GPIO16 y GPIO26/GPIO27 usando dos modulos TTL-RS485.\n");
    printf("Cableado TTL modulo A: GPIO17->RXD, GPIO16<-TXD.\n");
    printf("Cableado TTL modulo B: GPIO26->RXD, GPIO27<-TXD.\n");
    printf("Cableado RS485: A<->A, B<->B. Si falla, invertir solo A/B.\n\n");

    configure_uart(MASTER_UART, MASTER_TX_PIN, MASTER_RX_PIN, "UART2 maestro");
    configure_uart(TEST_UART, TEST_TX_PIN, TEST_RX_PIN, "UART1 prueba");

    const uint8_t master_to_test[] = {0x55, 0xAA, 0x11, 0x22, 0x33, 0x44};
    const uint8_t test_to_master[] = {0xA5, 0x5A, 0x99, 0x88, 0x77, 0x66};
    uint8_t rx[RX_BUF_SIZE];

    while (1) {
        memset(rx, 0, sizeof(rx));
        int rx_len = send_and_receive(MASTER_UART,
                                      TEST_UART,
                                      master_to_test,
                                      sizeof(master_to_test),
                                      rx,
                                      sizeof(rx));
        check_result("UART2 GPIO17/GPIO16 -> UART1 GPIO26/GPIO27",
                     master_to_test,
                     sizeof(master_to_test),
                     rx,
                     rx_len);

        vTaskDelay(pdMS_TO_TICKS(500));

        memset(rx, 0, sizeof(rx));
        rx_len = send_and_receive(TEST_UART,
                                  MASTER_UART,
                                  test_to_master,
                                  sizeof(test_to_master),
                                  rx,
                                  sizeof(rx));
        check_result("UART1 GPIO26/GPIO27 -> UART2 GPIO17/GPIO16",
                     test_to_master,
                     sizeof(test_to_master),
                     rx,
                     rx_len);

        printf("\n--- Nueva prueba en 2 segundos ---\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
