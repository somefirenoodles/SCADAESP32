#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "sdkconfig.h"

#define UART_PORT UART_NUM_2
#define RX_BUF_SIZE 256
#define NOISE_BUF_SIZE 64

#if CONFIG_SCADA_MODBUS_PARITY_EVEN
#define SCADA_UART_PARITY UART_PARITY_EVEN
#elif CONFIG_SCADA_MODBUS_PARITY_ODD
#define SCADA_UART_PARITY UART_PARITY_ODD
#else
#define SCADA_UART_PARITY UART_PARITY_DISABLE
#endif

static uint16_t modbus_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;

    for (size_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)data[pos];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

static void print_hex(const uint8_t *data, int len)
{
    for (int i = 0; i < len; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
}

static const char *parity_name(void)
{
#if CONFIG_SCADA_MODBUS_PARITY_EVEN
    return "8E1";
#elif CONFIG_SCADA_MODBUS_PARITY_ODD
    return "8O1";
#else
    return "8N1";
#endif
}

static void push_noise(uint8_t *noise, int *noise_len, uint8_t value)
{
    if (*noise_len < NOISE_BUF_SIZE) {
        noise[*noise_len] = value;
        (*noise_len)++;
    }
}

static int read_modbus_frame(uint8_t slave_id,
                             uint8_t function_code,
                             uint8_t *response,
                             int response_max_len,
                             int timeout_ms)
{
    int elapsed_ms = 0;
    int total = 0;
    int expected_len = 0;
    uint8_t noise[NOISE_BUF_SIZE] = {0};
    int noise_len = 0;

    while (elapsed_ms < timeout_ms) {
        uint8_t byte = 0;
        int len = uart_read_bytes(UART_PORT, &byte, 1, pdMS_TO_TICKS(20));

        if (len <= 0) {
            elapsed_ms += 20;
            continue;
        }

        if (total == 0) {
            if (byte != slave_id) {
                push_noise(noise, &noise_len, byte);
                continue;
            }

            response[total++] = byte;
            continue;
        }

        if (total == 1) {
            if (byte != function_code && byte != (uint8_t)(function_code | 0x80)) {
                push_noise(noise, &noise_len, response[0]);
                push_noise(noise, &noise_len, byte);
                total = 0;
                expected_len = 0;
                continue;
            }

            response[total++] = byte;

            if (byte == (uint8_t)(function_code | 0x80)) {
                expected_len = 5; // slave + exception function + exception code + crc2
            }
            continue;
        }

        if (total == 2 && response[1] == function_code) {
            int byte_count = byte;
            if (byte_count <= 0 || byte_count > (response_max_len - 5)) {
                push_noise(noise, &noise_len, response[0]);
                push_noise(noise, &noise_len, response[1]);
                push_noise(noise, &noise_len, byte);
                total = 0;
                expected_len = 0;
                continue;
            }

            response[total++] = byte;
            expected_len = 3 + byte_count + 2;
            continue;
        }

        if (total < response_max_len) {
            response[total++] = byte;
        } else {
            printf("Error: buffer RX lleno.\n");
            return -8;
        }

        if (expected_len > 0 && total >= expected_len) {
            return total;
        }
    }

    if (noise_len > 0) {
        printf("RX ruido descartado: ");
        print_hex(noise, noise_len);
        printf("Sin inicio de trama valido: esperado slave_id=0x%02X.\n", slave_id);
    } else {
        printf("RX: timeout. No hay respuesta valida.\n");
    }

    return -1;
}

static int modbus_read_registers(uint8_t slave_id,
                                 uint8_t function_code,
                                 uint16_t start_register,
                                 uint16_t register_count,
                                 uint8_t *response,
                                 int response_max_len)
{
    if (function_code != 0x03 && function_code != 0x04) {
        printf("Configuracion invalida: function code 0x%02X. Use 0x03 o 0x04.\n", function_code);
        return -10;
    }

    if (register_count == 0 || register_count > 64) {
        printf("Configuracion invalida: register_count=%u.\n", register_count);
        return -11;
    }

    uint8_t request[8] = {0};

    request[0] = slave_id;
    request[1] = function_code;
    request[2] = (start_register >> 8) & 0xFF;
    request[3] = start_register & 0xFF;
    request[4] = (register_count >> 8) & 0xFF;
    request[5] = register_count & 0xFF;

    uint16_t crc = modbus_crc16(request, 6);
    request[6] = crc & 0xFF;         // CRC low byte first in Modbus RTU
    request[7] = (crc >> 8) & 0xFF;

    uart_flush_input(UART_PORT);

    printf("\nTX Modbus RTU: ");
    print_hex(request, sizeof(request));

    uart_write_bytes(UART_PORT, (const char *)request, sizeof(request));
    uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(200));

    int total = read_modbus_frame(slave_id, function_code, response, response_max_len, 1000);
    if (total < 0) {
        return total;
    }

    printf("RX bruto: ");
    print_hex(response, total);

    if (total < 5) {
        printf("Error: trama demasiado corta.\n");
        return -2;
    }

    uint16_t crc_calc = modbus_crc16(response, total - 2);
    uint16_t crc_recv = response[total - 2] | ((uint16_t)response[total - 1] << 8);

    if (crc_calc != crc_recv) {
        printf("Error: CRC invalido. calc=0x%04X recv=0x%04X\n", crc_calc, crc_recv);
        printf("Diagnostico: trama detectada pero corrupta. Revise A/B, baudrate, paridad, GND comun o ruido del bus.\n");
        return -3;
    }

    if (response[1] == (uint8_t)(function_code | 0x80)) {
        printf("Excepcion Modbus. Codigo=0x%02X\n", response[2]);
        return -5;
    }

    int byte_count = response[2];
    int expected_byte_count = register_count * 2;

    if (byte_count != expected_byte_count) {
        printf("Error: byte_count inesperado. esperado=%d recibido=%d\n", expected_byte_count, byte_count);
        return -7;
    }

    printf("Trama Modbus valida.\n");

    for (int i = 0; i < register_count; i++) {
        uint16_t value = ((uint16_t)response[3 + i * 2] << 8) | response[4 + i * 2];
        printf("Registro 0x%04X = %u / 0x%04X\n",
               (unsigned int)(start_register + i),
               value,
               value);
    }

    return 0;
}

void app_main(void)
{
    const uart_config_t uart_config = {
        .baud_rate = CONFIG_SCADA_MODBUS_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = SCADA_UART_PARITY,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, RX_BUF_SIZE * 2, RX_BUF_SIZE * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT,
                                 CONFIG_SCADA_UART_TX_PIN,
                                 CONFIG_SCADA_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));

    printf("\n=== SCADAESP32: MODBUS RTU MASTER ===\n");
    printf("UART2 TX=GPIO%d RX=GPIO%d\n", CONFIG_SCADA_UART_TX_PIN, CONFIG_SCADA_UART_RX_PIN);
    printf("Serial=%d %s\n", CONFIG_SCADA_MODBUS_BAUDRATE, parity_name());
    printf("Slave ID=%d Function=0x%02X Start=0x%04X Count=%d Poll=%d ms\n",
           CONFIG_SCADA_MODBUS_SLAVE_ID,
           CONFIG_SCADA_MODBUS_FUNCTION_CODE,
           CONFIG_SCADA_MODBUS_START_REGISTER,
           CONFIG_SCADA_MODBUS_REGISTER_COUNT,
           CONFIG_SCADA_MODBUS_POLL_INTERVAL_MS);
    printf("Cableado: ESP32 GPIO17->RXD modulo, GPIO16<-TXD modulo, GND comun, VCC segun modulo.\n");
    printf("A/B deben ir a un esclavo RS485 real. Con A/B flotando puede aparecer ruido descartado o CRC invalido.\n");

    uint8_t response[RX_BUF_SIZE];

    while (1) {
        int result = modbus_read_registers((uint8_t)CONFIG_SCADA_MODBUS_SLAVE_ID,
                                           (uint8_t)CONFIG_SCADA_MODBUS_FUNCTION_CODE,
                                           (uint16_t)CONFIG_SCADA_MODBUS_START_REGISTER,
                                           (uint16_t)CONFIG_SCADA_MODBUS_REGISTER_COUNT,
                                           response,
                                           sizeof(response));

        if (result == 0) {
            printf("Lectura OK.\n");
        } else {
            printf("Lectura fallida. Codigo=%d\n", result);
        }

        vTaskDelay(pdMS_TO_TICKS(CONFIG_SCADA_MODBUS_POLL_INTERVAL_MS));
    }
}
