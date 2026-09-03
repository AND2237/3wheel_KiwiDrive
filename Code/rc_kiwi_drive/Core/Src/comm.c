#include "comm.h"
#include <string.h>

uint8_t crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0x00;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 8; i; --i) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}

#define RX_BUFFER_SIZE 64
static uint8_t rx_buf[RX_BUFFER_SIZE];
static volatile uint8_t rx_head = 0;
static volatile uint8_t rx_tail = 0;

static UART_HandleTypeDef *comm_uart = NULL;
static CommCommandCallback_t cmd_callback = NULL;

typedef enum {
    STATE_SYNC = 0,
    STATE_LENGTH,
    STATE_CMD,
    STATE_PAYLOAD,
    STATE_CRC
} RxState_t;

static RxState_t rx_state = STATE_SYNC;
static uint8_t rx_payload_len = 0;
static uint8_t rx_cmd = 0;
static uint8_t rx_payload[COMM_MAX_PAYLOAD];
static uint8_t rx_payload_idx = 0;

static uint32_t last_valid_packet_tick = 0;

void Comm_Init(UART_HandleTypeDef *huart) {
    comm_uart = huart;
    __HAL_UART_ENABLE_IT(comm_uart, UART_IT_RXNE);
    rx_state = STATE_SYNC;
}

void Comm_UART_IRQHandler(UART_HandleTypeDef *huart) {
    if (huart != comm_uart || comm_uart == NULL) return;

    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) ||
        __HAL_UART_GET_FLAG(huart, UART_FLAG_NE)  ||
        __HAL_UART_GET_FLAG(huart, UART_FLAG_FE))
    {
        volatile uint32_t dummy = huart->Instance->DR;
        (void)dummy;
        return;
    }

    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE)) {
        uint8_t byte = (uint8_t)(huart->Instance->DR & 0xFF);
        uint8_t next_head = (rx_head + 1) % RX_BUFFER_SIZE;
        if (next_head != rx_tail) {
            rx_buf[rx_head] = byte;
            rx_head = next_head;
        }
    }
}

static bool buf_read(uint8_t *byte) {
    if (rx_head == rx_tail) return false;
    *byte = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;
    return true;
}

void Comm_Process(void) {
    uint8_t byte;
    while (buf_read(&byte)) {
        switch (rx_state) {
            case STATE_SYNC:
                if (byte == COMM_SYNC_BYTE) {
                    rx_state = STATE_LENGTH;
                }
                break;

            case STATE_LENGTH:
                if (byte >= 1 && byte <= (COMM_MAX_PAYLOAD + 1)) {
                    rx_payload_len = byte - 1;
                    rx_state = STATE_CMD;
                } else {
                    rx_state = STATE_SYNC;
                }
                break;

            case STATE_CMD:
                rx_cmd = byte;
                rx_payload_idx = 0;
                if (rx_payload_len > 0) {
                    rx_state = STATE_PAYLOAD;
                } else {
                    rx_state = STATE_CRC;
                }
                break;

            case STATE_PAYLOAD:
                rx_payload[rx_payload_idx++] = byte;
                if (rx_payload_idx >= rx_payload_len) {
                    rx_state = STATE_CRC;
                }
                break;

            case STATE_CRC: {
                uint8_t crc_buf[1 + 1 + COMM_MAX_PAYLOAD];
                crc_buf[0] = rx_payload_len + 1;
                crc_buf[1] = rx_cmd;
                if (rx_payload_len > 0) {
                    memcpy(&crc_buf[2], rx_payload, rx_payload_len);
                }
                uint8_t calc_crc = crc8(crc_buf, 2 + rx_payload_len);

                if (calc_crc == byte) {
                    last_valid_packet_tick = HAL_GetTick();

                    if (cmd_callback != NULL) {
                        cmd_callback((CommCmd_t)rx_cmd, rx_payload, rx_payload_len);
                    }

                }
                rx_state = STATE_SYNC;
                break;
            }
            default:
                rx_state = STATE_SYNC;
                break;
        }
    }

}

bool Comm_IsConnected(void) {
    return (HAL_GetTick() - last_valid_packet_tick) < 200;
}

bool Comm_SendPacket(CommCmd_t cmd, const uint8_t *data, uint8_t len) {
    if (comm_uart == NULL || len > COMM_MAX_PAYLOAD) return false;

    uint8_t buffer[1 + 1 + 1 + COMM_MAX_PAYLOAD + 1];
    uint8_t idx = 0;
    buffer[idx++] = COMM_SYNC_BYTE;
    buffer[idx++] = 1 + len;
    buffer[idx++] = (uint8_t)cmd;
    if (len > 0 && data != NULL) {
        memcpy(&buffer[idx], data, len);
        idx += len;
    }
    uint8_t crc = crc8(&buffer[1], idx - 1);
    buffer[idx++] = crc;

    if (HAL_UART_Transmit(comm_uart, buffer, idx, 10) != HAL_OK) {
        return false;
    }
    return true;
}

void Comm_RegisterCommandCallback(CommCommandCallback_t callback) {
    cmd_callback = callback;
}
