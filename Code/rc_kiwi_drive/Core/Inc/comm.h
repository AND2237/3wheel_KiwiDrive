/*
 * comm.h
 *
 *  Created on: Aug 28, 2026
 *      Author: Alireza
 */

#ifndef COMM_H
#define COMM_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define COMM_SYNC_BYTE       0xAA

#define COMM_MAX_PAYLOAD     64   /* was 56 -- the 3-wheel telemetry
                                      packet needs 61 (see
                                      SendTelemetryLog in main.c);
                                      rounded up for headroom. */

typedef enum {
    CMD_SET_SPEED         = 0x01,
    CMD_REQUEST_TELEMETRY = 0x02,
    CMD_SET_PID = 0x03,
    CMD_STATUS = 0x04,
    CMD_HEARTBEAT = 0x05,
    CMD_SET_PWM_PSC = 0x06,
    CMD_RESET_ODOM = 0x07,
    CMD_GET_ODOM = 0x08,
    CMD_ODOMETRY_DATA = 0x09,

    CMD_MARK_VERTEX        = 0x0A,
    CMD_VERTEX_DATA        = 0x0B,
    CMD_CLEAR_VERTICES     = 0x0C,
    CMD_SET_MOTION_MODE    = 0x0D,
    CMD_TELEMETRY_LOG      = 0x0E,
    CMD_GET_ALL_VERTICES   = 0x0F,
    CMD_CLOSE_SURVEY       = 0x10,

    /* ---- Phase 3: waypoint patrol ---- */
    CMD_NAV_CLEAR          = 0x11,
    CMD_NAV_ADD_WP         = 0x12,
    CMD_NAV_START          = 0x13,
    CMD_NAV_STOP           = 0x14,
    CMD_NAV_STATUS         = 0x15,
	CMD_NAV_DEBUG          = 0x16,
    /* ---- Phase 4: field calibration ---- */
    CMD_CAL_SPIN           = 0x19,
    CMD_CAL_ROLL           = 0x1A,
    CMD_CAL_STATUS         = 0x1B,
    CMD_CAL_APPLY_SPIN     = 0x1C,
    CMD_CAL_APPLY_ROLL     = 0x1D,
	CMD_SET_HEADING_HOLD   = 0x1E,
	CMD_CAL_STOP           = 0x1F,

    /* ---- Waypoint capture (live, MCU-side, atomic -- mirrors
     * CMD_MARK_VERTEX/Survey_MarkVertex rather than trusting the
     * browser's last-polled odometry snapshot) ---- */
    CMD_NAV_MARK_WP        = 0x20,
    CMD_NAV_WP_DATA        = 0x21,
} CommCmd_t;

typedef void (*CommCommandCallback_t)(CommCmd_t cmd, const uint8_t *payload, uint8_t len);

void Comm_Init(UART_HandleTypeDef *huart);

void Comm_Process(void);

bool Comm_SendPacket(CommCmd_t cmd, const uint8_t *data, uint8_t len);

void Comm_RegisterCommandCallback(CommCommandCallback_t callback);

void Comm_UART_IRQHandler(UART_HandleTypeDef *huart);

bool Comm_IsConnected(void);

#endif
