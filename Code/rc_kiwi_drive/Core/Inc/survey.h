/*
 * survey.h
 *
 *  Created on: Aug 28, 2026
 *      Author: Alireza
 */

#ifndef SURVEY_H
#define SURVEY_H

#include <stdint.h>
#include <stdbool.h>

#define SURVEY_MAX_VERTICES   64u

/* Section 7.2 -- exactly 36 bytes now (was 32; grew by one int32 for
 * the 3rd wheel encoder), byte-exact wire layout. All members are
 * naturally 4-byte-aligned (float/uint32_t/int32_t), so the compiler adds
 * no padding on the ARM Cortex-M3 AAPCS ABI this project targets; the
 * static assertion in survey.c guarantees this at build time rather than
 * relying on that assumption silently. Do not reorder/insert/resize
 * fields without re-deriving the offsets in section 8's telemetry table
 * and the CMD_VERTEX_DATA parsers on the ESP/browser side. */
typedef struct {
    float    x, y, theta;                                /* 4+4+4 = 12, radians (see survey.c) */
    uint32_t timestamp_ms;                                /* 4 */
    int32_t  enc1, enc2, enc3;                            /* 4+4+4 = 12 */
    float    heading_disagreement_signed_deg;             /* 4 */
    float    heading_disagreement_abs_integral_deg;        /* 4 */
} Vertex_t;                                                /* = 36 bytes */

typedef enum {
    SURVEY_MARK_OK          = 0,
    SURVEY_MARK_BUFFER_FULL = 1,
    /* Extension beyond section 7.2's literal 0/1 status values: section 9
     * requires vertex marking to be disabled while IMU_IsFaulted(), and
     * the frozen Vertex_t protocol text didn't define a status code for
     * that case. Added here as the minimal, explicit way to report it
     * rather than silently dropping the command or overloading
     * BUFFER_FULL. Flagged for review. */
    SURVEY_MARK_IMU_FAULTED = 2
} SurveyMarkStatus_t;

/* vertex_count=0, survey_closed=false -- section 5.4. Does NOT touch
 * odometry or IMU state; StartSurvey() (main.c) calls this alongside
 * Odometry_ResetPose()/IMU_ResetDiagnostics(). */
void Survey_Clear(void);

bool    Survey_IsClosed(void);
void    Survey_Close(void);                 /* CMD_CLOSE_SURVEY -- section 7.1, 11.2 */

uint8_t Survey_GetVertexCount(void);
const Vertex_t *Survey_GetVertex(uint8_t index);   /* NULL if index >= count */

/* Records a vertex at the CURRENT instant: reads Odometry_GetPosition(),
 * Encoder_GetPosition() x2, Odometry_GetHeadingDisagreement(), and
 * HAL_GetTick() itself at the moment this is called -- never a timestamp
 * supplied by ESP/browser, since STM32 is the sole authoritative state
 * source (section 7.1). Rejects (SURVEY_MARK_IMU_FAULTED) if
 * IMU_IsFaulted() is currently true, before touching the buffer. On
 * SURVEY_MARK_OK, writes the assigned index to *out_index; on any other
 * status, *out_index is left unwritten. */
SurveyMarkStatus_t Survey_MarkVertex(uint8_t *out_index);

/* Starts an incremental "send every vertex" resync (CMD_GET_ALL_VERTICES).
 * Deliberately NOT a single tight loop over all vertices -- sending up to
 * SURVEY_MAX_VERTICES (64) CMD_VERTEX_DATA packets back-to-back would
 * block the UART for ~200ms+ inside one Comm_Process() call, well past
 * the 10ms control-loop budget and long enough to gap gyro integration
 * for that whole window -- exactly the class of blocking-call risk this
 * project has repeatedly had to design around (UART/I2C elsewhere). This
 * function only arms the resync; Survey_ResyncTick() drains it. */
void Survey_StartResync(void);

/* Call once per main-loop pass (outside the 10ms control block is fine --
 * this is not control-timing-critical). Sends at most one CMD_VERTEX_DATA
 * packet per call while a resync is in progress; a cheap no-op otherwise. */
void Survey_ResyncTick(void);

#endif /* SURVEY_H */
