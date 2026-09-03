/*
 * survey.c
 *
 *  Created on: Aug 28, 2026
 *      Author: Alireza
 */

#include "survey.h"
#include "odometry.h"
#include "encoder.h"
#include "imu.h"
#include "comm.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stddef.h>

typedef char Vertex_t_size_check[(sizeof(Vertex_t) == 36) ? 1 : -1];

extern Odometry_t g_odom;
extern Encoder_t  enc1;
extern Encoder_t  enc2;
extern Encoder_t  enc3;

static Vertex_t s_vertices[SURVEY_MAX_VERTICES];
static uint8_t  s_vertex_count  = 0;
static bool     s_survey_closed = false;

static bool     s_resync_active     = false;
static uint8_t  s_resync_next_index = 0;

void Survey_Clear(void)
{
    s_vertex_count  = 0;
    s_survey_closed = false;
    /* in-flight resync (if any) is also stale once vertices are cleared */
    s_resync_active = false;
}

bool Survey_IsClosed(void)
{
    return s_survey_closed;
}

void Survey_Close(void)
{
    s_survey_closed = true;
}

uint8_t Survey_GetVertexCount(void)
{
    return s_vertex_count;
}

const Vertex_t *Survey_GetVertex(uint8_t index)
{
    if (index >= s_vertex_count) return NULL;
    return &s_vertices[index];
}

SurveyMarkStatus_t Survey_MarkVertex(uint8_t *out_index)
{

    if (IMU_IsFaulted()) {
        return SURVEY_MARK_IMU_FAULTED;
    }

    if (s_vertex_count >= SURVEY_MAX_VERTICES) {
        return SURVEY_MARK_BUFFER_FULL;
    }

    Vertex_t *v = &s_vertices[s_vertex_count];

    Odometry_GetPosition(&g_odom, &v->x, &v->y, &v->theta);

    v->timestamp_ms = HAL_GetTick();
    v->enc1         = Encoder_GetPosition(&enc1);
    v->enc2         = Encoder_GetPosition(&enc2);
    v->enc3         = Encoder_GetPosition(&enc3);

    Odometry_GetHeadingDisagreement(&g_odom,
                                     &v->heading_disagreement_signed_deg,
                                     &v->heading_disagreement_abs_integral_deg);

    if (out_index) *out_index = s_vertex_count;
    s_vertex_count++;

    return SURVEY_MARK_OK;
}

void Survey_StartResync(void)
{
    s_resync_active     = true;
    s_resync_next_index = 0;
}

void Survey_ResyncTick(void)
{
    if (!s_resync_active) return;

    if (s_resync_next_index >= s_vertex_count) {
        s_resync_active = false;
        return;
    }

    const Vertex_t *v = &s_vertices[s_resync_next_index];

    uint8_t resp[2 + sizeof(Vertex_t)];
    resp[0] = (uint8_t)SURVEY_MARK_OK;
    resp[1] = s_resync_next_index;
    memcpy(resp + 2, v, sizeof(Vertex_t));

    Comm_SendPacket(CMD_VERTEX_DATA, resp, sizeof(resp));

    s_resync_next_index++;
}


