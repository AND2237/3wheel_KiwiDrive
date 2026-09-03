#include "nvs.h"
#include "stm32f1xx_hal.h"
#include <string.h>

#define NVS_PAGE_ADDR   0x0800FC00UL
#define NVS_MAGIC       0x4E565331UL
#define NVS_VERSION     3u   /* bumped: v2's track_width/wheel_diameter/
                                 wheel_scale_left/_right/rot_lurch_m_per_rad
                                 layout is semantically replaced by
                                 wheel_radius/robot_radius/wheel_scale1-3
                                 for the kiwi drive. Byte SIZE happens to
                                 stay 32 (6 floats either way), but the
                                 MEANING of those bytes changed, so old
                                 v2 flash content must still be rejected
                                 by the version check below rather than
                                 silently misread as new fields. */

#define NVS_SIZE        32u

void NVS_GetDefaults(NVS_Params_t *out)
{
    out->gyro_scale    = 1.0f;
    /* Placeholders -- measure your actual chassis and replace these,
     * the same way the old wheel_diameter/track_width defaults were
     * always meant to be measured-and-corrected, not final. */
    out->wheel_radius   = 0.05f;
    out->robot_radius   = 0.15f;
    out->wheel_scale1   = 1.0f;
    out->wheel_scale2   = 1.0f;
    out->wheel_scale3   = 1.0f;
}

static uint16_t crc16(const uint8_t *d, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= (uint16_t)(*d++) << 8;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

static void pack(uint8_t *buf, const NVS_Params_t *p)
{
    uint32_t magic = NVS_MAGIC; uint16_t ver = NVS_VERSION;
    memcpy(buf + 0, &magic, 4);
    memcpy(buf + 4, &ver,   2);
    memcpy(buf + 8,  &p->gyro_scale,    4);
    memcpy(buf + 12, &p->wheel_radius,  4);
    memcpy(buf + 16, &p->robot_radius,  4);
    memcpy(buf + 20, &p->wheel_scale1,  4);
    memcpy(buf + 24, &p->wheel_scale2,  4);
    memcpy(buf + 28, &p->wheel_scale3,  4);
    uint16_t c = crc16(buf + 8, 24);
    memcpy(buf + 6, &c, 2);
}

bool NVS_Load(NVS_Params_t *out)
{
    const uint8_t *mem = (const uint8_t *)NVS_PAGE_ADDR;
    uint32_t magic; uint16_t ver, crc;
    memcpy(&magic, mem + 0, 4);
    memcpy(&ver,   mem + 4, 2);
    memcpy(&crc,   mem + 6, 2);
    if (magic != NVS_MAGIC || ver != NVS_VERSION) return false;
    if (crc != crc16(mem + 8, 24)) return false;
    memcpy(&out->gyro_scale,   mem + 8,  4);
    memcpy(&out->wheel_radius, mem + 12, 4);
    memcpy(&out->robot_radius, mem + 16, 4);
    memcpy(&out->wheel_scale1, mem + 20, 4);
    memcpy(&out->wheel_scale2, mem + 24, 4);
    memcpy(&out->wheel_scale3, mem + 28, 4);
    return true;
}

bool NVS_Save(const NVS_Params_t *p)
{
    uint8_t buf[NVS_SIZE];
    pack(buf, p);

    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef er; uint32_t err = 0;
    er.TypeErase   = FLASH_TYPEERASE_PAGES;
    er.PageAddress = NVS_PAGE_ADDR;
    er.NbPages     = 1;
    if (HAL_FLASHEx_Erase(&er, &err) != HAL_OK) { HAL_FLASH_Lock(); return false; }

    for (uint16_t off = 0; off < NVS_SIZE; off += 2) {
        uint16_t hw; memcpy(&hw, buf + off, 2);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, NVS_PAGE_ADDR + off, hw) != HAL_OK) {
            HAL_FLASH_Lock(); return false;
        }
    }
    HAL_FLASH_Lock();

    return memcmp((const void *)NVS_PAGE_ADDR, buf, NVS_SIZE) == 0;
}