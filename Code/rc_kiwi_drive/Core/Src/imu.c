/*
 * imu.c
 *
 * MPU6050 gyro-Z driver. See sections 2.3 (calibration), 5.1 (API), 6.2
 * (register values) of phase2_perimeter_mapping_plan_FINAL_v4.md.

 */
#include "imu.h"
#include "stm32f1xx_hal.h"
#include "encoder.h"
#include "motor_controller.h"
#include <math.h>

extern I2C_HandleTypeDef  hi2c2;
extern Encoder_t          enc1;
extern Encoder_t          enc2;
extern Encoder_t          enc3;

extern MotorController_t  ctrl1;
extern MotorController_t  ctrl2;
extern MotorController_t  ctrl3;

#define MPU6050_I2C_ADDR          (0x68 << 1)
#define MPU6050_WHO_AM_I_EXPECT   0x68u
#define MPU6050_REG_PWR_MGMT_1    0x6Bu
#define MPU6050_REG_WHO_AM_I      0x75u
#define MPU6050_REG_SMPLRT_DIV    0x19u
#define MPU6050_REG_CONFIG        0x1Au
#define MPU6050_REG_GYRO_CONFIG   0x1Bu
#define MPU6050_REG_GYRO_ZOUT_H   0x47u

#define MPU6050_VAL_PWR_WAKE      0x01u
#define MPU6050_VAL_CONFIG_DLPF3  0x03u
#define MPU6050_VAL_GYRO_FS1000   0x10u
#define MPU6050_VAL_SMPLRT_100HZ  0x09u

#define IMU_GYRO_LSB_PER_DPS      32.8f
#define IMU_DEG_TO_RAD            (3.14159265359f / 180.0f)

#define IMU_I2C_TIMEOUT_MS        5u

/* A transient I2C glitch (this bus runs physically next to 3 motors
 * doing 20kHz PWM switching) must not permanently halt the robot.
 * Tolerate a short run of failures before declaring a hard fault, and
 * even once faulted, keep attempting to recover rather than requiring
 * a power cycle. */
#define IMU_I2C_FAIL_TOLERANCE     5u      /* consecutive failed reads
                                             * absorbed silently before
                                             * a fault is declared */
#define IMU_RECOVERY_RETRY_MS      1000u   /* while faulted, retry this often */

static float    s_gyro_bias              = 0.0f;
static float    s_scale_correction       = 1.0f;
static bool     s_imu_calibrated         = false;
static bool     s_faulted                = false;

static float    s_last_raw_dps           = 0.0f;
static bool     s_reading_valid_this_tick = false;

static bool     s_calib_active           = false;
static uint16_t s_calib_n                = 0;
static float    s_calib_sum              = 0.0f;
static float    s_calib_min              = 0.0f;
static float    s_calib_max              = 0.0f;

static uint8_t  s_consec_i2c_fail          = 0;
static uint32_t s_last_recovery_attempt_ms = 0;

static bool raw_conditions_met(void)
{
    if (s_faulted) return false;

    bool encoders_quiet = (fabsf(Encoder_GetSpeed(&enc1)) < IMU_EPS_SPEED_RPM) &&
                          (fabsf(Encoder_GetSpeed(&enc2)) < IMU_EPS_SPEED_RPM) &&
						  (fabsf(Encoder_GetSpeed(&enc3)) < IMU_EPS_SPEED_RPM) ;
    bool commanded_stop = (ctrl1.target_rpm == 0.0f) && (ctrl2.target_rpm == 0.0f) && (ctrl3.target_rpm == 0.0f);

    return encoders_quiet && commanded_stop;
}

/* ---- I2C bus recovery ----------------------------------------------------
 * STM32 I2C stuck-BUSY (SDA held low by a slave mid-byte, or a bus
 * glitch) is a documented silicon errata -- HAL_I2C_Init() alone does
 * not reliably clear it, and the HAL has no built-in recovery helper.
 * Standard remediation: release the bus with up to 9 SCL pulses driven
 * as plain GPIO, generate a STOP condition, then let HAL_I2C_Init()
 * bring the peripheral back up (its MSP callback reconfigures
 * PB10/PB11 back to I2C alternate-function mode regardless of what
 * state this leaves them in, so no manual pin-mode restore is needed
 * here). PB10=I2C2_SCL, PB11=I2C2_SDA per the CubeMX pin assignment. */
static void i2c_bus_recover(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_SET);

    for (int i = 0; i < 9 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    /* STOP condition: SDA low->high while SCL is high. */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(1);

    HAL_I2C_Init(&hi2c2);
}

static bool i2c_probe_who_am_i(void)
{
    uint8_t who = 0;
    return (HAL_I2C_Mem_Read(&hi2c2, MPU6050_I2C_ADDR, MPU6050_REG_WHO_AM_I,
                              I2C_MEMADD_SIZE_8BIT, &who, 1, IMU_I2C_TIMEOUT_MS) == HAL_OK)
           && (who == MPU6050_WHO_AM_I_EXPECT);
}

static void i2c_attempt_recovery(void)
{
    i2c_bus_recover();
    if (i2c_probe_who_am_i()) {
        s_faulted          = false;
        s_consec_i2c_fail  = 0;
    }
}

/* ---- Public API ---------------------------------------------------------- */

void IMU_Init(void)
{
    s_gyro_bias               = 0.0f;
    s_scale_correction        = 1.0f;
    s_imu_calibrated          = false;
    s_faulted                 = false;
    s_last_raw_dps            = 0.0f;
    s_reading_valid_this_tick = false;
    s_calib_active            = false;
    s_calib_n                 = 0;
    s_calib_sum               = 0.0f;
    s_consec_i2c_fail          = 0;
    s_last_recovery_attempt_ms = 0;

    uint8_t val;

    /* 1. Wake up (clear SLEEP bit) */
    val = MPU6050_VAL_PWR_WAKE;
    if (HAL_I2C_Mem_Write(&hi2c2, MPU6050_I2C_ADDR, MPU6050_REG_PWR_MGMT_1,
                           I2C_MEMADD_SIZE_8BIT, &val, 1, IMU_I2C_TIMEOUT_MS) != HAL_OK) {
        s_faulted = true;
        return;
    }
    HAL_Delay(100);

    uint8_t who_am_i = 0;
    if (HAL_I2C_Mem_Read(&hi2c2, MPU6050_I2C_ADDR, MPU6050_REG_WHO_AM_I,
                          I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, IMU_I2C_TIMEOUT_MS) != HAL_OK
        || who_am_i != MPU6050_WHO_AM_I_EXPECT) {
        s_faulted = true;
        return;
    }

    bool ok = true;

    val = MPU6050_VAL_CONFIG_DLPF3;
    if (HAL_I2C_Mem_Write(&hi2c2, MPU6050_I2C_ADDR, MPU6050_REG_CONFIG,
                           I2C_MEMADD_SIZE_8BIT, &val, 1, IMU_I2C_TIMEOUT_MS) != HAL_OK) ok = false;

    val = MPU6050_VAL_GYRO_FS1000;
    if (HAL_I2C_Mem_Write(&hi2c2, MPU6050_I2C_ADDR, MPU6050_REG_GYRO_CONFIG,
                           I2C_MEMADD_SIZE_8BIT, &val, 1, IMU_I2C_TIMEOUT_MS) != HAL_OK) ok = false;

    val = MPU6050_VAL_SMPLRT_100HZ;
    if (HAL_I2C_Mem_Write(&hi2c2, MPU6050_I2C_ADDR, MPU6050_REG_SMPLRT_DIV,
                           I2C_MEMADD_SIZE_8BIT, &val, 1, IMU_I2C_TIMEOUT_MS) != HAL_OK) ok = false;

    if (!ok) {
        s_faulted = true;
        return;
    }

    s_faulted = false;
}

void IMU_CalibrationTick(void)
{
    if (!s_reading_valid_this_tick) {

        return;
    }

    if (!raw_conditions_met()) {
        s_calib_active = false;
        s_calib_n = 0;
        s_calib_sum = 0.0f;
        return;
    }

    float raw = s_last_raw_dps;

    if (!s_calib_active) {
        s_calib_active = true;
        s_calib_n = 0;
        s_calib_sum = 0.0f;
        s_calib_min = s_calib_max = raw;
    }

    s_calib_sum += raw;
    if (raw < s_calib_min) s_calib_min = raw;
    if (raw > s_calib_max) s_calib_max = raw;
    s_calib_n++;

    if ((s_calib_max - s_calib_min) > IMU_GYRO_STATIONARY_RANGE_DPS) {
        s_calib_active = false;
        s_calib_n = 0;
        s_calib_sum = 0.0f;
        return;
    }

    if (s_calib_n >= IMU_BIAS_CALIBRATION_SAMPLES) {
        float new_bias = s_calib_sum / (float)s_calib_n;
        if (!s_imu_calibrated || fabsf(new_bias - s_gyro_bias) < IMU_MAX_BIAS_JUMP_DPS) {
            s_gyro_bias      = new_bias;
            s_imu_calibrated = true;
        }
        s_calib_active = false;
        s_calib_n = 0;
        s_calib_sum = 0.0f;
    }
}

bool IMU_IsCalibrated(void)
{
    return s_imu_calibrated;
}

float IMU_UpdateGyroZ(float dt)
{
    uint32_t now = HAL_GetTick();

    if (s_faulted) {
        if ((now - s_last_recovery_attempt_ms) < IMU_RECOVERY_RETRY_MS) {
            s_reading_valid_this_tick = false;
            return 0.0f;
        }
        s_last_recovery_attempt_ms = now;
        i2c_attempt_recovery();
        if (s_faulted) {
            s_reading_valid_this_tick = false;
            return 0.0f;
        }
        /* Recovered this tick -- fall through and take a real reading. */
    }

    uint8_t raw_bytes[2];
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c2, MPU6050_I2C_ADDR, MPU6050_REG_GYRO_ZOUT_H,
                                             I2C_MEMADD_SIZE_8BIT, raw_bytes, 2, IMU_I2C_TIMEOUT_MS);
    if (st != HAL_OK) {
        s_reading_valid_this_tick = false;
        s_consec_i2c_fail++;
        if (s_consec_i2c_fail >= IMU_I2C_FAIL_TOLERANCE) {
            s_faulted = true;
            s_last_recovery_attempt_ms = now;
            i2c_attempt_recovery();   /* may clear the fault immediately */
        }
        return 0.0f;
    }

    s_consec_i2c_fail = 0;
    s_reading_valid_this_tick = true;

    int16_t counts  = (int16_t)((raw_bytes[0] << 8) | raw_bytes[1]);
    float   raw_dps = (float)counts / IMU_GYRO_LSB_PER_DPS;
    s_last_raw_dps  = raw_dps;

    /* Bias first, then scale -- section 5.1 explicit ordering */
    float corrected_dps = (raw_dps - s_gyro_bias) * s_scale_correction;

    return corrected_dps * IMU_DEG_TO_RAD * dt;
}

float IMU_GetBiasRaw(void)
{
    return s_gyro_bias;
}

float IMU_GetLastRawDps(void)
{
    return s_last_raw_dps;
}

bool IMU_SetScaleCorrection(float factor)
{
    if (factor <= 0.0f) return false;
    s_scale_correction = factor;
    return true;
}

float IMU_GetScaleCorrection(void)
{
    return s_scale_correction;
}

void IMU_ResetDiagnostics(void)
{

}

bool IMU_IsFaulted(void)
{
    return s_faulted;
}
