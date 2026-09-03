/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "bsp_pwm.h"
#include "motor.h"
#include "encoder.h"
#include "motor_controller.h"
#include "comm.h"
#include "odometry.h"
#include "kiwi_kinematics.h"
#include "imu.h"
#include "survey.h"
#include "waypoint_nav.h"
#include "calib.h"
#include "nvs.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DT_NOMINAL_S 0.010f
#define DT_MAX_S 0.050f

#define TELEMETRY_LOG_ENABLED 1
#define TELEMETRY_LOG_PERIOD_MS 50u
#define RAD_TO_DEG_TELEM (180.0f / 3.14159265359f)

#define ENCODER_FAULT_SPEED_EPS_RPM 2.0f
#define ENCODER_FAULT_CMD_MIN_RPM 5.0f
#define ENCODER_FAULT_TICK_THRESHOLD 20u

/* Kiwi-drive mounting geometry: a fixed chassis design constant, not
 * runtime-calibratable (see nvs.h) -- SET THESE TO MATCH YOUR ACTUAL
 * CHASSIS, the same treatment ENCODER_PPR already gets in encoder.h.
 * Angle convention: body-frame CCW-from-+X (forward), matching
 * kiwi_kinematics.h. The values below (90/210/330) are a placeholder
 * symmetric 120-degree default -- correct them before relying on any
 * of this. */
#define KIWI_MOUNT_ANGLE1_DEG 90.0f
#define KIWI_MOUNT_ANGLE2_DEG 210.0f
#define KIWI_MOUNT_ANGLE3_DEG 330.0f
#define KIWI_DEG2RAD(d) ((d) * 3.14159265359f / 180.0f)

#define NAV_IDLE_SPEED_TOL_MPS 0.02f
#define NAV_IDLE_OMEGA_TOL_RADPS 0.15f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

Motor_t motor1 = {
    .IN1_Port = IN1_1_GPIO_Port,
    .IN1_Pin = IN1_1_Pin,
    .IN2_Port = IN2_1_GPIO_Port,
    .IN2_Pin = IN2_1_Pin,
    .pwmChannel = PWM_CH1};

Motor_t motor2 = {
    .IN1_Port = IN3_1_GPIO_Port,
    .IN1_Pin = IN3_1_Pin,
    .IN2_Port = IN4_1_GPIO_Port,
    .IN2_Pin = IN4_1_Pin,
    .pwmChannel = PWM_CH2};

Motor_t motor3 = {
    .IN1_Port = IN1_2_GPIO_Port,
    .IN1_Pin = IN1_2_Pin,
    .IN2_Port = IN2_2_GPIO_Port,
    .IN2_Pin = IN2_2_Pin,
    .pwmChannel = PWM_CH3};

Encoder_t enc1;
Encoder_t enc2;
Encoder_t enc3;
MotorController_t ctrl1;
MotorController_t ctrl2;
MotorController_t ctrl3;

KiwiKinematics_t g_kin;
Odometry_t g_odom;
static OdomMotionMode_t g_motion_mode = ODOM_IDLE;

/* Last commanded body-frame velocity (CMD_SET_SPEED), replacing the
 * old SyncDrive_t g_drive/target_left/target_right. There is no
 * separate L/R-sync loop left to run: every tick, all 3 wheels'
 * targets are derived from this ONE shared command via
 * KiwiKinematics_InverseKinematics, so they are synchronized by
 * construction (see kiwi_kinematics.h). */
static float g_cmd_vx = 0.0f;
static float g_cmd_vy = 0.0f;
static float g_cmd_omega = 0.0f;

static bool g_encoder_fault[3] = {false, false, false};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

void OnCommandReceived(CommCmd_t cmd, const uint8_t *payload, uint8_t len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void CheckEncoderFault(void)
{
  static uint16_t stall_count[3] = {0, 0, 0};
  static bool prev_fault[3] = {false, false, false};

  Encoder_t *encs[3] = {&enc1, &enc2, &enc3};
  MotorController_t *ctrls[3] = {&ctrl1, &ctrl2, &ctrl3};

  float speed[3], cmd[3];
  for (int i = 0; i < 3; i++)
  {
    speed[i] = fabsf(Encoder_GetSpeed(encs[i]));
    cmd[i] = fabsf(ctrls[i]->target_rpm);
  }

  bool safe_to_print = !Nav_IsActive();

  for (int i = 0; i < 3; i++)
  {
    bool stalled = (cmd[i] > ENCODER_FAULT_CMD_MIN_RPM) && (speed[i] < ENCODER_FAULT_SPEED_EPS_RPM);

    /* "at least one OTHER wheel is confirmed moving" proves this
     * specific wheel has a problem rather than the whole robot
     * simply being idle/disabled -- generalizes the old pairwise
     * FL-vs-FR check to 3 wheels. */
    bool other_moving = false;
    for (int j = 0; j < 3; j++)
    {
      if (j != i && speed[j] >= ENCODER_FAULT_SPEED_EPS_RPM)
        other_moving = true;
    }

    stall_count[i] = (stalled && other_moving) ? (uint16_t)(stall_count[i] + 1) : 0;
    g_encoder_fault[i] = (stall_count[i] >= ENCODER_FAULT_TICK_THRESHOLD);

    if (g_encoder_fault[i] && !prev_fault[i] && safe_to_print)
    {
      printf("ENCODER_FAULT: wheel %d stalled (cmd=%.1f rpm) while another wheel moves\r\n",
             i + 1, (double)cmd[i]);
    }
    prev_fault[i] = g_encoder_fault[i];
  }
}

#if TELEMETRY_LOG_ENABLED

/* Rolling "worst tick this telemetry period" watchdog. Updated every
 * control tick (see the main loop) with the RAW elapsed_ms BEFORE any
 * dt clamping, and reset to 0 each time it is packed into a telemetry
 * send below -- so each packet reports the worst stall observed in the
 * preceding TELEMETRY_LOG_PERIOD_MS window. A sustained ~10ms reading
 * is normal jitter-free operation; anything materially higher is direct
 * evidence of a blocking call (I2C retry, UART, etc.) eating into the
 * control loop's timing budget. */
static uint32_t s_max_elapsed_ms_since_telem = 0;

static void SendTelemetryLog(float dtheta_gyro_rad)
{
  int32_t d1_32, d2_32, d3_32;
  Odometry_GetLastDeltas(&g_odom, &d1_32, &d2_32, &d3_32);
  int16_t d1 = (int16_t)d1_32;
  int16_t d2 = (int16_t)d2_32;
  int16_t d3 = (int16_t)d3_32;

  float x, y, theta_rad;
  Odometry_GetPosition(&g_odom, &x, &y, &theta_rad);

  float hd_signed, hd_abs;
  Odometry_GetHeadingDisagreement(&g_odom, &hd_signed, &hd_abs);

  uint32_t ts = HAL_GetTick();
  int32_t enc1_pos = Encoder_GetPosition(&enc1);
  int32_t enc2_pos = Encoder_GetPosition(&enc2);
  int32_t enc3_pos = Encoder_GetPosition(&enc3);
  float gyro_raw = IMU_GetLastRawDps();
  float gyro_bias = IMU_GetBiasRaw();
  float dtheta_deg = dtheta_gyro_rad * RAD_TO_DEG_TELEM;
  float theta_deg = theta_rad * RAD_TO_DEG_TELEM;
  uint8_t mode_byte = (uint8_t)g_motion_mode;

  uint8_t is_calibrated = IMU_IsCalibrated() ? 1u : 0u;
  float scale_correction = IMU_GetScaleCorrection();
  uint32_t max_dt_ms = s_max_elapsed_ms_since_telem;
  uint8_t max_dt_ms_clamped = (max_dt_ms > 255u) ? 255u : (uint8_t)max_dt_ms;
  s_max_elapsed_ms_since_telem = 0;

  uint8_t buf[61];
  memcpy(buf + 0, &ts, 4);
  memcpy(buf + 4, &enc1_pos, 4);
  memcpy(buf + 8, &enc2_pos, 4);
  memcpy(buf + 12, &enc3_pos, 4);
  memcpy(buf + 16, &d1, 2);
  memcpy(buf + 18, &d2, 2);
  memcpy(buf + 20, &d3, 2);
  memcpy(buf + 22, &gyro_raw, 4);
  memcpy(buf + 26, &gyro_bias, 4);
  memcpy(buf + 30, &dtheta_deg, 4);
  memcpy(buf + 34, &theta_deg, 4);
  memcpy(buf + 38, &x, 4);
  memcpy(buf + 42, &y, 4);
  memcpy(buf + 46, &hd_signed, 4);
  memcpy(buf + 50, &hd_abs, 4);
  buf[54] = mode_byte;
  buf[55] = is_calibrated;
  memcpy(buf + 56, &scale_correction, 4);
  buf[60] = max_dt_ms_clamped;

  Comm_SendPacket(CMD_TELEMETRY_LOG, buf, sizeof(buf));
}
#endif /* TELEMETRY_LOG_ENABLED */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  Encoder_Init(&enc1, &htim2);
  Encoder_Init(&enc2, &htim3);
  Encoder_Init(&enc3, &htim4);

  /* Starting sign convention, same as the old FL/FR default -- verify
   * against real hardware (does each encoder count up while its wheel
   * spins in the commanded-forward direction?) and correct per-wheel
   * if any reads backwards. */
  enc1.dir_sign = -1;
  enc2.dir_sign = -1;
  enc3.dir_sign = -1;

  MotorController_Init(&ctrl1, &motor1, &enc1);
  MotorController_Init(&ctrl2, &motor2, &enc2);
  MotorController_Init(&ctrl3, &motor3, &enc3);

  IMU_Init();

  Comm_Init(&huart2);
  Comm_RegisterCommandCallback(OnCommandReceived);

  /* g_kin is the single shared source of truth for chassis geometry:
   * both the command-mixing path (KiwiKinematics_InverseKinematics,
   * used directly in the main loop below) and g_odom's own internal
   * copy (see Odometry_Init) are built from these same values, so the
   * two can't drift apart in practice even though Odometry_t owns its
   * copy rather than a shared pointer (see odometry.h). Wheel/robot
   * radius start at NVS_GetDefaults' placeholders and are corrected
   * below if calibrated values are on flash; mounting angles are a
   * fixed chassis constant (see the KIWI_MOUNT_ANGLE* defines) and are
   * never NVS-loaded. */
  {
    NVS_Params_t defaults;
    NVS_GetDefaults(&defaults);
    KiwiKinematics_Init(&g_kin, defaults.wheel_radius, defaults.robot_radius,
                        KIWI_DEG2RAD(KIWI_MOUNT_ANGLE1_DEG),
                        KIWI_DEG2RAD(KIWI_MOUNT_ANGLE2_DEG),
                        KIWI_DEG2RAD(KIWI_MOUNT_ANGLE3_DEG));
  }
  Odometry_Init(&g_odom, &g_kin,
                Encoder_GetPosition(&enc1), Encoder_GetPosition(&enc2), Encoder_GetPosition(&enc3));

  Nav_Init();
  Cal_Init();

  {
    NVS_Params_t nvs;
    if (NVS_Load(&nvs))
    {
      IMU_SetScaleCorrection(nvs.gyro_scale);
      KiwiKinematics_Init(&g_kin, nvs.wheel_radius, nvs.robot_radius,
                          KIWI_DEG2RAD(KIWI_MOUNT_ANGLE1_DEG),
                          KIWI_DEG2RAD(KIWI_MOUNT_ANGLE2_DEG),
                          KIWI_DEG2RAD(KIWI_MOUNT_ANGLE3_DEG));
      Odometry_SetParams(&g_odom, &g_kin);
      Odometry_SetWheelScale(&g_odom, nvs.wheel_scale1, nvs.wheel_scale2, nvs.wheel_scale3);
    }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint32_t lastControlTime = 0;
  uint32_t lastStatusTime = 0;
#if TELEMETRY_LOG_ENABLED
  uint32_t lastTelemetryLogTime = 0;
#endif

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    Comm_Process();
    Survey_ResyncTick();
    Nav_DebugTick();

    if ((now - lastControlTime) >= 10U)
    {
      uint32_t elapsed_ms = now - lastControlTime;
      float dt = (float)elapsed_ms * 0.001f;
      lastControlTime = now;

      if (elapsed_ms > s_max_elapsed_ms_since_telem)
      {
        s_max_elapsed_ms_since_telem = elapsed_ms;
      }

      if (dt <= 0.0f)
      {
        dt = DT_NOMINAL_S;
      }
      else if (dt > DT_MAX_S)
      {
        /* A real, known elapsed time exists -- use it (capped for
         * numerical safety) rather than discarding it in favour of
         * a fixed nominal value that would under-integrate whatever
         * actually happened during the stall. */
        dt = DT_MAX_S;
      }

      float dtheta_gyro = IMU_UpdateGyroZ(dt);
      IMU_CalibrationTick();

      float cur_x, cur_y, cur_theta;
      Odometry_GetPosition(&g_odom, &cur_x, &cur_y, &cur_theta);

      bool any_encoder_fault = g_encoder_fault[0] || g_encoder_fault[1] || g_encoder_fault[2];

      if (!Comm_IsConnected() || IMU_IsFaulted() || any_encoder_fault)
      {
        if (Nav_IsActive())
          Nav_AbortFailsafe();
        if (Cal_IsActive())
          Cal_Abort();
        g_cmd_vx = 0.0f;
        g_cmd_vy = 0.0f;
        g_cmd_omega = 0.0f;
        MotorController_Stop(&ctrl1);
        MotorController_Stop(&ctrl2);
        MotorController_Stop(&ctrl3);
      }
      else if (Nav_IsActive())
      {
        Nav_Update(dt);
      }
      else if (Cal_IsActive())
      {
        Cal_Update(dt, dtheta_gyro);
      }
      else
      {
        float w1, w2, w3;
        KiwiKinematics_InverseKinematics(&g_kin, g_cmd_vx, g_cmd_vy, g_cmd_omega, &w1, &w2, &w3);
        MotorController_SetSpeed(&ctrl1, w1);
        MotorController_SetSpeed(&ctrl2, w2);
        MotorController_SetSpeed(&ctrl3, w3);
        MotorController_Update(&ctrl1, dt);
        MotorController_Update(&ctrl2, dt);
        MotorController_Update(&ctrl3, dt);
      }
      {
        static bool auto_was_active = false;
        bool auto_now = Nav_IsActive() || Cal_IsActive();
        if (auto_was_active && !auto_now)
        {
          /* Come to a clean, known-zero state on the transition
           * back to manual control -- MotorController_Stop
           * clears each controller's own PID integrator (see
           * motor_controller.c), preventing whatever Nav/Cal
           * left behind from carrying over into manual drive. */
          g_cmd_vx = 0.0f;
          g_cmd_vy = 0.0f;
          g_cmd_omega = 0.0f;
          MotorController_Stop(&ctrl1);
          MotorController_Stop(&ctrl2);
          MotorController_Stop(&ctrl3);
        }
        auto_was_active = auto_now;
      }

      int32_t pos1 = Encoder_GetPosition(&enc1);
      int32_t pos2 = Encoder_GetPosition(&enc2);
      int32_t pos3 = Encoder_GetPosition(&enc3);

      OdomMotionMode_t mode_eff;
      if (Nav_IsActive())
        mode_eff = Nav_GetMotionMode();
      else if (Cal_IsActive())
        mode_eff = Cal_GetMotionMode();
      else
      {
        /* STRAIGHT vs ROTATING no longer changes Odometry_Update's
         * behavior (see odometry.h) -- only IDLE-vs-not matters
         * here, so this collapses to one stationary check against
         * NAV_IDLE_SPEED_TOL_MPS/NAV_IDLE_OMEGA_TOL_RADPS. */
        bool stationary = (sqrtf(g_cmd_vx * g_cmd_vx + g_cmd_vy * g_cmd_vy) < NAV_IDLE_SPEED_TOL_MPS) && (fabsf(g_cmd_omega) < NAV_IDLE_OMEGA_TOL_RADPS);
        mode_eff = stationary ? ODOM_IDLE : ODOM_STRAIGHT;
      }
      Odometry_Update(&g_odom, pos1, pos2, pos3, dtheta_gyro, mode_eff);

      CheckEncoderFault();

#if TELEMETRY_LOG_ENABLED
      if ((now - lastTelemetryLogTime) >= TELEMETRY_LOG_PERIOD_MS)
      {
        lastTelemetryLogTime = now;
        SendTelemetryLog(dtheta_gyro);
      }
#endif
    }

    if ((now - lastStatusTime) >= 100U)
    {
      lastStatusTime = now;

      float speeds[3];
      speeds[0] = MotorController_GetCurrentSpeed(&ctrl1);
      speeds[1] = MotorController_GetCurrentSpeed(&ctrl2);
      speeds[2] = MotorController_GetCurrentSpeed(&ctrl3);

      int32_t cnt1 = Encoder_GetPosition(&enc1);
      int32_t cnt2 = Encoder_GetPosition(&enc2);
      int32_t cnt3 = Encoder_GetPosition(&enc3);
      uint8_t duty1 = MotorController_GetDuty(&ctrl1);
      uint8_t duty2 = MotorController_GetDuty(&ctrl2);
      uint8_t duty3 = MotorController_GetDuty(&ctrl3);

      /* Layout (27 bytes):
       *   [0..3]   float  rpm_1
       *   [4..7]   float  rpm_2
       *   [8..11]  float  rpm_3
       *   [12..15] int32  enc_1  (raw count — drift diagnosis)
       *   [16..19] int32  enc_2
       *   [20..23] int32  enc_3
       *   [24]     uint8  duty_1 (%)
       *   [25]     uint8  duty_2
       *   [26]     uint8  duty_3
       */
      uint8_t status_buf[27];
      memcpy(status_buf, speeds, 12);
      memcpy(status_buf + 12, &cnt1, 4);
      memcpy(status_buf + 16, &cnt2, 4);
      memcpy(status_buf + 20, &cnt3, 4);
      status_buf[24] = duty1;
      status_buf[25] = duty2;
      status_buf[26] = duty3;
      Comm_SendPacket(CMD_STATUS, status_buf, 27);
    }
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 1799;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Internal_LED_GPIO_Port, Internal_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IN1_1_Pin | IN2_1_Pin | IN3_1_Pin | IN4_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IN1_2_Pin | IN2_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Internal_LED_Pin */
  GPIO_InitStruct.Pin = Internal_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Internal_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_1_Pin IN2_1_Pin IN3_1_Pin IN4_1_Pin */
  GPIO_InitStruct.Pin = IN1_1_Pin | IN2_1_Pin | IN3_1_Pin | IN4_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_2_Pin IN2_2_Pin */
  GPIO_InitStruct.Pin = IN1_2_Pin | IN2_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int __io_putchar(int ch)
{
    (void)ch;
    return ch;
}

static void StartSurvey(void)
{
    if (Nav_IsActive()) Nav_Stop();
    if (Cal_IsActive()) Cal_Abort();
    Nav_ClearWaypoints();
    Odometry_ResetPose(&g_odom,
                        Encoder_GetPosition(&enc1), Encoder_GetPosition(&enc2), Encoder_GetPosition(&enc3));
    IMU_ResetDiagnostics();
    Survey_Clear();
}

/* Persists the currently-applied calibration (gyro scale + odometry
 * geometry) to flash. Must be called after ANY CMD_CAL_APPLY_* that
 * actually changed a value -- otherwise the calibration only lives in
 * RAM and silently reverts to NVS_GetDefaults() on the next power
 * cycle, invalidating whatever field test produced it. */
static void PersistCalibration(void)
{
    NVS_Params_t nvs;
    nvs.gyro_scale    = IMU_GetScaleCorrection();
    nvs.wheel_radius  = g_odom.kin.wheel_radius_m;
    nvs.robot_radius  = g_odom.kin.robot_radius_m;
    nvs.wheel_scale1  = g_odom.wheel_scale[0];
    nvs.wheel_scale2  = g_odom.wheel_scale[1];
    nvs.wheel_scale3  = g_odom.wheel_scale[2];
    if (!NVS_Save(&nvs)) {
        printf("NVS_SAVE_FAILED: calibration not persisted\r\n");
    }
}

void OnCommandReceived(CommCmd_t cmd, const uint8_t *payload, uint8_t len)
{
    switch (cmd) {

    case CMD_SET_SPEED: {
        if (Nav_IsActive()) break;
        if (len == 12) {
            memcpy(&g_cmd_vx,    payload,     4);
            memcpy(&g_cmd_vy,    payload + 4, 4);
            memcpy(&g_cmd_omega, payload + 8, 4);
        }
        break;
    }

    case CMD_REQUEST_TELEMETRY: {
        float speeds[3];
        speeds[0] = MotorController_GetCurrentSpeed(&ctrl1);
        speeds[1] = MotorController_GetCurrentSpeed(&ctrl2);
        speeds[2] = MotorController_GetCurrentSpeed(&ctrl3);
        Comm_SendPacket(CMD_REQUEST_TELEMETRY, (uint8_t *)speeds, sizeof(speeds));
        break;
    }

    case CMD_SET_PID: {
        if (len == 12) {
            float kp, ki, kd;
            memcpy(&kp, payload,     4);
            memcpy(&ki, payload + 4, 4);
            memcpy(&kd, payload + 8, 4);
            PID_SetGains(&ctrl1.pid, kp, ki, kd);
            PID_SetGains(&ctrl2.pid, kp, ki, kd);
            PID_SetGains(&ctrl3.pid, kp, ki, kd);
        }
        break;
    }

    case CMD_RESET_ODOM: {
        StartSurvey();
        break;
    }

    case CMD_GET_ODOM: {
        float x, y, theta;
        Odometry_GetPosition(&g_odom, &x, &y, &theta);

        uint8_t odom_buf[12];
        memcpy(odom_buf,     &x,     4);
        memcpy(odom_buf + 4, &y,     4);
        memcpy(odom_buf + 8, &theta, 4);
        Comm_SendPacket(CMD_ODOMETRY_DATA, odom_buf, 12);
        break;
    }

    /* ---- Gate 2 ---- */
    case CMD_SET_MOTION_MODE: {
        if (Nav_IsActive()) break;
        if (len == 1) {
            bool robot_stopped = (g_cmd_vx == 0.0f) && (g_cmd_vy == 0.0f) && (g_cmd_omega == 0.0f);
            if (g_motion_mode == ODOM_IDLE || robot_stopped) {
                uint8_t requested = payload[0];
                if (requested <= (uint8_t)ODOM_STRAIGHT) {
                    g_motion_mode = (OdomMotionMode_t)requested;
                }
                /* else: out-of-range byte, silently ignored -- mode unchanged */
            }
            /* else: rejected -- section 11.1, no mode change mid-motion */
        }
        break;
    }

    case CMD_MARK_VERTEX: {
        uint8_t index = 0;
        SurveyMarkStatus_t status = Survey_MarkVertex(&index);

        uint8_t resp[2 + sizeof(Vertex_t)];
        resp[0] = (uint8_t)status;
        uint8_t resp_len = 1;

        if (status == SURVEY_MARK_OK) {
            const Vertex_t *v = Survey_GetVertex(index);
            if (v) {
                resp[1] = index;
                memcpy(&resp[2], v, sizeof(Vertex_t));
                resp_len = (uint8_t)(2 + sizeof(Vertex_t));
            }
        }

        Comm_SendPacket(CMD_VERTEX_DATA, resp, resp_len);
        break;
    }

    case CMD_CLEAR_VERTICES: {
        bool robot_stopped = (g_cmd_vx == 0.0f) && (g_cmd_vy == 0.0f) && (g_cmd_omega == 0.0f);
        if (g_motion_mode == ODOM_IDLE || robot_stopped) {
            Survey_Clear();
        }
        break;
    }

    case CMD_CLOSE_SURVEY: {
        Survey_Close();
        break;
    }

    case CMD_GET_ALL_VERTICES: {
        Survey_StartResync();
        break;
    }

    /* ---- Phase 3: autonomous waypoint patrol ---- */
    case CMD_NAV_CLEAR: {
        if (!Nav_IsActive()) Nav_ClearWaypoints();
        Nav_SendStatus();
        break;
    }
    case CMD_NAV_ADD_WP: {
        if (len == 8) {
            float wx, wy;
            memcpy(&wx, payload,     4);
            memcpy(&wy, payload + 4, 4);
            Nav_AddWaypoint(wx, wy);   /* rejected internally if active/full */
        }
        Nav_SendStatus();
        break;
    }
    case CMD_NAV_START: {
        if (len == 1) {
            if (Nav_Start(payload[0])) {
                /* zero any stale manual targets so the post-patrol
                 * hand-over is clean */
                g_cmd_vx = 0.0f; g_cmd_vy = 0.0f; g_cmd_omega = 0.0f;
                MotorController_Stop(&ctrl1);
                MotorController_Stop(&ctrl2);
                MotorController_Stop(&ctrl3);
            }
        }
        Nav_SendStatus();
        break;
    }
    case CMD_NAV_STOP: {
        if (Nav_IsActive()) Nav_Stop();
        Nav_SendStatus();
        break;
    }
    case CMD_NAV_MARK_WP: {
        float x, y, th;
        Odometry_GetPosition(&g_odom, &x, &y, &th);
        bool ok = Nav_AddWaypoint(x, y);   /* rejected internally if active/full */

        uint8_t resp[10];
        resp[0] = ok ? 1u : 0u;
        resp[1] = ok ? (uint8_t)(Nav_GetWaypointCount() - 1u) : 0xFFu;
        memcpy(resp + 2, &x, 4);
        memcpy(resp + 6, &y, 4);
        Comm_SendPacket(CMD_NAV_WP_DATA, resp, sizeof(resp));
        Nav_SendStatus();
        break;
    }
    /* ---- Phase 4: field calibration ---- */

    case CMD_CAL_SPIN: {
        if (!Nav_IsActive() && !Cal_IsActive() && len == 12) {
            float deg_ccw, deg_cw, power_pct;
            memcpy(&deg_ccw,   payload,     4);
            memcpy(&deg_cw,    payload + 4, 4);
            memcpy(&power_pct, payload + 8, 4);
            Cal_StartSpin(deg_ccw, deg_cw, power_pct);
        }
        break;
    }
    case CMD_CAL_ROLL: {
        if (!Nav_IsActive() && !Cal_IsActive() && len == 8) {
            float dist_m, power_pct;
            memcpy(&dist_m,    payload,     4);
            memcpy(&power_pct, payload + 4, 4);
            Cal_StartRoll(dist_m, power_pct);
        }
        break;
    }
    case CMD_CAL_STOP: {
        if (Cal_IsActive()) Cal_Abort();
        break;
    }

    /* Gyro scale and robot radius are both derived from the SAME
     * completed 2-leg spin test -- calib.c already holds its own
     * cached gyro-integrated vs kinematics-implied per-leg figures
     * (see Cal_GetSpinResult); the operator supplies only the
     * ground-truth measured angles (protractor/floor marks), never
     * retransmitting anything the STM32 already measured itself. */
    case CMD_CAL_APPLY_SPIN: {
        float g1_gyro, g1_kin, g2_gyro, g2_kin;
        if (len == 8 && Cal_GetSpinResult(&g1_gyro, &g1_kin, &g2_gyro, &g2_kin)) {
            float measured_ccw, measured_cw;
            memcpy(&measured_ccw, payload,     4);
            memcpy(&measured_cw,  payload + 4, 4);

            float gyro_sum     = g1_gyro + g2_gyro;
            float kin_sum       = g1_kin  + g2_kin;
            float measured_sum = measured_ccw + measured_cw;

            /* Guard against a degenerate cached test or nonsensical
             * operator input driving either formula's denominator
             * toward zero -- same spirit as the pre-migration
             * handler's d_cmd > 0.05f sanity check. */
            if (gyro_sum > 1.0f && measured_sum > 1.0f && kin_sum > 0.0f) {
                float new_gyro_scale   = IMU_GetScaleCorrection() * measured_sum / gyro_sum;
                float new_robot_radius = g_kin.robot_radius_m * kin_sum / measured_sum;

                if (IMU_SetScaleCorrection(new_gyro_scale)) {
                    KiwiKinematics_Init(&g_kin, g_kin.wheel_radius_m, new_robot_radius,
                                         g_kin.mount_angle_rad[0], g_kin.mount_angle_rad[1], g_kin.mount_angle_rad[2]);
                    Odometry_SetParams(&g_odom, &g_kin);
                    PersistCalibration();
                    Cal_ClearSpinResult();
                }
            }
        }
        break;
    }
    case CMD_CAL_APPLY_ROLL: {
        float dist_estimated;
        if (len == 4 && Cal_GetRollResult(&dist_estimated) && dist_estimated > 0.01f) {
            float d_true;
            memcpy(&d_true, payload, 4);

            if (d_true > 0.01f) {
                float new_wheel_radius = g_kin.wheel_radius_m * d_true / dist_estimated;
                KiwiKinematics_Init(&g_kin, new_wheel_radius, g_kin.robot_radius_m,
                                     g_kin.mount_angle_rad[0], g_kin.mount_angle_rad[1], g_kin.mount_angle_rad[2]);
                Odometry_SetParams(&g_odom, &g_kin);
                PersistCalibration();
                Cal_ClearRollResult();
            }
        }
        break;
    }

    default:
        break;
    }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
