/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : LQG Motor Control using USB CDC - GA25-370 280RPM
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "i2c-lcd.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;  // PWM PA8
TIM_HandleTypeDef htim2;  // Encoder PA0, PA1
TIM_HandleTypeDef htim3;  // Timer interrupt 10ms

// ================= THONG SO DONG CO GA25-370 =================
// GA25-370 12V 280RPM
// Encoder: 11 xung / 1 kenh / 1 vong motor
// Doc quadrature x4, ty so truyen 21.3:1
// PPR truc ra = 11 * 4 * 21.3 = 937.2
#define PULSE_PER_REV 937.2f
#define TS 0.01f
#define PWM_MAX 999.0f

// Motor thuc te cua ban: Manual PWM 999 dat khoang 250 RPM
// Nen gioi han LQG thap hon max that de dieu khien on dinh hon
#define RPM_SAFE_MAX 250.0f

volatile float current_rpm = 0.0f;

// LQG: bien nay la RPM muc tieu
volatile float setpoint_rpm = 0.0f;

// Manual: bien nay la PWM muc tieu
volatile float manual_pwm_cmd = 0.0f;

// ================= TRANG THAI DIEU KHIEN =================
// 0: Dung, 1: Quay thuan, -1: Quay nguoc
volatile int motor_direction = 1;

// 0: Manual PWM, 1: Auto LQG
volatile int pid_enable = 1;

// Giu lai de tranh loi link neu file usbd_cdc_if.c con tham chieu PID cu
float Kp = 5.0f;
float Ki = 0.1f;
float Kd = 0.01f;

int16_t pwm_output = 0;

// ================= THONG SO LQG =================
float LQG_A = 0.9621f;
float LQG_B = 0.0102f;
float LQG_C = 1.0f;

// Do loi Kalman Filter (L)
float LQG_L  = 0.3397f;

// Do loi LQR (Kx)
float LQG_Kx = 6.5793f;
float LQG_Ki = 0.80f;
float LQG_Kr = 1.00f;

float x_hat = 0.0f;
float x_pred = 0.0f;
float y_pred = 0.0f;
float integral_error = 0.0f;
float u_prev = 0.0f;

// Loc toc do encoder
float rpm_filtered = 0.0f;
float rpm_alpha = 0.10f;

// Ramp rieng cho LQG RPM va Manual PWM
float ramped_rpm_target = 0.0f;
float ramped_pwm_target = 0.0f;

float rpm_ramp_step = 3.0f;     // RPM moi 10ms
float pwm_ramp_step = 12.0f;    // PWM moi 10ms

// Gioi han toc do doi PWM cua LQG
float max_pwm_step = 12.0f;

// Bu ma sat tinh cho GA25-370 + L298N
int16_t pwm_min = 80;

char usb_tx_buffer[128];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
void Error_Handler(void);

float LQG_Compute(float target_speed, float measured_speed);
void Motor_Control_Compute(void);

/* USER CODE BEGIN 0 */
void USB_ForceReEnumeration(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_Delay(100);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12);
    HAL_Delay(50);
}
/* USER CODE END 0 */

/* =============================================================================
   HAM TINH LQG
   ============================================================================= */
float LQG_Compute(float target_speed, float measured_speed)
{
    float error;
    float control_signal;

    // 1. Du doan trang thai
    x_pred = LQG_A * x_hat + LQG_B * u_prev;

    // 2. Du doan ngo ra
    y_pred = LQG_C * x_pred;

    // 3. Observer / Kalman estimator
    x_hat = x_pred + LQG_L * (measured_speed - y_pred);

    // 4. Sai so toc do
    error = target_speed - x_hat;
    integral_error += error * TS;

    // 5. Anti-windup
    if (integral_error > 3000.0f) integral_error = 3000.0f;
    if (integral_error < -3000.0f) integral_error = -3000.0f;

    // 6. Luat dieu khien LQG/LQI
    control_signal = (LQG_Kr * target_speed)
                   - (LQG_Kx * x_hat)
                   + (LQG_Ki * integral_error);

    // 7. Gioi han PWM
    if (control_signal > PWM_MAX) control_signal = PWM_MAX;
    if (control_signal < -PWM_MAX) control_signal = -PWM_MAX;

    // 8. Gioi han toc do thay doi PWM
    // Bien max_pwm_step nay da dua ra global de GUI tuning duoc
    if (control_signal > u_prev + max_pwm_step)
    {
        control_signal = u_prev + max_pwm_step;
    }

    if (control_signal < u_prev - max_pwm_step)
    {
        control_signal = u_prev - max_pwm_step;
    }

    u_prev = control_signal;
    return control_signal;
}

/* =============================================================================
   HAM DIEU KHIEN DONG CO
   ============================================================================= */
void Motor_Control_Compute(void)
{
    static uint16_t prev_encoder_count = 0;
    static int prev_mode = 1;

    uint16_t current_encoder_count = __HAL_TIM_GET_COUNTER(&htim2);
    int16_t encoder_count = (int16_t)(current_encoder_count - prev_encoder_count);
    prev_encoder_count = current_encoder_count;

    // Tinh toc do truc ra RPM
    float raw_rpm = ((float)encoder_count / PULSE_PER_REV) * (60.0f / TS);

    // Loc toc do
    rpm_filtered = rpm_alpha * raw_rpm + (1.0f - rpm_alpha) * rpm_filtered;
    current_rpm = rpm_filtered;

    // ================= DONG BO KHI DOI MODE =================
    if (pid_enable != prev_mode)
    {
        if (pid_enable == 1)
        {
            // Manual -> LQG
            // LQG bat dau tu toc do hien tai de khong giat
            ramped_rpm_target = current_rpm;

            x_hat = current_rpm;
            x_pred = current_rpm;
            y_pred = current_rpm;
            u_prev = pwm_output;
        }
        else
        {
            // LQG -> Manual
            // Manual bat dau tu PWM hien tai de khong giat
            ramped_pwm_target = (float)pwm_output;
        }

        prev_mode = pid_enable;
    }

    // ================= CHE DO DUNG =================
    if (motor_direction == 0)
    {
        // Cho ca target RPM va PWM ve 0 mem
        if (ramped_rpm_target > 0.0f)
        {
            ramped_rpm_target -= rpm_ramp_step;
            if (ramped_rpm_target < 0.0f) ramped_rpm_target = 0.0f;
        }
        else if (ramped_rpm_target < 0.0f)
        {
            ramped_rpm_target += rpm_ramp_step;
            if (ramped_rpm_target > 0.0f) ramped_rpm_target = 0.0f;
        }

        if (ramped_pwm_target > 0.0f)
        {
            ramped_pwm_target -= pwm_ramp_step;
            if (ramped_pwm_target < 0.0f) ramped_pwm_target = 0.0f;
        }
        else if (ramped_pwm_target < 0.0f)
        {
            ramped_pwm_target += pwm_ramp_step;
            if (ramped_pwm_target > 0.0f) ramped_pwm_target = 0.0f;
        }

        if (fabs(ramped_rpm_target) < 1.0f && fabs(ramped_pwm_target) < 1.0f)
        {
            pwm_output = 0;
            x_hat = 0.0f;
            x_pred = 0.0f;
            y_pred = 0.0f;
            integral_error = 0.0f;
            u_prev = 0.0f;
        }
    }

    // ================= CHE DO LQG =================
    else if (pid_enable == 1)
    {
        // LQG: setpoint la RPM
        float final_rpm_target = setpoint_rpm;

        if (final_rpm_target > RPM_SAFE_MAX) final_rpm_target = RPM_SAFE_MAX;
        if (final_rpm_target < 0.0f) final_rpm_target = 0.0f;

        if (motor_direction == -1)
        {
            final_rpm_target = -final_rpm_target;
        }

        // Ramp RPM target
        if (ramped_rpm_target < final_rpm_target)
        {
            ramped_rpm_target += rpm_ramp_step;
            if (ramped_rpm_target > final_rpm_target) ramped_rpm_target = final_rpm_target;
        }
        else if (ramped_rpm_target > final_rpm_target)
        {
            ramped_rpm_target -= rpm_ramp_step;
            if (ramped_rpm_target < final_rpm_target) ramped_rpm_target = final_rpm_target;
        }

        float control_signal = LQG_Compute(ramped_rpm_target, current_rpm);

        int16_t pwm_base = (int16_t)control_signal;

        // Bu ma sat tinh cho GA25-370 qua L298N
				if (ramped_rpm_target > 5.0f)
				{
						pwm_output = pwm_base + pwm_min;
				}
				else if (ramped_rpm_target < -5.0f)
				{
						pwm_output = pwm_base - pwm_min;
				}
				else
				{
						pwm_output = pwm_base;
				}

        if (pwm_output > 999) pwm_output = 999;
        if (pwm_output < -999) pwm_output = -999;
    }

    // ================= CHE DO MANUAL PWM =================
    else
    {
        // Manual: setpoint GUI la PWM, khong phai RPM
        float final_pwm_target = manual_pwm_cmd;

        if (final_pwm_target > PWM_MAX) final_pwm_target = PWM_MAX;
        if (final_pwm_target < 0.0f) final_pwm_target = 0.0f;

        if (motor_direction == -1)
        {
            final_pwm_target = -final_pwm_target;
        }

        // Ramp PWM target
        if (ramped_pwm_target < final_pwm_target)
        {
            ramped_pwm_target += pwm_ramp_step;
            if (ramped_pwm_target > final_pwm_target) ramped_pwm_target = final_pwm_target;
        }
        else if (ramped_pwm_target > final_pwm_target)
        {
            ramped_pwm_target -= pwm_ramp_step;
            if (ramped_pwm_target < final_pwm_target) ramped_pwm_target = final_pwm_target;
        }

        pwm_output = (int16_t)ramped_pwm_target;

        if (pwm_output > 999) pwm_output = 999;
        if (pwm_output < -999) pwm_output = -999;

        // Bumpless transfer dung cong thuc:
        // u = Kr*r - Kx*x_hat + Ki*I
        // I = (u - Kr*r + Kx*x_hat) / Ki
        x_hat = current_rpm;
        x_pred = current_rpm;
        y_pred = current_rpm;

        float target_speed_manual = current_rpm;

        if (fabs(LQG_Ki) > 0.0001f)
        {
            integral_error = ((float)pwm_output
                            - (LQG_Kr * target_speed_manual)
                            + (LQG_Kx * x_hat)) / LQG_Ki;
        }
        else
        {
            integral_error = 0.0f;
        }

        if (integral_error > 3000.0f) integral_error = 3000.0f;
        if (integral_error < -3000.0f) integral_error = -3000.0f;

        u_prev = pwm_output;
    }

    // ================= XUAT RA L298N =================
    if (pwm_output >= 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_output);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -pwm_output);
    }
}

/* =============================================================================
   CALLBACK TIMER 10ms
   ============================================================================= */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        Motor_Control_Compute();
    }
}

/* =============================================================================
   MAIN
   ============================================================================= */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    USB_ForceReEnumeration();

    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_I2C1_Init();
    MX_USB_DEVICE_Init();

    HAL_Delay(500);

    lcd_init();
    lcd_put_cur(0, 0);
    lcd_send_string("LQG GA25-370");

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_MOE_ENABLE(&htim1);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Base_Start_IT(&htim3);

    while (1)
    {
        char lcd_buf[32];

        float display_target = (pid_enable == 1) ? setpoint_rpm : manual_pwm_cmd;

        sprintf(lcd_buf, "S:%d R:%d   ",
                (int)display_target,
                (int)fabs(current_rpm));

        lcd_put_cur(1, 0);
        lcd_send_string(lcd_buf);

        // Format GUI:
        // target_display,current_rpm,pwm_output,x_hat,mode,dir
        sprintf(usb_tx_buffer, "%d,%d,%d,%d,%d,%d\r\n",
                (int)display_target,
                (int)fabs(current_rpm),
                (int)pwm_output,
                (int)x_hat,
                pid_enable,
                motor_direction);

        uint8_t result = CDC_Transmit_FS((uint8_t*)usb_tx_buffer, strlen(usb_tx_buffer));

        if (result == USBD_OK)
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        }

        HAL_Delay(100);
    }
}

/* =============================================================================
   SYSTEM CLOCK CONFIG
   ============================================================================= */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                  RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* =============================================================================
   I2C1 INIT
   ============================================================================= */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* =============================================================================
   TIM1 PWM PA8 INIT
   ============================================================================= */
static void MX_TIM1_Init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 71;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 999;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
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
}

/* =============================================================================
   TIM2 ENCODER PA0 PA1 INIT
   ============================================================================= */
static void MX_TIM2_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

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
}

/* =============================================================================
   TIM3 INTERRUPT 10ms INIT
   ============================================================================= */
static void MX_TIM3_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 71;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 9999;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/* =============================================================================
   GPIO INIT
   ============================================================================= */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* =============================================================================
   ERROR HANDLER
   ============================================================================= */
void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}