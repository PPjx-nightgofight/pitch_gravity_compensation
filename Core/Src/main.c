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
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "bsp_dwt.h"
#include "dmmotor.h"
#include "bmi088_simple.h"
#include "bsp_usart.h"
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static BMI088_t		imu_data;
static DMMotorInstance *pitch_motor;

float current_angle ;
float current_velocity ;
float current_torque ;
float current_deg ;

/* 位置模式参数（在 debug 里实时修改调参） */
float K_p = 7.0f;    // 初始值，根据实测调整
float K_d = 0.25f;   // 初始值，根据实测调整
float Target_angle = 0.0f;

#define SWEEP_START_DEG     (-30.0f)   // 起始角度
#define SWEEP_END_DEG       ( 20.0f)   // 终止角度
#define SWEEP_STEP_DEG      (  2.0f)   // 每步步长
#define SWEEP_SETTLE_COUNT  (500)      // 每个角度等待稳定的循环次数（500×2ms=500ms）
#define TORQUE_AVG_COUNT    (50)       // 稳定后取最后50次力矩均值
#define MAX_SWEEP_CYCLES    (30)        // 需要循环的次数
#define TORQUE_STABLE_THRESHOLD  0.005f  // 力矩稳定判定阈值 (Nm)，可调

//重力矩参数
#define GRAVITY_COMP_A1  -0.4369f   // cos系数
#define GRAVITY_COMP_A2  -0.2728f   // sin系数
#define GRAVITY_COMP_B   -0.0163f   // 结构偏置 (Nm)
#define COULOMB_FRICTION 0.0903f   // 库仑摩擦力 (Nm)
#define FRICTION_VEL_THRESHOLD  0.1f // 死区阈值 (rad/s)，低于此速度摩擦线性过渡


typedef enum {
    SWEEP_IDLE    = 0,  // 空闲（不自动扫，手动控制 Target_angle）
    SWEEP_RUNNING = 1,  // 正在自动扫角
    SWEEP_DONE    = 2,  // 完成扫描
} SweepState_e;

/* 在 debug 里把这个改成 1 启动扫描，完成后自动变成 2 */
volatile SweepState_e sweep_state = SWEEP_IDLE;

/* ============================================================
 *  重力矩，摩擦力前馈计算函数
 * ============================================================ */
/**
 * @brief 计算重力+摩擦的综合前馈力矩
 * @param pitch_rad  当前实际角度 (rad弧度)
 * @param velocity   当前电机角速度 (rad/s)，来自 measure.velocity
 * @return 前馈力矩 (Nm)
 *
 * 考虑到质心与机械水平存在一个差角，更新了算法使用
 * 残差R²为0.8276 (越接近1越好)，较之前0.81...有实际优化
 */
static float ComputeFeedforward(float pitch_rad, float velocity)
{
    float gravity_ff = GRAVITY_COMP_A1 * cosf(pitch_rad)
                     + GRAVITY_COMP_A2 * sinf(pitch_rad)
                     + GRAVITY_COMP_B;

    float v_abs = fabsf(velocity);
	
    float friction_ff;
	
    if (v_abs >= FRICTION_VEL_THRESHOLD)
        friction_ff = (velocity > 0.0f ? 1.0f : -1.0f) * COULOMB_FRICTION;
    else
        friction_ff = (velocity > 0.0f ? 1.0f : -1.0f)
                    * COULOMB_FRICTION * (v_abs / FRICTION_VEL_THRESHOLD);

	//gravity_ff + friction_ff
    return gravity_ff + friction_ff ;
}


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

  DWT_Init(168);
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_TIM10_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  
  
  
  uint8_t bmi_status = BMI088_Init(&imu_data);
  
  if (bmi_status != 0)
  {
      
      while(1)
      {
       
      }
  }
  
  BMI088_CalibrateGyro(&imu_data);
  
  DMMotor_Init_Config_s pitch_conf = {
        .can_handle = &hcan1,
        .tx_id      = 0x02,
        .rx_id      = 0xF2,  // �����λ�� Master ID
    };
    pitch_motor = DMMotorInit(&pitch_conf);
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/* ---- 1. 读取 IMU ---- */
        BMI088_ReadSensor(&imu_data);
        BMI088_UpdateAttitude(&imu_data);
 
        /* ---- 2. 读取电机反馈 ---- */
        current_angle    = pitch_motor->measure.total_angle;
        current_velocity = pitch_motor->measure.velocity;
        current_torque   = pitch_motor->measure.torque;
        current_deg      = pitch_motor->measure.position * 57.29578f;
 
	  float gravity_data = ComputeFeedforward( pitch_motor->measure.position , pitch_motor->measure.velocity );
        
  
//两种方法
//1，直接力控的
//2，位置速度模式里面的补偿
	  
/****
*第一种
*****/

DMMotorSetTorque( pitch_motor , gravity_data );
DMMotorSendTorque( pitch_motor );
	  
/****
*第二种
*****/
//        DMMotorSendPosVelTorque(pitch_motor,
//                                 Target_angle,
//                                 0.0f,
//                                 0.0f,
//                                 0.0f,
//                                 gravity_data);
 
        DWT_Delay_ms(2); // 2ms 控制周期
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
