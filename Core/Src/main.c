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

static BMI088_t		imu_data;

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

static DMMotorInstance *pitch_motor;

float current_angle ;
float current_velocity ;
float current_torque ;
float current_deg ;

/* 位置模式参数（在 debug 里实时修改调参） */
float K_p = 0.0f;    // 初始值，根据实测调整
float K_d = 0.0f;   // 初始值，根据实测调整
float Target_angle = 0.0f;

#define SWEEP_START_DEG     (-30.0f)   // 起始角度
#define SWEEP_END_DEG       ( 20.0f)   // 终止角度
#define SWEEP_STEP_DEG      (  2.0f)   // 每步步长
#define SWEEP_SETTLE_COUNT  (500)      // 每个角度等待稳定的循环次数（500×2ms=500ms）
#define TORQUE_AVG_COUNT    (50)       // 稳定后取最后50次力矩均值
#define MAX_SWEEP_CYCLES    (30)        // 需要循环的次数
#define TORQUE_STABLE_THRESHOLD  0.005f  // 力矩稳定判定阈值 (Nm)，可调

typedef enum {
    SWEEP_IDLE    = 0,  // 空闲（不自动扫，手动控制 Target_angle）
    SWEEP_RUNNING = 1,  // 正在自动扫角
    SWEEP_DONE    = 2,  // 完成扫描
} SweepState_e;



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
 
        DMMotorSendPosVelTorque(pitch_motor,
                                 Target_angle,
                                 0.0f,
                                 K_p,
                                 K_d,
                                 0.0f);
	  
//		DMMotorSetTorque(pitch_motor, 1.0f);   // 给5Nm
//		DMMotorSendTorque(pitch_motor);
 
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
