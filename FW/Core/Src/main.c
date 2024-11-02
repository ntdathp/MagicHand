/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pca9685.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVO_COUNT	5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
uint8_t ActiveServo;
char uart_rx_buffer[256];
char *buffer_ptr = uart_rx_buffer;
uint8_t received_byte;
uint8_t uart_received_flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void process_received_data()
	  {
	      if (uart_received_flag)
	      {
	          uart_received_flag = 0;  // Xóa cờ

	          int finger_index = 0;  // Chỉ số ngón tay (0 đến 4)
	          int value = -1;        // Giá trị điều khiển (1: lên, 0: xuống)

	          // Duyệt qua từng ký tự trong chuỗi nhận được
	          for (char *ptr = uart_rx_buffer; *ptr != '\0'; ptr++)
	          {
	              if (*ptr >= '0' && *ptr <= '1') // Kiểm tra ký tự là '0' hoặc '1'
	              {
	                  value = *ptr - '0';  // Chuyển ký tự thành số nguyên

	                  // Điều khiển servo dựa trên giá trị
	                  if (value == 1)
	                  {
	                      switch (finger_index)
	                      {
	                          case 0: finger0_up(); break;
	                          case 1: finger1_up(); break;
	                          case 2: finger2_up(); break;
	                          case 3: finger3_up(); break;
	                          case 4: finger4_up(); break;
	                      }
	                  }
	                  else if (value == 0)
	                  {
	                      switch (finger_index)
	                      {
	                          case 0: finger0_down(); break;
	                          case 1: finger1_down(); break;
	                          case 2: finger2_down(); break;
	                          case 3: finger3_down(); break;
	                          case 4: finger4_down(); break;
	                      }
	                  }

	                  finger_index++; // Tăng chỉ số ngón tay

	                  // Kiểm tra nếu đã xử lý đủ 5 ngón tay
	                  if (finger_index >= 5)
	                  {
	                      break;
	                  }
	              }
	          }

	          // Đặt lại nội dung mảng để sẵn sàng nhận chuỗi mới
	          for (int i = 0; i < sizeof(uart_rx_buffer); i++)
	          {
	              uart_rx_buffer[i] = '\0';
	          }
	      }
	  }
void finger0_down(void)
{
	PCA9685_SetServoAngle(0, 120);
};

void finger0_up(void)
{
	PCA9685_SetServoAngle(0, 180);
};

void finger1_down(void)
{
	PCA9685_SetServoAngle(1, 50);
};

void finger1_up(void)
{
	PCA9685_SetServoAngle(1, 180);
};

void finger2_down(void)
{
	PCA9685_SetServoAngle(2, 0);
};

void finger2_up(void)
{
	PCA9685_SetServoAngle(2, 180);
};

void finger3_down(void)
{
	PCA9685_SetServoAngle(3, 40);
};

void finger3_up(void)
{
	PCA9685_SetServoAngle(3, 180);
};

void finger4_down(void)
{
	PCA9685_SetServoAngle(4, 30);
};

void finger4_up(void)
{
	PCA9685_SetServoAngle(4, 180);
};

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
  MX_I2C1_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  PCA9685_Init(&hi2c1);
  PCA9685_SetServoAngle(0, 180); //120 -180
  PCA9685_SetServoAngle(1, 180); //180 -50
  PCA9685_SetServoAngle(2, 180);   //180-0
  PCA9685_SetServoAngle(3, 180);   //180-40
  PCA9685_SetServoAngle(4, 180);   //180-30

  HAL_Delay(2000);

  HAL_UART_Receive_IT(&huart4, &received_byte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  process_received_data();

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)
    {
        if (received_byte != '\n')
        {
            *buffer_ptr++ = received_byte;  // Lưu byte vào vị trí hiện tại của con trỏ và tăng con trỏ
        }
        else
        {
            *buffer_ptr = '\0';             // Kết thúc chuỗi bằng ký tự null
            uart_received_flag = 1;         // Đặt cờ báo hiệu chuỗi đã nhận hoàn chỉnh
            buffer_ptr = uart_rx_buffer;    // Đặt lại con trỏ về đầu mảng cho lần nhận tiếp theo
        }

        // Tiếp tục nhận byte tiếp theo
        HAL_UART_Receive_IT(&huart4, &received_byte, 1);
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

#ifdef  USE_FULL_ASSERT
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
