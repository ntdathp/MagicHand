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

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t ActiveServo;

char uart_rx_buffer[256];
char *buffer_ptr = uart_rx_buffer;
uint8_t received_byte;
uint8_t uart_received_flag = 0;

uint8_t mode_flag = 0;

volatile uint8_t button_pressed = 0;
uint32_t button_press_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_short(volatile uint32_t delay)
{
    while (delay--)
    {
        // Vòng lặp trống tạo độ trễ ngắn
    }
}
void led_chasing(void)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	                delay_short(100000);  // Điều chỉnh giá trị để thay đổi độ trễ
	                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

	                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	                delay_short(100000);
	                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

	                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	                delay_short(100000);
	                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

	                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
	                delay_short(100000);
	                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
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

void process_received_data()
{
    if (uart_received_flag)
    {
        uart_received_flag = 0;  // Xóa cờ
        int finger_index = 0;    // Chỉ số ngón tay (0 đến 4)
        int value = -1;          // Giá trị điều khiển (1: lên, 0: xuống)

        if (mode_flag == 0)
        {
            // Duyệt qua từng ký tự trong chuỗi nhận được
            for (char *ptr = uart_rx_buffer; *ptr != '\0' && finger_index < 5; ptr++)
            {
                if (*ptr == '0' || *ptr == '1') // Kiểm tra ký tự là '0' hoặc '1'
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
                    finger_index++;  // Tăng chỉ số ngón tay
                }
            }
        }
        else
        {
            int all_zeros = 1;  // Cờ kiểm tra nếu toàn là '0'
            int all_ones = 1;   // Cờ kiểm tra nếu toàn là '1'
            int match_pattern = 1; // Cờ kiểm tra định dạng 0,1,1,0,0

            // Chuỗi mẫu 0,1,1,0,0
            const char *pattern = "0,1,1,0,0";

            // Lặp qua từng ký tự trong chuỗi
            for (int i = 0; uart_rx_buffer[i] != '\0'; i++)
            {
                if (uart_rx_buffer[i] == '0')
                {
                    all_ones = 0;  // Nếu có '0', không thể là toàn '1'
                }
                else if (uart_rx_buffer[i] == '1')
                {
                    all_zeros = 0;  // Nếu có '1', không thể là toàn '0'
                }
                else if (uart_rx_buffer[i] != ',')
                {
                    // Nếu có ký tự không phải '0', '1', hoặc dấu phẩy, thì không hợp lệ
                    return;
                }

                // Kiểm tra xem chuỗi có khớp với mẫu không
                if (uart_rx_buffer[i] != pattern[i])
                {
                    match_pattern = 0;  // Nếu có ký tự không khớp mẫu, đặt cờ thành 0
                }
            }

            // Kiểm tra các trường hợp
            if (all_zeros)
            {
                // Trường hợp toàn là '0'
                // Thực hiện hành động cho trường hợp toàn 0
                // Ví dụ:
            	finger0_up;
            	finger1_up;
            	finger2_up;
            	finger3_up;
            	finger4_up;
            }
            else if (all_ones)
            {
                // Trường hợp toàn là '1'
                // Thực hiện hành động cho trường hợp toàn 1
                // Ví dụ:
            	finger0_down();
            	finger1_up;
            	finger2_up;
            	finger3_down();
            	finger4_down();

            }
            else if (match_pattern)
            {
                // Trường hợp khớp với mẫu "0,1,1,0,0"
                // Thực hiện hành động cho trường hợp này
            	finger0_down();
            	finger1_down();
            	finger2_down();
            	finger3_down();
            	finger4_down();
            }
            else
            {
                // Trường hợp khác không hợp lệ, bỏ qua
                printf("Buffer does not match any condition.\n");
                return;
            }
        }

        // Đặt lại nội dung mảng để sẵn sàng nhận chuỗi mới
        memset(uart_rx_buffer, 0, sizeof(uart_rx_buffer));
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0)  // Kiểm tra nếu ngắt từ PA0
    {
    	 button_pressed = 1;             // Đặt cờ báo hiệu nút đã nhấn
    	  button_press_time = HAL_GetTick();  // Ghi nhận thời gian nhấn

    }
}

void check_button_press()
{
    if (button_pressed)
    {
        // Kiểm tra nếu đã qua thời gian chống rung (ví dụ: 50ms)
        if ((HAL_GetTick() - button_press_time) > 10)
        {
            // Thực hiện hiệu ứng LED chạy
            for (int i = 0; i < 3; i++)  // Số lần lặp lại hiệu ứng
            {
                // Bật đèn LED lần lượt trên các chân PD12 - PD15
            	led_chasing();
            }

            // Thay đổi trạng thái của `mode_flag`
            mode_flag ^= 1;
            if (mode_flag)
            	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET);

            // Reset cờ sau khi xử lý
            button_pressed = 0;
        }
    }
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
//  PCA9685_Init(&hi2c1);
//  PCA9685_SetServoAngle(0, 180); //120 -180
//  PCA9685_SetServoAngle(1, 180); //180 -50
//  PCA9685_SetServoAngle(2, 180);   //180-0
//  PCA9685_SetServoAngle(3, 180);   //180-40
//  PCA9685_SetServoAngle(4, 180);   //180-30

//  HAL_Delay(2000);

  HAL_UART_Receive_IT(&huart1, &received_byte, 1);
  led_chasing();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  process_received_data();
	  check_button_press();


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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (received_byte != '\n')
        {
            *buffer_ptr++ = received_byte;  // Lưu byte vào vị trí hiện tại của con tr�? và tăng con tr�?
        }
        else
        {
            *buffer_ptr = '\0';             // Kết thúc chuỗi bằng ký tự null
            uart_received_flag = 1;         // �?ặt c�? báo hiệu chuỗi đã nhận hoàn chỉnh
            buffer_ptr = uart_rx_buffer;    // �?ặt lại con tr�? v�? đầu mảng cho lần nhận tiếp theo
        }

        // Tiếp tục nhận byte tiếp theo
        HAL_UART_Receive_IT(&huart1, &received_byte, 1);
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
