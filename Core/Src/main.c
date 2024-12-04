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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Define sensor thresholds
#define SENSOR_THRESHOLD 2000
#define RED_THRESHOLD    150   // Adjust for the detected red level
#define GREEN_THRESHOLD  150   // Adjust for the detected green level
#define BLUE_THRESHOLD   150   // Adjust for the detected blue level

// Color sensor address (e.g. TCS34725)
#define COLOR_SENSOR_ADDRESS TCS3200 // what is our color sensor's I2C address
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Global handles
ADC_HandleTypeDef hadc1;       // Handle for ADC1 to read IR sensor values
TIM_HandleTypeDef htim2;       // Handle for Timer 2 to control PWM
I2C_HandleTypeDef hi2c1;       // Handle for I2C1 to communicate with the color sensor

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_TIM2_Init(void);
void MX_ADC1_Init(void);
void MX_I2C1_Init(void);

uint32_t readIRSensor(ADC_HandleTypeDef *hadc, uint32_t channel);
void readColorSensor(uint16_t *red, uint16_t *green, uint16_t *blue);
void moveForward(void);
void turnLeft(void);
void turnRight(void);
void stopMotors(void);
void solveMaze(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM2_Init();                // Initialize Timer 2 for PWM
  MX_ADC1_Init();                // Initialize ADC1 for IR sensors
  MX_I2C1_Init();                // Initialize I2C1 for color sensor communication
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  // Start LEFT_MOTOR_PWM
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);  // Start RIGHT_MOTOR_PWM
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	 solveMaze();

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * Reads an analog value from a specified ADC channel for IR sensors.
 * hadc: Pointer to ADC handle (e.g., hadc1)
 * channel: ADC channel to read (e.g., ADC_CHANNEL_0)
 * Analog value from the specified channel
 */
uint32_t readIRSensor(ADC_HandleTypeDef *hadc, uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = channel;                  // Specify the ADC channel
    sConfig.Rank = ADC_REGULAR_RANK_1;          // Set rank for single conversion
    HAL_ADC_ConfigChannel(hadc, &sConfig);      // Apply configuration

    HAL_ADC_Start(hadc);                        // Start ADC conversion
    HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY); // Wait for conversion to complete
    uint32_t value = HAL_ADC_GetValue(hadc);    // Retrieve the ADC value
    HAL_ADC_Stop(hadc);                         // Stop the ADC

    return value;                               // Return the sensor value
}

/**
 * Reads RGB values from a color sensor via I2C.
 * red: Pointer to store the red value
 * green: Pointer to store the green value
 * blue: Pointer to store the blue value
 */
void readColorSensor(uint16_t *red, uint16_t *green, uint16_t *blue) {
    uint8_t buffer[6] = {0};

    // Read 6 bytes of RGB data from the color sensor
    HAL_I2C_Mem_Read(&hi2c1, COLOR_SENSOR_ADDRESS, 0x14 | 0x80, I2C_MEMADD_SIZE_8BIT, buffer, 6, HAL_MAX_DELAY);

    *red = (buffer[1] << 8) | buffer[0];    // Combine high and low bytes for red
    *green = (buffer[3] << 8) | buffer[2];  // Combine high and low bytes for green
    *blue = (buffer[5] << 8) | buffer[4];   // Combine high and low bytes for blue
}

/**
 * Moves the robot forward.
 */
void moveForward() {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  // LEFT_MOTOR_DIR: Forward
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);  // RIGHT_MOTOR_DIR: Forward

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 200);   // LEFT_MOTOR_PWM
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 200);   // RIGHT_MOTOR_PWM
}

/**
 * Turns the robot left.
 */
void turnLeft() {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // LEFT_MOTOR_DIR: Reverse
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);   // RIGHT_MOTOR_DIR: Forward

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 200);    // LEFT_MOTOR_PWM
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 200);    // RIGHT_MOTOR_PWM
    HAL_Delay(500);  // Delay for turning duration (adjust as needed)
}

/**
 * Turns the robot right.
 */
void turnRight() {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);   // LEFT_MOTOR_DIR: Forward
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // RIGHT_MOTOR_DIR: Reverse

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 200);    // LEFT_MOTOR_PWM
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 200);    // RIGHT_MOTOR_PWM
    HAL_Delay(500);  // Delay for turning duration (adjust as needed)
}

/**
 * Stops both motors.
 */
void stopMotors() {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);  // Stop LEFT_MOTOR_PWM
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);  // Stop RIGHT_MOTOR_PWM
}

/**
 * Implements a simple maze-solving algorithm using IR sensors and a color sensor.
 */
void solveMaze() {
    uint16_t red, green, blue;

    while (1) {
        // Read sensor values
        uint32_t front = readIRSensor(&hadc1, ADC_CHANNEL_0);  // Front IR sensor
        uint32_t left = readIRSensor(&hadc1, ADC_CHANNEL_1);   // Left IR sensor
        uint32_t right = readIRSensor(&hadc1, ADC_CHANNEL_2);  // Right IR sensor
        readColorSensor(&red, &green, &blue);                 // Read color sensor

        // Check color sensor values for specific actions (e.g., stop on red)
        if (red > RED_THRESHOLD && green < GREEN_THRESHOLD && blue < BLUE_THRESHOLD) {
            stopMotors();  // Stop on red surface
            HAL_Delay(2000);  // Pause for 2 seconds
        } else if (front < SENSOR_THRESHOLD) {
            moveForward();  // No wall ahead, move forward
        } else if (left < SENSOR_THRESHOLD) {
            turnLeft();     // Wall ahead, no wall to the left, turn left
        } else if (right < SENSOR_THRESHOLD) {
            turnRight();    // Wall ahead, no wall to the right, turn right
        } else {
            turnRight();    // Dead end, perform a U-turn
            turnRight();
        }

        HAL_Delay(100);  // Delay for smoother movements (adjust as needed)
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
