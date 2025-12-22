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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <ssd1306.h>
#include <ssd1306_fonts.h>
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
// Wheel Encoder
volatile int32_t left_wheel_count = 0; // Left wheel pulse count
volatile int32_t right_wheel_count = 0; // Right wheel pulse count
volatile int8_t left_backwards = 0; // Indicates if the left wheel is moving backward
volatile int8_t right_backwards = 0; // Indicates if the right wheel is moving backward

// USART Communication
#define RXBUFFERSIZE 12 // Buffer size for receiving the complete message
uint8_t Command_Message[RXBUFFERSIZE]; // Buffer to store received data

// UART ring buffer for streaming command parser
#define UART_RING_SIZE 50
#define UART_HEADER 0x0D
#define UART_FOOTER 0x20
#define FRAME_LEN 11
volatile uint8_t uart_ring[UART_RING_SIZE];
volatile uint16_t uart_head = 0; // next write index
volatile uint16_t uart_tail = 0; // next read index

// USART Transmission Message
char Encoder_Message[20];

// ADC Measurements
uint16_t ADCArray[2];  // Array to store ADC readings for voltage and current
// Sliding Window (Moving Average) for voltage and current
#define WINDOW_SIZE 50  // Number of readings to average
uint16_t adc_voltage_buffer[WINDOW_SIZE] = { 0 }; // Buffer for voltage readings
uint16_t adc_current_buffer[WINDOW_SIZE] = { 0 }; // Buffer for current readings
uint8_t adc_index = 0;  // Current index for buffer
// Smoothed ADC Values
uint16_t smoothed_ADCArray[2]; // Array to store the smoothed voltage and current values

// OLED Display
char OLED_Message[20]; // String buffer for formatted output on the OLED screen

// PCB Button
int button = 0; // Tracks button state (e.g. pressed or not)


int32_t left_cmd = 0;  // Motor command values
int32_t right_cmd = 0; // Motor command values
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Function to send encoder counts via USART
void Send_Encoder_Counts(void) {
	int len = sprintf(Encoder_Message, "[%ld,%ld]\r\n",
			(int32_t) __HAL_TIM_GET_COUNTER(&htim2),
			(int32_t) __HAL_TIM_GET_COUNTER(&htim5));
	HAL_UART_Transmit_IT(&huart2, (uint8_t*) Encoder_Message, len);
}

// Remote control
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        // Append the newly received byte (stored in Command_Message[0]) into the ring buffer
        uint8_t b = Command_Message[0];
        uart_ring[uart_head] = b;
        uart_head = (uart_head + 1) % UART_RING_SIZE;
        // If buffer overflow, advance tail to drop oldest byte
        if (uart_head == uart_tail) {
            uart_tail = (uart_tail + 1) % UART_RING_SIZE;
        }

        // Try to parse as many complete frames as possible
        uint16_t count = (uart_head + UART_RING_SIZE - uart_tail) % UART_RING_SIZE;
        uint16_t offset = 0; // offset from current tail within available bytes
        // Use a temporary linear view to simplify parsing logic
        while ((count - offset) >= FRAME_LEN) {
            // Read potential header at (tail + offset)
            uint16_t idx = (uart_tail + offset) % UART_RING_SIZE;
            if (uart_ring[idx] != UART_HEADER) {
                // not a header, skip this byte
                offset++;
                continue;
            }
            // Ensure footer is in place at header+10
            uint16_t footer_idx = (idx + FRAME_LEN - 1) % UART_RING_SIZE; // header + 10
            if (uart_ring[footer_idx] != UART_FOOTER) {
                // footer mismatch -> this header is invalid; skip this header byte
                offset++;
                continue;
            }

            // We have a candidate valid frame; copy payload bytes (big-endian)
            uint8_t payload[9];
            for (int i = 0; i < 9; ++i) {
                payload[i] = uart_ring[(idx + 1 + i) % UART_RING_SIZE];
            }

            // Decode 16-bit big-endian values
            uint32_t m1 = ((uint32_t)payload[0] << 8) | payload[1];
            uint32_t m2 = ((uint32_t)payload[2] << 8) | payload[3];
            uint32_t s1 = ((uint32_t)payload[4] << 8) | payload[5];
            uint32_t s2 = ((uint32_t)payload[6] << 8) | payload[7];
            uint8_t dir = payload[8];


            // Clamp servo values to safe range 1000..1400 (per request)
            if (s1 < 1000U) s1 = 1000U;
            else if (s1 > 1400U) s1 = 1400U;
            if (s2 < 1000U) s2 = 1000U;
            else if (s2 > 1400U) s2 = 1400U;

            // Map directions: bit0 -> motor1, bit1 -> motor2. Per requirement: 0 = backward, 1 = forward
            // ASSUMPTION: motor1 maps to left motor (htim12 channels), motor2 maps to right motor (htim4 channels).
            left_cmd = (dir & 0x01) ? (int32_t)m1 : -(int32_t)m1;
            right_cmd = (dir & 0x02) ? (int32_t)m2 : -(int32_t)m2;

            // Apply motors and servos
            motor(left_cmd, right_cmd);

            // Servos: directly set TIM8 compare registers
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (uint32_t)s1);
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, (uint32_t)s2);

            // Advance offset past this frame
            offset += FRAME_LEN;
        }

        // Advance tail by offset (discard parsed or skipped bytes)
        uart_tail = (uart_tail + offset) % UART_RING_SIZE;

        // Restart reception for next byte
        HAL_UART_Receive_IT(&huart2, (uint8_t*) Command_Message, 1);
    }
}
// PCB button interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == SW2_Pin) // Handle SW2 (PB12)
	{

	} else if (GPIO_Pin == SW3_Pin) // Handle SW3 (PB13)
	{

	}
}
// Motor control function
void motor(int32_t left, int32_t right) {
	// Clamp the values to be within -65535 and +65535
	left = fminf(fmaxf(left, -65535), 65535);
	right = fminf(fmaxf(right, -65535), 65535);

	// Handle the right wheel
	if (right >= 0) {
		// Forward
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0); // Backward
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, right); // Forward
	} else {
		// Backward
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, -right); // Backward
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0); // Forward
	}

	// Handle the left wheel
	if (left >= 0) {
		// Forward
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0); // Backward
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, left); // Forward
	} else {
		// Backward
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, -left); // Backward
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0); // Forward
	}
}
// Timer interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// 10ms timer to send our the encoder counts
	if (htim->Instance == TIM13) {
		Send_Encoder_Counts();
	}
}
// Callback function of SysTick
void HAL_SYSTICK_Callback(void) {
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4); // 1ms toggle pin
}
// ADC Callback
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1)  // Check which ADC triggered the interrupt
			{
		// Update the ADC buffers with the new readings from DMA
		adc_voltage_buffer[adc_index] = ADCArray[0]; // Store voltage ADC reading
		adc_current_buffer[adc_index] = ADCArray[1]; // Store current ADC reading

		// Move the index forward, wrapping around when reaching the buffer size
		adc_index = (adc_index + 1) % WINDOW_SIZE;

		// Calculate the moving average for voltage
		uint32_t voltage_sum = 0;
		for (int i = 0; i < WINDOW_SIZE; i++) {
			voltage_sum += adc_voltage_buffer[i];
		}
		smoothed_ADCArray[0] = voltage_sum / WINDOW_SIZE; // Store smoothed voltage

		// Calculate the moving average for current
		uint32_t current_sum = 0;
		for (int i = 0; i < WINDOW_SIZE; i++) {
			current_sum += adc_current_buffer[i];
		}
		smoothed_ADCArray[1] = current_sum / WINDOW_SIZE; // Store smoothed current
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_TIM4_Init();
	MX_TIM2_Init();
	MX_TIM5_Init();
	MX_TIM12_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_TIM13_Init();
	MX_TIM8_Init();
	/* USER CODE BEGIN 2 */

	// Start right PWM channels
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	// Start left PWM channels
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

	// Start PWM on TIM8 CH1 and CH2
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	// Set servo angles (e.g., 90 degrees for both)
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 1000);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 1400);

	// Example on count encoder pulses using interrupts
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1 | TIM_CHANNEL_2);

	// Start UART reception in interrupt mode
-    HAL_UART_Receive_IT(&huart2, (uint8_t*) Command_Message, RXBUFFERSIZE); // UASRT 2
+    // Use single-byte interrupt-driven reception and a software ring buffer for framing
+    HAL_UART_Receive_IT(&huart2, (uint8_t*) Command_Message, 1); // USART2 single-byte receive

	// ADC Values for voltage and current
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADCArray, 2);

	HAL_TIM_Base_Start_IT(&htim13); // Encoder Message

	HAL_ADC_Start_IT(&hadc1); // ADC interrupt handler

	ssd1306_Init();
	ssd1306_Fill(White);
	ssd1306_UpdateScreen();
	ssd1306_Fill(Black);

	ssd1306_SetCursor(0, 0);  // Set cursor to an appropriate position
	ssd1306_WriteString("3360 251221", Font_11x18, White);
	ssd1306_UpdateScreen();

	HAL_Delay(1000);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		// Clear the display
		ssd1306_Fill(Black);

		// Format and display the ADC voltage and current values
		// ADCArray[0]*(3.3/4096)*((4700+1000)/1000)
		// ((ADCArray[1]/4096*3300)-2500)/185
		sprintf(OLED_Message, "%.1fV %s%.3fA",
				(smoothed_ADCArray[0] * 0.00459228515 + 0.22), // Voltage calculation
				(smoothed_ADCArray[0] * 0.00459228515 + 0.22) < 7.5 ?
						"TooLow" : "", // Check if voltage is lower than 7.5V
				((smoothed_ADCArray[1] * 0.8 - 2500) / 185) < 0 ?
						0 : ((smoothed_ADCArray[1] * 0.8 - 2500) / 185)); // Current calculation

		ssd1306_SetCursor(0, 0);  // Set cursor to the top of the display
		ssd1306_WriteString(OLED_Message, Font_11x18, White);

		sprintf(OLED_Message, "L:%d%s", __HAL_TIM_GET_COUNTER(&htim2),
				__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2) ? "B" : "F");
		ssd1306_SetCursor(0, 15);
		ssd1306_WriteString(OLED_Message, Font_11x18, White);

		sprintf(OLED_Message, "R:%d%s", __HAL_TIM_GET_COUNTER(&htim5),
				__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim5) ? "B" : "F");
		ssd1306_SetCursor(0, 30);
		ssd1306_WriteString(OLED_Message, Font_11x18, White);

		//sprintf(buffer, "%s", );
		ssd1306_SetCursor(0, 45);
		ssd1306_WriteString(OLED_Message, Font_11x18, White);

		// Update the OLED screen to reflect all changes
		ssd1306_UpdateScreen();

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
