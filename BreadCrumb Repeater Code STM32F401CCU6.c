/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for the LoRa Repeater Module
  ******************************************************************************
  * @attention
  *
  * This code implements a simple LoRa repeater. It listens for a specific
  * data packet from an upstream node (like a drone or another repeater),
  * validates it, and forwards it to the next node in the chain (another
  * repeater or the base station).
  *
  * Hardware: STM32F401CCU6 "Black Pill"
  * LoRa Module: SX1278 (e.g., Ra-02)
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LoRa.h" // <-- IMPORTANT: You must add an SX1278 LoRa library to your project.
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// This is the data structure for the LoRa communication packet.
// It MUST be identical on the drone, all repeaters, and the base station.
typedef struct {
    uint8_t   source_id;      // ID of the original sender or last repeater
    uint8_t   destination_id; // ID of the final destination (the base station)
    uint8_t   packet_type;    // Can be used to differentiate data types, e.g., 0x01 for Drone Data
    float     gas_value;      // Gas sensor reading from the drone
    float     pos_x;          // Drone's 3D plot X-coordinate
    float     pos_y;          // Drone's 3D plot Y-coordinate
    float     pos_z;          // Drone's 3D plot Z-coordinate
    uint8_t   checksum;       // Optional: for data integrity check (not implemented in this example)
} LoRa_Packet;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// --- NETWORK CONFIGURATION ---
// IMPORTANT: You must change these values for each repeater you program.
#define REPEATER_ID         0x01  // The unique ID for THIS repeater. Use 0x02 for the second, 0x03 for the third, etc.
#define UPSTREAM_NODE_ID    0x00  // The ID of the node this repeater LISTENS TO.
                                  // For Repeater 0x01, this is the Drone (0x00).
                                  // For Repeater 0x02, this would be Repeater 0x01.
#define BASE_STATION_ID     0xFF  // The final destination address for all data packets.

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1; // Assuming you are using SPI1

/* USER CODE BEGIN PV */
LoRa lora; // The LoRa object from the library.
LoRa_Packet rx_packet; // A buffer to store the received packet.
LoRa_Packet tx_packet; // A buffer for the packet to be transmitted.
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  // --- LORA MODULE INITIALIZATION ---
  // 1. Create a new LoRa object instance.
  lora = LoRa_new();

  // 2. Assign the STM32 hardware handles and pins to the LoRa object.
  //    These names (e.g., LORA_NSS_GPIO_Port) are defined in main.h by CubeIDE
  //    based on the "User Labels" you set in the IOC pin configuration.
  lora.hSPIx                 = &hspi1;
  lora.CS_port               = LORA_NSS_GPIO_Port;
  lora.CS_pin                = LORA_NSS_Pin;
  lora.RESET_port            = LORA_RESET_GPIO_Port;
  lora.RESET_pin             = LORA_RESET_Pin;
  lora.DIO0_port			   = LORA_DIO0_GPIO_Port; // Interrupt pin for RxDone
  lora.DIO0_pin				   = LORA_DIO0_Pin;

  // 3. Initialize the LoRa module with the configured settings.
  //    The LoRa_init function will communicate over SPI to set frequency,
  //    spreading factor, bandwidth, etc.
  uint16_t lora_status = LoRa_init(&lora);

  // 4. Check if initialization was successful.
  if (lora_status != LORA_OK) {
	  // If initialization fails, trap the MCU here.
	  // You can add an error indicator, like blinking an LED rapidly.
	  while(1) {
		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // Assuming LD2 is your board's LED
		  HAL_Delay(100);
	  }
  }

  // 5. Put the LoRa module into continuous receive mode.
  //    The repeater's primary job is to listen, so we start this right away.
  LoRa_startReceiving(&lora);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	// --- MAIN REPEATER LOGIC ---

	// 1. LISTEN: Poll the LoRa module to see if a new packet has arrived.
	// The LoRa_receive function checks the DIO0 pin or status registers.
	// It returns the number of bytes received if a packet is available.
	uint8_t bytes_received = LoRa_receive(&lora, (uint8_t*)&rx_packet, sizeof(LoRa_Packet));

	// 2. CHECK: Was a complete packet received?
	if (bytes_received == sizeof(LoRa_Packet)) {

		// A packet was received. Now, validate it.
		// 3. VALIDATE: Is this packet from the correct source and for the correct destination?
		// This prevents the repeater from forwarding stray or incorrect packets.
		if (rx_packet.source_id == UPSTREAM_NODE_ID && rx_packet.destination_id == BASE_STATION_ID) {

			// If we are here, the packet is valid and intended for our network.

			// Optional: Blink an LED to indicate a valid packet was received.
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

			// 4. PREPARE FOR FORWARDING:
			// Copy the received packet to the transmit buffer.
			tx_packet = rx_packet;
			// CRITICAL: Update the source_id to this repeater's ID.
			// This allows the base station to trace the path the data took.
			tx_packet.source_id = REPEATER_ID;

			// 5. FORWARD (TRANSMIT): Send the updated packet onward.
			// The LoRa_transmit function will put the module in TX mode, send the data,
			// and wait for the transmission to complete.
			LoRa_transmit(&lora, (uint8_t*)&tx_packet, sizeof(LoRa_Packet), 500); // 500ms timeout

			// 6. RETURN TO LISTEN MODE: After transmitting, immediately switch back
			// to receive mode to be ready for the next packet.
			LoRa_startReceiving(&lora);

			// Turn off the LED after forwarding is complete.
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		}
	}
	// If no packet is received, the loop continues, and we check again.
	// A HAL_Delay() is not needed here as the LoRa_receive function provides sufficient polling delay.

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; // Adjust as needed
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, LORA_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin; // This is an example, use your board's LED pin
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_NSS_Pin */
  GPIO_InitStruct.Pin = LORA_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LORA_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_RESET_Pin */
  GPIO_InitStruct.Pin = LORA_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_DIO0_Pin */
  GPIO_InitStruct.Pin = LORA_DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LORA_DIO0_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
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
