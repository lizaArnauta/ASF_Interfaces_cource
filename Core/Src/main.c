/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usb_device.h"
#include "rng.h"



/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ILI9341/ILI9341_GFX.h"
#include "ILI9341/ILI9341_STM32_Driver.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#include "string.h"
#include "stdbool.h"
#include "stdlib.h"
#include "stdio.h"

#define BUFFER_SIZE 512
#define NMEA_MAX_SENTENCE_SIZE (uint8_t)82u
#define NMEA_SENTENCE_CRC_START '*'
#define NMEA_SENTENCE_END "\r\n"
#define NMEA_G "$GNRMC"
#define STAT_RECORDS_COUNT 128u
#define NMEA_SENTENCE_COUNT 128u
#define TERM_UTC_OUTPUT_FORMAT "\r\nUTC time: %s\r\n"
#define TERM_DATE_OUTPUT_FORMAT "\r\nCurrent date: %s\r\n"
#define TERM_STATUS_OUTPUT_FORMAT "\r\nStatus: %c\r\n"
#define TERM_POS_OUTPUT_FORMAT "\r\nLat: %s, %c; Lon: %s, %c\r\n"


#define VERSION_STR "\r\nVersion: 0.0.1, build time: 9/18/2025, UART baudrate: 9600, USB speed: FS\r\n"


#define TERM_CMD_VERSION "VER"
#define TERM_CMD_GET_POS "GET POS"
#define TERM_CMD_GET_TIME "GET TIME"
#define TERM_CMD_GET_DATE "GET DATE"
#define TERM_CMD_GET_STATUS "GET STATUS"
#define TERM_CMD_RAW_ON "RAW ON"
#define TERM_CMD_RAW_OFF "RAW OFF"


uint8_t usb_term_buffer[BUFFER_SIZE];
uint8_t usb_buffer_index = 0;
uint32_t usb_received_len = 0;

uint8_t dma_rx_buffer[BUFFER_SIZE];
uint16_t data_size = 0;
_Bool is_received_new_data = false;
_Bool is_raw_on = true;
char utc[16] = "";
char year[16] = "";
char status_A_V = '-';
char lon[16] = "";
char lon_stat = '-';
char lat[16] = "";
char lat_stat = '-';
uint32_t uid = 0U;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

char str_data[BUFFER_SIZE + 1u] = {0};

typedef struct
{
	uint16_t received_bytes;
} STAT_record_t;

typedef struct
{
	STAT_record_t records[STAT_RECORDS_COUNT];
	uint16_t write_index;
	uint16_t records_count;
	uint16_t ne_count;
	uint16_t ore_count;
	uint16_t fe_count;
	uint16_t pe_count;
} STAT_t;

STAT_t statistic = {0};

void STAT_AddRecord(uint16_t bytes_count)
{
	if (statistic.write_index >= STAT_RECORDS_COUNT)
	{
		statistic.write_index = 0;
		statistic.records[statistic.write_index].received_bytes = bytes_count;

	}
	else
	{
		statistic.records[statistic.write_index].received_bytes = bytes_count;
		statistic.write_index++;
	}
	statistic.records_count++;
}

typedef struct
{
	uint8_t packet[NMEA_MAX_SENTENCE_SIZE];
	uint8_t packet_crc;
	uint8_t calculated_crc;
	_Bool is_crc_valid;
} NMEA_sentence_t;


typedef struct
{
	NMEA_sentence_t sentences[NMEA_SENTENCE_COUNT];
	uint64_t sentence_count;
	uint8_t write_index;
	uint8_t read_index;
} NMEA_que_t;


typedef struct
{
	NMEA_que_t que;
} NMEA_t;

NMEA_t nmea = {0};

void GetDataFromNMEA(NMEA_sentence_t* sentence)
{
	if (!sentence)
		return;

	const char delimiter = ',';
	char copy_string[128];
	char* token = NULL;
	char* start_pos = NULL;
	char* end_pos = NULL;
	uint8_t frame = 0;
	uint8_t string_index = 0;

	strcpy(copy_string, sentence->packet);
	char* tmp_ptr = copy_string;
	start_pos = strchr(tmp_ptr, delimiter);
	start_pos++;
	if (*start_pos == '\0')
	{
		return;
	}

	while ((*start_pos != '\0') && (end_pos = strchr(start_pos, delimiter)))
	{
		frame++;
		if (frame == 1)
		{
			memcpy(utc, start_pos, (end_pos - start_pos));
		}
		else if (frame == 2)
		{
			if (end_pos - start_pos == 1)
				status_A_V = *start_pos;
		}
		else if (frame == 3)
		{
			memcpy(lat, start_pos, (end_pos - start_pos));
		}
		else if (frame == 4)
		{
			if (end_pos - start_pos == 1)
				lat_stat = *start_pos;
		}
		else if (frame == 5)
		{
			memcpy(lon, start_pos, (end_pos - start_pos));
		}
		else if (frame == 6)
		{
			if (end_pos - start_pos == 1)
				lon_stat = *start_pos;
		}
		else if (frame == 9)
		{
			memcpy(year, start_pos, (end_pos - start_pos));
		}

		start_pos = end_pos + 1;

	}
}


static void CheckSentenceCrc(NMEA_sentence_t* sentence)
{
	if (!sentence)
		return;

	uint8_t crc = 0;
	uint8_t index = 1;
	uint8_t packet_crc = 0;

	char* end_ptr = NULL;

	while(sentence->packet[index] != NMEA_SENTENCE_CRC_START)
	{
		crc ^= sentence->packet[index];
		index++;
	}

	packet_crc = strtol(&sentence->packet[index + 1], &end_ptr, 16);

	sentence->packet_crc = packet_crc;
	sentence->calculated_crc = crc;
	sentence->is_crc_valid = packet_crc == crc;
}


void NMEA_AddtoQueue(uint8_t* start_pos, uint8_t* end_pos)
{
	uint16_t size = end_pos - start_pos;

	if (size > NMEA_MAX_SENTENCE_SIZE)
		return;

	if (nmea.que.write_index >= NMEA_SENTENCE_COUNT)
	{
		nmea.que.write_index = 0;
	}
	memcpy(nmea.que.sentences[nmea.que.write_index].packet, start_pos, size);
	nmea.que.sentences[nmea.que.write_index].packet[size] = '\0';
	CheckSentenceCrc(&nmea.que.sentences[nmea.que.write_index]);
	GetDataFromNMEA(&nmea.que.sentences[nmea.que.write_index]);
	nmea.que.write_index++;
	nmea.que.sentence_count++;
}


NMEA_sentence_t* NMEA_ReadFromQueue(void)
{
	NMEA_sentence_t* sentence_ptr = NULL;

	if (nmea.que.sentence_count > 0)
	{
		nmea.que.sentence_count--;

		sentence_ptr = &nmea.que.sentences[nmea.que.read_index];

		nmea.que.read_index++;

		if (nmea.que.read_index == NMEA_SENTENCE_COUNT)
		{
			nmea.que.read_index = 0;
		}
	}

	return sentence_ptr;
}


void ParseGpsData(uint8_t* data_ptr, uint32_t length)
{
	char* start_pos = NULL;
	char* end_pos = NULL;
	char* is_rmc_string = NULL;


	if (data_ptr == NULL)
	{
		return;
	}

	memcpy(str_data, data_ptr, length);
	str_data[length] = '\0';
	char* tmp_ptr = str_data;

	while ((start_pos = strstr(tmp_ptr, NMEA_G)))
	{
		if ((end_pos = strstr(start_pos, NMEA_SENTENCE_END)))
		{
			NMEA_AddtoQueue(start_pos, end_pos);
			tmp_ptr = end_pos;
		}
		else
		{
			return;
		}
	}
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart == &huart2)
	{
		is_received_new_data = true;
		data_size = Size;
	}
	else
	{
		Error_Handler();
	}
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->ErrorCode & HAL_UART_ERROR_PE)
	{
		statistic.pe_count++;
	}
	if (huart->ErrorCode & HAL_UART_ERROR_NE)
	{
		statistic.ne_count++;
	}
	if (huart->ErrorCode & HAL_UART_ERROR_FE)
	{
		statistic.fe_count++;
	}
	if (huart->ErrorCode & HAL_UART_ERROR_ORE)
	{
		statistic.ore_count++;
	}
}

void SystemClock_Config(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);

static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void LCD_Init(void)
{
	uint16_t y_init_pos = 0u;
	char term_output[256] = {0};

	(void)snprintf(term_output, 256, "UID: %x", uid);

	ILI9341_Draw_Text(term_output, 7, y_init_pos, WHITE, 2, BLACK);
	y_init_pos += 16u;
	ILI9341_Draw_Text("UTC time: ", 7, y_init_pos, WHITE, 2, BLACK);
	y_init_pos += 16u;
	ILI9341_Draw_Text("Date: ", 7, y_init_pos, WHITE, 2, BLACK);
	y_init_pos += 16u;
	ILI9341_Draw_Text("Lat: ", 7, y_init_pos, WHITE, 2, BLACK);
	y_init_pos += 16u;
	ILI9341_Draw_Text("Lon: ", 7, y_init_pos, WHITE, 2, BLACK);
	y_init_pos += 16u;
}

void LCD_Update(void)
{
	uint16_t y_init_pos = 16u;
	const uint16_t x_update_pos = 10* 6 * 2;
	char term_output[256] = {0};

	(void)snprintf(term_output, 256, "%s", utc);
	ILI9341_Draw_Text(term_output, x_update_pos, y_init_pos, WHITE, 2, BLACK);
	y_init_pos += 16u;

	(void)snprintf(term_output, 256, "%s", year);
	ILI9341_Draw_Text(term_output, x_update_pos, y_init_pos, WHITE, 2, BLACK);
	y_init_pos += 16u;

	(void)snprintf(term_output, 256, "%s, %c", lat, lat_stat);
	ILI9341_Draw_Text(term_output, x_update_pos, y_init_pos, WHITE, 2, BLACK);
	y_init_pos += 16u;

	(void)snprintf(term_output, 256, "%s, %c", lon, lon_stat);
	ILI9341_Draw_Text(term_output, x_update_pos, y_init_pos, WHITE, 2, BLACK);
	y_init_pos += 16u;



}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t buffer[] = "\r\n";

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  DWT_Init();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  __enable_irq();
  rng_seed(DWT->CYCCNT);
  uid = xorshift32();
  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dma_rx_buffer, BUFFER_SIZE);
  ILI9341_Init();
  ILI9341_Fill_Screen(BLACK);
  ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  LCD_Init();
  while (1)
  {

    /* USER CODE END WHILE */
      if(is_received_new_data == true)
      {
   	   is_received_new_data = false;

   	   ParseGpsData(dma_rx_buffer, data_size);
   	   STAT_AddRecord(data_size);
   	   LCD_Update();
   	   NMEA_sentence_t* sentence_ptr = NMEA_ReadFromQueue();
   	   char cdc_string[128] = {0};
   	   char term_output[256] = {0};

   	   if (sentence_ptr != NULL && is_raw_on)
   	   {
   		   CDC_Transmit_FS(sentence_ptr->packet, strlen((char*)sentence_ptr->packet));
   		   CDC_Transmit_FS(buffer, strlen(buffer));

   	   }

      }
      if (usb_received_len > 0)
      {
   	   char term_output[256] = {0};

   	   if (strcmp(usb_term_buffer, TERM_CMD_VERSION) == 0)
   	   {
   		   CDC_Transmit_FS(VERSION_STR, strlen(VERSION_STR));
   	   }
   	   else if (strcmp(usb_term_buffer, TERM_CMD_GET_POS) == 0)
   	   {
   		   (void)snprintf(term_output, 256, TERM_POS_OUTPUT_FORMAT, lat, lat_stat, lon, lon_stat);
   		   CDC_Transmit_FS(term_output, sizeof(term_output));
   	   }
   	   else if (strcmp(usb_term_buffer, TERM_CMD_GET_TIME) == 0)
   	   {
   		   (void)snprintf(term_output, 256, TERM_UTC_OUTPUT_FORMAT, utc);
   		   CDC_Transmit_FS(term_output, strlen(term_output));
   	   }
   	   else if (strcmp(usb_term_buffer, TERM_CMD_GET_DATE) == 0)
   	   {
   	       (void)snprintf(term_output, 256, TERM_DATE_OUTPUT_FORMAT, year);
   	       CDC_Transmit_FS(term_output, strlen(term_output));
   	   }
   	   else if (strcmp(usb_term_buffer, TERM_CMD_RAW_ON) == 0)
   	   {
   		   is_raw_on = true;
   	   }
   	   else if (strcmp(usb_term_buffer, TERM_CMD_RAW_OFF) == 0)
   	   {
   	       is_raw_on = false;
   	   }
		   else if (strcmp(usb_term_buffer, TERM_CMD_GET_STATUS) == 0)
		   {
			   (void)snprintf(term_output, 256, TERM_STATUS_OUTPUT_FORMAT, status_A_V);
			   CDC_Transmit_FS(term_output, strlen(term_output));
		   }
   	   else
   	   {

   	   }
   	   usb_received_len = 0;
   	   memset(usb_term_buffer, 0, sizeof(usb_term_buffer));
      }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
void MX_SPI1_Init(void)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  huart2.Init.BaudRate = 9600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DATA_Ready_Pin */
  GPIO_InitStruct.Pin = DATA_Ready_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DATA_Ready_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_Pin INT2_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = INT1_Pin|INT2_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

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
