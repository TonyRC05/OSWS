/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
// UART2: DEBUG
// UART1: DUST SENSOR

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmp280.h"
#include "sht21.h"
#include <stdio.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//DEFINITIONS FOR STATES
#define IDLE 			0x01
#define GET_TIMEDATE 	0x02
#define READ_SENSORS 	0x03
#define READ_ANALOG 	0x04
#define SEND_DATA		0x05
#define STORE_SD	 	0x06
#define SEND_INFO 		0X07
#define REMOVE_SD 		0x08
#define INSERT_SD 		0x09

//DEFINITIONS FOR INTERVAL MEASUREMENT
#define SEG_30	1
#define MIN_1   2
#define MIN_10  20
#define MIN_30  60
#define HR_1   	120
#define HR_12  1440
#define HR_24   2880

//DEFINITIONS FOR CO2 SENSOR
#define SCD30_I2C_ADDR		0xC2 //The real one is 0x61
#define START_MEASURE_MSB 	0x00
#define START_MEASURE_LSB 	0x10
#define PRESSURE_MSB 		0x00
#define PRESSURE_LSB 		0x00
#define STOP_MEASURE_MSB 	0x01
#define STOP_MEASURE_LSB 	0x04
#define GET_STATUS_MSB 		0x02
#define GET_STATUS_LSB 		0x02
#define READ_MSB 			0x03
#define READ_LSB 			0x00
#define FIRMWARE_MSB		0xD1
#define FIRMWARE_LSB 		0x00
#define RESET_MSB			0xD3
#define RESET_LSB			0x04
#define CRC	0x81


//DEFINITIONS FOR COMMANDS


typedef struct {
	UART_HandleTypeDef *PMS_huart;
	uint8_t PMS_MODE;
	uint16_t PM1_0_atmospheric; // concentration unit * μg/m3 - under atmospheric environment
	uint16_t PM2_5_atmospheric;
	uint16_t PM10_atmospheric;
	uint16_t density_2_5um;
	uint16_t density_5_0um;
	uint16_t density_10um;
}Sensor_PMS;

typedef struct {
	bool BMP280;
	bool SHT21;
	bool dust;
	bool SCD30;
}Sensors;

union {
    float value;
    uint8_t bytes[4];
}u;

//union{
//	uint8_t command;
//	uint8_t byteCommand[1];
//}c;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
FATFS fs;
FIL fil;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

//CONTROL VARIABLES
I2C_HandleTypeDef * scd30_i2c;
volatile uint8_t state = IDLE; //Initial State
Sensors SensorList = {false}; //Array stores sensor status.
volatile bool storeSD = true; //By default, the system store the data on the SD.
volatile uint8_t counterSD = 0; //Count the the times the button was pressed for the SD.
volatile uint8_t counterInterval=0; //Count the times the button was pressed to set the timeLimit.
volatile uint8_t intervalSet=false; //Detect if interval was changed.

//*************************************************** TFT //***************************************************
volatile bool tim2First=true; //To catch the first time the tim2 IT is activated. 30s is the base
volatile bool tim3First=true; //To catch the first time the tim3 IT is activated.

volatile bool txFirstFinished=false;
volatile bool txSecondFinished=false;

uint8_t counter200=0;
bool counter200Flag=true;

uint8_t counterSend=0;
//***************************************************//***************************************************

volatile uint16_t timeLimit = SEG_30; //Default: 30s for each measurement
volatile uint16_t counter30s=0; //Count the times the timer for 30s is deployed.
volatile bool timActive = false; //Flag to know if Main timer is counting seconds.
volatile bool showTFT = true; //Flag to enable or disable TFT display
volatile bool storeFinished = false; //Flag to know if the data was stored on SD.
volatile bool showFinished = false; //Flag to know if the data was displayed on TFT.
volatile bool readFinished = false; //Flag to know if the sensors were read correctly.
volatile uint16_t size=0;  //Size of the sent file

uint32_t counterLog = 0;  //Count for the file generation on SD
uint8_t logStored= 0;  //Count for the file generation on SD
uint8_t counterRx = 0;

volatile uint8_t Data[256];  //Data to be sent
volatile uint8_t DataTFT[256]; //Data sent to the TFT
volatile uint8_t nameFile[256]; //Var to store file name on the SD
volatile uint8_t bytes[4];

Sensor_PMS PMS5003 = {0}; //Structure for dust concentration
uint8_t rxbuf[32] = {0}; // Buffer to recieve the response of PMS5003

BMP280_HandleTypedef bmp280; //handler for sensor
uint8_t id=0; //ID for the BME280

RTC_TimeTypeDef time; //For RTC time
RTC_DateTypeDef date; //For RTC date

float pressure, temperature, humidity; //Variables stored
float temperature2, humidity2;
float temperature3, humidity3, CO2;
float light = 99.9; //Test

//Commands sent to the TFT module
uint8_t Commands[4]={0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

Status_Flag Init_SD(void);
Status_Flag Write_SD(char* p);
Status_Flag Init_Sensors(void);
Status_Flag Init_BMP280(void);
Status_Flag SCD30_Init(I2C_HandleTypeDef *hi2c);
Status_Flag Read_BMP280(void);
Status_Flag Read_Dust(void);
Status_Flag Read_All_Sensors(void);
Status_Flag Set_Status(void);
Status_Flag Leer_TFTSwitch(void);
void SCD30_Start(void);
void SCD30_Restart(void);
Status_Flag SCD30_Read(float *temperature, float* humidity, float* CO2);
void WriteMsg(char message[]);
void Calculate_Time(uint16_t time);
void Calculate_SdStat(void);
void Send_Commands(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void set_time(void){

	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef DateToUpdate = {0};

	sTime.Hours = 0x08;
	sTime.Minutes = 0x48;
	sTime.Seconds = 0x0;
	WriteMsg("Setting time and date...");
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
	{
	Error_Handler();
	}

	DateToUpdate.WeekDay = RTC_WEEKDAY_FRIDAY;
	DateToUpdate.Month = RTC_MONTH_MARCH;
	DateToUpdate.Date = 0x31;
	DateToUpdate.Year = 0x23;

	if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
	{
	Error_Handler();
	}

	/* USER CODE BEGIN Check_RTC_BKUP */
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F0);
	counterLog=1; //Starts with 0 and then 1. //To create new log file on SD
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, counterLog); //First stores counterLog=1.. Always
	/* USER CODE END Check_RTC_BKUP */
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
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  I2C_HandleTypeDef *scd30_i2c;

  if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32F0){ //This is made to avoid reset the time again and again
	  WriteMsg("Setting for 1st time the RTC Time and file name!\r\n");
	  set_time();
	  size=sprintf( (char *)Data, "LOG stored %d.csv\r\n", counterLog);
	  HAL_UART_Transmit(&huart2, Data, size, 1000);
	  __HAL_TIM_SET_COUNTER(&htim2,43689);
  } else { //Solo se reseteo
	  WriteMsg("Just RESET! Was not the 1st time\r\n");
	  counterLog = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);
	  counterLog++;
	  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, counterLog); //Stores the value for future files
	  size=sprintf( (char *)Data, "LOG stored %d.csv\r\n", counterLog);
	  HAL_UART_Transmit(&huart2, Data, size, 1000);
	  __HAL_TIM_SET_COUNTER(&htim2,43689);
	  intervalSet= true; //CHECK THIS!!!!
  }

  if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR4)== 0x1010){ //The limit was modified
	  timeLimit = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3);
	  size=sprintf( (char *)Data, "TimeLimit RECOVERED %d\r\n", timeLimit);
	  HAL_UART_Transmit(&huart2, Data, size, 1000);
	  Calculate_Time(timeLimit);
  } else {
	  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, timeLimit); //30s default
	  Calculate_Time(timeLimit);
  }

  WriteMsg("Welcome to the sensing platform \r\n");
  HAL_GPIO_WritePin(TFT_GPIO_Port, TFT_Pin, 0); //Send 1 to indicate i will send data
  HAL_GPIO_WritePin(SD_STATUS_GPIO_Port, SD_STATUS_Pin, 0); //By default, SD store is enabled.
  HAL_GPIO_WritePin(SENSORS_STATUS_GPIO_Port, SENSORS_STATUS_Pin, 0); //By default, SD store is enabled.

  WriteMsg("Starting... Please Wait\r\n");
  WriteMsg("Checking tft switch\r\n");
  Leer_TFTSwitch(); //Check for the button TFT status
  HAL_Delay(100);
  WriteMsg("Deinit I2C\r\n"); //To get SDC30 work.
  HAL_I2C_DeInit(&hi2c1);
  HAL_Delay(100);
  WriteMsg("Init I2C again\r\n");
  HAL_I2C_Init(&hi2c1);
  HAL_Delay(1000); //before:100

  Init_Sensors(); //Maybe set a condition to stop measurements when some sensor is failing
  Init_SD();
  HAL_ADC_Start(&hadc1);

  Calculate_SdStat();

  WriteMsg("Sending initial commands \r\n");
  Send_Commands();

  HAL_GPIO_WritePin(SD_STATUS_GPIO_Port, SD_STATUS_Pin, 1); //By default, SD store is enabled.
  HAL_GPIO_WritePin(SENSORS_STATUS_GPIO_Port, SENSORS_STATUS_Pin, 1); //By default, SD store is enabled.
  HAL_Delay(500);
  Set_Status();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	switch (state) {
		case IDLE: //Read_Dust
			if (!timActive){
				HAL_TIM_Base_Start_IT(&htim2); //Start counting 30s
				timActive=true;
			}
			Read_Dust();
			break;

		case GET_TIMEDATE:
			HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
			state = READ_SENSORS;
			break;

		case READ_SENSORS:
			//Check this and calculate time!
			Read_All_Sensors();
			Set_Status();
			if (storeSD){
				state=STORE_SD;
			} else {
				state = SEND_DATA;
			}
			break;

//		case READ_ANALOG:
////			WriteMsg("ReadAnalog State\r\n");
//			HAL_ADC_PollForConversion(&hadc1, 100);
//			light=HAL_ADC_GetValue(&hadc1);
//			size=sprintf((char *)Data, "The light is: %d \r\n",light);
//			HAL_UART_Transmit(&huart2, Data, size, 1000);
//
//			if (sdPresent){
//				HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, 1);
//				state=STORE_SD;
//			} else {
//				HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, 0);
//				state=SEND_INFO;
//			}
//			break;

		case STORE_SD:
			memset(Data, 0, sizeof(Data)); //Borra la info de "Data"
			if (PMS5003.PM1_0_atmospheric > 600){
				PMS5003.PM1_0_atmospheric = 0;
			}
			if (PMS5003.PM2_5_atmospheric > 600){
						PMS5003.PM2_5_atmospheric = 0;
					}
			if (PMS5003.PM10_atmospheric > 600){
						PMS5003.PM10_atmospheric = 0;
					}
			size=sprintf((char *)Data, "%d,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%.2f \r\n",time.Hours, time.Minutes, time.Seconds, date.Date, date.Month, temperature, humidity, pressure, CO2,light,PMS5003.PM1_0_atmospheric, PMS5003.PM2_5_atmospheric, PMS5003.PM10_atmospheric,temperature2);
			HAL_UART_Transmit(&huart2, Data, size, 1000); //Debug
			Write_SD(&Data); //Store the frame captured on the SD
			storeFinished=true;
			state=SEND_DATA;
			break;

		case REMOVE_SD:
			memset(Data, 0, sizeof(Data)); //Borra la info de "Data"
			WriteMsg("Extracting SD. Wait...\r\n");
			size=sprintf((char*)Data,"Finishing session\r\n");
			f_open(&fil,nameFile, FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
			f_lseek(&fil, fil.fsize);
			f_puts(Data, &fil);
			f_close(&fil);

			HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, counterLog); //Here the SD is extracted!
			storeSD=false; //Because the SD was removed.
			HAL_GPIO_WritePin(SD_STATUS_GPIO_Port, SD_STATUS_Pin,0);
			Calculate_SdStat();
			Send_Commands();
			WriteMsg("You can extract the SD...\r\n");

			HAL_TIM_Base_Start_IT(&htim2); //Start counting 30s or x time again
			timActive=true;
			tim2First=true; //Check this
			WriteMsg("Timer started again \r\n");
			state=IDLE;
			break;

		case INSERT_SD:
			HAL_GPIO_TogglePin(SD_STATUS_GPIO_Port, SD_STATUS_Pin);
			counterSD=0;
			WriteMsg("Reinserting the SD!\r\n");
			WriteMsg("Restarting...!\r\n");
//			counterLog=1;
//			HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, counterLog); //Here the SD is inserted again!
			NVIC_SystemReset();
			break;

		case SEND_DATA:
			HAL_GPIO_WritePin(TFT_GPIO_Port, TFT_Pin, 1); //Send 1 to indicate i will send data
			WriteMsg("LINE HIGH \r\n");

			//temperature=25.2;
			u.value = temperature;
			HAL_UART_Transmit(&huart3, u.bytes, 4,HAL_MAX_DELAY); //Transmit to the TFT
			WriteMsg("Temperature Sent \r\n");
			//memset(bytes, 0, 4);

			//humidity=48.57;
			u.value=humidity;
			HAL_UART_Transmit(&huart3, u.bytes, 4,HAL_MAX_DELAY); //Transmit to the TFT
			WriteMsg("HUM1 Sent \r\n");

			//pressure = 90.99;
			u.value=pressure;
			HAL_UART_Transmit(&huart3, u.bytes, 4,HAL_MAX_DELAY); //Transmit to the TFT
			WriteMsg("Pressure Sent \r\n");

			u.value = PMS5003.PM1_0_atmospheric;
			HAL_UART_Transmit(&huart3, u.bytes, 4,HAL_MAX_DELAY); //Transmit to the TFT
			WriteMsg("PM1.0 SENT \r\n");
			//memset(bytes, 0, 4);

			u.value=PMS5003.PM2_5_atmospheric;
			HAL_UART_Transmit(&huart3, u.bytes, 4,HAL_MAX_DELAY); //Transmit to the TFT
			WriteMsg("PM2.5 SENT \r\n");

			u.value=PMS5003.PM10_atmospheric;
			HAL_UART_Transmit(&huart3, u.bytes, 4,HAL_MAX_DELAY); //Transmit to the TFT
			WriteMsg("PM10 SENT \r\n");

			u.value = CO2;
			HAL_UART_Transmit(&huart3, u.bytes, 4,HAL_MAX_DELAY); //Transmit to the TFT
			WriteMsg("CO2 SENT \r\n");
			//memset(bytes, 0, 4);

			u.value=light;
			HAL_UART_Transmit(&huart3, u.bytes, 4,HAL_MAX_DELAY); //Transmit to the TFT
			WriteMsg("LIGHT SENT \r\n");

			u.value=temperature2;
			HAL_UART_Transmit(&huart3, u.bytes, 4,HAL_MAX_DELAY); //Transmit to the TFT
			WriteMsg("TEMP2 SENT \r\n");

			HAL_GPIO_WritePin(TFT_GPIO_Port, TFT_Pin, 0); //Send 1 to indicate i will send data
			WriteMsg("LINE LOW \r\n");

			state=IDLE;
			WriteMsg("Going to IDLE... \r\n");
			break;

		default:
			state = IDLE;
			break;
	}

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 30000 ;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 64000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 6400-1;
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TFT_Pin|GPIO_PIN_4|SENSORS_STATUS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SD_STATUS_Pin|LED_SD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TFT_Pin */
  GPIO_InitStruct.Pin = TFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TFT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 SENSORS_STATUS_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|SENSORS_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_STATUS_Pin LED_SD_Pin */
  GPIO_InitStruct.Pin = SD_STATUS_Pin|LED_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_SD_Pin BTN_INTERVAL_Pin */
  GPIO_InitStruct.Pin = BTN_SD_Pin|BTN_INTERVAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TFT_EN_Pin */
  GPIO_InitStruct.Pin = TFT_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TFT_EN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
Status_Flag Init_SD(void){ //Creates the file and store the name in "nameFile" for future writings
	WriteMsg("Initializing the SD. Please, wait...\r\n");
	f_mount(&fs, "", 1);
	size=sprintf( (char *)Data, "log %d.csv\r\n", counterLog); //Creates the file name
//	HAL_UART_Transmit(&huart2, Data, size, 1000); //Show the name of the file
	memset(nameFile, 0, sizeof(nameFile)); //Borra la info de "Data"
	for(int i= 0; i < 256; i++) {
		nameFile[i] = Data[i];
	}
	HAL_UART_Transmit(&huart2, nameFile, size, 1000);
	f_open(&fil,Data, FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
	f_lseek(&fil, fil.fsize);
	size=sprintf( (char *)Data, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n","Hour","Minute","Seconds","Day","Month","Temperature","Humidity","Pressure","C02","Light","PM1","PM2.5", "PM10","Temp2");
	f_puts(Data, &fil);
	f_close(&fil);
	return STATUS_OK;
}

Status_Flag Write_SD(char* p){
	f_open(&fil,nameFile, FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
	f_lseek(&fil, fil.fsize);
	f_puts(p, &fil);
	f_close(&fil);
	return STATUS_OK;
}// END FN.

Status_Flag Init_Sensors(void){
	WriteMsg("Initializing all the sensors. Please Wait...\r\n");

	if (SCD30_Init(&hi2c1)){ //CO2 sensor
		SensorList.SCD30 = true;
		WriteMsg("SCD30 sensor works fine.\r\n");
		SCD30_Start(); //CHECK THIS!
	} else {
		SensorList.SCD30 = false;
		WriteMsg("Error with SCD30 sensor! Please check it.\r\n");
	}

	if (Init_BMP280()){ //Temp, Hum and Pressure sensor
		SensorList.BMP280= true;
		WriteMsg("BMP280 sensor works fine.\r\n");
	} else {
		SensorList.BMP280= false;
		WriteMsg("Error with  BMP280 sensor! Please check it.\r\n");
	}

	if (SHT2x_Init(&hi2c1)){ //Temp and Hum Sensor
		SHT2x_SetResolution(RES_12_8);
		SensorList.SHT21=true;
		WriteMsg("SHT21 sensor works fine.\r\n");
	} else {
		SensorList.SHT21= false;
		WriteMsg("Error with the SHT21 sensor! Please check it.\r\n");
	}

}//END FN.

Status_Flag Read_All_Sensors(void){
	if (SensorList.BMP280) {
		Read_BMP280();
	} else {
		Init_BMP280();
	}
	if (SensorList.SHT21) {
		temperature2=SHT2x_GetTemperature();
	} else {
		SHT2x_SoftReset();
	}
	SCD30_Read(&temperature3, &humidity3, &CO2);

	readFinished = true;
	WriteMsg("Reading finished\r\n");

	/*
	size=sprintf( (char *)Data, "The CO2 is %.2f\r\n", CO2);
	HAL_UART_Transmit(&huart2, Data, size, 1000);
	size=sprintf( (char *)Data, "The Temp3 is %.2f\r\n", temperature3);
	HAL_UART_Transmit(&huart2, Data, size, 1000);
	size=sprintf( (char *)Data, "The HUM3 is %.2f\r\n", humidity3);
	HAL_UART_Transmit(&huart2, Data, size, 1000);
	*/

} //END FN.

Status_Flag Init_BMP280(void){
	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c= &hi2c1;
	if (bmp280_init(&bmp280, &bmp280.params)) {
		bool bme280p = bmp280.id == BME280_CHIP_ID;
		if (bme280p){
			WriteMsg("BME280 Found\r\n");
		} else {
			WriteMsg("BMP280 Found\r\n");
		}
		SensorList.BMP280= true;
		return STATUS_OK;
	} else {
		SensorList.BMP280= false;
		return STATUS_FAIL;
	}
}//END FN.

Status_Flag Read_BMP280(void){
	if (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)){
		WriteMsg("A problem occurred with BME280\r\n");
		SensorList.BMP280=false;
		return STATUS_FAIL;
	} else {
		WriteMsg("Correct reading from BME280\r\n");
		pressure=pressure/100;
		SensorList.BMP280=true;
		return STATUS_OK;
	}
}//END FN.

void WriteMsg(char message[]){
	size = sprintf((char *)Data, message);
	HAL_UART_Transmit(&huart2, Data, size, 1000);
}//END FN.

Status_Flag Read_Dust(void){
	__HAL_UART_FLUSH_DRREGISTER(&huart1);
	memset(rxbuf, 0, sizeof(rxbuf)); //Clean the buffer for reception
	HAL_UART_Receive(&huart1, rxbuf,32,2000);


//	PMS5003.density_2_5um= 0;
//	PMS5003.density_5_0um= 0;
//	PMS5003.density_10um= 0;
	PMS5003.PM1_0_atmospheric= 0;
	PMS5003.PM2_5_atmospheric= 0;
	PMS5003.PM10_atmospheric= 0;


	PMS5003.PM1_0_atmospheric= (rxbuf[10]<<8) + rxbuf[11];
	PMS5003.PM2_5_atmospheric= (rxbuf[12]<<8) + rxbuf[13];
	PMS5003.PM10_atmospheric= (rxbuf[14]<<8) + rxbuf[15];
//	PMS5003.density_2_5um= (rxbuf[22]<<8) + rxbuf[23];
//	PMS5003.density_5_0um= (rxbuf[24]<<8) + rxbuf[25];
//	PMS5003.density_10um= (rxbuf[26]<<8) + rxbuf[27];

	return STATUS_OK;
} //END FN.

Status_Flag Set_Status(void){

	if (SensorList.BMP280 && SensorList.SCD30 && SensorList.SHT21){
		HAL_GPIO_WritePin(SENSORS_STATUS_GPIO_Port, SENSORS_STATUS_Pin, 1);
		WriteMsg("Read finished TRUE\r\n");
	} else {
		HAL_GPIO_WritePin(SENSORS_STATUS_GPIO_Port, SENSORS_STATUS_Pin, 0);
		WriteMsg("Read finished FALSE \r\n");
	}

	if (storeSD) {
		HAL_GPIO_WritePin(SD_STATUS_GPIO_Port, SD_STATUS_Pin, 1);
		WriteMsg("SD Status TRUE\r\n");
	} else {
		HAL_GPIO_WritePin(SD_STATUS_GPIO_Port, SD_STATUS_Pin, 0);
		WriteMsg("SD Status FALSE\r\n");
	}
}//END FN.

Status_Flag Leer_TFTSwitch(void){
	if (HAL_GPIO_ReadPin(TFT_EN_GPIO_Port, TFT_EN_Pin) == 0){ //If pressed, 0V (Pull up)
		showTFT=true;
		Commands[0]=0xA0;
		WriteMsg("TFT ENABLED\r\n");
	} else {
		showTFT=false;
		Commands[0]=0xA1;
		WriteMsg("TFT Disabled\r\n");
	}
}//END FN.

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){ //Each x seconds. Default: 30s

	if (htim == &htim2){ //Cada 30s
		if(tim2First){ //If it was the first time IT
			tim2First=false;
		} else {
			WriteMsg("TIM2 reached (30s)\r\n");
			timeLimit = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3); //Default = 30s
//			counter30s++;

			if (intervalSet){ //If the interval was changed for the first time.
				counter30s=timeLimit;
				intervalSet=false;
			} else {
				counter30s++;
			}

			if (counter30s == timeLimit){
				WriteMsg("Counter30s igual a Limit\r\n");
				counter30s=0;
				state = GET_TIMEDATE;
			}
		}
	}

//	if (htim==&htim3){ //Cada 200ms
//		WriteMsg("TIM3 IT\r\n");
//		if(tim3First){
//			WriteMsg("Set tim3First false\r\n");
//			tim3First=false;
//		}
//
//		if(txFirstFinished && counter200Flag){ //Check this!! maybe need to be deleted
//			WriteMsg("Sending count 200ms\r\n");
//			counter200++;
//			if (counter200==10) { //2 seconds
//				counter200Flag=false;
//			}
//		}
//
//		if(!tim3First && !txFirstFinished){
//			counterSend++;
////			counterSend=1; //debug. erase later
//			switch (counterSend) {
//				case 1:
//					WriteMsg("1st value sending... \r\n");
//					temperature=25.2;
//					u.value = temperature;
//					HAL_GPIO_TogglePin(TFT_GPIO_Port, TFT_Pin);
//					HAL_UART_Transmit(&huart3, u.bytes, 4, HAL_MAX_DELAY); //Transmit to the TFT
//					WriteMsg("TX 2... \r\n");
////					WriteMsg("El MSJ fue:... \r\n");
////					HAL_UART_Transmit(&huart2, u.bytes, 4, HAL_MAX_DELAY); //Transmit to the debug
//					memset(bytes, 0, 4);
////					counterSend=2; //debug, erase later
////					NVIC_SystemReset();
//					break;
//
//				case 2:
//					WriteMsg("2nd value sending... \r\n");
//					humidity=47.89;
//					u.value = humidity;
//					HAL_GPIO_TogglePin(TFT_GPIO_Port, TFT_Pin);
//					HAL_UART_Transmit(&huart3, u.bytes, 4, HAL_MAX_DELAY); //Transmit to the TFT
//					WriteMsg("TX 3... \r\n");
////					WriteMsg("El MSJ fue:... \r\n");
////					HAL_UART_Transmit(&huart2, u.bytes, 4, HAL_MAX_DELAY); //Transmit to the debug
//					memset(bytes, 0, 4);
////					counterSend=1; //debug, erase later
//					//NVIC_SystemReset();
//					break;
//
//				case 3:
//					HAL_GPIO_TogglePin(TFT_GPIO_Port, TFT_Pin);
//					size=sprintf( (char *)DataTFT, "Press:780.5\n");
//					HAL_UART_Transmit(&huart3, DataTFT, 256,1000);
//					memset(DataTFT, 0, 255);
//					counterSend=0;
//					txFirstFinished=true;
//					break;
//
//				default:
//					//NOTHING
//					break;
//			}
//		}
//
//		if(counter200Flag==false && !txSecondFinished){
//			counter200=0;
//			counterSend++;
//			WriteMsg("Sending 2nd block\r\n");
//			switch (counterSend) {
//				case 1:
//					HAL_GPIO_TogglePin(TFT_GPIO_Port, TFT_Pin);
//					size=sprintf( (char *)DataTFT, "PM2.5:%d\n",PMS5003.PM2_5_atmospheric);
//					HAL_UART_Transmit(&huart3, DataTFT, 256,1000);
//					memset(DataTFT, 0, 255);
//					break;
//				case 2:
//					HAL_GPIO_TogglePin(TFT_GPIO_Port, TFT_Pin);
//					size=sprintf( (char *)DataTFT, "PM10:%d\n",PMS5003.PM10_atmospheric);
//					HAL_UART_Transmit(&huart3, DataTFT, 256,1000);
//					memset(DataTFT, 0, 255);
//					break;
//
//				case 3:
//					HAL_GPIO_TogglePin(TFT_GPIO_Port, TFT_Pin);
//					size=sprintf( (char *)DataTFT, "CO2:%.2f\n",CO2);
//					HAL_UART_Transmit(&huart3, DataTFT, 256,1000);
//					memset(DataTFT, 0, 255);
//					counterSend=0;
//					txSecondFinished=true;
//					state=SEND_DATA;
//					break;
//				default:
//					//NOTHING
//					break;
//			}
//		}
//
//	}
}//END FN.

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin){
//	WriteMsg("EXTI GRAL!\r\n");

	if (GPIO_Pin == BTN_SD_Pin){ //Remove or Insert the SD.
		WriteMsg("Botonazo de SD!\r\n");
			counterSD++;
			if(counterSD == 1){ //Pressed to remove the SD
				HAL_TIM_Base_Stop(&htim2); //Stop the main Timer for x seconds
				timActive=false;
				WriteMsg("Timer detenido \r\n");
				state=REMOVE_SD;
			}
			else if(counterSD == 2){
				state=INSERT_SD;
			}
	}
	else if (GPIO_Pin == TFT_EN_Pin){
			WriteMsg("Leyendo pin TFT\r\n");
			Leer_TFTSwitch();
			Send_Commands();
	}

	else if (GPIO_Pin == BTN_INTERVAL_Pin){

		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR4, 0x1010); //To indicate that the interval button was pressed
		counterInterval++;

		switch (counterInterval) {
			case 1:
				HAL_TIM_Base_Stop(&htim2); //Stop the main Timer for x seconds
				timActive=false;
				timeLimit= SEG_30;
				counter30s=0;
				HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, timeLimit); //Stores the value for future files
				Calculate_Time(timeLimit);
				Send_Commands();
				__HAL_TIM_SET_COUNTER(&htim2,43689); //Force to show the data within the next 10s after select each interval
				WriteMsg("set 30 seconds \r\n");
				intervalSet=true;
				state=IDLE;
				break;

			case 2:
				HAL_TIM_Base_Stop(&htim2); //Stop the main Timer for x seconds
				timActive=false;
				timeLimit= MIN_1;
				counter30s=0;
				HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, timeLimit); //Stores the value for future files
				Calculate_Time(timeLimit);
				Send_Commands();
				__HAL_TIM_SET_COUNTER(&htim2,43689);
				WriteMsg("Set 1 Min \r\n");
				intervalSet=true;
				state=IDLE;
				break;

			case 3:
				HAL_TIM_Base_Stop(&htim2); //Stop the main Timer for x seconds
				timActive=false;
				timeLimit= MIN_10;
				counter30s=0;
				HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, timeLimit); //Stores the value for future files
				Calculate_Time(timeLimit);
				Send_Commands();
				__HAL_TIM_SET_COUNTER(&htim2,43689);
				WriteMsg("Set 10 Min \r\n");
				intervalSet=true;
				state=IDLE;
				break;

			case 4:
				HAL_TIM_Base_Stop(&htim2); //Stop the main Timer for x seconds
				timActive=false;
				timeLimit= MIN_30;
				counter30s=0;
				HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, timeLimit); //Stores the value for future files
				Calculate_Time(timeLimit);
				Send_Commands();
				__HAL_TIM_SET_COUNTER(&htim2,43689);
				WriteMsg("Set 30 Min \r\n");
				intervalSet=true;
				state=IDLE;
				break;

			case 5:
				HAL_TIM_Base_Stop(&htim2); //Stop the main Timer for x seconds
				timActive=false;
				timeLimit= HR_1;
				counter30s=0;
				HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, timeLimit); //Stores the value for future files
				Calculate_Time(timeLimit);
				Send_Commands();
				__HAL_TIM_SET_COUNTER(&htim2,43690);
				WriteMsg("Set 1 HR \r\n");
				intervalSet=true;
				state=IDLE;
				break;

			case 6:
				HAL_TIM_Base_Stop(&htim2); //Stop the main Timer for x seconds
				timActive=false;
				timeLimit= HR_12; //12hr
				counter30s=0;
				HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, timeLimit); //Stores the value for future files
				Calculate_Time(timeLimit);
				Send_Commands();
				__HAL_TIM_SET_COUNTER(&htim2,43690);
				WriteMsg("Set 12 hr \r\n");
				intervalSet=true;
				state=IDLE;
				break;

			case 7:
				HAL_TIM_Base_Stop(&htim2); //Stop the main Timer for x seconds
				timActive=false;
				timeLimit= HR_24; //24hr
				counter30s=0;
				HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, timeLimit); //Stores the value for future files
				Calculate_Time(timeLimit);
				Send_Commands();
				__HAL_TIM_SET_COUNTER(&htim2,43690);
				WriteMsg("Set 24 hr \r\n");
				intervalSet=true;
				counterInterval=0;
				state=IDLE;
				break;

			default:
				timeLimit= SEG_30;
				break;
		}
	}


//	if (sdPresent) {
//		state=REMOVE_SD;
//		sdPresent = false; //SD was removed
//		HAL_GPIO_TogglePin(LED_SD_GPIO_Port, LED_SD_Pin);
//	} else {
//		state=INSERT_SD;
//		sdPresent=true;
//		HAL_GPIO_TogglePin(LED_SD_GPIO_Port, LED_SD_Pin);
//	}
}//END FN.

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	WriteMsg("IT UART EXECUTED!");
	counterRx++;
	switch (counterRx) {
		case 1:
			humidity= 47.89;
			u.value = humidity;
			WriteMsg("humidity value sending... \r\n");
			break;
		case 2:
			WriteMsg("Reseeet!... \r\n");
			NVIC_SystemReset();
			break;

		default:
			break;
	}

	HAL_UART_Transmit_IT(&huart3, u.bytes, 4);
	WriteMsg("TX 1... \r\n");
	memset(bytes, 0, 4);
}


Status_Flag SCD30_Init(I2C_HandleTypeDef *hi2c){
	WriteMsg("Iniciando SCD30 ...\r\n");
	uint8_t cmd[2]={FIRMWARE_MSB, FIRMWARE_LSB};
	uint8_t value[3]={0};
	scd30_i2c=hi2c;
	HAL_I2C_Master_Transmit(scd30_i2c, SCD30_I2C_ADDR, cmd, 2, 1000);
	HAL_Delay(1000);
	HAL_I2C_Master_Receive(scd30_i2c, SCD30_I2C_ADDR, &value, 3, 1000);
	HAL_Delay(1000);
	/*size=sprintf( (char *)Data, "Value 0 %x\r\n", value[0]);
	HAL_UART_Transmit(&huart2, Data, size, 1000);
	size=sprintf( (char *)Data, "Value 1 %x\r\n", value[1]);
	HAL_UART_Transmit(&huart2, Data, size, 1000);
	*/
	if ((value[0]<<8 | value[1]) == 834){
		return STATUS_OK;
		WriteMsg("regresa ok\r\n");
	} else{
		return STATUS_FAIL;
		WriteMsg("regresa Fail\r\n");
	}
}//END FN.

void SCD30_Start(){
	uint8_t cmd[5]={START_MEASURE_MSB, START_MEASURE_LSB, PRESSURE_MSB, PRESSURE_LSB, CRC};
	HAL_I2C_Master_Transmit(scd30_i2c, SCD30_I2C_ADDR, cmd, 5, 1000);
}//END FN.

void SCD30_Restart(){
	uint8_t cmd[2]={RESET_MSB, RESET_LSB};
	HAL_I2C_Master_Transmit(scd30_i2c, SCD30_I2C_ADDR, cmd, 2, 1000);
	WriteMsg("RESTART CO2 \r\n");
}//END FN.

Status_Flag SCD30_Read(float *temperature3, float* humidity3, float* CO2){
	uint8_t value[18]={0};
	unsigned char buffer[4];
	unsigned int tempU32;
	float result=0;

	uint8_t cmd[2]={READ_MSB, READ_LSB};
	HAL_I2C_Master_Transmit(scd30_i2c, SCD30_I2C_ADDR, cmd, 2, 1000);
	HAL_I2C_Master_Receive(scd30_i2c, SCD30_I2C_ADDR, &value, 18, 1000);

	tempU32 = (unsigned int) ((((unsigned int)value[0]) << 24) | (((unsigned int)value[1]) << 16) |
								 (((unsigned int)value[3]) << 8) |
								 ((unsigned int)value[4]));
	result= *(float*)&tempU32;
	*CO2= result;
	tempU32=0;

	tempU32 = (unsigned int) ((((unsigned int)value[6]) << 24) | (((unsigned int)value[7]) << 16) |
								 (((unsigned int)value[9]) << 8) |
								 ((unsigned int)value[10]));
	result= *(float*)&tempU32;
	*temperature3= result;
	tempU32=0;

	tempU32 = (unsigned int) ((((unsigned int)value[12]) << 24) | (((unsigned int)value[13]) << 16) |
								 (((unsigned int)value[15]) << 8) |
								 ((unsigned int)value[16]));
	result= *(float*)&tempU32;
	*humidity3= result;
	tempU32=0;

//	buffer[0]=value[6];
//	buffer[1]=value[7];
//	buffer[2]=value[9];
//	buffer[3]=value[10];
//	result= *(float*)&tempU32;
	return STATUS_OK;
}//END FN.

void Calculate_Time(uint16_t time){
	switch (time) {
		case SEG_30:
			Commands[1]=0xB0;
			break;

		case MIN_1:
			Commands[1]=0xB1;
			break;

		case MIN_10:
			Commands[1]=0xB2;
			break;

		case MIN_30:
			Commands[1]=0xB3;
			break;

		case HR_1:
			Commands[1]=0xB4;
			break;

		case HR_12:
			Commands[1]=0xB5;
			break;

		case HR_24:
			Commands[1]=0xB6;
			break;

		default:
			break;
	}
}

void Calculate_SdStat(void){

	if (storeSD){
		Commands[2]=0xC0;
	} else {
		Commands[2]=0xC1;
	}

	if (SensorList.BMP280 || SensorList.SCD30 ||SensorList.SHT21 || SensorList.dust){
		Commands[3]=0xD0;
	} else {
		Commands[3]=0xD1;
	}

} //END FN.

void Send_Commands(void){
	WriteMsg("Sending instructions");
	HAL_GPIO_WritePin(TFT_GPIO_Port, TFT_Pin, 0);
//	Commands[0]=0xA0; //Debug
//	Commands[1]=0xB0;
//	Commands[2]=0xC0;
//	Commands[3]=0xD0;
	HAL_UART_Transmit(&huart3,Commands,4,HAL_MAX_DELAY); //Transmit to the TFT

} //END FN.

//void float2Bytes(float val, BYTE* bytes_array){
//  // Create union of shared memory space
//  union {
//    float value;
//    uint8_t array[4];
//  } u;
//
//  u.value = val;
//  memcpy(bytes_array, u.temp_array, 4);// Assign bytes to input array
//} //END FN

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
