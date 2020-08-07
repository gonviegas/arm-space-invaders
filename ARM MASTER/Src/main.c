/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f769i_discovery.h"
#include "stm32f769i_discovery_lcd.h"
#include <stdio.h>
#include <stdbool.h>
#include "bmp_lib.h"
//#include "invader_0.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum id {
	X_AXIS,
	A_BUTTON
}Id;

typedef struct joystick {
	Id id;
	uint8_t val;

}Joystick;

typedef enum status {
	OFF,
	DESTROYED,
	SHOT,
	LIVE
}Status;


typedef struct invader {
	Status status;
	uint16_t x;
	uint16_t y;
}Invader;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

DMA2D_HandleTypeDef hdma2d;

DSI_HandleTypeDef hdsi;

LTDC_HandleTypeDef hltdc;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

SDRAM_HandleTypeDef hsdram1;

osThreadId IdleTaskHandle;
/* USER CODE BEGIN PV */
const uint8_t x_unit_px = 6;
const uint8_t y_unit_px = 20;
Invader invader[11][5];
bool flag_endgame = true;
bool flag_tx_DMA = false;
bool flag_half_tx_DMA = false;
bool flag_rx_DMA = false;
bool flag_half_rx_DMA = false;
uint8_t data2receive[4];
uint8_t data2send[200];
uint16_t x_master = 300;
uint16_t y_master = 460;
uint16_t x_laser;
uint16_t y_laser;

char a[100];
char x[100];
char y[100];
//uint16_t joy_master = 0;
//uint16_t button_joy_master;


//uint16_t y_slave = 460;

//uint16_t y_laser_2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_LTDC_Init(void);
static void MX_DSIHOST_DSI_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_SPI2_Init(void);
void StartIdleTask(void const * argument);

/* USER CODE BEGIN PFP */
static void LCD_Config(void);
void get_joystick_master_SPI_sdr(void * parameters);
void receive_slave_joystick_UART(void * parameters);
void send_slave_UART(void * parameters);
void cannon_master_rcv(void * parameters);
void cannon_slave_rcv(void * parameters);
void invaders_sdr (void *parameters);
//void send_slave(void);
//void get_joystick_master(void * parameters);
//void lcd_cannon_player_1(void);
//void lcd_cannon_player_2(void);
//void lcd_laser_1(void);
//void lcd_laser_2(void);
//void lcd_laser_1 (void* parameters);
//void lcd_cannon_player_1 (void* parameters);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
xQueueHandle xQueue_joy_master;
xQueueHandle xQueue_joy_slave;
xQueueHandle xQueue_invaders;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_LTDC_Init();
  MX_DSIHOST_DSI_Init();
  MX_USART6_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
	//
	//  BSP_LED_Init(LED1);
	BSP_LED_Init(LED2);

	//  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);
	LCD_Config();

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	xQueue_joy_slave = xQueueCreate(10, sizeof(Joystick));
	xQueue_joy_master = xQueueCreate(10, sizeof(Joystick));
	xQueue_invaders = xQueueCreate(10, sizeof(Invader));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of IdleTask */
  osThreadDef(IdleTask, StartIdleTask, osPriorityIdle, 0, 128);
  IdleTaskHandle = osThreadCreate(osThread(IdleTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	//if (xQueue_joy_slave != NULL) {

  	xTaskCreate(get_joystick_master_SPI_sdr, "get Joystick MASTER SPI", 400, NULL, tskIDLE_PRIORITY + 3, NULL);
	xTaskCreate(receive_slave_joystick_UART, "get Joystick Slave UART", 400, NULL, tskIDLE_PRIORITY + 3, NULL);
	xTaskCreate(send_slave_UART, "Send to Slave", 400, NULL, tskIDLE_PRIORITY + 3, NULL);
	xTaskCreate(cannon_master_rcv, "Joystick_master input", 400, NULL, tskIDLE_PRIORITY + 2, NULL);
	xTaskCreate(cannon_slave_rcv, "Joystick Slave input", 400, NULL, tskIDLE_PRIORITY + 2, NULL);
	xTaskCreate(invaders_sdr,"Invaders", 1000, NULL, tskIDLE_PRIORITY + 1, NULL);

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//}

	while (1)
	{


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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_USART6;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  hdma2d.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA;
  hdma2d.LayerCfg[1].RedBlueSwap = DMA2D_RB_REGULAR;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief DSIHOST Initialization Function
  * @param None
  * @retval None
  */
static void MX_DSIHOST_DSI_Init(void)
{

  /* USER CODE BEGIN DSIHOST_Init 0 */

  /* USER CODE END DSIHOST_Init 0 */

  DSI_PLLInitTypeDef PLLInit = {0};
  DSI_HOST_TimeoutTypeDef HostTimeouts = {0};
  DSI_PHY_TimerTypeDef PhyTimings = {0};
  DSI_LPCmdTypeDef LPCmd = {0};
  DSI_CmdCfgTypeDef CmdCfg = {0};

  /* USER CODE BEGIN DSIHOST_Init 1 */

  /* USER CODE END DSIHOST_Init 1 */
  hdsi.Instance = DSI;
  hdsi.Init.AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_DISABLE;
  hdsi.Init.TXEscapeCkdiv = 4;
  hdsi.Init.NumberOfLanes = DSI_ONE_DATA_LANE;
  PLLInit.PLLNDIV = 20;
  PLLInit.PLLIDF = DSI_PLL_IN_DIV1;
  PLLInit.PLLODF = DSI_PLL_OUT_DIV1;
  if (HAL_DSI_Init(&hdsi, &PLLInit) != HAL_OK)
  {
    Error_Handler();
  }
  HostTimeouts.TimeoutCkdiv = 1;
  HostTimeouts.HighSpeedTransmissionTimeout = 0;
  HostTimeouts.LowPowerReceptionTimeout = 0;
  HostTimeouts.HighSpeedReadTimeout = 0;
  HostTimeouts.LowPowerReadTimeout = 0;
  HostTimeouts.HighSpeedWriteTimeout = 0;
  HostTimeouts.HighSpeedWritePrespMode = DSI_HS_PM_DISABLE;
  HostTimeouts.LowPowerWriteTimeout = 0;
  HostTimeouts.BTATimeout = 0;
  if (HAL_DSI_ConfigHostTimeouts(&hdsi, &HostTimeouts) != HAL_OK)
  {
    Error_Handler();
  }
  PhyTimings.ClockLaneHS2LPTime = 28;
  PhyTimings.ClockLaneLP2HSTime = 33;
  PhyTimings.DataLaneHS2LPTime = 15;
  PhyTimings.DataLaneLP2HSTime = 25;
  PhyTimings.DataLaneMaxReadTime = 0;
  PhyTimings.StopWaitTime = 0;
  if (HAL_DSI_ConfigPhyTimer(&hdsi, &PhyTimings) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_ConfigFlowControl(&hdsi, DSI_FLOW_CONTROL_BTA) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_SetLowPowerRXFilter(&hdsi, 10000) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_ConfigErrorMonitor(&hdsi, HAL_DSI_ERROR_NONE) != HAL_OK)
  {
    Error_Handler();
  }
  LPCmd.LPGenShortWriteNoP = DSI_LP_GSW0P_DISABLE;
  LPCmd.LPGenShortWriteOneP = DSI_LP_GSW1P_DISABLE;
  LPCmd.LPGenShortWriteTwoP = DSI_LP_GSW2P_DISABLE;
  LPCmd.LPGenShortReadNoP = DSI_LP_GSR0P_DISABLE;
  LPCmd.LPGenShortReadOneP = DSI_LP_GSR1P_DISABLE;
  LPCmd.LPGenShortReadTwoP = DSI_LP_GSR2P_DISABLE;
  LPCmd.LPGenLongWrite = DSI_LP_GLW_DISABLE;
  LPCmd.LPDcsShortWriteNoP = DSI_LP_DSW0P_DISABLE;
  LPCmd.LPDcsShortWriteOneP = DSI_LP_DSW1P_DISABLE;
  LPCmd.LPDcsShortReadNoP = DSI_LP_DSR0P_DISABLE;
  LPCmd.LPDcsLongWrite = DSI_LP_DLW_DISABLE;
  LPCmd.LPMaxReadPacket = DSI_LP_MRDP_DISABLE;
  LPCmd.AcknowledgeRequest = DSI_ACKNOWLEDGE_DISABLE;
  if (HAL_DSI_ConfigCommand(&hdsi, &LPCmd) != HAL_OK)
  {
    Error_Handler();
  }
  CmdCfg.VirtualChannelID = 0;
  CmdCfg.ColorCoding = DSI_RGB888;
  CmdCfg.CommandSize = 640;
  CmdCfg.TearingEffectSource = DSI_TE_EXTERNAL;
  CmdCfg.TearingEffectPolarity = DSI_TE_RISING_EDGE;
  CmdCfg.HSPolarity = DSI_HSYNC_ACTIVE_LOW;
  CmdCfg.VSPolarity = DSI_VSYNC_ACTIVE_LOW;
  CmdCfg.DEPolarity = DSI_DATA_ENABLE_ACTIVE_HIGH;
  CmdCfg.VSyncPol = DSI_VSYNC_FALLING;
  CmdCfg.AutomaticRefresh = DSI_AR_ENABLE;
  CmdCfg.TEAcknowledgeRequest = DSI_TE_ACKNOWLEDGE_DISABLE;
  if (HAL_DSI_ConfigAdaptedCommandMode(&hdsi, &CmdCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_SetGenericVCID(&hdsi, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DSIHOST_Init 2 */

  /* USER CODE END DSIHOST_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};
  LTDC_LayerCfgTypeDef pLayerCfg1 = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 7;
  hltdc.Init.VerticalSync = 3;
  hltdc.Init.AccumulatedHBP = 14;
  hltdc.Init.AccumulatedVBP = 5;
  hltdc.Init.AccumulatedActiveW = 654;
  hltdc.Init.AccumulatedActiveH = 485;
  hltdc.Init.TotalWidth = 660;
  hltdc.Init.TotalHeigh = 487;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 0;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 0;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg.Alpha = 0;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0;
  pLayerCfg.ImageWidth = 0;
  pLayerCfg.ImageHeight = 0;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 0;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 0;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg1.Alpha = 0;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg1.FBStartAdress = 0;
  pLayerCfg1.ImageWidth = 0;
  pLayerCfg1.ImageHeight = 0;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart6.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart6.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}
/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_13;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_32;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_1;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 16;
  SdramTiming.ExitSelfRefreshDelay = 16;
  SdramTiming.SelfRefreshTime = 16;
  SdramTiming.RowCycleDelay = 16;
  SdramTiming.WriteRecoveryTime = 16;
  SdramTiming.RPDelay = 16;
  SdramTiming.RCDDelay = 16;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(JST_EN_GPIO_Port, JST_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : JST_EN_Pin */
  GPIO_InitStruct.Pin = JST_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(JST_EN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void LCD_Config(void)
{
	uint32_t  lcd_status = LCD_OK;

	/* Initialize the LCD */
	lcd_status = BSP_LCD_Init();
	while(lcd_status != LCD_OK);

	BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);

	/* Clear the LCD */
	//	BSP_LCD_Clear(LCD_COLOR_WHITE);
	//	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_Clear(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
}


void get_joystick_master_SPI_sdr(void *parameters) { /* Receber X de Joystick de ARM Master*/
	Joystick joystick_x;
	joystick_x.id = X_AXIS;
	Joystick joystick_a;
	joystick_a.id = A_BUTTON;
	uint8_t spiTxBuff[5] = {0b10000011, 0, 0, 0, 0};
	uint8_t spiRxBuff[5];

	portTickType xNextWakeTime;
	const portTickType xTicksToWait = 0 / portTICK_RATE_MS;
	const portTickType xCycleFrequency = 20 / portTICK_RATE_MS;
	xNextWakeTime = xTaskGetTickCount ();

	for (;;){
		HAL_GPIO_WritePin(JST_EN_GPIO_Port, JST_EN_Pin, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi2, spiTxBuff, spiRxBuff, 5, 100);
		HAL_GPIO_WritePin(JST_EN_GPIO_Port, JST_EN_Pin, GPIO_PIN_SET);

		joystick_x.val = ((spiRxBuff[1] << 8 ) + spiRxBuff[0]) /10;
		joystick_a.val = spiRxBuff[4];

		xQueueSendToBack(xQueue_joy_master, &joystick_x , xTicksToWait);
		xQueueSendToBack(xQueue_joy_master, &joystick_a , xTicksToWait);

		vTaskDelayUntil(&xNextWakeTime, xCycleFrequency);
	}
}




void cannon_master_rcv(void * parameters)  {
	Joystick joy_master;

	bool btn_joy_flag = false;
	bool laser_lcd_flag = false;

	for (;;){
		portBASE_TYPE xStatus=xQueueReceive(xQueue_joy_master, &joy_master, portMAX_DELAY);
		if(xStatus==pdTRUE) {
			if (flag_endgame == true) {
				if (joy_master.id == X_AXIS && joy_master.val < 30) {
					if (x_master < 200) {
						x_master = 200;
						BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
						BSP_LCD_FillRect(x_master-4,y_master-1,40,20);

						BSP_LCD_DrawBitmap(x_master,y_master,cannon_master);

					}
					else {
						BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
						BSP_LCD_FillRect(x_master-4,y_master-1,40,20);
						x_master = x_master - 2;

						BSP_LCD_DrawBitmap(x_master,y_master,cannon_master);
					}
				}else if (joy_master.id == X_AXIS && joy_master.val > 70) {
					if (x_master > 750) {
						x_master = 750;
						BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
						BSP_LCD_FillRect(x_master-4,y_master-1,40,20);

						BSP_LCD_DrawBitmap(x_master,y_master,cannon_master);
					} else {
						BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
											BSP_LCD_FillRect(x_master-4,y_master-1,40,20);
						x_master = x_master + 2;

						BSP_LCD_DrawBitmap(x_master,y_master,cannon_master);
					}
				}
				else {
					BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
										BSP_LCD_FillRect(x_master-4,y_master-1,40,20);
					BSP_LCD_DrawBitmap(x_master,y_master,cannon_master);
				}

				if (laser_lcd_flag == false) {
					if (joy_master.id == A_BUTTON && (joy_master.val == 4 || joy_master.val == 2 )) {
						btn_joy_flag = true;
						x_laser = x_master + 14;
						y_laser = y_master -2;
					}
				}

				if (btn_joy_flag == true) {
					laser_lcd_flag = true;
					BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
					BSP_LCD_FillRect(x_laser,y_laser,2,12);
					y_laser = y_laser - 4;
					BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
					BSP_LCD_FillRect(x_laser,y_laser,2,12);

					for (int c = 0; c < 11; c++) {		//board init
						for (int l = 0; l < 5; l++) {
							if (invader[c][l].status == LIVE && x_laser >= invader[c][l].x && x_laser <= invader[c][l].x + 30 &&
									y_laser >= invader[c][l].y &&  y_laser <= invader[c][l].y + 25) {
								invader[c][l].status = SHOT;
								btn_joy_flag = false;
								laser_lcd_flag = false;
							}
						}
					}
					if (y_laser < 5 || y_laser > 480)  {
						BSP_LCD_DrawBitmap(x_laser-15,y_laser,laser_master_end);
						laser_lcd_flag = false;
					}
				}
			}
		}
		else {
			Error_Handler();
		}
	}
}

void cannon_slave_rcv(void * parameters)  {
	uint16_t x_slave = 500;
	uint16_t y_slave = 460;
	Joystick joy_slave;

	uint16_t x_laser;
	uint16_t y_laser;
	bool btn_joy_flag = false;
	bool laser_lcd_flag = false;

	for (;;){
		portBASE_TYPE xStatus=xQueueReceive(xQueue_joy_slave, &joy_slave, portMAX_DELAY);
		if(xStatus==pdTRUE) {
			if (flag_endgame == true) {
				if (joy_slave.id == X_AXIS && joy_slave.val < 10) {
					if (x_slave < 200) {
						x_slave = 200;
						BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
						BSP_LCD_FillRect(x_slave-4,y_slave-1,40,20);
						BSP_LCD_DrawBitmap(x_slave,y_slave,cannon_slave);
					}
					else {
						BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
												BSP_LCD_FillRect(x_slave-4,y_slave-1,40,20);
						x_slave = x_slave - 2;
						BSP_LCD_DrawBitmap(x_slave,y_slave,cannon_slave);
					}
				}else if (joy_slave.id == X_AXIS && joy_slave.val > 45) {
					if (x_slave > 750) {
						x_slave = 750;
						BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
												BSP_LCD_FillRect(x_slave-4,y_slave-1,40,20);
						BSP_LCD_DrawBitmap(x_slave,y_slave,cannon_slave);
					} else {
						BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
												BSP_LCD_FillRect(x_slave-4,y_slave-1,40,20);
						x_slave = x_slave + 2;
						BSP_LCD_DrawBitmap(x_slave,y_slave,cannon_slave);
					}
				}
				else {
					BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
											BSP_LCD_FillRect(x_slave-4,y_slave-1,40,20);
					BSP_LCD_DrawBitmap(x_slave,y_slave,cannon_slave);
				}

				if (laser_lcd_flag == false) {
					if (joy_slave.id == A_BUTTON && joy_slave.val == 1 ) {
						btn_joy_flag = true;
						x_laser = x_slave + 14;
						y_laser = y_slave -2;
					}
				}
				if (btn_joy_flag == true) {
					laser_lcd_flag = true;
					BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
					BSP_LCD_FillRect(x_laser,y_laser,2,12);
					y_laser = y_laser - 4;
					BSP_LCD_SetTextColor(LCD_COLOR_MAGENTA);
					BSP_LCD_FillRect(x_laser,y_laser,2,12);

					for (int c = 0; c < 11; c++) {		//board init
						for (int l = 0; l < 5; l++) {
							if (invader[c][l].status == LIVE && x_laser >= invader[c][l].x && x_laser <= invader[c][l].x + 30 &&
									y_laser >= invader[c][l].y &&  y_laser <= invader[c][l].y + 25) {
								invader[c][l].status = SHOT;
								btn_joy_flag = false;
								laser_lcd_flag = false;
							}
						}
					}
					if (y_laser < 5 || y_laser > 480)  {
						BSP_LCD_DrawBitmap(x_laser-15,y_laser,laser_slave_end);
						laser_lcd_flag = false;
					}
				}
			}
		}
		else {
			Error_Handler();
		}
	}
}

void invaders_sdr (void *parameters) {
	int16_t x_start = 252;
	int16_t y_start = 80;
	int8_t rows_right = 0;
	int8_t rows_left = 0;
	int8_t direction = 1;
	int8_t x_steps = 0;
	int8_t y_steps = 0;
	bool flag_y_steps = false;
	uint8_t ctr_shot = 0;

	for (int c = 0; c < 11; c++) {		//board init
		for (int l = 0; l < 5; l++) {
			invader[c][l].status = LIVE;
		}
	}
	portTickType xNextWakeTime;

	portTickType xCycleFrequency = 800 / portTICK_RATE_MS;
	xNextWakeTime = xTaskGetTickCount ();

	for (;;) {
		if (flag_endgame == true) {

			if (flag_y_steps == true) {
				y_steps++;
				flag_y_steps = false;
			}
			if(direction == 1 ) { // Right direction
				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
				BSP_LCD_FillRect(160,0,619,480);
				x_steps ++;
				rows_right = 0;
				for (int c = 10; c >= 0; c--) {
					rows_right++;
					for (int l = 5; l >= 0; l--) {
						if (invader[c][l].status == LIVE) {
							if(x_steps == 8 * rows_right) {
								flag_y_steps = true;
								direction = -1;
							}
						}
					}
				}
			}

			if(direction == -1 ) { // Left Direction
				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
				BSP_LCD_FillRect(160,0,619,480);
				x_steps --;
				rows_left = 0;
				for (int c = 0; c < 11; c++) {
					rows_left++;
					for (int l = 0; l < 5; l++) {
						if (invader[c][l].status == LIVE) {
							if(x_steps == (-8) * rows_left) {
								flag_y_steps = true;
								direction = 1;
							}
						}
					}
				}
			}
			for (int c = 0; c < 11; c++) { // 2nd and 3rd row of invaders
				for (int l = 0; l < 5; l++) {
					if (invader[c][l].status == LIVE) {
						invader[c][l].x = x_start+(45*c)+(x_unit_px*x_steps);
						invader[c][l].y = y_start+(50*l)+(y_unit_px*y_steps);
						if (x_steps %2 == 0 && l==0) {
							BSP_LCD_DrawBitmap(invader[c][l].x,invader[c][l].y,invader_a1);
						}
						else if (l==0)
						{
							BSP_LCD_DrawBitmap(invader[c][l].x,invader[c][l].y,invader_a2);
						}
						if (x_steps %2 == 0 && (l==1 || l==2)){
							BSP_LCD_DrawBitmap(invader[c][l].x,invader[c][l].y,invader_b1);
						}
						else if (l==1 || l==2)
						{
							BSP_LCD_DrawBitmap(invader[c][l].x,invader[c][l].y,invader_b2);
						}
						if(x_steps%2 ==0 && (l==3 || l==4)){
							BSP_LCD_DrawBitmap(invader[c][l].x,invader[c][l].y,invader_c1);
						}
						else if(l==3 || l==4) {
							BSP_LCD_DrawBitmap(invader[c][l].x,invader[c][l].y,invader_c2);
						}
					}
				}
			}

			for (int c = 0; c < 11; c++) {
				for (int l = 0; l < 5; l++) {
					if (invader[c][l].status == DESTROYED) {
						invader[c][l].status = OFF;
					}
					if (invader[c][l].status == SHOT) {
						BSP_LCD_DrawBitmap(x_start+(45*c)+(x_unit_px*x_steps),y_start+(50*l)+(y_unit_px*y_steps),invader_dstroy);
						invader[c][l].status = DESTROYED;
						ctr_shot++;
					}
					if (ctr_shot == 55) {
						flag_endgame = false;
					}
					if (invader[c][l].status == LIVE) {
						if (invader[c][l].y >= 430) {
							flag_endgame = false;
						}
					}
				}
			}
			if(ctr_shot == 6) {
				xCycleFrequency = 700;
			}
			if(ctr_shot == 12) {
				xCycleFrequency = 600;
			}

			if(ctr_shot == 18) {
				xCycleFrequency = 500;
			}
			if(ctr_shot == 24) {
				xCycleFrequency = 400;
			}
			if(ctr_shot == 30) {
				xCycleFrequency = 300;
			}
			if(ctr_shot == 36) {
				xCycleFrequency = 200;
			}
			if(ctr_shot == 42) {
				xCycleFrequency = 100;
			}

			if(ctr_shot == 48) {
				xCycleFrequency = 50;
			}
			if(ctr_shot >= 54) {
				xCycleFrequency = 25;
			}
		}
		vTaskDelayUntil(&xNextWakeTime, xCycleFrequency);
	}
}

void send_slave_UART(void * parameters) { /* Enviar Joystick Slave para ARM Master */
	for (int i = 0; i < 200; i++) {
		data2send[i] = ' ';
	}
	portTickType xNextWakeTime;

	const portTickType xCycleFrequency = 20 / portTICK_RATE_MS;
	xNextWakeTime = xTaskGetTickCount ();

	HAL_UART_Transmit_DMA(&huart6, data2send, 200);
	for (;;){

		if (flag_half_tx_DMA == true) {
			flag_half_tx_DMA = false;
			data2send[0] = 'x';
			data2send[1] = x_master;
			data2send[2] = 'y';
			data2send[3] = y_master;

		}

		if (flag_tx_DMA == true) {
			flag_tx_DMA = false;

			data2send[4] = 'x';
			data2send[5] = x_laser;


			data2send[6] = 'y';
			data2send[7] = y_laser;
		}
		vTaskDelayUntil(&xNextWakeTime, xCycleFrequency);
	}
}

void receive_slave_joystick_UART(void * parameters) { /* Receber X de Joystick de ARM Slave*/ //
	Joystick joystick_x;
	joystick_x.id = X_AXIS;
	Joystick joystick_a;
	joystick_a.id = A_BUTTON;

	portTickType xNextWakeTime;
	const portTickType xTicksToWait = 0 / portTICK_RATE_MS;
	const portTickType xCycleFrequency = 20 / portTICK_RATE_MS;
	xNextWakeTime = xTaskGetTickCount ();
	for (int i = 0; i < 4; i++) {
		data2receive[i] = '0';
	}
	HAL_UART_Receive_DMA(&huart6, data2receive, 4);

	for (;;){
		//				if (flag_half_rx_DMA == true) {
		//					flag_half_rx_DMA = false;
		//					for (int i = 0; i < 2; i++) {
		//						if (data2receive[i] == 'x') {
		//							joystick_x.val = data2receive[i+1];
		//							sprintf(a, "X_AXIS = %d", data2receive[i+1]);
		//								BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
		//								BSP_LCD_DisplayStringAt(0, 200, (int*)a, LEFT_MODE);
		//							xQueueSendToBack(xQueue_joy_slave, &joystick_x, xTicksToWait);
		//						}
		//
		//					}
		//				}
		//				if (flag_rx_DMA == true) {
		//					flag_rx_DMA = false;
		//					HAL_UART_Receive_DMA(&huart6, data2receive, 4);
		//					for (int i = 2; i < 4; i++) {
		//						if (data2receive[i] == 'a') {
		//							joystick_a.val = data2receive[i+1];
		//							printf(y, "A_BUTTON = %d", data2receive[i+1]);
		//							BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
		//							BSP_LCD_DisplayStringAt(0, 300, (int*)y, LEFT_MODE);
		//							xQueueSendToBack(xQueue_joy_slave, &joystick_a, xTicksToWait);
		//						}
		//					}
		//				}


		if (flag_rx_DMA == true) {
			flag_rx_DMA = false;
			HAL_UART_Receive_DMA(&huart6, data2receive, 4);
			for (int i = 0; i < 4; i++) {
				if (data2receive[i] == 'a') {
					joystick_a.val = data2receive[i+1];

					xQueueSendToBack(xQueue_joy_slave, &joystick_a, xTicksToWait);
				}
				if (data2receive[i] == 'x') {
					if (i == 3) {

						joystick_x.val = data2receive[i];
					}
					else {
						xQueueSendToBack(xQueue_joy_slave, &joystick_x, xTicksToWait);
					}

				}
			}
		}
		vTaskDelayUntil(&xNextWakeTime, xCycleFrequency);
	}
}


void DMA1_Stream6_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart6_tx);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart6) {
		flag_tx_DMA = true;
	}
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart6) {
		flag_half_tx_DMA = true;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart6) {
		flag_rx_DMA = true;
	}
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart6) {
		flag_half_rx_DMA = true;
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartIdleTask */
/**
 * @brief  Function implementing the IdleTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartIdleTask */
void StartIdleTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
  /* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	BSP_LED_On(LED2);
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
