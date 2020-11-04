
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
//#include "task.h"

#include "PlatformType.h"
#include "SubRoutines.h"
#include "Robot.h"

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

osThreadId Task_DefaultHandle;
uint32_t Task_DefaultBuffer[ 512 ];
osStaticThreadDef_t Task_DefaultControlBlock;
osThreadId Task_ActivityHandle;
uint32_t Task_ActivityBuffer[ 256 ];
osStaticThreadDef_t Task_ActivityControlBlock;
osThreadId Task_SystemHandle;
uint32_t Task_SystemBuffer[ 512 ];
osStaticThreadDef_t Task_SystemControlBlock;
osThreadId Task_FlexOledHandle;
uint32_t Task_FlexOledBuffer[ 512 ];
osStaticThreadDef_t Task_FlexOledControlBlock;
osThreadId Task_PwrMngtHandle;
uint32_t Task_PwrMngtBuffer[ 512 ];
osStaticThreadDef_t Task_PwrMngtControlBlock;
osThreadId Task_SensorsHandle;
uint32_t Task_SensorsBuffer[ 512 ];
osStaticThreadDef_t Task_SensorsControlBlock;
osThreadId Task_RpiHandle;
uint32_t Task_RpiBuffer[ 512 ];
osStaticThreadDef_t Task_RpiControlBlock;
osThreadId Task_BluetoothHandle;
uint32_t Task_BluetoothBuffer[ 256 ];
osStaticThreadDef_t Task_BluetoothControlBlock;
osThreadId Task_BehaviorHandle;
uint32_t Task_BehaviorBuffer[ 512 ];
osStaticThreadDef_t Task_BehaviorControlBlock;
osThreadId Task_KinematicHandle;
uint32_t Task_KinematicBuffer[ 512 ];
osStaticThreadDef_t Task_KinematicControlBlock;
osMessageQId myQueue01Handle;
uint8_t myQueue01Buffer[ 16 * sizeof( uint16_t ) ];
osStaticMessageQDef_t myQueue01ControlBlock;
osSemaphoreId myBinarySem01Handle;
osStaticSemaphoreDef_t myBinarySem01ControlBlock;
osSemaphoreId myBinarySem02Handle;
osStaticSemaphoreDef_t myBinarySem02ControlBlock;
osSemaphoreId myBinarySem03Handle;
osStaticSemaphoreDef_t myBinarySem03ControlBlock;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
void StartTask_Default(void const * argument);
void StartTask_Activity(void const * argument);
void StartTask_System(void const * argument);
void StartTask_FlexOled(void const * argument);
void StartTask_PwrMngt(void const * argument);
void StartTask_Sensors(void const * argument);
void StartTask_Rpi(void const * argument);
void StartTask_Bluetooth(void const * argument);
void StartTask_Behavior(void const * argument);
void StartTask_Kinematic(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                
/*******************************************************************************************************************************************************/
/*******************************************************************************************************************************************************/

/* User functions prototypes ----------------------------------------*/
void CheckInputs(void);
void I2C_Device_Check(void);

extern void vTaskGetRunTimeStats( char *pcWriteBuffer );

/* User variables ---------------------------------------------------*/



Input_st Input;
Gesture_st Gesture;
Sensors_st Sensor;


DeviceStatus_enum ITG3205_Status;
DeviceStatus_enum ADXL345_Status;
DeviceStatus_enum SSD1306_Status;
DeviceStatus_enum SSD1320_Status;

I2C2_Bus_enum I2C2_Bus = I2C2_Free;

uint8_t CPUChargeBuff[8];
uint8_t Flex_OLED_Initialized = FALSE;
Flex_Oled_Menu_em Flex_Oled_Menu = Battery;

uint16_t Rpi_Shutdown_ctr = 0;
uint16_t Board_Shutdown_ctr = 0;
uint16_t LowBattery_ctr = 0;

#define BUFFERSIZE 8
uint8_t RPI_TxBuff[BUFFERSIZE];
uint8_t RPI_RxBuff[BUFFERSIZE];
uint16_t j=0;


/*******************************************************************************************************************************************************/
/*******************************************************************************************************************************************************/

int main(void)
{

	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C2_Init();
	MX_SPI1_Init();
	MX_SPI2_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_SPI3_Init();
	MX_TIM4_Init();
	MX_USART3_UART_Init();
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_USART2_UART_Init();

  /* definition and creation of myBinarySem01 */
  osSemaphoreStaticDef(myBinarySem01, &myBinarySem01ControlBlock);
  myBinarySem01Handle = osSemaphoreCreate(osSemaphore(myBinarySem01), 1);

  /* definition and creation of myBinarySem02 */
  osSemaphoreStaticDef(myBinarySem02, &myBinarySem02ControlBlock);
  myBinarySem02Handle = osSemaphoreCreate(osSemaphore(myBinarySem02), 1);

  /* definition and creation of myBinarySem03 */
  osSemaphoreStaticDef(myBinarySem03, &myBinarySem03ControlBlock);
  myBinarySem03Handle = osSemaphoreCreate(osSemaphore(myBinarySem03), 1);


  /* Create the thread(s) */
  /* definition and creation of Task_Default */
  osThreadStaticDef(Task_Default, StartTask_Default, osPriorityNormal, 0, 512, Task_DefaultBuffer, &Task_DefaultControlBlock);
  Task_DefaultHandle = osThreadCreate(osThread(Task_Default), NULL);

  /* definition and creation of Task_Activity */
  osThreadStaticDef(Task_Activity, StartTask_Activity, osPriorityLow, 0, 256, Task_ActivityBuffer, &Task_ActivityControlBlock);
  Task_ActivityHandle = osThreadCreate(osThread(Task_Activity), NULL);

  /* definition and creation of Task_System */
  osThreadStaticDef(Task_System, StartTask_System, osPriorityNormal, 0, 512, Task_SystemBuffer, &Task_SystemControlBlock);
  Task_SystemHandle = osThreadCreate(osThread(Task_System), NULL);

  /* definition and creation of Task_FlexOled */
  osThreadStaticDef(Task_FlexOled, StartTask_FlexOled, osPriorityNormal, 0, 512, Task_FlexOledBuffer, &Task_FlexOledControlBlock);
  Task_FlexOledHandle = osThreadCreate(osThread(Task_FlexOled), NULL);

  /* definition and creation of Task_PwrMngt */
  osThreadStaticDef(Task_PwrMngt, StartTask_PwrMngt, osPriorityHigh, 0, 512, Task_PwrMngtBuffer, &Task_PwrMngtControlBlock);
  Task_PwrMngtHandle = osThreadCreate(osThread(Task_PwrMngt), NULL);

  /* definition and creation of Task_Sensors */
  osThreadStaticDef(Task_Sensors, StartTask_Sensors, osPriorityHigh, 0, 512, Task_SensorsBuffer, &Task_SensorsControlBlock);
  Task_SensorsHandle = osThreadCreate(osThread(Task_Sensors), NULL);

  /* definition and creation of Task_Rpi */
  osThreadStaticDef(Task_Rpi, StartTask_Rpi, osPriorityHigh, 0, 512, Task_RpiBuffer, &Task_RpiControlBlock);
  Task_RpiHandle = osThreadCreate(osThread(Task_Rpi), NULL);

  /* definition and creation of Task_Bluetooth */
  osThreadStaticDef(Task_Bluetooth, StartTask_Bluetooth, osPriorityNormal, 0, 256, Task_BluetoothBuffer, &Task_BluetoothControlBlock);
  Task_BluetoothHandle = osThreadCreate(osThread(Task_Bluetooth), NULL);

  /* definition and creation of Task_Behavior */
  osThreadStaticDef(Task_Behavior, StartTask_Behavior, osPriorityHigh, 0, 512, Task_BehaviorBuffer, &Task_BehaviorControlBlock);
  Task_BehaviorHandle = osThreadCreate(osThread(Task_Behavior), NULL);

  /* definition and creation of Task_Kinematic */
  osThreadStaticDef(Task_Kinematic, StartTask_Kinematic, osPriorityHigh, 0, 512, Task_KinematicBuffer, &Task_KinematicControlBlock);
  Task_KinematicHandle = osThreadCreate(osThread(Task_Kinematic), NULL);

	/* definition and creation of myQueue01 */
  osMessageQStaticDef(myQueue01, 16, uint16_t, myQueue01Buffer, &myQueue01ControlBlock);
	myQueue01Handle = osMessageCreate(osMessageQ(myQueue01), NULL);
	osKernelStart();

	while (1)
	{

	}


}



/*******************************************************************************************************************************************************/
/*******************************************************************************************************************************************************/
/*******************************************************************************************************************************************************/
/*******************************************************************************************************************************************************/
/*******************************************************************************************************************************************************/


void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 128;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 600000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function - (Raspberry Pi) */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function - (Flex OLED) */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI3 init function - (Bluetooth Module) */
static void MX_SPI3_Init(void)
{

  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 32; //1MHz -> Cnt increases by 1000000 every 1sec -> 1=1µs so 1.5ms = 1500µs | old=32MHz/640000 = 50Hz - 640
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 32; //640; // 32MHz/333Hz = 96096
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, SERVO_PWR_EARS_Pin|SERVO_PWR_NECK_Pin|SERVO_PWR_HOOD_Pin|REG_3V3_EN_Pin
						  |PI_GPIO_A_Pin|PI_SIG_OFF_Pin|PI_SIG_ON_Pin|PI_PWR_ON_OFF_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PI_I2C_LINE_Pin|FLEX_OLED_CS_Pin|FLEX_OLED_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, MAIN_SWITCH_Pin|EYES_RST_Pin|REG_5V_EN_Pin|BT_CS_Pin
						  |BT_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_ACT_Pin|LED_B0_Pin|LED_ERR_Pin|PSU_EN_Pin, GPIO_PIN_RESET);
	/*Configure GPIO pins : SERVO_PWR_EARS_Pin SERVO_PWR_NECK_Pin SERVO_PWR_HOOD_Pin REG_3V3_EN_Pin */
	GPIO_InitStruct.Pin = SERVO_PWR_EARS_Pin|SERVO_PWR_NECK_Pin|SERVO_PWR_HOOD_Pin|REG_3V3_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : LTC4015_ALERT_Pin SIG_SYSTEM_Pin HODD_SW1_Pin HOOD_SW2_Pin */
	GPIO_InitStruct.Pin = LTC4015_ALERT_Pin|SIG_SYSTEM_Pin|HODD_SW1_Pin|HOOD_SW2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


	/*Configure GPIO pins : PI_I2C_LINE_Pin FLEX_OLED_CS_Pin FLEX_OLED_RST_Pin */
	GPIO_InitStruct.Pin = PI_I2C_LINE_Pin|FLEX_OLED_CS_Pin|FLEX_OLED_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PI_I2C_BUSY_Pin PI_GPIO_B_Pin SW_PI_BOOT_Pin SW_HOOD_Pin */
	GPIO_InitStruct.Pin = PI_I2C_BUSY_Pin|PI_GPIO_B_Pin|SW_PI_BOOT_Pin|SW_HOOD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : PI_GPIO_A_Pin PI_SIG_OFF_Pin PI_SIG_ON_Pin PI_PWR_ON_OFF_Pin */
	GPIO_InitStruct.Pin = PI_GPIO_A_Pin|PI_SIG_OFF_Pin|PI_SIG_ON_Pin|PI_PWR_ON_OFF_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : MAIN_SWITCH_Pin REG_5V_EN_Pin */
	GPIO_InitStruct.Pin = MAIN_SWITCH_Pin|REG_5V_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : EYES_RST_Pin BT_CS_Pin BT_RESET_Pin */
	GPIO_InitStruct.Pin = EYES_RST_Pin|BT_CS_Pin|BT_RESET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : SW_USER_Pin */
	GPIO_InitStruct.Pin = SW_USER_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SW_USER_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LED_ACT_Pin LED_B0_Pin LED_ERR_Pin PSU_EN_Pin */
	GPIO_InitStruct.Pin = LED_ACT_Pin|LED_B0_Pin|LED_ERR_Pin|PSU_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : DOCK_SENS_Pin */
	GPIO_InitStruct.Pin = DOCK_SENS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(DOCK_SENS_GPIO_Port, &GPIO_InitStruct);

	/* Flex OLED Display */
    GPIO_InitStruct.Pin = FLEX_OLED_SCK_Pin|FLEX_OLED_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}


/*******************************************************************************************************************************************************/
/*******************************************************************************************************************************************************/
/*******************************************************************************************************************************************************/
/*******************************************************************************************************************************************************/
/*******************************************************************************************************************************************************/


void StartTask_PwrMngt(void const * argument)
{
	LTC4015_ChargerState chargerState = 0x0000;
	LTC4015_ChargeStatus chargeStatus = 0x0000;
	LTC4015_LimitAlerts limitAlerts = 0x0000;
	LTC4015_ChargeStateAlerts stateAlerts = 0x0000;
	LTC4015_ChargeStatusAlerts statusAlerts = 0x0000;
	LTC4015_SystemStatus systemStatus = 0x0000;

	set_PSU_3V3(ON);
	set_MAIN_SWITCH(ON);
	osDelay(250);
	set_PSU_5V(ON);
	osDelay(250);
	set_PSU_DRS0101(ON);
	BIP_0();


	for(;;)
	{
		RPi_PwrMngt();
		Board_PwrMngt();

		//LTC4015_GetChargerState(&chargerState);
		//LTC4015_GetChargeStatus(&chargeStatus);
		//LTC4015_GetLimitAlerts(&limitAlerts);
		LTC4015_GetPowerVal();

		if( ((Charger.Power.BatVoltage > 7) && (Charger.Power.BatVoltage < 9.50)) && (Input.SIG_SYS == GPIO_PIN_SET) )
		{
			set_LED_ERR(ON);
			LowBattery_ctr ++;

			if(LowBattery_ctr>=50)
			{
				LowBattery_ctr = 0;
				BIP_4();
			}
		}

		PWM_FAN = (uint16_t)(Sensor.TempPSU*15);
		Sensor.Fan_Speed = PWM_FAN;

		osDelay(100);
	}

}



void StartTask_System(void const * argument)
{

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);	// FAN
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);	// Rpi LED
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);	// BUZZ

	SSD1306_Init();
	LTC4015_Init();
	EarsServos_Init();
	NeckServos_Init();
	//HoodServos_Init();

	PWM_LED_PI = 0;
	PWM_FAN = 500;
	PWM_BUZZ = 0;

	for(;;)
	{
		CheckInputs();
		osDelay(100);
	}

}



void StartTask_Activity(void const * argument)
{
	for(;;)
	{
		set_LED_ACT(ON);
		osDelay(75);
		set_LED_ACT(OFF);
		osDelay(150);
		set_LED_ACT(ON);
		osDelay(75);
		set_LED_ACT(OFF);
		osDelay(1000);
	}
}



void StartTask_Default(void const * argument)
{
	uint16_t RegVAl;
	uint16_t IR_Val;
	uint16_t ChargerState = 0xFF;
	float x, y, z;

	for(;;)
	{

		if(Input.SW_OLED_MENU == GPIO_PIN_SET)
		{
			set_LED_B0(ON);

			Flex_Oled_Menu ++;
			if(Flex_Oled_Menu > 3) { Flex_Oled_Menu = 0; }
			Flex_OLED_clearDisplay(CLEAR_ALL);
			Flex_OLED_Update();
			osDelay(100);
		}
		else
		{
			set_LED_B0(OFF);
		}


		if(Input.SW1 == GPIO_PIN_SET)
		{


			Gaits_DefaultPosition(100);
			osDelay(2000);

			IK_LegInit();

			Robot.BodyCmd.TransX	= 0;
			Robot.BodyCmd.TransY	= 0;
			Robot.BodyCmd.TransZ	= -20;
			Robot.BodyCmd.RotX	= 0;
			Robot.BodyCmd.RotY	= 0;
			Robot.BodyCmd.RotZ	= 0;
			IK_Run();

			osDelay(1000);
			Robot.BodyCmd.RotZ	= 5;
			IK_Run();
			osDelay(250);
			Head_SetPosition(PitchNeutral, YawNeutral+75, megaSlow);

			osDelay(1000);
			Robot.BodyCmd.RotZ	= -5;
			IK_Run();
			osDelay(250);
			Head_SetPosition(PitchNeutral, YawNeutral-75, megaSlow);

			osDelay(1000);
			Robot.BodyCmd.RotZ	= 0;
			IK_Run();
			osDelay(250);
			Head_SetPosition(PitchNeutral, YawNeutral, megaSlow);

			/*
			Robot.BodyCmd.TransX	= 20;
			osDelay(2000);
			Robot.BodyCmd.TransY	= 0;
			osDelay(2000);
			Robot.BodyCmd.RotX	= 10;
			osDelay(2000);
			Robot.BodyCmd.RotY	= 10;
			osDelay(2000);
			Robot.BodyCmd.RotZ	= 10;
*/
			/*
			Eyes_SetExpression(Sad, fast);
			Ears_SetPosition(EarL_Down, EarR_Down, slow);
			osDelay(2000);
			Eyes_SetExpression(Neutral, fast);
			Ears_SetPosition(EarL_Middle, EarL_Middle, slow);
			osDelay(2000);
			Eyes_SetExpression(Furious, fast);
			Ears_SetPosition(EarL_Up, EarR_Up, slow);
			osDelay(2000);
			Eyes_SetExpression(Neutral, fast);
			Ears_SetPosition(EarL_Middle, EarL_Middle, slow);
			 */
			//NeckServos_Init();
			//Gaits_StandPosition(75);
		}
		else
		{

		}

		osDelay(125);
	}

}



void Board_PwrMngt(void)
{
	uint8_t i;

	if(Input.SIG_SYS == GPIO_PIN_RESET)
	{
		set_LED_ERR(ON);
		Board_Shutdown_ctr++;
	}
	else
	{
		set_LED_ERR(OFF);
		Board_Shutdown_ctr = 0;
	}

	if(Board_Shutdown_ctr == 15)
	{
		for(i =0 ; i<10 ; i++)
		{
			set_LED_ERR(OFF);
			osDelay(50);
			set_LED_ERR(ON);
			osDelay(50);
		}

		BoardShutdownProcedure();
	}

}


void RPi_PwrMngt(void)
{
	uint16_t i, j;

	if( (Input.SW_PI_BOOT == GPIO_PIN_SET) && (Rpi_Shutdown_ctr < RPI_SHTDWN_MAX_TIMEOUT) )
	{
		Rpi_Shutdown_ctr++;
		set_RPI_PWR(ON);
		set_PI_SIG_ON(TRUE);
		set_PI_SIG_OFF(FALSE);
	}
	else
	{
		Rpi_Shutdown_ctr=0;
	}


	if(Rpi_Shutdown_ctr == 1)
	{
		for(i=0 ; i<500 ; i++)
		{
			PWM_LED_PI = i;
			HAL_Delay(3);
		}
	}


	if( (Rpi_Shutdown_ctr >= RPI_SHTDWN_MAX_TIMEOUT) /*&& (Input.RPI_RUNNING == GPIO_PIN_SET)*/ )
	{
		set_RPI_PWR(FALSE);
		set_PI_SIG_OFF(TRUE);

		set_LED_ERR(ON);

		for(j=0 ; j<=3 ; j++)
		{
			PWM_LED_PI = 0;
			HAL_Delay(50);
			PWM_LED_PI = 500;
			HAL_Delay(50);
		}

		// Delay for the Rpi to shutdown (30ms x 500 = 15s)
		for(i=500 ; i>0 ; i--)
		{
			PWM_LED_PI = i;
			HAL_Delay(30);
		}
		PWM_LED_PI = 0;

		set_LED_ERR(OFF);

		set_RPI_PWR(OFF);
		set_PI_SIG_OFF(FALSE);

		Rpi_Shutdown_ctr = 0;
	}

}



void StartTask_FlexOled(void const * argument)
{
	uint8_t OLED_Ready = 0;

	osDelay(100);
	Flex_OLED_Init();
	Robot.OLED.OLED_Contrast = 50;
	Flex_OLED_setContrast(Robot.OLED.OLED_Contrast);
	OLED_Ready = Flex_OLED_StartupAnimation(10);

	for(;;)
	{
		if( (OLED_Ready == OK) && (Input.SIG_SYS == GPIO_PIN_SET) )
		{
			switch(Flex_Oled_Menu)
			{
				case Battery:
					Flex_OLED_Menu_Battery();
				break;
				case Modes:
					Flex_OLED_Menu_Modes();
				break;
				case Sensors:
					Flex_OLED_Menu_Sensors();
				break;
				case SPI_1:
					Flex_OLED_Menu_SPI1(&RPI_RxBuff);
				break;

				default:
				break;
			}
		}
		osDelay(100);
	}

}



void StartTask_Sensors(void const * argument)
{
	float Pitch, Roll;
	float rotX, rotY, rotZ;
	uint32_t val;
	float Temp_PSU = 0;
	float Temp_CHG = 0;
	float Temp_AVG = 0;
	uint16_t IR_Val;
	uint8_t IR_Dist;
	uint16_t LTC_VIN;

	osDelay(250);
	//I2C_Device_Check();

	LM75B_Init();
	ADC101_Init();
	LSM9DS1_Init();
	osDelay(100);

	for(;;)
	{
		LM75B_CHG_GetTemp(&Temp_CHG);
		LM75B_PSU_GetTemp(&Temp_PSU);
		Temp_AVG = (Temp_CHG + Temp_PSU)/2;
		ADC101_ReadIR(&IR_Val);
		LSM9DS1_ReadAngle(&Roll, &Pitch);
		//LSM9DS1_ReadGyro(&rotX, &rotY, &rotZ);

		Sensor.IMU.Pitch   	= Pitch;
		Sensor.IMU.Roll    	= Roll;
		Sensor.TempCharger 	= Temp_CHG;
		Sensor.TempPSU		= Temp_PSU;
		Sensor.TempAverage  = Temp_AVG;
		Sensor.dist_IR		= IR_Val;

		osDelay(50);
	}
}


void StartTask_Rpi(void const * argument)
{
	uint8_t aTxBuffer[16];
	uint8_t aRxBuffer[16];

	SPI1_init_IT();

	for(;;)
	{
		HAL_SPI_TransmitReceive_IT(&hspi1, RPI_TxBuff, RPI_RxBuff, BUFFERSIZE);
		osDelay(75);
	}

}


void StartTask_Bluetooth(void const * argument)
{
	uint8_t header_master1[5U] = {0x0A, 0x00, 0x00, 0x00, 0x00};
	uint8_t header_master2[5U] = {0x0B, 0x00, 0x00, 0x00, 0x00};
	uint8_t header_slave[5U]  = {0x00, 0x00, 0x00, 0x00, 0x00};
	/*
	osDelay(1500);

	set_BT_RST(ON);
	osDelay(10);

	set_BT_CS(LOW);
	osDelay(1);
	//HAL_SPI_TransmitReceive(&hspi3, &header_master1, &header_slave, 5, 100);
	HAL_SPI_Transmit(&hspi3, &header_master2, 5, 100);
	HAL_SPI_Receive(&hspi3, &header_slave,5, 100);
	asm("NOP");
	HAL_SPI_TransmitReceive(&hspi3, &header_master2, &header_slave, 5, 100);
	asm("NOP");
	set_BT_CS(HIGH);
	asm("NOP");
 	*/

	for(;;)
	{

		osDelay(1000);
	}

}


void StartTask_Behavior(void const * argument)
{
	int8_t i=0,j=0;

	Robot.Eyes.Contrast = 50;

	osDelay(1500);
	Eyes_WakingUp(fast);
	Ears_SetPosition(EarL_Middle, EarR_Middle, verySlow);

	osDelay(1500);
	//Gaits_DefaultPosition(100);

	for(;;)
	{
		osDelay(7500);
		Eyes_Blink();
	}

}


void StartTask_Kinematic(void const * argument)
{

	osDelay(1000);
	IK_LegInit();
	DRS0101_Clear(BROADCAST_ID);
	DRS0101_setTorque(BROADCAST_ID, TORQUE_ON);

	//Gaits_DefaultPosition(70);

	Robot.BodyCmd.TransX	= 0;
	Robot.BodyCmd.TransY	= 0;
	Robot.BodyCmd.TransZ	= 0;
	Robot.BodyCmd.RotX	= 0;
	Robot.BodyCmd.RotY	= 0;
	Robot.BodyCmd.RotZ	= 0;

	for(;;)
	{

		//IK_Run();
		osDelay(100);
	}

}

/*******************************************************************************************************************************************************/
/*******************************************************************************************************************************************************/
/*******************************************************************************************************************************************************/


// SPI first call occurs first, put 00 into the buffer until the host to take
void SPI1_init_IT(void)
{
	memset(RPI_TxBuff, 0, BUFFERSIZE);
	memset(RPI_RxBuff, 0, BUFFERSIZE);
	HAL_SPI_TransmitReceive_IT(&hspi1, RPI_TxBuff, RPI_RxBuff, BUFFERSIZE);
}


void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi==&hspi1)
	{
		set_LED_ERR(ON);
		BIP_3();
	}
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	uint32_t i;

	if(hspi==&hspi1)
	{
		RPI_TxBuff[0] = 0x01+j;
		RPI_TxBuff[1] = 0x02+j;
		RPI_TxBuff[2] = 0x03+j;
		RPI_TxBuff[3] = 0x04+j;
		RPI_TxBuff[4] = 0x05+j;
		RPI_TxBuff[5] = 0x06+j;
		RPI_TxBuff[6] = 0x07+j;
		RPI_TxBuff[7] = 0x08+j;

		j++;
		HAL_SPI_TransmitReceive_IT(&hspi1, RPI_TxBuff, RPI_RxBuff, 8);

		for(i=0; i<=1000000; i++)
		{
			asm("NOP");
		}
	}


}


void CheckInputs(void)
{
	Input.SW1 			= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
	//Input.SW_HOOD		= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15);
	Input.SW_OLED_MENU	= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15);
	Input.SW_PI_BOOT	= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14);
	Input.RPI_RUNNING   = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9);
	Input.SIG_HOOD1		= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);
	Input.SIG_HOOD2		= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9);
	Input.SIG_SYS		= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5);
	Input.SIG_DOCK		= HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1);
	Input.CHARGER_FAULT = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);
	Input.PI_GPIO_B		= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_8);
}


void I2C_Device_Check(void)
{
	uint8_t addr, ctr, i, j;
	volatile uint8_t valid_addr[24];

	set_LED_B0(ON);

	for(j=0 ; j<24; j++)
	{
		valid_addr[j] = 0x00;
	}

	ctr = 0;
	addr= 0;

	HAL_I2C_DeInit(&hi2c2);
	HAL_Delay(10);
	HAL_I2C_Init(&hi2c2);
	HAL_Delay(10);

	for (addr=0 ; addr<=254 ; addr++)
	{
		if(HAL_I2C_IsDeviceReady(&hi2c2, addr, 5, 100) == HAL_OK)
		{
			valid_addr[ctr] = addr;
			ctr++;
		}
		HAL_Delay(10);
	}

	set_LED_B0(OFF);
}



/*******************************************************************************************************************************************************/
/*******************************************************************************************************************************************************/
/*******************************************************************************************************************************************************/


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
	set_LED_ERR(ON);
	while(1)
	{

	}
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
