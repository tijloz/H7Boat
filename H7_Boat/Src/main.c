/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DACResolution 12
#define TurnOn 3000
#define OutMax 4096
#define Averages 5
#define SetpointStep 10
#define MaximumSD 50
#define CalibrationDelay 10000
#define MaxCommands 10
#define Threshold 8
#define SampleTimeInSec 0.1
#define AUTOMATIC 1
#define MANUAL 0
#define UpdateStall 30
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

int len, PWM;
char  Rx_index, Rx_data[2], Rx_Buffer[100], Transfer_cplt, buffer[100], bufferEcho[100];
char strt = '!', del = '!', Start, Timer, Channel, Brightness[4]="300", Rx_String[100];

unsigned char j;
uint8_t Device, State;

/* Buffer used for transmission */

unsigned Setpoint = TurnOn;
unsigned MaxSetpoint = 4086;
float PreviousVoltages[Averages];
float MeanVoltage;
float AverageVoltage;
unsigned Calibrating = 0;

char *Command[MaxCommands];
unsigned WordIndex;

unsigned ADC_raw[3];
float ADC_float[3];
float LOAD_CURRENT;
float LOAD_VOLTAGE;
float TEMPERATURE;

float mVoltsPerBit = 0.00005035;	//(3.300/65535)
float mAmpsPerBit = 0.00005035;	//(3.300/65535)
float DegsPerBit = ((0.107421875/256));	//(3.3/65535)/0.01
float ScaleFactor = 1;
float offset = -1024;
unsigned Vout = 128;

uint8_t inAuto = 0;
int Input, lastInput, Output, kp = 10, kd = 1; updateCount = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM13_Init(void);
/* USER CODE BEGIN PFP */
static void StringSort(void);
static void CalculateValues(void);
static void SendValues(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void CalculateValues(void){
	LOAD_VOLTAGE = (((ADC_float[2]*ScaleFactor)+offset)*mVoltsPerBit);
	if (Calibrating == 1){return;}
	LOAD_CURRENT = (((ADC_float[0]*ScaleFactor)+offset)*mAmpsPerBit);
	TEMPERATURE = (((ADC_float[1]*ScaleFactor)+offset)*DegsPerBit);
	updatePID();
	updateCount++;
	if(updateCount==UpdateStall){
		updateCount==0;
		SendValues();
	}
}

void SendValues(void){

	len=sprintf(buffer, "!Voltage!%.3f!\n", (LOAD_VOLTAGE));
	HAL_UART_Transmit(&huart3, buffer, len, 1000);

	len=sprintf(buffer, "!Current!%.3f!\n", LOAD_CURRENT);
	HAL_UART_Transmit(&huart3, buffer, len, 1000);

	len=sprintf(buffer, "!Temperature!%.3f!\n", TEMPERATURE);
	HAL_UART_Transmit(&huart3, buffer, len, 1000);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef*hadc){
	uint8_t i;
	 if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC))
	  {
	  ADC_raw[i] = HAL_ADC_GetValue(hadc);
	  ADC_float[i] = (float)ADC_raw[i];
	  i++;

	  }
	if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOS))
		{
			ADC_raw[i] = HAL_ADC_GetValue(hadc);
			ADC_float[i] = (float)ADC_raw[i];
			i=0;
			CalculateValues();
		}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance==TIM8) //check if the interrupt comes from TIM8
        {
			len=sprintf(buffer, "!Protection 1 Alive!!\n");
			HAL_UART_Transmit(&huart3, buffer, len, 1000);
        }
    if (htim->Instance==TIM1) {
    	if (Calibrating == 1){return;}
        	HAL_ADC_Start_IT(&hadc1);
        }
}

void Calibrate(void){
	Calibrating=1;											// Enable the Calibrating flag to disable normal voltage/current/temperature measurements
	float VoltageSum = 0;
	float SD = 5;
	float Difference;
	float Varsum;
	float Variance;
	float MeanVoltages[100];
	unsigned k=0;
	unsigned m=0;

recalibrate:
	for (Setpoint=TurnOn; Setpoint < MaxSetpoint; Setpoint = Setpoint + SetpointStep){

		while (SD > MaximumSD) {
			VoltageSum = 0;
			Difference = 0;
			Variance = 0;
			Varsum = 0;
			k=0;

			while (k<Averages){
				HAL_ADC_Start_IT(&hadc1);					// Take ADC readings
				PreviousVoltages[k]=LOAD_VOLTAGE;
				HAL_Delay(CalibrationDelay);
				k++;
				VoltageSum = VoltageSum + LOAD_VOLTAGE;		// Add the current LOAD_VOLTAGE to the running sum
				MeanVoltage = (VoltageSum/k);				// and take the mean
				Difference = LOAD_VOLTAGE - MeanVoltage;	// The difference is the current LOAD_VOLTAGE less the running MeanVoltage
				Varsum = Varsum + pow(Difference,2);		// and Varsum is the running sum of the squares of the Difference
			}
		  Variance = Varsum / (float)k;
		  SD = sqrt(Variance);
		}
		MeanVoltages[m]=MeanVoltage;
		m++;
	}

	for (m=0; m<100; m++) {
		VoltageSum = 0;
		Difference = 0;
		Variance = 0;
		Varsum = 0;
		k=0;

		while (k<5){
			k++;
			MeanVoltages[(m+k)];
			VoltageSum = VoltageSum + LOAD_VOLTAGE;
			MeanVoltage = (VoltageSum/k);
			Difference = LOAD_VOLTAGE - MeanVoltage;
			Varsum = Varsum + pow(Difference,2);
		}
	  Variance = Varsum / (float)k;
	  SD = sqrt(Variance);
	  if (SD<Threshold ){
		  Setpoint=(TurnOn+(m*SetpointStep));				// Set the new Setpoint
		  Calibrating=0;									// Disable the Calibrating flag to resume normal voltage/current/temperature measurements
		  return;											// break and return from the loop
	  }
	}
	/*
	 * If this is reached calibration failed
	 */
	len=sprintf(buffer, "!CalibrationFailed!1!");			// Report the failure
	HAL_UART_Transmit(&huart3, buffer, len, 1000);
	goto recalibrate;										// and attempt to recalibrate
	SetMode(AUTOMATIC);
}

void updatePID(void){
	   if(!inAuto) return;

	  /*Compute all the working error variables*/
	  Input = ((ADC_float[2]*ScaleFactor)+offset);			// Take the corrected ADC value for voltage and store as input
	  double error = Setpoint - Input;						// Find the error value
	  double dInput = (Input - lastInput);					// Input difference

	  /*Compute PID Output*/
	  Output = kp * error - kd * dInput;					// Find the new output
	  if(Output > OutMax) Output = OutMax;					// Stop wind-up
	  else if(Output < TurnOn) Output = TurnOn;				// Stop wind-down
	  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, Output);
}

void SetTunings(double Kp, double Kd)
{
   if (Kp<0 || Kd<0) return;
   kp = Kp;
   kd = Kd / SampleTimeInSec;
}

void InitialisePID(){
	lastInput=Input;
}

void SetMode(int Mode)
{
    uint8_t newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto)
    {  /*we just went from manual to auto*/
        Initialize();
    }
    inAuto = newAuto;
}

void UpdateSetpoint(void){
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, Setpoint);
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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM1_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_IT(&hadc1);
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, Vout);
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim8);

  HAL_UART_Receive_IT(&huart3, Rx_data, 1);	// Activate USART rx interrupt

//  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
//  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
//  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
//  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
//  HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_3);
//  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);

//  AnchorPWM();
//  NavPWM();
//  DeckPWM();
//  //GalleyPWM();
//  Cabin1PWM();
//  Cabin2PWM();
//  RidePWM();


  Calibrate();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if (Transfer_cplt==1) {
		  StringSort();
		  Transfer_cplt=0;
	  }

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

  /**Supply configuration update enable 
  */
  MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);
  /**Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while ((PWR->D3CR & (PWR_D3CR_VOSRDY)) != PWR_D3CR_VOSRDY) 
  {
    
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_ADC
                              |RCC_PERIPHCLK_USB|RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_CLKP;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /**Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.BoostMode = DISABLE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure the ADC multi-mode 
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_16CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /**DAC Initialization 
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /**DAC channel OUT1 config 
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 48000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19960;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 480;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 499;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 480;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 499;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 480;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 499;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 480;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 499;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 48000;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 29940;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 480;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 333;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

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
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.Prescaler = UART_PRESCALER_DIV1;
  huart3.Init.FIFOMode = UART_FIFOMODE_DISABLE;
  huart3.Init.TXFIFOThreshold = UART_TXFIFO_THRESHOLD_1_8;
  huart3.Init.RXFIFOThreshold = UART_RXFIFO_THRESHOLD_1_8;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.ep0_mps = DEP0CTL_MPS_64;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PF12 PF14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PE9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//void SetLight(void) {
//	__HAL_TIM_SetCompare(Timer, Channel, Brightness);
//}
//
//void AnchorPWM(void) {
//	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1,PWM);
//}
//void NavPWM(void) {
//	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2,PWM);
//}
//void DeckPWM(void) {
//	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2,PWM);
//}
////void GalleyPWM(void) {
////	__HAL_TIM_SetCompare(&htim15, TIM_CHANNEL_1,PWM);
////}
//void Cabin1PWM(void) {
//	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3,PWM);
//}
//void Cabin2PWM(void) {
//	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1,PWM);
//}
//void RidePWM(void) {
//	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3,PWM);
//}

void *StringSlice(void){

	    int m = 0;
	    char *p = strtok (Rx_Buffer, "!");

	    while (p != "\r")
	    {
	        Command[m++] = p;
	        printf("%s\n", p);
	    	p = strtok (NULL, "!");
	    }

	return 0;
}



void StringSort(void)
{
	/*
	 * Split the string into components
	 */

	StringSlice();
	Device=(uint8_t)(Command[1]);				// Determine the device type
	Timer = Command[2];							// Load the timer address
	Channel=Command[3];							// Set the channel address
	State = (uint8_t)(Command[4]);				// Boolean identifier for State

	char strLen = strlen(Rx_Buffer);			// Find the string length of the Rx_Buffer
	switch (strLen) {
		case 5 :		Brightness[0] = '0';
						Brightness[1] = '0';
						strcpy(Brightness, Command[5]);

		case 6 :		Brightness[0] = '0';
						strcpy(Brightness, Command[5]);

		case 7 :		strcpy(Brightness, Command[5]);
	}
	PWM = atoi(Brightness);
	/*
	 * Perform the operation
	 */
	switch(Device)
	{
	case '1':									// Case 1 is a light

		switch (Timer) {
//			case '1':
//				switch (Channel) {
//				case '1': __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,PWM);
//				switch (State) {case '0': HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1); case '1': HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);}
//				case '2': __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2,PWM);
//				switch (State) {case '0': HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2); case '2': HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);}
//				case '3': __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,PWM);
//				switch (State) {case '0': HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3); case '3': HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);}
//				case '4': __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4,PWM);
//				switch (State) {case '0': HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4); case '1': HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);}
//					}
		case '2':
			switch (Channel) {
				case '1': __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1,PWM);
					switch (State) {
						case '0': HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1); break;
						case '1': HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
					}break;
				case '2': __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2,PWM);
					switch (State) {
						case '0': HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2); break;
						case '1': HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
					} break;
				case '3': __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3,PWM);
					switch (State) {
						case '0': HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3); break;
						case '1': HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
					}break;
				case '4': __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4,PWM);
					switch (State) {
						case '0': HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4); break;
						case '1': HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
					} break;
			} break;

		case '3':
			switch (Channel) {
				case '1': __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1,PWM);
					switch (State) {
						case '0': HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1); break;
						case '1': HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
					} break;
				case '2': __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2,PWM);
					switch (State) {
						case '0': HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2); break;
						case '1': HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
					} break;
				case '3': __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3,PWM);
					switch (State) {
						case '0': HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3); break;
						case '1': HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
					} break;
				case '4': __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4,PWM);
					switch (State) {
						case '0': HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4); break;
						case '1': HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
					} break;
			} break;

		case '4':
			switch (Channel) {
				case '1': __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1,PWM);
					switch (State) {
						case '0': HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1); break;
						case '1': HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
					} break;
				case '2': __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2,PWM);
					switch (State) {
						case '0': HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2); break;
						case '1': HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
					} break;
				case '3': __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3,PWM);
					switch (State) {
						case '0': HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3); break;
						case '1': HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
					} break;
				case '4': __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4,PWM);
					switch (State) {
						case '0': HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4); break;
						case '1': HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
					} break;
			} break;

		case '5':
			switch (Channel) {
				case '1': __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1,PWM);
					switch (State) {
						case '0': HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1); break;
						case '1': HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
					}break;
				case '2': __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2,PWM);
					switch (State) {
						case '0': HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2); break;
						case '1': HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
					} break;
				case '3': __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3,PWM);
					switch (State) {
						case '0': HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_3); break;
						case '1': HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
					} break;
				case '4': __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4,PWM);
					switch (State) {
						case '0': HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_4); break;
						case '1': HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
					} break;
			} break;

//			case '8':
//				switch (Channel) {
//					case '1': __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,PWM);
//						switch (State) {
//							case '0': HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
//							case '1': HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);}
//					case '2': __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,PWM);
//						switch (State) {
//							case '0': HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
//							case '1': HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);}
//					case '3': __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3,PWM);
//						switch (State) {
//							case '0': HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
//							case '1': HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);}
//					case '4': __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4,PWM);
//						switch (State) {
//							case '0': HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_4);
//							case '1': HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);}
//				}
		} break;
	}
	for (uint8_t i=0;i<100;i++) {Rx_String[i]=0;}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t i;
    if (huart->Instance == USART3)  //current UART
	{
    	if (Rx_index==0) {for (i=0;i<100;i++) {
			Rx_Buffer[i]=0;   //clear Rx_Buffer before receiving new data
			Rx_String[i]=0;
		}}


	if (Rx_data[0]!=13) //if received data different from ascii 13 (enter)
	{
		Rx_Buffer[Rx_index++]=Rx_data[0];    //add data to Rx_Buffer
	}
	else            //if received data = 13
	{
		Rx_Buffer[Rx_index++]=Rx_data[0];    //add data to Rx_Buffer
		Rx_Buffer[Rx_index++]="!";    //add data to Rx_Buffer
		Rx_index=0;
		Transfer_cplt=1;//transfer complete, data is ready to read
	}

	HAL_UART_Receive_IT(&huart3, Rx_data, 1);   //activate UART receive interrupt every time
	strcpy (Rx_String, Rx_Buffer);
//	if(Transfer_cplt == 1)
//	{
//		len=sprintf(bufferEcho, Rx_Buffer);
//		HAL_UART_Transmit_IT(&huart3, bufferEcho, len);
//	}

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
