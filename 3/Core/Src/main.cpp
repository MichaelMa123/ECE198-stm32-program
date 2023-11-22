/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <iostream>
#include <cmath>
#include <vector>
#include <climits>
#include <algorithm>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define ADC_BUF_LED 4096
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
i2cLcd_HandleTypeDef h_lcd;

/* USER CODE BEGIN PV */
uint16_t adc_buf[ADC_BUF_LED];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void delay(uint32_t tick);
bool Button_press();
bool peak_finder();
void pushvalue(long long n,bool a);
long beat_per_mins();
//void pushvalue(long n,bool a);
void enable_delay(void);
void pushWarning(long n);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int mean(int array[],std::size_t n)// calculate the mean of the value.
{
	int sum{0};
	for(std::size_t m{};m<n;m++)
	{
		sum+=array[m];
	}
	sum=sum/n;
	return sum;
}
int standerd_dev(int array[],std::size_t n,int mean)
{
	int sum{0};
	for(unsigned int a{0};a<n;a++)
	{
		sum+=(array[a]-mean)*(array[a]-mean);
	}
	int final{0};
	final=sum/n;
	final=sqrt(final);
	return final;
}

//bool peak_finder()
//{
//	long long max_value=0;
//	bool ispeak=false;
//	bool result=false;
//	long long raw_value{};
//	int a{};
//	long long reader=0;
//	long start=HAL_GetTick();
//	long now=HAL_GetTick();
////	for(int n{0};now<(start+20);n++)
////		//reader=0
////		//for(long long n{0};n<200;n++)
////	{
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
//		HAL_ADC_Start(&hadc1);
//		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//				//pushvalue(HAL_ADC_GetValue(&hadc1),true);
//		raw_value = HAL_ADC_GetValue(&hadc1);					//pushvalue(n,true);
////				//++n;					a=n;
////		//pushvalue(a,true);
////		now=HAL_GetTick();
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
////		a++;
////	}
//	raw_value=reader/a;
//	if(raw_value*2<max_value)
//	{
//		max_value=raw_value*0.8;
//	}
//	if(raw_value>max_value-5)
//	{
//		if(raw_value>max_value)
//		{
//			max_value=raw_value;
//		}
//		if(ispeak==false)
//		{
//			result=true;
//		}
//		else if (raw_value<max_value-10)
//		{
//			ispeak=false;
//			max_value=max_value-10;
//		}
//		return result;
//	}
//	return 0;
//}
//long beat_per_mins()
//{
//	bool n=true;
//	int beat_per_mls{};
//	while(n)
//	{
//		//int beat_per_m{};
//		int rate{};
//		if(peak_finder())
//		{
//			rate=60000/beat_per_mls;
//			pushvalue(rate,true);
//			return rate;
//		}
//		else
//		{
//			enable_delay();
//			delay(840000);
//		}
//		beat_per_mls+=10;
//
//	}
//	//return rate;
//	return 0;
//}
void enable_delay(void)
{
	DWT->CYCCNT = 0;
	CoreDebug->DEMCR |= 0x01000000;
	DWT->CTRL |= 1;
}
void delay(uint32_t tick)
{
	uint32_t start = DWT->CYCCNT;
	uint32_t current = 0;
do
{
	current = DWT->CYCCNT;
}
while((current - start) < tick);
}
bool Button_press()
{
	 if(HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
		 {
			 return true;
		 }
	 return false;
}
//void pushvalue(int n,bool a)
//{
//	if(a==true)
//	{
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
//		HAL_ADC_Start(&hadc1);
//		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//		int reader = HAL_ADC_GetValue(&hadc1);
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
//
//		i2cLcd_Init(&h_lcd);
//		i2cLcd_ClearDisplay(&h_lcd);
//		std::string s = std::to_string(reader);
//		std::size_t i =0;
//		while(s[i])
//		{
//			  i2cLcd_SendChar(&h_lcd, s[i]);
//	  //HAL_Delay(100);
//			  i++;
//		}
//		//delete [] str;
//	}
//
//}
void pushvalue(long long n,bool a)
{
	if(a==true)
	{
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
//		HAL_ADC_Start(&hadc1);
//		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//		int reader = HAL_ADC_GetValue(&hadc1);
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

		i2cLcd_Init(&h_lcd);
		i2cLcd_ClearDisplay(&h_lcd);
		std::string s = std::to_string(n);
		//std::string s = std::to_string(reader);
		std::size_t i =0;
		while(s[i])
		{
			  i2cLcd_SendChar(&h_lcd, s[i]);
	  //HAL_Delay(100);
			  i++;
		}
		//delete [] str;
	}

}

void pushWarning(long n)
{
	//i2cLcd_Init(&h_lcd);
		i2cLcd_ClearDisplay(&h_lcd);
		std::string s = "Warning, abnormal heart rate ";
		std::string u {};
		if(n>160)
		{
			u="Heart rate higer than 160";
		}
		else if(n<40&&n>10)
		{
			u="Warning Heart rate Low ";
		}
		else if(n<=10)
		{
			u="warning heart rate lower that 10";
		}

		std::size_t i =0;
		while(s[i])
		{
			  i2cLcd_SendChar(&h_lcd, s[i]);
			  //HAL_Delay(100);
			  i++;
		}
		i=0;
		if(n<40||n>160)
		{
			while(u[i])
			{
				i2cLcd_SendChar(&h_lcd, u[i]);
			}
		}


}
//int beat20()
//{
//	const float Threshold = 0.7;
//	const int min_diff = 10;
//	int const buffer{250};
//	int nsap=0;
//	int index=0;
//	int sample_buffer[buffer];
//	long last=0;
//	int max_value=0;
//	int min_value=1024;
//	while(true)
//		{
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
//	HAL_ADC_Start(&hadc1);
//	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//	sample_buffer[index] = HAL_ADC_GetValue(&hadc1);
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
//	if(nsap>=buffer&&HAL_GetTick()-last>500)
//	{
//		last=HAL_GetTick();
//
//	int max_value=0;
//	int min_value=1024;
//	for(int i = 0; i <buffer; i++) {
//	      max_value = std::max(sample_buffer[i], max_value);
//	      min_value = std::min(sample_buffer[i], min_value);
//	    }
//	float difference=std::max(max_value-min_value,min_diff);
//	float threshold = difference * Threshold + min_value;
//	int heart_beata{0};
//	int heart_beatb{0};
//	int heart_beatc{0};
//	for(int i = 1; i < buffer; i++)
//	{
//		if(sample_buffer[(index+i+1)]%buffer >= threshold && sample_buffer[(index+i)%buffer] < threshold)
//		{
//			if(heart_beatc && i-heart_beatc > 15 && i-heart_beatc < 150)
//			{
//			heart_beatb += i-heart_beatc;
//			heart_beata++;
//			}
//			heart_beatc = i;
//		}
//		float bpm = 60000 * heart_beata/ (heart_beatb * 20);
//		if(heart_beata>3)
//		{
//
//		}
//	}
//	}
//	else
//	{
//		nsap++;
//	}
//	index=(index+1)%buffer;
//	enable_delay();
//	delay(1680000);
//}
//}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//uint8_t i2c_lcd_addr = (0x27<<1);
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
  MX_ADC1_Init();
  uint8_t i2c_lcd_addr = (0x27<<1);
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  std::size_t cap{100};
  int heart[cap]{0};
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_buf,ADC_BUF_LED);
  std::vector<long> Time{0};
  unsigned int count{0};
  const float Threshold = 0.7;
  	const int min_diff = 10;
  	int const buffer{250};
  	int nsap=0;
  	int index=0;
  	int sample_buffer[buffer];
  	long last=0;
  	int max_value=0;
  	int min_value=1024;
  //unsigned int cycle;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  bool compaired {false};
  int mean_rate{0};
  int std_dev{0};
  int warning{0};
  unsigned int cycle_count{};
  unsigned int warning_cycle{};
  //int beat_per_mins2{0};
  bool pressed{false};
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET);
  i2cLcd_CreateHandle(&h_lcd, &hi2c1, i2c_lcd_addr);
  i2cLcd_Init(&h_lcd);
//  int dif{};
//  long now{};
//  long start{};
  //long time2_beat{peak_finder()};
  while (1)
  {
	  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
	  	HAL_ADC_Start(&hadc1);
	  	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  	sample_buffer[index] = HAL_ADC_GetValue(&hadc1);
	  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
	  	if(nsap>=buffer&&(HAL_GetTick()-last)>500)
	  	{
	  		last=HAL_GetTick();

	  	int max_value=0;
	  	int min_value=1024;
	  	for(int i = 0; i <buffer; i++)
	  	{
	  	      max_value = std::max(sample_buffer[i], max_value);
	  	      min_value = std::min(sample_buffer[i], min_value);
	  	}
	  	float difference=std::max(max_value-min_value,min_diff);
	  	float threshold = difference * Threshold + min_value;
	  	int heart_beata{0};
	  	int heart_beatb{0};
	  	int heart_beatc{0};
	  	for(int i = 1; i < buffer; i++)
	  	{
	  		if(sample_buffer[(index+i+1)%buffer] >= threshold && sample_buffer[(index+i)%buffer] < threshold)
	  		{
	  			if(heart_beatc && i-heart_beatc > 15 && i-heart_beatc < 150)
	  			{
	  			heart_beatb += i-heart_beatc;
	  			heart_beata++;
	  			}
	  			heart_beatc = i;
	  		}
	  		int bpm = 60000 * heart_beata / (heart_beatb * 20);
	  		if(heart_beata>3)
	  		{
	  			heart[count]=bpm;
	  			count++;
				if(count==100)
				{
					count=0;
					compaired=true;
				}
	  			pushvalue(heart[count],pressed);

	  		}
	  	}
	  	}
	  	else
	  	{
	  		nsap++;
	  	}
	  	index=(index+1)%buffer;
	  if(pressed==false)
	  {
	  	pressed=Button_press();
	  }
	  //heart[count]=beat_per_mins();
	  pushvalue(heart[count],pressed);
//	  if(count==100)
//	  {
//		  count=0;
//		  compaired=true;
//
//	  }

	  	  	  	//cycle++;
	  	  pushvalue(heart[count],pressed);
	  	  	  	  if(compaired&&(count%10==0))
	  	  	  	  {
	  	  	  		  mean_rate=mean(heart,cap);
	  	  	  		  std_dev=standerd_dev(heart,cap,mean_rate);
	  	  	  	  }
	  	  	  	if(abs(heart[count]-mean_rate)>std_dev)
	  	  	  		{
	  	  	  			warning++;
	  	  	  			warning_cycle=cycle_count;
	  	  	  		}
	  	  	  	if(warning>10)
	  	  	  	{
	  	  	  		pushWarning(beat_per_mins());
	  	  	  		pressed=true;
	  	  	  	}
	  	  	  	  if(((cycle_count-warning_cycle)%1000==0)&&warning!=0)
	  	  	  	  {
	  	  	  		  warning--;

	  	  	  	  }
	  	  	  	if(((cycle_count)%1000==0))
	  	  	  		  	  {
	  	  	  		  		  pressed=false;

	  	  	  		  		  i2cLcd_ClearDisplay(&h_lcd);

	  	  	  		  	  }
	  	  	  	  cycle_count++;
	  	  	  if(pressed==false)
	  	  	  	  {
	  	  	  	  	  	  pressed=Button_press();
	  	  	  	  }
	  	  	if(cycle_count==(UINT_MAX-15))
	  	  	{
	  	  		cycle_count=0;
	  	  	}
	  	  //count++;
	  	enable_delay();
	  	delay(168000);
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(USART_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA10 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_SET);
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_RESET);
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
