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
long peak_finder();
void pushvalue(int n,bool a);
void enable_delay(void);
void pushWarning(int n);
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

long peak_finder()
{
	long millis {};
	//enable_delay();
	//int raw{};
	//int raw2{};
	//return millis;
	//uint32_t times{860};
	//int t2{1000};
	//bool peak{true};
	//delay(times);
	std::vector<long long> slops{};
	unsigned long reader{};
	unsigned long recorder[5]{0};
	while(true)
	{
		unsigned long now=HAL_GetTick();
		unsigned int n =0;
		unsigned long start=HAL_GetTick();
		for(int p{0};p<5;p++)
		{
			while(now<(start+20))
		{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		reader += HAL_ADC_GetValue(&hadc1);
		n++;
		now=HAL_GetTick();
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
		}
			reader/=n;
			recorder[p]=reader;
			//return millies;
		}
		//HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		//raw = HAL_ADC_GetValue(&hadc1);
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
		//delay(times);
		//HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		//raw2=HAL_ADC_GetValue(&hadc1);
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
		long long slope{(recorder[4]-recorder[0])};

		slops.push_back(slope);
		if(	slops[slops. size()-2]>0&&slops[slops.size()-1]<0)
		{
			millis = HAL_GetTick();
			return millis;
		}
		if(slops.size()>10)
		{
			slops.erase(slops.begin(), slops.end() - 2);
		}
	}
	return 0;
}
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
void pushvalue(int n,bool a)
{
	if(a==true)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		int reader = HAL_ADC_GetValue(&hadc1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

		i2cLcd_Init(&h_lcd);
		i2cLcd_ClearDisplay(&h_lcd);
		std::string s = std::to_string(n);
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
void pushWarning(int n)
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
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t i2c_lcd_addr = (0x27<<1);
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
  //uint8_t i2c_lcd_addr = (0x27<<1);
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  std::size_t cap{100};
  int heart[cap]{0};
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_buf,ADC_BUF_LED);
  std::vector<long> Time{0};
  unsigned int count{0};
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  bool compaired {false};
  int mean_rate{0};
  int std_dev{0};
  int warning{0};
  unsigned int cycle_count{};
  unsigned int warning_cycle{};
  int beat_per_mins{0};
  bool pressed{false};
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET);
  i2cLcd_CreateHandle(&h_lcd, &hi2c1, i2c_lcd_addr);
  i2cLcd_Init(&h_lcd);
  int dif{};
  long now{};
  long start{};
  //long time2_beat{peak_finder()};
  while (1)
  {
	  now=HAL_GetTick();
	  		//unsigned int n =0;
	  		//unsigned long start=HAL_GetTick();
	  long time_beat{peak_finder()};
	  start=HAL_GetTick();

	  //pushvalue(beat_per_mins,pressed);
	  //pushvalue(time_beat/1000,pressed);
	  //HAL_Delay(400);
	  if(pressed==false)
	  {
	  	pressed=Button_press();
	  }
	  Time.push_back(start-now);
	  //if(Time[Time. size()-2]!=0)
	  	  	  	  //{
	  	  	  		  dif=start-now;
	  	  	  	pushvalue(dif,pressed);

	  	  	  		  //count++;
	  	  	  		  if(count==100)
	  	  	  		  {
	  	  	  			  count=0;
	  	  	  			  compaired=true;

	  	  	  		  }
	  	  	  		  double time_per_beat{dif/1000};
	  	  	  		  if(time_per_beat!=0)
	  	  	  		  {
	  	  	  		  beat_per_mins=time_per_beat;
	  	  	  		  heart[count]=beat_per_mins;
	  	  	  		  }
	  	  	  	count++;
	  	  	  	  //}

	  	  	  	//pushvalue(beat_per_mins,pressed);
	  	  	  //pushvalue(dif,pressed);
	  	  	pushvalue(dif,pressed);
	  	  	  	  if(compaired&&(count%10==0))
	  	  	  	  {
	  	  	  		  mean_rate=mean(heart,cap);
	  	  	  		  std_dev=standerd_dev(heart,cap,mean_rate);
	  	  	  	  }
	  	  	  	//pushvalue(beat_per_mins,pressed);
	  	  	  //pushvalue(time_beat/1000,pressed);
	  	  	  	if(abs(heart[count]-mean_rate)>std_dev)
	  	  	  		{
	  	  	  			warning++;
	  	  	  			warning_cycle=cycle_count;
	  	  	  		}
	  	  	  	//pushvalue(beat_per_mins,pressed);
	  	  	//pushvalue(time_beat/1000,pressed);
	  	  	  	if(warning>10)
	  	  	  	{
	  	  	  		pushWarning(beat_per_mins);
	  	  	  		pressed=true;
	  	  	  	}
	  	  	  	  if(((cycle_count-warning_cycle)%10==0)&&warning!=0)
	  	  	  	  {
	  	  	  		  warning--;

	  	  	  	  }
	  	  	  	if(((cycle_count)%20==0))
	  	  	  		  	  {
	  	  	  		  		  pressed=false;

	  	  	  		  		  i2cLcd_ClearDisplay(&h_lcd);

	  	  	  		  	  }
	  	  	  	  cycle_count++;
	  	  	  //pushvalue(time_beat/1000,pressed);
	  	  	  	//pushvalue(beat_per_mins,pressed);
	  	  	  	  if(Time.size()>20)
	  	  	  	  {
	  	  	  	     Time.erase(Time.begin(), Time.end() - 1);
	  	  	  	  }
	  	  	  if(pressed==false)
	  	  	  	  {
	  	  	  	  	  	  pressed=Button_press();
	  	  	  	  }
	  	  	if(count==(UINT_MAX-15))
	  	  	{
	  	  		count=0;
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
