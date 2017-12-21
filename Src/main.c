/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"
#include "adc.h"
#include "lptim.h"
#include "usart.h"
#include "rtc.h"
#include "gpio.h"
#include "tim.h"
#include "communication.h"
//#include <stdio.h>

/* USER CODE BEGIN Includes */
#define TIMER_FREQUENCY_HZ		((uint32_t)1000)

/* Temperature sensor calibration value adress */
#define TEMP30_CAL_ADDR ((uint16_t*)((uint32_t) 0x1FF8007A))
#define TEMP130_CAL_ADDR ((uint16_t*)((uint32_t) 0x1FF8007E))
#define VDD_APPLI ((uint16_t) (3300))
#define VDD_CALIB ((uint16_t) (3000))
#define RANGE_12BITS ((uint16_t) (4095))
#define IGNORE_KEY ((uint16_t) (0xB00))
//macros
#define BITS_TO_VOLTAGE(ADC_DATA) \
	((ADC_DATA) * VDD_APPLI / RANGE_12BITS)
#define TemperatureCalculate(data) \
	((((data * VDD_APPLI / VDD_CALIB) - (int32_t) *TEMP30_CAL_ADDR) * (int32_t)(130-30)) / (int32_t)(*TEMP130_CAL_ADDR - *TEMP30_CAL_ADDR));
#define PacketData(ID) \
					(((Rx_Buffer[(ID * 3)] << 0x4) & 0xFF0) + ((Rx_Buffer[(ID * 3) + 1] >> 0x4) & 0xF)) // extract paket data



/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
	char  buffer[15], Rx_indx, Rx_data[2], Rx_Buffer[200], Transfer_cplt;
	int len, i, j, step, Cell_Temperature, Cell_Voltage, Packet_Id, Packet_Id_cplt, Cell_Temperature_Calibrate_Coeff, Cell_Voltage_Calibrate_Coeff;
	int PWM_Value = 0, Cycles = 0, Byte_To_Receive = 2;
	uint32_t Tx_Data;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
// int32_t TemperatureCalculate(uint32_t data);
int32_t GetTemp();
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
uint8_t Calculate_CRC(uint16_t data_crc);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

	ADC_ChannelConfTypeDef sConfig;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_ADC_Init();
  MX_LPTIM1_Init();
  MX_TIM2_Init();
  MX_LPUART1_UART_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();

  /* USER CODE BEGIN 2 */
i = 0;

HAL_UART_Receive_IT(&huart2, Rx_data, 1); //enable uart rx interrupt every time receiving 1 byte

HAL_ADC_Start(&hadc); // start ADC conversion

/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

//	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
//	  HAL_Delay(200);

if (Transfer_cplt == 1)// receive data completed
{

	for (j=0; j<Rx_Buffer[0]; j++)
	{
		if ((Rx_Buffer[((j*3)+1)] != 0xFC) && ((Rx_Buffer[((j*3)+1)] & 0xF) != 0xF) && (Packet_Id_cplt == 0))
		{
			Packet_Id = j;
			Packet_Id_cplt = 1;

		}
	}


	if ((Rx_Buffer[(Packet_Id * 3) + 1] & 0x7) == 0x1)
	{
		ADC1->CHSELR = ADC_CHSELR_CHSEL1; // select AD1 channel

		Cell_Voltage = BITS_TO_VOLTAGE(ADC1->DR) * 1.925;// convert adc data to millivolts
		/*
		 * need to add calibration correction algorithm
		 */
		if ((Rx_Buffer[(Packet_Id * 3) + 1] & 0x8) == 0x8)
		{
//			Cell_Voltage_Calibrate_Coeff = (((Rx_Buffer[(Packet_Id * 3)] << 0x4) & 0xFF0) \
//					+ ((Rx_Buffer[(Packet_Id * 3) + 1] >> 0x4) & 0xF)) - Cell_Voltage; // calculate offset coefficient for voltage correction
			Cell_Voltage_Calibrate_Coeff = PacketData(Packet_Id) - Cell_Voltage; // calculate offset coefficient for voltage correction
		}

		Send_Updated_Packet(Cell_Voltage+Cell_Voltage_Calibrate_Coeff);// send cell voltage

		/* human readable cell voltage info send
		len=sprintf(buffer,"Cell %i mV\r\n",Cell_Voltage)-1;
		HAL_UART_Transmit(&huart2, buffer , len, 1000);
		 */

	}


	if ((Rx_Buffer[(Packet_Id * 3) + 1] & 0x7) == 0x2)
	{
		ADC1->CHSELR = ADC_CHSELR_CHSEL18; // select temp sensor channel

		Cell_Temperature = TemperatureCalculate((int32_t) ADC1->DR);// convert adc data to temperture in C
		/*
		 * need to add calibration correction algorithm
		 */
		Send_Updated_Packet(Cell_Temperature);// send cell temperature

		/* human readable cell temperature info send
		len=sprintf(buffer,"Temp %i C\r\n",Cell_Temperature);
		HAL_UART_Transmit(&huart2, buffer , len, 1000);
		 */

	}

	if ((Rx_Buffer[(Packet_Id * 3) + 1] & 0x7) == 0x3)
	{
		if ((Rx_Buffer[(Packet_Id * 3) + 1] & 0x8) == 0x8)
		{
			PWM_Value = PacketData(Packet_Id); //(((Rx_Buffer[(Packet_Id * 3)] << 0x4) & 0xFF0) + ((Rx_Buffer[(Packet_Id * 3) + 1] >> 0x4) & 0xF));
			/* human readable update load state info
			len=sprintf(buffer,"Update \r\n");
			HAL_UART_Transmit(&huart2, buffer , len, 1000);
			 */
		}
		TIM2_CH2_PWM_Setvalue(100 - PWM_Value); // setting PWM (inverted signal)

		Send_Updated_Packet(PWM_Value);// send load state

		/* human readable load state info
		len=sprintf(buffer,"Load  %i %% \r\n",PWM_Value);
		HAL_UART_Transmit(&huart2, buffer , len, 1000);
		 */
	}

	if ((Rx_Buffer[(Packet_Id * 3) + 1] & 0x7) == 0x5)
	{
		if ((Rx_Buffer[(Packet_Id * 3) + 1] & 0x8) == 0x8)
		{
			Cycles++;// in future release need to save this value in flash
		}

		Send_Updated_Packet(Cycles);// send cycles state

		/* human readable cycles state info
		len=sprintf(buffer,"Cycles  %i %% \r\n",Cycles);
		HAL_UART_Transmit(&huart2, buffer , len, 1000);
		 */
	}

	  Transfer_cplt = 0;
      Byte_To_Receive = 2;

}
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */ //2Mhz
  /*RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  */

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_4;


  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;//MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_LPUART1
                              |RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_LPTIM1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.LptimClockSelection = RCC_LPTIM1CLKSOURCE_PCLK;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
/* int32_t TemperatureCalculate(uint32_t data)
{
	int32_t temperature;

	temperature = ((data * VDD_APPLI / VDD_CALIB) - (int32_t) *TEMP30_CAL_ADDR);
	temperature = temperature * (int32_t)(130-30);
	temperature = temperature / (int32_t)(*TEMP130_CAL_ADDR - *TEMP30_CAL_ADDR);
//	temperature = temperature + 30;
	return(temperature);
}
*/

int32_t GetTemp()
{
	/* (2) Select the auto off mode */
	/* (3) Select CHSEL17 for VRefInt */
	/* (4) Select a sampling mode of 111 i.e. 239.5 ADC clk to be greater than
	17.1us */
	/* (5) Wake-up the VREFINT (only for Temp sensor and VRefInt) */
	ADC1->CFGR1 |= ADC_CFGR1_AUTOFF; /* (2) */
	ADC1->CHSELR = ADC_CHSELR_CHSEL18; /* (3) */
	ADC1->SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2; /* (4) */
	ADC->CCR |= ADC_CCR_VREFEN; /* (5) */
	/* Performs the AD conversion */
	ADC1->CR |= ADC_CR_ADSTART; /* start the ADC conversion */
	while ((SYSCFG->CFGR3 & SYSCFG_CFGR3_SENSOR_ADC_RDYF) == 0) /* wait end of conversion */
	{
		/* For robust implementation, add here time-out management */
	}
	return(ADC1->DR);
}

//Interrupt callback routine
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t i;
    if (huart->Instance == USART2)  //current UART
        {
        if (Rx_indx==0) {for (i=0;i<200;i++) Rx_Buffer[i]=0;}   //clear Rx_Buffer before receiving new data

//        if (Rx_data[0]!=13) //if received data different from ascii 13 (enter)
        if (Rx_indx < Byte_To_Receive) //if received packet number less than need to be received
            {
            Rx_Buffer[Rx_indx++]=Rx_data[0];    //add data to Rx_Buffer
            if (Rx_Buffer[1] == 0xFC)// receive starting packet
            {
            	Byte_To_Receive = ((Rx_Buffer[0]) * 3) + 5;// each data packet consist from 3 bytes plus start packet and end packet
            }
            }
        else            //if received data = 13
            {
            Rx_Buffer[Rx_indx]=Rx_data[0];    //add data to Rx_Buffer
            Rx_indx=0;
            Transfer_cplt=1;//transfer complete, data is ready to read
            }

        HAL_UART_Receive_IT(&huart2, Rx_data, 1);   //activate UART receive interrupt every time
        }

}

void Send_Updated_Packet(uint16_t data)
{
	Tx_Data = ((data << 0xC) & 0x00FFF000) + IGNORE_KEY;
	Tx_Data = Tx_Data + Calculate_CRC(Tx_Data);
//	buffer[2] = Tx_Data & 0x0000FF;
//	buffer[1] = (Tx_Data >> 0x8) & 0xFF;
//	buffer[0] = (Tx_Data >> 0x10) & 0xFF;
//	HAL_UART_Transmit(&huart2, buffer , 3, 1000);

	Rx_Buffer[Packet_Id * 3 + 2] = Tx_Data & 0x0000FF;
	Rx_Buffer[Packet_Id * 3 + 1] = (Tx_Data >> 0x8) & 0xFF;
	Rx_Buffer[Packet_Id * 3] = (Tx_Data >> 0x10) & 0xFF;
	HAL_UART_Transmit(&huart2, Rx_Buffer , Byte_To_Receive+1, 1000);
	HAL_UART_Transmit(&hlpuart1, Rx_Buffer , Byte_To_Receive + 1, 1000);
	Packet_Id_cplt = 0;

}


//calculate 8-bit CRC
uint8_t Calculate_CRC(uint16_t data_crc)
{

	return(0xFF);// temporary data
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
