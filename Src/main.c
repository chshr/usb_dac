
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include "dac.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/


/* USER CODE BEGIN PV */
#define INTERMSG_BYTES 4

/* Private variables ---------------------------------------------------------*/
typedef struct Array{
	uint16_t *array;
	size_t used;
	size_t size;
} Array;

typedef struct Array8{
	uint8_t *array;
	size_t used;
	size_t size;
} Array8;

typedef struct ArrHolder{
	Array* x;
	Array* y;
	Array* t;
	char* eraseFlag;
} ArrHolder;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void parseString(uint8_t *Buf, uint32_t *Len, ArrHolder Arr, uint8_t *interMsg, int *interMsgSize);
void interpolInsert2Arr(int p1, int p2, uint16_t t, Array* resArr);

void initArray(Array *a, size_t initialSize);
void insert2Array(Array *a, int element);
void freeArray(Array *a);

void USB_DEVICE_MasterHardReset(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
ArrHolder Arrs;
uint8_t interMsgSave[INTERMSG_BYTES];
int cyclesToWait, startCycle;
int interMsgSizeGlob = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_USART3_UART_Init();
//  USB_DEVICE_MasterHardReset();
  MX_USB_DEVICE_Init();
  MX_DAC_Init();
  /* USER CODE BEGIN 2 */

  // Setting processor cycle-based timer
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->LAR = 0xC5ACCE55;	// Lock Access Register
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // enable the counter
  DWT->CYCCNT = 0; // reset the counter
  cyclesToWait = (SystemCoreClock/1000000L)*20; // wait 20 us


  HAL_DAC_Start(&hdac,DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac,DAC_CHANNEL_2);

  HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R, 2048);
  HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R, 2048);

  Array xarr, yarr, tarr;
  initArray(&xarr, 1024);
  initArray(&yarr, 1024);
  initArray(&tarr, 1024);
  *(xarr.array) = 0;
  *(yarr.array) = 0;
  *(tarr.array) = 0;
  Arrs.x = &xarr;
  Arrs.y = &yarr;
  Arrs.t = &tarr;
  *(Arrs.eraseFlag) = 0;

  uint16_t t = 0;
  uint16_t xcur = 0;
  uint16_t ycur = 0;
  uint16_t tcur = 0;
  uint16_t p = 0;
  uint16_t x = 0;
  uint16_t xprev = 0;
  uint16_t y = 0;
  uint16_t yprev = 0;

  double xStepDiff = 0;
  double yStepDiff = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  startCycle = DWT->CYCCNT;

//// To be used with pre-made arrays of points
//	  HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R, xarr.array[p]);
//	  HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R, yarr.array[p]);
//	  if (p>=(int)xarr.used-1)
//	  {
//		  p = 0;
//	  } else {
//		  p++;
//	  }

	  // To be used with new interpolation Array calculated for every point
	  if (tcur == 0)
	  {
		  if (xarr.used > 1)
		  {
			  xprev = x;
			  yprev = y;
		  } else {
			  xprev = xarr.array[0];
			  yprev = yarr.array[0];
		  }

		  x = xarr.array[p];
		  xcur = xprev;
		  y = yarr.array[p];
		  ycur = yprev;
		  t = tarr.array[p];

		  if (xprev > 0)
		  {
			  xStepDiff = (x - xprev) / (double)(t + 1);
			  yStepDiff = (y - yprev) / (double)(t + 1);
		  }

	  } else
	  {
		  xcur = xprev + xStepDiff * tcur;
		  ycur = yprev + yStepDiff * tcur;
	  }

	  if (tcur == t)
	  {
		  tcur = 0;
		  if ((xarr.used==0) | (p>=xarr.used-1))
		  {
			  p = 0;
		  } else if (xarr.used > 0)
		  {
			  p++;
		  }
	  } else
	  {
		  tcur++;
	  }

	  HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R, xcur);
	  HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R, ycur);

	  do {
	  }while(DWT->CYCCNT - startCycle < cyclesToWait);

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
void initArray(Array *a, size_t initialSize)
{
	a->array = (uint16_t*)malloc(initialSize * sizeof(uint16_t));
	a->array[0] = 0;
	a->used = 0;
	a->size = initialSize;
}

void insert2Array(Array *a, int element)
{
	if (a->used == a->size)
	{
		a->size += 64;
		a->array = (uint16_t*) realloc(a->array, a->size * sizeof(uint16_t));
	}
	a->array[a->used++] = element;
}

void freeArray(Array *a)
{
	free(a->array);
	a->array = NULL;
	a->used = a->size = 0;
}

void initArray8(Array8 *a, size_t initialSize)
{
	a->array = (uint8_t*)malloc(initialSize * sizeof(uint8_t));
	a->array[0] = 0;
	a->used = 0;
	a->size = initialSize;
}

void insert2Array8(Array8 *a, int element)
{
	if (a->used == a->size)
	{
		a->size += 8;
		a->array = (uint8_t*) realloc(a->array, a->size * sizeof(uint8_t));
	}
	a->array[a->used++] = element;
}

void freeArray8(Array8 *a)
{
	free(a->array);
	a->array = NULL;
	a->used = a->size = 0;
}

void interpolInsert2Arr(int p1, int p2, uint16_t t, Array* resArr)
// Makes t points between p1 and p2 and inserts to the Array resArr
{
	int pdiff, newpoint, i;
	double d;

	pdiff = p2 - p1;
	d = pdiff / (double)(t + 1);

	for (i = 1; i<(t+1); ++i)
	{
		newpoint = p1 + d * i;
		insert2Array(resArr, newpoint);
	}
}

void parseString(uint8_t *Buf, uint32_t *Len, ArrHolder Arrs, uint8_t *interMsg, int *interMsgSize)
{

	uint8_t i, j;
	i = j = 0;
	uint8_t send_res;
	uint16_t x = 0;
	uint16_t y = 0;
	uint16_t t = 0;
	Array* xarr = Arrs.x;
	Array* yarr = Arrs.y;
	Array* tarr = Arrs.t;
	Array8 tempBuf;
	int tempBufFlag = 0;

	void eraseAllArrs(void)
	{
		freeArray(xarr);
		freeArray(yarr);
		freeArray(tarr);
		initArray(xarr, 64);
		initArray(yarr, 64);
		initArray(tarr, 64);
		*(Arrs.eraseFlag) = 0;
	}

	if (*(Arrs.eraseFlag))
	{
		eraseAllArrs();
	}

	// work only with information with even number of bytes
	// odd number of bytes probably means \n in the end
	if (*Len % 2 == 1)
	{
		(*Len)--;
	}

	if (*interMsgSize > 0)
	{
		*Len += *interMsgSize;
		initArray8(&tempBuf, *Len);
		tempBufFlag = 1;

		for (j=0; j<*interMsgSize; ++j)
		{
			insert2Array8(&tempBuf, *(interMsg+j));
		}

		for (j=0; j<(*Len - *interMsgSize); ++j)
		{
			insert2Array8(&tempBuf, *(Buf+j));
		}
		Buf = tempBuf.array;
	}

	// parse the data
	while ((*Len - i) / 6 > 0)
	{
		x = (uint16_t) ((*(Buf+i) << 8) | (*(Buf+i+1)));
		y = (uint16_t) ((*(Buf+i+2) << 8) | (*(Buf+i+3)));
		t = (uint16_t) ((*(Buf+i+4) << 8) | (*(Buf+i+5)));
		i += 6;

//		if (xarr->used > 0)
//		{
//			interpol(xarr->array[xarr->used-1], x, t, xarr);
//		}
		insert2Array(xarr, x);

//		if (yarr->used > 0)
//		{
//			interpol(yarr->array[yarr->used-1], y, t, yarr);
//		}
		insert2Array(yarr, y);

		insert2Array(tarr, t);
	}

	*interMsgSize = *Len - i;

	// check if bytes that left are command
	if (*(Buf+i) == 0xEE)
	{
		if (*(Buf+i+1) == 0xEE)
		{
			// erase all data upon receiving the next message
			*(Arrs.eraseFlag) = 1;
			*interMsgSize = 0;
		} else if (*(Buf+i+1) == 0x01)
		{
			// erase all data immediately
			eraseAllArrs();
			*interMsgSize = 0;
		}
	}

	while ((*Len - i) > 0)
	{
		*interMsg++ = *(Buf+i++);
	}

	if (tempBufFlag)
		freeArray8(&tempBuf);

}

// called from function CDC_Receive_FS in file usbd_cdc_if.c
void receivedUsbCallback(uint8_t* Buf, uint32_t *Len)
{
	parseString(Buf, Len, Arrs, &interMsgSave, &interMsgSizeGlob);
	HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
}

/*pull down the USB RP pin to let think of master that device have been
 *disconnected and force new device identification
 *when calling MX_USB_DEVICE_Init() */
void USB_DEVICE_MasterHardReset(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,0);
    HAL_Delay(1000);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
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
