/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "socket.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */
#define W5500_select() HAL_GPIO_WritePin(GPIO_W5500_CS_GPIO_Port, GPIO_W5500_CS_Pin, GPIO_PIN_RESET);
#define W5500_release() HAL_GPIO_WritePin(GPIO_W5500_CS_GPIO_Port, GPIO_W5500_CS_Pin, GPIO_PIN_SET);

#define W5500_rx() W5500_rxtx(0xff)
#define W5500_tx(data) W5500_rxtx(data)

uint8_t W5500_rxtx(uint8_t data);
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

/* Definitions for ethernetTask */
osThreadId_t ethernetTaskHandle;
const osThreadAttr_t ethernetTask_attributes = {
  .name = "ethernetTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */char writeValue[60];
#define DATA_BUF_SIZE   4096
uint8_t gDATABUF[DATA_BUF_SIZE];

///////////////////////////////////
// Default Network Configuration //
///////////////////////////////////
wiz_NetInfo gWIZNETINFO = { .mac = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED},
							.ip = {192, 168, 0, 231},
							.sn = {255,255,255,0},
							.gw = {192, 168, 0, 1},
							.dns = {0,0,0,0},
							.dhcp = NETINFO_STATIC };

//states for multythread http
#define HTTP_IDLE 0
#define HTTP_SENDING 1

//variables for multythread http
uint32_t sentsize[_WIZCHIP_SOCK_NUM_];
uint32_t filesize[_WIZCHIP_SOCK_NUM_];
uint8_t http_state[_WIZCHIP_SOCK_NUM_];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C2_Init(void);
void StartEthernet(void *argument);

/* USER CODE BEGIN PFP */
void  wizchip_select(void);
void  wizchip_deselect(void);
void  wizchip_write(uint8_t wb);
uint8_t wizchip_read();
void network_init(void);								// Initialize Network information and display it
int32_t tcp_http_mt(uint8_t, uint8_t*, uint16_t);		// Multythread TCP server
void HTTP_reset(uint8_t sockn);
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
	uint8_t i;
	uint8_t memsize[2][8] = {{2,2,2,2,2,2,2,2},{2,2,2,2,2,2,2,2}};
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
  MX_SPI2_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  ////////////////////////////////////////////////////////////////////////////////////////////////////
     // First of all, Should register SPI callback functions implemented by user for accessing WIZCHIP //
     ////////////////////////////////////////////////////////////////////////////////////////////////////

     /* Chip selection call back */
  	reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);

  	 /* SPI Read & Write callback function */
  	reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write);

  	////////////////////////////////////////////////////////////////////////
  	/* WIZCHIP SOCKET Buffer initialize */
  	if(ctlwizchip(CW_INIT_WIZCHIP,(void*)memsize) == -1)
  	{
  	   //init fail
  	   while(1);
  	}

  	/* Network initialization */
  	network_init();

  	//all connections inactive
  	for(i=0;i<_WIZCHIP_SOCK_NUM_;i++)
  	HTTP_reset(i);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ethernetTask */
  ethernetTaskHandle = osThreadNew(StartEthernet, NULL, &ethernetTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, OUT1_OP9_Pin|OUT1_OP10_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ON_LED1_GPIO_Port, ON_LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ON_BUZZ_Pin|GPIO_W5500_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Rst_GPIO_Port, Rst_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : OUT1_OP9_Pin OUT1_OP10_Pin */
  GPIO_InitStruct.Pin = OUT1_OP9_Pin|OUT1_OP10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ON_LED1_Pin */
  GPIO_InitStruct.Pin = ON_LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ON_LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT1_OP4_Pin OUT1_OP3_Pin OUT1_OP2_Pin OUT1_OP1_Pin
                           OUT1_OP8_Pin OUT1_OP7_Pin OUT1_OP6_Pin OUT1_OP5_Pin */
  GPIO_InitStruct.Pin = OUT1_OP4_Pin|OUT1_OP3_Pin|OUT1_OP2_Pin|OUT1_OP1_Pin
                          |OUT1_OP8_Pin|OUT1_OP7_Pin|OUT1_OP6_Pin|OUT1_OP5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ON_BUZZ_Pin GPIO_W5500_CS_Pin */
  GPIO_InitStruct.Pin = ON_BUZZ_Pin|GPIO_W5500_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT1_OP12_Pin OUT1_OP11_Pin OUT1_OP15_Pin OUT1_OP16_Pin
                           OUT1_OP14_Pin OUT1_OP13_Pin */
  GPIO_InitStruct.Pin = OUT1_OP12_Pin|OUT1_OP11_Pin|OUT1_OP15_Pin|OUT1_OP16_Pin
                          |OUT1_OP14_Pin|OUT1_OP13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Rst_Pin */
  GPIO_InitStruct.Pin = Rst_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Rst_GPIO_Port, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_PD01_ENABLE();

}

/* USER CODE BEGIN 4 */
uint8_t W5500_rxtx(uint8_t data)
{
	uint8_t rxdata;

	HAL_SPI_TransmitReceive(&hspi2, &data, &rxdata, 1, 50);

	return (rxdata);
}

//////////
// TODO //
/////////////////////////////////////////////////////////////////
// SPI Callback function for accessing WIZCHIP                 //
// WIZCHIP user should implement with your host spi peripheral //
/////////////////////////////////////////////////////////////////
void  wizchip_select(void)
{
	W5500_select();
}

void  wizchip_deselect(void)
{
	W5500_release();
}

void  wizchip_write(uint8_t wb)
{
	W5500_tx(wb);
}

uint8_t wizchip_read()
{
   return W5500_rx();
}
//////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////
// Intialize the network information to be used in WIZCHIP //
/////////////////////////////////////////////////////////////
void network_init(void)
{
   uint8_t tmpstr[6];

	ctlnetwork(CN_SET_NETINFO, (void*)&gWIZNETINFO);

	ctlwizchip(CW_GET_ID,(void*)tmpstr);
}
/////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////
// HTTP Multythread Example Code using ioLibrary_BSD         //
///////////////////////////////////////////////////////////////
#define len(some) (sizeof(some)/sizeof(some[0]))

//http server

void HTTP_reset(uint8_t sockn)
{
    sentsize[sockn]=0;
	http_state[sockn]=HTTP_IDLE;
}

int32_t tcp_http_mt(uint8_t sn, uint8_t* buf, uint16_t port)
{
   int32_t ret;
   uint32_t size = 0;
   uint8_t flagHtmlGen = 0;
   char *url,*p;
   uint16_t blocklen=0;

   switch(getSn_SR(sn))
   {
      case SOCK_ESTABLISHED :

         if(getSn_IR(sn) & Sn_IR_CON)
         {
            setSn_IR(sn,Sn_IR_CON);
         }

         if((size = getSn_RX_RSR(sn)) > 0)
         {
            if(size > DATA_BUF_SIZE) size = DATA_BUF_SIZE;
            ret = recv(sn,buf,size);

            HTTP_reset(sn);

            if(ret <= 0)
            return ret;

            url =(char*) buf + 4;

            if((http_state[sn]==HTTP_IDLE)&&(memcmp(buf, "GET ", 4)==0)&&((p = strchr(url, ' '))))// extract URL from request header
            {
              *(p++) = 0;//making zeroed url string

				if ((url != NULL)&&(strncmp("/favicon.ico",url,12) != 0))
				{
					flagHtmlGen = 1;

					//Gera��o da HTML
					if(flagHtmlGen == 1)
					{
						if(strncmp("/rede", url, 6) == 0)
						{
							strcpy((char*)buf,"HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n");
							strcat((char*)buf, "<html><head>");
							strcat((char*)buf, "<title>Mestre Config</title>");
							strcat((char*)buf, "</head>");
							strcat((char*)buf, "<body>");
							strcat((char*)buf, "<b><center>Configuracao da Placa de rede</b></center><br>");
							strcat((char*)buf, "<center><form action=''>");
							strcat((char*)buf, "IP: <input type='text' name='ip'>		Mascara: <input type='text' name='mascara'><br><br>");
							strcat((char*)buf, "Porta: <input type='text' name='porta'>		Gateway: <input type='text' name='gateway'><br><br>");
							strcat((char*)buf, "DNS 1: <input type='text' name='dns1'>		DNS 2: <input type='text' name='dns2'><br><br>");
							strcat((char*)buf, "DHCP: <button>On</button>  <button>Off</button>");
							strcat((char*)buf, "</center></form>");
							strcat((char*)buf, "<center><button>Salvar</button><br><br><a href='http://192.168.0.231'>Voltar</a><br></center>");

							strcat((char*)buf, "</body>");
							strcat((char*)buf, "</html>");
						}
						else if(strncmp("/interfaces", url, 6) == 0){
							strcpy((char*)buf,"HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n");
							strcat((char*)buf, "<html><head>");
							strcat((char*)buf, "<title>Escrava Config</title>");
							strcat((char*)buf, "</head>");
							strcat((char*)buf, "<body>");
							strcat((char*)buf, "<div class = 'entradas'>");

							strcat((char*)buf, "<b><center>Estado das Entradas</b> </center>");
							strcat((char*)buf, "<p></p>");
							strcat((char*)buf, "<center><table border='1'>");
							strcat((char*)buf, "<tr>");
							strcat((char*)buf, "<td><b>Entrada Numero</b></td>");
							strcat((char*)buf, "<td><b>Estado da entrada</b></td>");
							strcat((char*)buf, "</tr>");
							strcat((char*)buf, "<tr>");
							strcat((char*)buf, "<td><center>1</center></td>");
							strcat((char*)buf, "</tr>");
							strcat((char*)buf, "<tr>");
							strcat((char*)buf, "<td><center>2</center></td>");
							strcat((char*)buf, "</tr>");
							strcat((char*)buf, "<tr>");
							strcat((char*)buf, "<td><center>3</center></td>");
							strcat((char*)buf, "</tr>");
							strcat((char*)buf, "<tr>");
							strcat((char*)buf, "<td><center>4</center></td>");
							strcat((char*)buf, "</tr>");
							strcat((char*)buf, "<tr>");
							strcat((char*)buf, "<td><center>5</center></td>");
							strcat((char*)buf, "</tr>");
							strcat((char*)buf, "<tr>");
							strcat((char*)buf, "<td><center>6</center></td>");
							strcat((char*)buf, "</tr>");
							strcat((char*)buf, "<tr>");
							strcat((char*)buf, "<td><center>7</center></td>");
							strcat((char*)buf, "</tr>");
							strcat((char*)buf, "<tr>");
							strcat((char*)buf, "<td><center>8</center></td>");
							strcat((char*)buf, "</tr>");
							strcat((char*)buf, "</table></center>");
							strcat((char*)buf, "<p></p>");
							strcat((char*)buf, "<center>");
							strcat((char*)buf, "<a href='http://192.168.0.231'>Voltar</a><br>");
							strcat((char*)buf, "</center>");

							strcat((char*)buf, "</div>");
							strcat((char*)buf, "</body>");
							strcat((char*)buf, "</html>");
						}
						else
						{
							strcpy((char*)buf,"HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n");
							strcat((char*)buf, "<html><head>");
							strcat((char*)buf, "<title>Escrava Config</title>");
							strcat((char*)buf, "<link href='https://maxcdn.bootstrapcdn.com/bootstrap/3.3.1/css/bootstrap.min.css' rel='stylesheet'></link>");
							strcat((char*)buf, "</head>");
							strcat((char*)buf, "<body>");
							strcat((char*)buf, "<div class = 'entradas'>");

							strcat((char*)buf, "<b><center>Menus para configuracao</b><br><br>");
							strcat((char*)buf, "<a href='http://192.168.0.231/rede'>Configurar rede</a><br><br>");
							strcat((char*)buf, "<a href='http://192.168.0.231/interfaces'>Configurar interfaces\n</a><br>");

							strcat((char*)buf, "</div>");
							strcat((char*)buf, "</body>");
							strcat((char*)buf, "</html>");
						}

						blocklen = strlen((char*)buf);

						ret = send(sn,buf,blocklen);
						if(ret < 0)
						{
							close(sn);
							return ret;
						}
						else
						{
							HTTP_reset(sn);
							disconnect(sn);
						}
					}
				}
				else
				{
					HTTP_reset(sn);
					disconnect(sn);
				}

        	  }
         }
         break;
      case SOCK_CLOSE_WAIT :

    	  HTTP_reset(sn);

         if((ret=disconnect(sn)) != SOCK_OK)
         return ret;

         break;
      case SOCK_INIT :

    	  HTTP_reset(sn);

         if( (ret = listen(sn)) != SOCK_OK) return ret;
         break;
      case SOCK_CLOSED:

    	  HTTP_reset(sn);

         if((ret=socket(sn,Sn_MR_TCP,port,0x00)) != sn)
         return ret;

         break;

      default:
    	  HTTP_reset(sn);
         break;
   }
   return 1;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartEthernet */
/**
  * @brief  Function implementing the ethernetTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartEthernet */
void StartEthernet(void *argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t i;
  /* Infinite loop */
  for(;;)
  {
	  for(i=4;i<_WIZCHIP_SOCK_NUM_;i++)
	  {
	  	tcp_http_mt(i, gDATABUF, 80);
	  }
	  //4 sockets para modbus TCP
	  for(i=0;i<4;i++)
	  {
	  	tcp_http_mt(i, gDATABUF, 502);
	  }
  }
  /* USER CODE END 5 */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
