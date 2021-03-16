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
#define W5500_select() HAL_GPIO_WritePin(GPIO_W5500_CS_GPIO_Port, GPIO_W5500_CS_Pin, GPIO_PIN_RESET);
#define W5500_release() HAL_GPIO_WritePin(GPIO_W5500_CS_GPIO_Port, GPIO_W5500_CS_Pin, GPIO_PIN_SET);

#define W5500_rx() W5500_rxtx(0xff)
#define W5500_tx(data) W5500_rxtx(data)

#define NETWORK_STORAGE 0x0800C000
#define	INTERFACE_STORAGE 0x0800D000
#define page_size 0x800
/* USER CODE END PD */

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
/* Definitions for readInput1Task */
osThreadId_t readInput1TaskHandle;
const osThreadAttr_t readInput1Task_attributes = {
  .name = "readInput1Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for readInput2Task */
osThreadId_t readInput2TaskHandle;
const osThreadAttr_t readInput2Task_attributes = {
  .name = "readInput2Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for readInput3Task */
osThreadId_t readInput3TaskHandle;
const osThreadAttr_t readInput3Task_attributes = {
  .name = "readInput3Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for readInput4Task */
osThreadId_t readInput4TaskHandle;
const osThreadAttr_t readInput4Task_attributes = {
  .name = "readInput4Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */
#define DATA_BUF_SIZE   4096
uint8_t gDATABUF[DATA_BUF_SIZE];
uint8_t config_data[300];

uint8_t inputs[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t ip[4], mask[4], gateway[4], dns1[4], dns2[4], port = 80;
char ipStr[15];
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
void StartReadInput1(void *argument);
void StartReadInput2(void *argument);
void StartReadInput3(void *argument);
void StartReadInput4(void *argument);

/* USER CODE BEGIN PFP */
uint8_t W5500_rxtx(uint8_t data);
void  wizchip_select(void);
void  wizchip_deselect(void);
void  wizchip_write(uint8_t wb);
uint8_t wizchip_read();
void network_init(void);								// Initialize Network information and display it
int32_t tcp_http_mt(uint8_t, uint8_t*, uint16_t);		// Multythread TCP server
void HTTP_reset(uint8_t sockn);
void read_flash(uint8_t* data, uint32_t ADDRESS);		//Reads data from the flash memory
void save_to_flash(uint8_t* data, uint32_t ADDRESS);		//Writes data to the flash memory
uint8_t *strremove(uint8_t *str, const uint8_t *sub);
void configNetwork();
void configInterfaces();
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
  configNetwork();
  configInterfaces();
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

  /* creation of readInput1Task */
  readInput1TaskHandle = osThreadNew(StartReadInput1, NULL, &readInput1Task_attributes);

  /* creation of readInput2Task */
  readInput2TaskHandle = osThreadNew(StartReadInput2, NULL, &readInput2Task_attributes);

  /* creation of readInput3Task */
  readInput3TaskHandle = osThreadNew(StartReadInput3, NULL, &readInput3Task_attributes);

  /* creation of readInput4Task */
  readInput4TaskHandle = osThreadNew(StartReadInput4, NULL, &readInput4Task_attributes);

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
					if (strncmp("/ip:",url,4) == 0)
					{
						memset(config_data, 0, sizeof(config_data));
						strcpy(config_data, url);
						save_to_flash(config_data, NETWORK_STORAGE);
						configNetwork();
					}
					if(strncmp("/interfaces2/ip", url, 14) == 0)
					{
						memset(config_data, 0, sizeof(config_data));
						strcpy(config_data, url);
						save_to_flash(config_data, INTERFACE_STORAGE);
					}
					if(strncmp("/con/ip", url, 7) == 0)
					{
						strcat(config_data, url);
						save_to_flash(config_data, INTERFACE_STORAGE);
					}

					//Gera��o da HTML
					if(flagHtmlGen == 1)
					{
						if(strncmp("/rede", url, 4) == 0)
						{
							strcpy((char*)buf,"HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n");
							strcat((char*)buf, "<html><head>");
							strcat((char*)buf, "<title>Mestre Config</title>");
							strcat((char*)buf, "</head>");
							strcat((char*)buf, "<body>");
							strcat((char*)buf, "<b><center>Configuracao da Placa de rede</b></center><br>");
							strcat((char*)buf, "<center><form action=''>");
							strcat((char*)buf, "IP: <input type='text' id='ip'>		Mascara: <input type='text' id='mascara'><br><br>");
							strcat((char*)buf, "Porta: <input type='text' id='porta'>		Gateway: <input type='text' id='gateway'><br><br>");
							strcat((char*)buf, "DNS 1: <input type='text' id='dns1'>		DNS 2: <input type='text' id='dns2'><br><br>");
							strcat((char*)buf, "DHCP: <button>On</button>  <button>Off</button>");
							strcat((char*)buf, "</center></form>");

							strcat((char*)buf, "<script>function configNet() {");
							strcat((char*)buf, "window.open('http://");
							strcat((char*)buf, &ipStr);
							strcat((char*)buf, "/ip:' + document.getElementById('ip').value + ',mask:' + document.getElementById('mascara').value + ',port:' +");
							strcat((char*)buf, "document.getElementById('porta').value + ',gateway:' + document.getElementById('gateway').value + ',dns1:' + document.getElementById('dns1').value +");
							strcat((char*)buf, "',dns2:' + document.getElementById('dns2').value, '_self');}</script>");

							strcat((char*)buf, "<center><button onClick = 'configNet()'>Salvar</button><br><br><a href='http://");
							strcat((char*)buf, &ipStr);
							strcat((char*)buf, "'>Voltar</a><br></center>");

							strcat((char*)buf, "</body>");
							strcat((char*)buf, "</html>");
						}
						else if(strncmp("/interfaces1", url, 12) == 0)
						{
							strcpy((char*)buf,"HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n");
							strcat((char*)buf, "<html><head><title>Escrava Config</title>");
							strcat((char*)buf, "<center><table border='1'><tr>");
							strcat((char*)buf, "<td><b>Entrada / IP da placa destino</b></td>");
							strcat((char*)buf, "<td><b>Rele da placa destino</b></td></tr>");

							strcat((char*)buf, "<tr><td>1:  <input type='text' id='ip1'></td>");
							strcat((char*)buf, "<td><input type='text' id='r1'></td></tr>");

							strcat((char*)buf, "<tr><td>2:  <input type='text' id='ip2'></td>");
							strcat((char*)buf, "<td><input type='text' id='r2'></td></tr>");

							strcat((char*)buf, "<tr><td>3:  <input type='text' id='ip3'></td>");
							strcat((char*)buf, "<td><input type='text' id='r3'></td></tr>");

							strcat((char*)buf, "<tr><td>4:  <input type='text' id='ip4'></td>");
							strcat((char*)buf, "<td><input type='text' id='r4'></td></tr>");

							strcat((char*)buf, "<tr><td>5:  <input type='text' id='ip5'></td>");
							strcat((char*)buf, "<td><input type='text' id='r5'></td></tr>");

							strcat((char*)buf, "<tr><td>6:  <input type='text' id='ip6'></td>");
							strcat((char*)buf, "<td><input type='text' id='r6'></td></tr>");

							strcat((char*)buf, "<tr><td>7:  <input type='text' id='ip7'></td>");
							strcat((char*)buf, "<td><input type='text' id='r7'></td></tr>");

							strcat((char*)buf, "<tr><td>8:  <input type='text' id='ip8'></td>");
							strcat((char*)buf, "<td><input type='text' id='r8'></td></tr></table><br>");

							strcat((char*)buf, "<script>function configInt() {");
							strcat((char*)buf, "window.open('http://");
							strcat((char*)buf, &ipStr);
							strcat((char*)buf, "/interfaces2/ip1:' + document.getElementById('ip1').value + ',ip2:' + document.getElementById('ip2').value + ',ip3:' +");
							strcat((char*)buf, "document.getElementById('ip3').value + ',ip4:' + document.getElementById('ip4').value + ',ip5:' + document.getElementById('ip5').value +");
							strcat((char*)buf, "',ip6:' + document.getElementById('ip6').value + ',ip7:' + document.getElementById('ip7').value + ',ip8:' + document.getElementById('ip8').value");
							strcat((char*)buf, "+ ',r1:' + document.getElementById('r1').value + ',r2:' + document.getElementById('r2').value ");
							strcat((char*)buf, "+ ',r3:' + document.getElementById('r3').value + ',r4:' + document.getElementById('r4').value ");
							strcat((char*)buf, "+ ',r5:' + document.getElementById('r5').value + ',r6:' + document.getElementById('r6').value ");
							strcat((char*)buf, "+ ',r7:' + document.getElementById('r7').value + ',r8:' + document.getElementById('r8').value, '_self');}</script>");

							strcat((char*)buf, "<button onClick='configInt()'>Proximo</button><br><br><a href='http://");
							strcat((char*)buf, &ipStr);
							strcat((char*)buf, "'>Voltar</a><br></center>");

							strcat((char*)buf, "</body>");
							strcat((char*)buf, "</html>");
						}
						else if(strncmp("/interfaces2", url, 12) == 0)
						{
							strcpy((char*)buf,"HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n");
							strcat((char*)buf, "<html><head><title>Escrava Config</title>");
							strcat((char*)buf, "<center><table border='1'><tr>");
							strcat((char*)buf, "<td><b>Entrada / IP da placa destino</b></td>");
							strcat((char*)buf, "<td><b>Rele da placa destino</b></td></tr>");

							strcat((char*)buf, "<tr><td>9:  <input type='text' id='ip9'></td>");
							strcat((char*)buf, "<td><input type='text' id='r9'></td></tr>");

							strcat((char*)buf, "<tr><td>10:  <input type='text' id='ip10'></td>");
							strcat((char*)buf, "<td><input type='text' id='r10'></td></tr>");

							strcat((char*)buf, "<tr><td>11:  <input type='text' id='ip11'></td>");
							strcat((char*)buf, "<td><input type='text' id='r11'></td></tr>");

							strcat((char*)buf, "<tr><td>12:  <input type='text' id='ip12'></td>");
							strcat((char*)buf, "<td><input type='text' id='r12'></td></tr>");

							strcat((char*)buf, "<tr><td>13:  <input type='text' id='ip13'></td>");
							strcat((char*)buf, "<td><input type='text' id='r13'></td></tr>");

							strcat((char*)buf, "<tr><td>14:  <input type='text' id='ip14'></td>");
							strcat((char*)buf, "<td><input type='text' id='r14'></td></tr>");

							strcat((char*)buf, "<tr><td>15:  <input type='text' id='ip15'></td>");
							strcat((char*)buf, "<td><input type='text' id='r15'></td></tr>");

							strcat((char*)buf, "<tr><td>16:  <input type='text' id='ip16'></td>");
							strcat((char*)buf, "<td><input type='text' id='r16'></td></tr></table><br>");

							strcat((char*)buf, "<script>function configInt() {");
							strcat((char*)buf, "window.open('http://");
							strcat((char*)buf, &ipStr);
							strcat((char*)buf, "/con/ip9:' + document.getElementById('ip9').value + ',ip10:' + document.getElementById('ip10').value + ',ip11:' +");
							strcat((char*)buf, "document.getElementById('ip11').value + ',ip12:' + document.getElementById('ip12').value + ',ip13:' + document.getElementById('ip13').value +");
							strcat((char*)buf, "',ip14:' + document.getElementById('ip14').value + ',ip15:' + document.getElementById('ip15').value + ',ip16' + document.getElementById('ip16').value");
							strcat((char*)buf, "+ ',r9:' + document.getElementById('r9').value + ',r10:' + document.getElementById('r10').value ");
							strcat((char*)buf, "+ ',r11:' + document.getElementById('r11').value + ',r12:' + document.getElementById('r12').value ");
							strcat((char*)buf, "+ ',r13:' + document.getElementById('r13').value + ',r14:' + document.getElementById('r14').value ");
							strcat((char*)buf, "+ ',r15:' + document.getElementById('r15').value + ',r16:' + document.getElementById('r16').value, '_self');}</script>");
							strcat((char*)buf, "<button onClick='configInt()'>Salvar</button><br><br><a href='http://");
							strcat((char*)buf, &ipStr);
							strcat((char*)buf, "'>Voltar</a><br></center>");

							strcat((char*)buf, "</body>");
							strcat((char*)buf, "</html>");

						}
						else
						{
							strcpy((char*)buf,"HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n");
							strcat((char*)buf, "<html><head>");
							strcat((char*)buf, "<title>Escrava Config</title>");
							strcat((char*)buf, "</head>");
							strcat((char*)buf, "<body>");
							strcat((char*)buf, "<div class = 'entradas'>");

							strcat((char*)buf, "<b><center>Menus para configuracao</b><br><br>");
							strcat((char*)buf, "<a href='http://");
							strcat((char*)buf, &ipStr);
							strcat((char*)buf,"/rede'>Configurar rede</a><br><br>");
							strcat((char*)buf, "<a href='http://");
							strcat((char*)buf, &ipStr);
							strcat((char*)buf, "/interfaces1'>Configurar interfaces\n</a><br>");

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

void save_to_flash(uint8_t *data, uint32_t ADDRESS)
{
	volatile uint32_t data_to_FLASH[(strlen((char*)data)/4)	+ (int)((strlen((char*)data) % 4) != 0)];
	memset((uint8_t*)data_to_FLASH, 0, strlen((char*)data_to_FLASH));
	strcpy((char*)data_to_FLASH, (char*)data);

	volatile uint32_t data_length = (strlen((char*)data_to_FLASH) / 4)
									+ (int)((strlen((char*)data_to_FLASH) % 4) != 0);
	volatile uint16_t pages = (strlen((char*)data)/page_size)
									+ (int)((strlen((char*)data)%page_size) != 0);
	  /* Unlock the Flash to enable the flash control register access *************/
	  HAL_FLASH_Unlock();

	  /* Allow Access to option bytes sector */
	  HAL_FLASH_OB_Unlock();

	  /* Fill EraseInit structure*/
	  FLASH_EraseInitTypeDef EraseInitStruct;
	  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	  EraseInitStruct.PageAddress = ADDRESS;
	  EraseInitStruct.NbPages = pages;
	  uint32_t PageError;

	  volatile uint32_t write_cnt=0, index=0;

	  volatile HAL_StatusTypeDef status;
	  status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
	  while(index < data_length)
	  {
		  if (status == HAL_OK)
		  {
			  status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADDRESS+write_cnt, data_to_FLASH[index]);
			  if(status == HAL_OK)
			  {
				  write_cnt += 4;
				  index++;
			  }
		  }
	  }

	  HAL_FLASH_OB_Lock();
	  HAL_FLASH_Lock();
}

void read_flash(uint8_t* data, uint32_t ADDRESS)
{
	volatile uint32_t read_data;
	volatile uint32_t read_cnt=0;
	do
	{
		read_data = *(uint32_t*)(ADDRESS + read_cnt);
		if(read_data != 0xFFFFFFFF)
		{
			data[read_cnt] = (uint8_t)read_data;
			data[read_cnt + 1] = (uint8_t)(read_data >> 8);
			data[read_cnt + 2] = (uint8_t)(read_data >> 16);
			data[read_cnt + 3] = (uint8_t)(read_data >> 24);
			read_cnt += 4;
		}
	}while(read_data != 0xFFFFFFFF);
}

uint8_t *strremove(uint8_t *str, const uint8_t *sub) {
    uint8_t *p, *q, *r;
    if ((q = r = strstr(str, sub)) != NULL) {
        size_t len = strlen(sub);
        while ((r = strstr(p = r + len, sub)) != NULL) {
            memmove(q, p, r - p);
            q += r - p;
        }
        memmove(q, p, strlen(p) + 1);
    }
    return str;
}

void configNetwork()
{
	uint8_t i;
	uint8_t memsize[2][8] = {{2,2,2,2,2,2,2,2},{2,2,2,2,2,2,2,2}};
	read_flash(config_data, NETWORK_STORAGE);
	if(config_data[0] == '/' && config_data[1] == 'i' && config_data[2] == 'p')
	{
		strremove(config_data, "/ip:");
		strremove(config_data, "mask:");
		strremove(config_data, "port:");
		strremove(config_data, "gateway:");
		strremove(config_data, "dns1:");
		strremove(config_data, "dns2:");
		uint8_t *p, *ipPointer, *maskPointer, *portPointer, *gatewayPointer, *dns1Pointer, *dns2Pointer;
		p = strtok(config_data, ",");

		for(int i = 0; p != NULL; i++)
		{
			switch(i)
			{
				case 0:
					ipPointer = p;
					break;
				case 1:
					maskPointer = p;
					break;
				case 2:
					portPointer = p;
					break;
				case 3:
					gatewayPointer = p;
					break;
				case 4:
					dns1Pointer = p;
					break;
				case 5:
					dns2Pointer = p;
					break;
				default:
					break;
			}
			p = strtok(NULL, ",");
		}

		p = strtok(ipPointer, ".");

		for(int i = 0; p != NULL; i++)
		{
			ip[i] = atoi(p);
			p = strtok(NULL, ".");
		}

		p = strtok(maskPointer, ".");

		for(int i = 0; p != NULL; i++)
		{
			mask[i] = atoi(p);
			p = strtok(NULL, ".");
		}

		p = strtok(gatewayPointer, ".");

		for(int i = 0; p != NULL; i++)
		{
			gateway[i] = atoi(p);
			p = strtok(NULL, ".");
		}

		p = strtok(dns1Pointer, ".");

		for(int i = 0; p != NULL; i++)
		{
			dns1[i] = atoi(p);
			p = strtok(NULL, ".");
		}

		p = strtok(dns2Pointer, ".");

		for(int i = 0; p != NULL; i++)
		{
			dns2[i] = atoi(p);
			p = strtok(NULL, ".");
		}

		port = atoi(portPointer);

		wiz_NetInfo wizNet = { .mac = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED},
				.ip = {ip[0], ip[1], ip[2], ip[3]},
				.sn = {mask[0], mask[1], mask[2], mask[3]},
				.gw = {gateway[0], gateway[1], gateway[2], gateway[3]},
				.dns = {dns1[0], dns1[1], dns1[2], dns1[3]},
				.dhcp = NETINFO_STATIC };

		gWIZNETINFO = wizNet;

	  	HAL_GPIO_WritePin(Rst_GPIO_Port, Rst_Pin, GPIO_PIN_RESET);
	  	HAL_Delay(500);
	  	HAL_GPIO_WritePin(Rst_GPIO_Port, Rst_Pin, GPIO_PIN_SET);
	  	HAL_Delay(50);
	}
	else
	{
		ip[0] = gWIZNETINFO.ip[0];
		ip[1] = gWIZNETINFO.ip[1];
		ip[2] = gWIZNETINFO.ip[2];
		ip[3] = gWIZNETINFO.ip[3];
	}

	uint8_t aux;
	int c = 14;
	ipStr[3] = '.';
	ipStr[7] = '.';
	ipStr[11] = '.';
	for(int j = 3; j >= 0; j --)
	{
		for(int i = 0; i < 3; i++)
		{
			ipStr[c - i] = ip[j] % 10 + '0';
			ip[j] /= 10;
		}
		c -= 4;
	}
	/* Chip selection call back */
	reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);

	/* SPI Read & Write callback function */
	reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write);

	///////////////////////////////////////////////////////////////////////
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
}

void configInterfaces()
{
	read_flash(config_data, INTERFACE_STORAGE);
	if(config_data[0] == '/' && config_data[1] == 'i' && config_data[2] == 'n')
	{
		strremove(config_data, "/interfaces2/");
		strremove(config_data, "/con/");
		strremove(config_data, "ip1:");
		strremove(config_data, "ip2:");
		strremove(config_data, "ip3:");
		strremove(config_data, "ip4:");
		strremove(config_data, "ip5:");
		strremove(config_data, "ip6:");
		strremove(config_data, "ip7:");
		strremove(config_data, "ip8:");
		strremove(config_data, "ip9:");
		strremove(config_data, "ip10:");
		strremove(config_data, "ip11:");
		strremove(config_data, "ip12:");
		strremove(config_data, "ip13:");
		strremove(config_data, "ip14:");
		strremove(config_data, "ip15:");
		strremove(config_data, "ip16:");
		strremove(config_data, "r1:");
		strremove(config_data, "r2:");
		strremove(config_data, "r3:");
		strremove(config_data, "r4:");
		strremove(config_data, "r5:");
		strremove(config_data, "r6:");
		strremove(config_data, "r7:");
		strremove(config_data, "r8:");
		strremove(config_data, "r9:");
		strremove(config_data, "r10:");
		strremove(config_data, "r11:");
		strremove(config_data, "r12:");
		strremove(config_data, "r13:");
		strremove(config_data, "r14:");
		strremove(config_data, "r15:");
		strremove(config_data, "r16:");

		uint8_t *ip1, *ip2, *ip3, *ip4, *ip5, *ip6, *ip7, *ip8, *ip9, *ip10, *ip11, *ip12, *ip13, *ip14, *ip15, *ip16;
		uint8_t *r1, *r2, *r3, *r4, *r5, *r6, *r7, *r8, *r9, *r10, *r11, *r12, *r13, *r14, *r15, *r16, *p;

		p = strtok(config_data, ",");

		for(int i = 0; p != NULL; i++)
		{
			switch(i)
			{
				case 0:
					ip1 = p;
					break;
				case 1:
					ip2 = p;
					break;
				case 2:
					ip3= p;
					break;
				case 3:
					ip4 = p;
					break;
				case 4:
					ip5 = p;
					break;
				case 5:
					ip6 = p;
					break;
				case 6:
					ip7 = p;
					break;
				case 7:
					ip8 = p;
					break;
				case 8:
					r1 = p;
					break;
				case 9:
					r2 = p;
					break;
				case 10:
					r3 = p;
					break;
				case 11:
					r4 = p;
					break;
				case 12:
					r5 = p;
					break;
				case 13:
					r6 = p;
					break;
				case 14:
					r7 = p;
					break;
				case 15:
					r8 = p;
					break;
				case 16:
					ip9 = p;
					break;
				case 17:
					ip10 = p;
					break;
				case 18:
					ip11 = p;
					break;
				case 19:
					ip12 = p;
					break;
				case 20:
					ip13 = p;
					break;
				case 21:
					ip14 = p;
					break;
				case 22:
					ip15 = p;
					break;
				case 23:
					ip16 = p;
					break;
				case 24:
					r9 = p;
					break;
				case 25:
					r10 = p;
					break;
				case 26:
					r11 = p;
					break;
				case 27:
					r12 = p;
					break;
				case 28:
					r13 = p;
					break;
				case 29:
					r14 = p;
					break;
				case 30:
					r15 = p;
					break;
				case 31:
					r16 = p;
					break;
				default:
					break;
			}
			p = strtok(NULL, ",");
		}
	}
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
	  	tcp_http_mt(i, gDATABUF, port);
	  }
	  //4 sockets para modbus TCP
	  for(i=0;i<4;i++)
	  {
	  	tcp_http_mt(i, gDATABUF, 502);
	  }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartReadInput1 */
/**
* @brief Function implementing the readInput1Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadInput1 */
void StartReadInput1(void *argument)
{
  /* USER CODE BEGIN StartReadInput1 */
  /* Infinite loop */
  for(;;)
  {
	  inputs[0] = HAL_GPIO_ReadPin(OUT1_OP1_GPIO_Port, OUT1_OP1_Pin);
	  inputs[1] = HAL_GPIO_ReadPin(OUT1_OP2_GPIO_Port, OUT1_OP2_Pin);
	  inputs[2] = HAL_GPIO_ReadPin(OUT1_OP3_GPIO_Port, OUT1_OP3_Pin);
	  inputs[3] = HAL_GPIO_ReadPin(OUT1_OP4_GPIO_Port, OUT1_OP4_Pin);
    osDelay(1);
  }
  /* USER CODE END StartReadInput1 */
}

/* USER CODE BEGIN Header_StartReadInput2 */
/**
* @brief Function implementing the readInput2Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadInput2 */
void StartReadInput2(void *argument)
{
  /* USER CODE BEGIN StartReadInput2 */
  /* Infinite loop */
  for(;;)
  {
	  inputs[4] = HAL_GPIO_ReadPin(OUT1_OP5_GPIO_Port, OUT1_OP5_Pin);
	  inputs[5] = HAL_GPIO_ReadPin(OUT1_OP6_GPIO_Port, OUT1_OP6_Pin);
	  inputs[6] = HAL_GPIO_ReadPin(OUT1_OP7_GPIO_Port, OUT1_OP7_Pin);
	  inputs[7] = HAL_GPIO_ReadPin(OUT1_OP8_GPIO_Port, OUT1_OP8_Pin);
    osDelay(1);
  }
  /* USER CODE END StartReadInput2 */
}

/* USER CODE BEGIN Header_StartReadInput3 */
/**
* @brief Function implementing the readInput3Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadInput3 */
void StartReadInput3(void *argument)
{
  /* USER CODE BEGIN StartReadInput3 */
  /* Infinite loop */
  for(;;)
  {
	  inputs[8] = HAL_GPIO_ReadPin(OUT1_OP8_GPIO_Port, OUT1_OP9_Pin);
	  inputs[9] = HAL_GPIO_ReadPin(OUT1_OP9_GPIO_Port, OUT1_OP10_Pin);
	  inputs[10] = HAL_GPIO_ReadPin(OUT1_OP10_GPIO_Port, OUT1_OP11_Pin);
	  inputs[11] = HAL_GPIO_ReadPin(OUT1_OP11_GPIO_Port, OUT1_OP12_Pin);
    osDelay(1);
  }
  /* USER CODE END StartReadInput3 */
}

/* USER CODE BEGIN Header_StartReadInput4 */
/**
* @brief Function implementing the readInput4Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadInput4 */
void StartReadInput4(void *argument)
{
  /* USER CODE BEGIN StartReadInput4 */
  /* Infinite loop */
  for(;;)
  {
	  inputs[12] = HAL_GPIO_ReadPin(OUT1_OP13_GPIO_Port, OUT1_OP13_Pin);
	  inputs[13] = HAL_GPIO_ReadPin(OUT1_OP14_GPIO_Port, OUT1_OP14_Pin);
	  inputs[14] = HAL_GPIO_ReadPin(OUT1_OP15_GPIO_Port, OUT1_OP15_Pin);
	  inputs[15] = HAL_GPIO_ReadPin(OUT1_OP16_GPIO_Port, OUT1_OP16_Pin);
    osDelay(1);
  }
  /* USER CODE END StartReadInput4 */
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
