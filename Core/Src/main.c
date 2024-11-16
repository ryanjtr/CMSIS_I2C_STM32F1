/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include <stdbool.h>
#include "stm32f1xx_it.h"
#include "string.h"
#include "stdarg.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void i2c_I2C1_GPIO_config(void);
void i2c_I2C1_config(void);
bool i2c_I2C1_isSlaveAddressExist(uint8_t Addr);

bool i2c_I2C1_masterTransmit_IT(uint8_t Addr, uint8_t reg, uint8_t *pData, uint8_t len, uint32_t timeout);
bool i2c_I2C1_masterTransmit(uint8_t Addr, uint8_t reg, uint8_t *pData, uint8_t len, uint32_t timeout);
bool i2c_I2C1_masterReceive(uint8_t Addr, uint8_t reg, uint8_t *pData, uint8_t len, uint32_t timeout);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void uart_print(const char *str)
{
  uint32_t length = strlen(str);
  for (uint32_t i = 0; i < length; i++)
  {
    // Ch�? cho đến khi bộ truy�?n sẵn sàng
    while (!LL_USART_IsActiveFlag_TXE(USART1))
      ;
    LL_USART_TransmitData8(USART1, (uint8_t)str[i]);
  }
  // �?ảm bảo gửi xong byte cuối cùng
  while (!LL_USART_IsActiveFlag_TC(USART1))
    ;
}

void uart_printf(const char *format, ...)
{
  char buffer[128]; // Tạo buffer đủ lớn để chứa chuỗi kết quả
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args); // Dùng vsnprintf để format chuỗi
  va_end(args);
  uart_print(buffer); // In chuỗi format qua UART
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
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  */
  LL_GPIO_AF_Remap_SWJ_NOJTAG();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  i2c_I2C1_GPIO_config();
  i2c_I2C1_config();

  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);

  NVIC_EnableIRQ(I2C1_EV_IRQn); // Kích hoạt ngắt sự kiện I2C1
  NVIC_EnableIRQ(I2C1_ER_IRQn); // Kích hoạt ngắt lỗi I2C1
  //  uint8_t data = 5;
  uint8_t data1[8] = {8, 1, 2, 3, 4, 5, 6, 7};
//  uint8_t rxdata[3] = {6, 7, 8};
  //  if (i2c_I2C1_masterTransmit(0x68 << 1, 0x03, data1, 7, 1000))
  //  {
  //    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_5);
  //  }
  //  if (i2c_I2C1_masterReceive(0x68 << 1, 0x03, &rxdata[1], 1000))
  //  {
  //    if (data1[2] == rxdata[1])
  //    {
  //      LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);
  //    }
  //  }
  //  if (i2c_I2C1_masterReceive(0x68 << 1, 0x03, &rxdata[1], 1, 1000))
  //  {
  //    if (6 == rxdata[1])
  //    {
  //      LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);
  //    }
  //  }
//  if (i2c_I2C1_masterTransmit_IT(0x68 << 1, 0x03, data1, 8, 1000))
//  {
//    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);
//  }
  //  i2c_I2C1_masterTransmit_IT(0x68 << 1, 0x03, data, 1000);

  //  uint8_t rx_data[1];
  //  if (i2c_I2C1_masterTransmit(0x68 << 1, 0x03, &data[1],1000))
  //  {
  //    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);
  //  }
  //  if (i2c_I2C1_masterReceive(0x68 << 1, 0x03, &rx_data[0],1000))
  //  {
  //    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_5);
  //  }
  //  i2c_I2C1_masterReceive(117, &data, 1);
  uart_print("master\r\n");
//  LL_mDelay(10000);
  uint8_t count=0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	    if (i2c_I2C1_masterTransmit(0x55 << 1, 0x03, data1, 1, 1000))
	    {
	      LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
	      uart_printf("count=%d\r\n",count);
	      count++;
	    }
	    LL_mDelay(5000);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(8000000);
  LL_SetSystemCoreClock(8000000);
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

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * The function configures GPIO pins PB6 (SCL) and PB7 (SDA) for I2C communication on I2C1 interface.
 */
void i2c_I2C1_GPIO_config(void)
{
  // PB6 (SCL), PB7(SDA)
  // Bật xung clock PORTB
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
  // Chế độ: xuất 10Mhz
  GPIOB->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_MODE7); // Xóa MODE6 và MODE7
  GPIOB->CRL |= (GPIO_CRL_MODE6_0 | GPIO_CRL_MODE7_0);
  // CNF: Alternate function Open-Drain
  GPIOB->CRL |= (GPIO_CRL_CNF6 | GPIO_CRL_CNF7);
}

/**
 * The function `i2c_I2C1_config` configures the I2C1 peripheral for communication at a speed of
 * 100KHz.
 */
void i2c_I2C1_config(void)
{
  // Bật xung clock I2C
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

  // Cấu hình tần số I2C dùng thanh ghi I2C_CR2
  // tần số này trùng với tần số APB1 mà bạn đã cấu hình trước đó
  I2C1->CR2 &= ~(I2C_CR2_FREQ);
  I2C1->CR2 |= (8UL << 0);
  //  Cấu hình TRISE dùng thanh ghi I2C_TRISE
  //  như đã thấy ở phần ví dụ bit 5:0 của thanh ghi I2C_TRISE, ta dùng tần số 8Mhz nên TRISE= 0x09
  I2C1->TRISE &= ~(0xFF);
  I2C1->TRISE |= 0x09;
  // Cấu hình tốc độ I2C (100KHz SCL) dùng thanh ghi I2C_CCR
  // ta có f=8Mhz -> T=0.125us, ta muốn T_high=T_low=5us => CCR=5us/0.125us=40 <=> 0x28
  //ta có f=8Mhz -> T=0.125us, ta muốn T_high=T_low=50us => CCR=50us/0.125us=400 <=> 0x190 (10KHz)
  I2C1->CCR = 0x190;
  // Bật ngoại vi I2C dùng I2C_CR1 bằng cách đặt PE=1
  I2C1->CR1 |= I2C_CR1_PE;
}

/**
 * The function `i2c_I2C1_isSlaveAddressExist` checks if a slave address exists on the I2C1 bus by
 * sending a start condition, transmitting the address, and checking for acknowledgment before sending
 * a stop condition.
 *
 * @param Addr The function `i2c_I2C1_isSlaveAddressExist` is checking if a slave address exists on the
 * I2C1 bus. The parameter `Addr` is the 7-bit address of the slave device that you want to check for
 * existence on the I2C bus. The
 *
 * @return The function `i2c_I2C1_isSlaveAddressExist` returns a boolean value - `true` if the slave
 * address exists on the I2C bus, and `false` if it does not.
 */
bool i2c_I2C1_isSlaveAddressExist(uint8_t Addr)
{
  uint32_t count = 0;

  // Bit POS được xóa để đảm bảo I2C hoạt động trong chế độ chuẩn (standard mode).
  I2C1->CR1 &= ~(I2C_CR1_POS);
  // Gửi đi�?u kiện Start ra
  I2C1->CR1 |= I2C_CR1_START;
  // Ch�? bit start được tạo
  while (!(I2C1->SR1 & I2C_SR1_SB))
  {
    if (++count > 20)
      return false;
  }
  count = 0;
  // Xóa SB bằng cách đ�?c thanh ghi SR1, sau đó ghi địa chỉ vào thanh ghi DR
  //  Gửi địa chỉ slave ra
  I2C1->DR = Addr;
  //  LL_I2C_TransmitData8(I2C1, Addr);
  // Ch�? ACK
  while (!(I2C1->SR1 & I2C_SR1_ADDR))
    ;
  {
    if (++count > 100)
      return false;
  }
  count = 0;
  // Tạo đi�?u kiện Stop
  I2C1->CR1 |= I2C_CR1_STOP;
  // Xóa c�? Addr bằng cách đ�?c SR1 trước rồi tiếp đến SR2
  (void)I2C1->SR1;
  (void)I2C1->SR2;
  // Ch�? I2C vào trạng thái bận
  while ((I2C1->SR1 & I2C_SR2_BUSY))
  {
    if (++count > 20)
      return false;
  }
  return true;
}

uint8_t Address_slave;
uint8_t reg_slave;
uint8_t *tx_data;
uint8_t tx_len;
uint8_t tx_index = 0;
bool i2c_I2C1_masterTransmit_IT(uint8_t Addr, uint8_t reg, uint8_t *pData, uint8_t len, uint32_t timeout)
{
  uint32_t count = 0;
  // Ch�? I2C vào trạng thái bận
  while ((I2C1->SR1 & I2C_SR2_BUSY))
  {
    if (++count > 20)
      return false;
  }
  count = 0;
  // Bit POS được xóa để đảm bảo I2C hoạt động trong chế độ chuẩn (standard mode).
  I2C1->CR1 &= ~(I2C_CR1_POS);
  // Bật ngắt bộ đệm
  I2C1->CR2 |= I2C_CR2_ITBUFEN;
  // Bật ngắt sự kiện
  I2C1->CR2 |= I2C_CR2_ITEVTEN;
  // Bật ngắt phát hiện lỗi
  I2C1->CR2 |= I2C_CR2_ITERREN;

  Address_slave = Addr;
  reg_slave = reg;
  tx_data = pData;
  tx_len = len;
  // Tạo đi�?u kiện Start
  I2C1->CR1 |= I2C_CR1_START;
  return true;
}

bool i2c_I2C1_masterReceive(uint8_t Addr, uint8_t reg, uint8_t *pData, uint8_t len, uint32_t timeout)
{
  uint32_t count = 0;
  uint8_t dataIndex = 0;
  // Ch�? I2C vào trạng thái bận
  while ((I2C1->SR1 & I2C_SR2_BUSY))
  {
    if (++count > timeout)
      return false;
  }
  count = 0;

  // Bit POS được xóa để đảm bảo I2C hoạt động trong chế độ chuẩn (standard mode).
  I2C1->CR1 &= ~(I2C_CR1_POS);
  // Trả v�? ACK sau mỗi lần nhận được địa chỉ đúng và dữ liệu
  I2C1->CR1 |= I2C_CR1_ACK;
  // Tạo đi�?u kiện Start
  I2C1->CR1 |= I2C_CR1_START;
  // Ch�? bit start được tạo
  while (!(I2C1->SR1 & I2C_SR1_SB))
  {
    if (++count > timeout)
      return false;
  }
  count = 0;
  // Gửi địa chỉ slave
  I2C1->DR = Addr | 0x01; // �?�?c địa chỉ
  // Ch�? ACK
  while (!(I2C1->SR1 & I2C_SR1_ADDR))
  {
    if (++count > timeout)
      return false;
  }
  count = 0;
  // Nhận dữ liệu
  if (len == 0)
  {
    // Xóa c�? Addr
    (void)I2C1->SR1;
    (void)I2C1->SR2;
    // Tạo đi�?u kiện dừng
    I2C1->CR1 |= I2C_CR1_STOP;
    return true;
  }
  else if (len == 1)
  {
    // Xóa bit ACK
    I2C1->CR1 &= ~(I2C_CR1_ACK);

    // Xóa c�? Addr
    __IO uint32_t tempRd = I2C1->SR1;
    tempRd = I2C1->SR2;
    (void)tempRd;
    // Tạo đi�?u kiện dừng
    I2C1->CR1 |= I2C_CR1_STOP;
  }
  else if (len == 2)
  {
    // Bit POS được xóa để đảm bảo I2C hoạt động trong chế độ chuẩn (standard mode).
    I2C1->CR1 |= (I2C_CR1_POS);
    // Xóa c�? Addr
    __IO uint32_t tempRd = I2C1->SR1;
    tempRd = I2C1->SR2;
    (void)tempRd;
    // Xóa ACK
    I2C1->CR1 &= ~(I2C_CR1_ACK);
    // Tạo đi�?u kiện dừng
    I2C1->CR1 |= I2C_CR1_STOP;
  }
  else
  {
    // Xóa c�? Addr
    __IO uint32_t tempRd = I2C1->SR1;
    tempRd = I2C1->SR2;
    (void)tempRd;
  }

  while (dataIndex < len)
  {
    if (len <= 3)
    {
      if (len == 1)
      {
        // Ch�? bộ đệm nhận trống

        while (!(I2C1->SR1 & I2C_SR1_RXNE))
        {
          if (++count > timeout)
            return false;
        }
        count = 0;
        pData[dataIndex] = (uint8_t)I2C1->DR;
        dataIndex++;
      }
      else if (len == 2)
      {
        // Ch�? BTF=1
        while (!(I2C1->SR1 & I2C_SR1_BTF))
        {
          if (++count > timeout)
            return false;
        }
        count = 0;

        // �?�?c DR 2 lần
        pData[dataIndex] = (uint8_t)I2C1->DR;
        dataIndex++;
        pData[dataIndex] = (uint8_t)I2C1->DR;
        dataIndex++;
      }
      else
      {
        // Ch�? BTF=1
        while (!(I2C1->SR1 & I2C_SR1_BTF))
        {
          if (++count > timeout)
            return false;
        }
        count = 0;
        // Xóa ACK
        I2C1->CR1 &= ~(I2C_CR1_ACK);
        // �?�?c dữ liệu
        pData[dataIndex] = (uint8_t)I2C1->DR;
        dataIndex++;
        // �?�?c thêm
        pData[dataIndex] = (uint8_t)I2C1->DR;
        dataIndex++;
        // Ch�? bộ đệm nhận trống
        while (!(I2C1->SR1 & I2C_SR1_RXNE))
        {
          if (++count > timeout)
            return false;
        }
        count = 0;
        pData[dataIndex] = (uint8_t)I2C1->DR;
        dataIndex++;
      }
    }
    else // len>3
    {
      // Ch�? bộ đệm nhận trống

      while (!(I2C1->SR1 & I2C_SR1_RXNE))
      {
        if (++count > timeout)
          return false;
      }
      count = 0;
      pData[dataIndex] = (uint8_t)I2C1->DR;
      dataIndex++;
    }
  }
  return true;
}

void I2C1_EV_IRQHandler(void)
{

  // Kiểm tra c�? SB (Start Bit) được set
  if (I2C1->SR1 & I2C_SR1_SB)
  {
    // Gửi địa chỉ thiết bị với bit ghi (0)
    I2C1->DR = Address_slave;
  }

  // Kiểm tra c�? ADDR (Address Sent)
  else if (I2C1->SR1 & I2C_SR1_ADDR)
  {
    // �?�?c SR1 và SR2 để xóa c�? ADDR
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    // Gửi địa chỉ thanh ghi đầu tiên
    I2C1->DR = reg_slave;
  }

  // Kiểm tra c�? TXE (Transmit Data Register Empty)
  else if (I2C1->SR1 & I2C_SR1_TXE)
  {

    if (tx_index < tx_len)
    {
      // Gửi byte dữ liệu tiếp theo
      I2C1->DR = tx_data[tx_index++];
    }
    else
    {
      // Tạo đi�?u kiện Stop khi gửi xong
      I2C1->CR1 |= I2C_CR1_STOP;
      LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_5);
      // Tắt các ngắt để tránh kích hoạt ngắt không cần thiết
      I2C1->CR2 &= ~(I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
    }
  }
}
bool i2c_I2C1_masterTransmit(uint8_t Addr, uint8_t reg, uint8_t *pData, uint8_t len, uint32_t timeout)
{
  uint32_t count = 0;
  uint8_t index = 0;
  // Ch�? I2C vào trạng thái bận
  while ((I2C1->SR1 & I2C_SR2_BUSY))
  {
    if (++count > timeout)
      return false;
  }

  // Bit POS được xóa để đảm bảo I2C hoạt động trong chế độ chuẩn (standard mode).
  I2C1->CR1 &= ~(I2C_CR1_POS);
  // Tạo đi�?u kiện Start
  I2C1->CR1 |= I2C_CR1_START;
  // Ch�? bit start được tạo
  while (!(I2C1->SR1 & I2C_SR1_SB))
  {
    if (++count > timeout)
      return false;
  }
  count = 0;
  // Gửi địa chỉ slave
  I2C1->DR = Addr;
  // Ch�? ACK
  while (!(I2C1->SR1 & I2C_SR1_ADDR))
  {
    if (++count > timeout)
      return false;
  }
  count = 0;
  // Xóa c�? Addr
  (void)I2C1->SR1;
  (void)I2C1->SR2;
  // Gửi thanh ghi thiết bị cần ghi ra
  I2C1->DR = reg;
  // Truy�?n dữ liệu
  while (len > 0U)
  {
    // Kiểm tra bộ đệm Tx có trống không
    while (!(I2C1->SR1 & I2C_SR1_TXE))
    {
      if (++count > timeout)
        return false;
    }
    count = 0;
    // Gửi dữ liệu ra
    I2C1->DR = pData[index];
    len--;
    index++;
    // Nếu truy�?n xong BTF=1 và len != 0 thì truy�?n tiếp
    if ((I2C1->SR1 & I2C_SR1_BTF) && (len != 0))
    {
      // Gửi dữ liệu ra
      I2C1->DR = pData[index];
      len--;
      index++;
    }
  }
  // Tạo đi�?u kiện dừng
  I2C1->CR1 |= I2C_CR1_STOP;
  return true;
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
