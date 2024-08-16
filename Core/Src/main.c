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
/* USER CODE BEGIN PFP */
void i2c_I2C1_GPIO_config(void);
void i2c_I2C1_config(void);
bool i2c_I2C1_isSlaveAddressExist(uint8_t Addr);
bool i2c_I2C1_masterReceive(uint8_t Addr, uint8_t *pData, uint8_t len);
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
  /* USER CODE BEGIN 2 */
  i2c_I2C1_GPIO_config();
  i2c_I2C1_config();
  //  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);

  LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);
  //  for (int i = 0; i < 1000; ++i)
  //    ;
  if (i2c_I2C1_isSlaveAddressExist(0x68 << 1))
  {
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);
  }
  //  uint8_t data;
  //  i2c_I2C1_masterReceive(117, &data, 1);
  /* USER CODE END 2 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

  /**/
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
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
  // ta có f=8Mhz -> T=0.125ns, ta muốn T_high=T_low=5us => CCR=5us/0.125ns=40
  I2C1->CCR = 0x28;
  // Bật ngoại vi I2C dùng I2C_CR1 bằng cách đặt PE=1
  I2C1->CR1 |= I2C_CR1_PE;
  //  // Bật bit ACK để nhận ACK mỗi khi nhận được byte dữ liệu hoặc địa chỉ
  //  I2C1->CR1 |= I2C_CR1_ACK;
}

bool i2c_I2C1_isSlaveAddressExist(uint8_t Addr)
{
  uint32_t count = 0;
  // Gửi đi�?u kiện Start ra

  I2C1->CR1 &= ~(I2C_CR1_POS);

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
  __IO uint32_t tempRd = I2C1->SR1;
  tempRd = I2C1->SR2;
  (void)tempRd; // B�? qua biến tempRd
  // Ch�? I2C vào trạng thái bận
  while ((I2C1->SR1 & I2C_SR2_BUSY))
  {
    if (++count > 20)
      return false;
  }
  return true;
}

bool i2c_I2C1_masterTransmit(uint8_t Addr, uint8_t *pData, uint8_t len)
{
  uint32_t count = 0;
  // Chờ I2C vào trạng thái bận
  while ((I2C1->SR1 & I2C_SR2_BUSY))
  {
    if (++count > 20)
      return false;
  }
  // Tạo điều kiện Start
  I2C1->CR1 &= ~(I2C_CR1_POS);
  I2C1->CR1 |= I2C_CR1_START;
  // Chờ bit start được tạo
  while (!(I2C1->SR1 & I2C_SR1_SB))
  {
    if (++count > 20)
      return false;
  }
  count = 0;
  // Gửi địa chỉ slave
  I2C1->DR = Addr;
  // Chờ ACK
  while (!(I2C1->SR1 & I2C_SR1_ADDR))
  {
    if (++count > 20)
      return false;
  }
  count = 0;
  // Xóa cờ Addr
  __IO uint32_t tempRd = I2C1->SR1;
  tempRd = I2C1->SR2;
  (void)tempRd;
  // Gửi dữ liệu
  uint8_t dataIndex = 0;
  while (dataIndex < len)
  {
    // Kiểm tra bộ đệm Tx có trống không
    while (!(I2C1->SR1 & I2C_SR1_TXE))
    {
      if (++count > 20)
        return false;
    }
    I2C1->DR = pData[dataIndex];
    dataIndex++;
    // Chờ cờ BTF của thanh ghi SR1
    count = 0;
    while (!(I2C1->SR1 & I2C_SR1_BTF))
    {
      if (++count > 20)
        return false;
    }
  }

  // Tạo điều kiện dừng
  I2C1->CR1 |= I2C_CR1_STOP;
  return true;
}
bool i2c_I2C1_masterReceive(uint8_t Addr, uint8_t *pData, uint8_t len)
{
  uint32_t count = 0;
  uint8_t dataIndex = 0;
  // Chờ I2C vào trạng thái bận
  while ((I2C1->SR1 & I2C_SR2_BUSY))
  {
    if (++count > 20)
      return false;
  }
  count = 0;
  // Tạo điều kiện Start
  I2C1->CR1 &= ~(I2C_CR1_POS);
  I2C1->CR1 |= I2C_CR1_ACK;
  I2C1->CR1 |= I2C_CR1_START;
  // Chờ bit start được tạo
  while (!(I2C1->SR1 & I2C_SR1_SB))
  {
    if (++count > 20)
      return false;
  }
  count = 0;
  // Gửi địa chỉ slave
  I2C1->DR = Addr | 0x01; // Đọc địa chỉ
  // Chờ ACK
  while (!(I2C1->SR1 & I2C_SR1_ADDR))
  {
    if (++count > 20)
      return false;
  }
  count = 0;
  // Nhận dữ liệu
  if (len == 0)
  {
    // Xóa cờ Addr
    __IO uint32_t tempRd = I2C1->SR1;
    tempRd = I2C1->SR2;
    (void)tempRd;
    // Tạo điều kiện dừng
    I2C1->CR1 |= I2C_CR1_STOP;
    return true;
  }
  else if (len == 1)
  {
    // Xóa bit ACK
    I2C1->CR1 &= ~(I2C_CR1_ACK);

    // Xóa cờ Addr
    __IO uint32_t tempRd = I2C1->SR1;
    tempRd = I2C1->SR2;
    (void)tempRd;
    // Tạo điều kiện dừng
    I2C1->CR1 |= I2C_CR1_STOP;
  }
  else if (len == 2)
  {
    // Đặt bit POS=1
    I2C1->CR1 |= (I2C_CR1_POS);
    // Xóa cờ Addr
    __IO uint32_t tempRd = I2C1->SR1;
    tempRd = I2C1->SR2;
    (void)tempRd;
    // Xóa ACK
    I2C1->CR1 &= ~(I2C_CR1_ACK);
    // Tạo điều kiện dừng
    I2C1->CR1 |= I2C_CR1_STOP;
  }
  else
  {
    // Xóa cờ Addr
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
        // Chờ bộ đệm nhận trống

        while (!(I2C1->SR1 & I2C_SR1_RXNE))
        {
          if (++count > 20)
            return false;
        }
        count = 0;
        pData[dataIndex] = (uint8_t)I2C1->DR;
        dataIndex++;
      }
      else if (len == 2)
      {
        // Chờ BTF=1
        while (!(I2C1->SR1 & I2C_SR1_BTF))
        {
          if (++count > 20)
            return false;
        }
        count = 0;

        // Đọc DR 2 lần
        pData[dataIndex] = (uint8_t)I2C1->DR;
        dataIndex++;
        pData[dataIndex] = (uint8_t)I2C1->DR;
        dataIndex++;
      }
      else
      {
        // Chờ BTF=1
        while (!(I2C1->SR1 & I2C_SR1_BTF))
        {
          if (++count > 20)
            return false;
        }
        count = 0;
        // Xóa ACK
        I2C1->CR1 &= ~(I2C_CR1_ACK);
        // Đọc dữ liệu
        pData[dataIndex] = (uint8_t)I2C1->DR;
        dataIndex++;
        // Đọc thêm
        pData[dataIndex] = (uint8_t)I2C1->DR;
        dataIndex++;
        // Chờ bộ đệm nhận trống
        while (!(I2C1->SR1 & I2C_SR1_RXNE))
        {
          if (++count > 20)
            return false;
        }
        count = 0;
        pData[dataIndex] = (uint8_t)I2C1->DR;
        dataIndex++;
      }
    }
    else // len>3
    {
      // Chờ bộ đệm nhận trống

      while (!(I2C1->SR1 & I2C_SR1_RXNE))
      {
        if (++count > 20)
          return false;
      }
      count = 0;
      pData[dataIndex] = (uint8_t)I2C1->DR;
      dataIndex++;
    }
  }
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

#ifdef USE_FULL_ASSERT
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
