/*
 * i2c.c
 *
 *  Created on: Nov 19, 2024
 *      Author: dell
 */

#include "i2c.h"

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
  //  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_6, LL_GPIO_PULL_UP);
  //  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_7, LL_GPIO_PULL_UP);
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
  // ta có f=8Mhz -> T=0.125us, ta muốn T_high=T_low=5us => CCR=5us/0.125us=40 <=> 0x28 (100KHz)
  // ta có f=8Mhz -> T=0.125us, ta muốn T_high=T_low=50us => CCR=50us/0.125us=400 <=> 0x190 (10KHz)
  // ta có f=72Mhz -> T=..us, ta muốn T_high=T_low=5us => CCR=5us/..us=360 <=> 0x168 (100KHz)
  I2C1->CCR = 0x168;
  // Bật ngoại vi I2C dùng I2C_CR1 bằng cách đặt PE=1
  I2C1->CR1 |= I2C_CR1_PE;

  //  NVIC_EnableIRQ(I2C1_EV_IRQn);
  //  NVIC_EnableIRQ(I2C1_ER_IRQn);
}

bool i2c_I2C1_masterTransmit(uint8_t Addr, uint8_t *pData, uint8_t len, uint32_t timeout)
{
  uint32_t count = 0;
  uint8_t index = 0;

  // Chờ I2C vào trạng thái bận
  while ((I2C1->SR2 & I2C_SR2_BUSY))
  {
    if (++count > timeout)
      return false;
  }


  // Xóa POS và tạo điều kiện Start
  I2C1->CR1 &= ~(I2C_CR1_POS);
  I2C1->CR1 |= I2C_CR1_START;

  // Chờ bit Start được set
  while (!(I2C1->SR1 & I2C_SR1_SB))
  {
    if (++count > timeout)
      return false;
  }
  count = 0;

  // Gửi địa chỉ Slave
  I2C1->DR = Addr << 1;

  while (!(I2C1->SR1 & I2C_SR1_ADDR))
  {
    if (++count > timeout)
    {
      uart_printf("ADDR timeout\r\n");
      return false;
    }
  }
  count = 0;

  // Xóa bit ADDR
  (void)I2C1->SR1;
  (void)I2C1->SR2;

  // Truyền dữ liệu
  while (len > 0U)
  {
    // Chờ bộ đệm trống
    while (!(I2C1->SR1 & I2C_SR1_TXE))
    {
      if (++count > timeout)
        return false;
    }
    count = 0;

    // Gửi dữ liệu
    I2C1->DR = pData[index];
    index++;
    len--;

    // Nếu còn dữ liệu và BTF=1, gửi tiếp byte tiếp theo
    if ((len > 0U) && (I2C1->SR1 & I2C_SR1_BTF))
    {
      I2C1->DR = pData[index];
      index++;
      len--;
    }
  }

  // Chờ byte cuối cùng hoàn tất
  while (!(I2C1->SR1 & I2C_SR1_BTF))
  {
    if (++count > timeout)
      return false;
  }

  // Tạo điều kiện STOP
  I2C1->CR1 |= I2C_CR1_STOP;

  return true;
}

uint8_t DS3231_Read(uint8_t Addr, uint8_t *pData, uint8_t reg,uint8_t len, uint32_t timeout)
{
  uint32_t count = 0;
  uint8_t dataIndex = 0;
  uint8_t dataSize = len;

  // Wait until BUSY flag is reset
  while ((I2C1->SR1 & I2C_SR2_BUSY))
  {
    if (++count > timeout)
      return false;
  }
  count = 0;

  // Disable POS
  I2C1->CR1 &= ~(I2C_CR1_POS);

  // Enable ACK
  I2C1->CR1 |= I2C_CR1_ACK;

  // Generate start condition
  I2C1->CR1 |= I2C_CR1_START;

  // Wait until SB flag is set
  while (!(I2C1->SR1 & I2C_SR1_SB))
  {
    if (++count > timeout)
      return false;
  }
  count = 0;

  // Send slave address write
  I2C1->DR = (Addr << 1);


  // Wait until ADDR flag is set
  while (!(I2C1->SR1 & I2C_SR1_ADDR))
  {
    if (++count > timeout)
    {
      uart_printf("ADDR timeout1\r\n");
      return false;
    }
  }
  count = 0;

  //	  Clear ADDR flag
  (void)I2C1->SR1;
  (void)I2C1->SR2;

  // register slave
  I2C1->DR = reg;

  if (I2C1->SR1 & I2C_SR1_ARLO)
  {
    uart_printf("arlo 2\r\n");
  }
  while (!(I2C1->SR1 & I2C_SR1_TXE))
  {
    if (++count > timeout)
    {
      uart_printf("error txe\r\n");
      return false;
    }
  }
  count = 0;
  if (I2C1->SR1 & I2C_SR1_ARLO)
  {
    uart_printf("arlo 3\r\n");
  }
  // Generate start condition
  I2C1->CR1 |= I2C_CR1_START;
  // Wait until SB flag is set
  while (!(I2C1->SR1 & I2C_SR1_SB))
  {
    if (++count > timeout)
      return false;
  }
  count = 0;
  if (I2C1->SR1 & I2C_SR1_ARLO)
  {
    uart_printf("arlo 4\r\n");
  }
  // Send slave address
  I2C1->DR = (Addr << 1) | 1; // R/W=1 (read)
  if (I2C1->SR1 & I2C_SR1_ARLO)
  {
    uart_printf("arlo 5\r\n");
  }
  // Wait until ADDR flag is set
  while (!(I2C1->SR1 & I2C_SR1_ADDR))
  {
    if (++count > timeout)
    {
      uart_printf("ADDR timeout2\r\n");
      return false;
    }
  }
  count = 0;
  if (I2C1->SR1 & I2C_SR1_ARLO)
  {
    uart_printf("arlo 6\r\n");
  }
  if (dataSize == 1)
  {

    // Disable Acknowledge
    I2C1->CR1 &= ~(I2C_CR1_ACK);
    __disable_irq();

    // Clear ADDR flag
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    // Generate Stop
    I2C1->CR1 |= I2C_CR1_STOP;

    /* Re-enable IRQs */
    __enable_irq();
  }

  while (dataSize > 0)
  {
    if (dataSize <= 3)
    {
      /* One byte */
      if (dataSize == 1)
      {
        // Wait until RXNE flag is set
        while (!(I2C1->SR1 & I2C_SR1_RXNE))
        {
          if (++count > timeout)
          {
            uart_printf("wait rx 1 byte fail\r\n");
            return false;
          }
        }
        count = 0;

        // Read data from DR
        pData[dataIndex] = (uint8_t)I2C1->DR;
        dataIndex++;
        dataSize--;
      }
    }
  }
  return true;
}

bool i2c_I2C1_masterTransmit_IT(uint8_t Addr, uint8_t *pData, uint8_t len, uint32_t timeout)
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

  // Tạo đi�?u kiện Start
  I2C1->CR1 |= I2C_CR1_START;
  return true;
}

bool i2c_I2C1_masterReceive(uint8_t Addr, uint8_t *pData, uint8_t len, uint32_t timeout)
{
  uint32_t count = 0;
  uint8_t dataIndex = 0;
  uint8_t dataSize = len;

  // Wait until BUSY flag is reset
  while ((I2C1->SR1 & I2C_SR2_BUSY))
  {
    if (++count > timeout)
      return false;
  }
  count = 0;

  // Disable POS
  I2C1->CR1 &= ~(I2C_CR1_POS);

  // Enable ACK
  I2C1->CR1 |= I2C_CR1_ACK;

  // Generate start condition
  I2C1->CR1 |= I2C_CR1_START;

  // Wait until SB flag is set
  while (!(I2C1->SR1 & I2C_SR1_SB))
  {
    if (++count > timeout)
      return false;
  }
  count = 0;

  // Send slave address
  I2C1->DR = (Addr << 1) | 0x01; // R/W=1 (read)

  // Wait until ADDR flag is set
  while (!(I2C1->SR1 & I2C_SR1_ADDR))
  {
    if (++count > timeout)
    {
      uart_printf("ADDR timeout\r\n");
      return false;
    }
  }
  count = 0;

  if (I2C1->SR1 & I2C_SR1_BERR)
  {
    uart_printf("berr 1\r\n");
  }

  if (dataSize == 0)
  {
    //	  Clear ADDR flag
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    // Generate Stop
    I2C1->CR1 |= I2C_CR1_STOP;
    return false;
  }
  else if (dataSize == 1)
  {

    // Disable Acknowledge
    I2C1->CR1 &= ~(I2C_CR1_ACK);

    if (I2C1->SR1 & I2C_SR1_BERR)
    {
      uart_printf("berr 2\r\n");
    }

    /* Disable all active IRQs around ADDR clearing and STOP programming because the EV6_3
    software sequence must complete before the current byte end of transfer */
    __disable_irq();

    // Clear ADDR flag
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    // Generate Stop
    I2C1->CR1 |= I2C_CR1_STOP;

    if (I2C1->SR1 & I2C_SR1_BERR)
    {
      uart_printf("berr 3\r\n");
    }

    /* Re-enable IRQs */
    __enable_irq();
  }
  else if (dataSize == 2)
  {
    // Enable Pos
    I2C1->CR1 |= (I2C_CR1_POS);

    /* Disable all active IRQs around ADDR clearing and STOP programming because the EV6_3
          software sequence must complete before the current byte end of transfer */
    __disable_irq();

    // Clear ADDR flag
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    // Disable Acknowledge
    I2C1->CR1 &= ~(I2C_CR1_ACK);

    /* Re-enable IRQs */
    __enable_irq();
  }
  else // len>2
  {
    // Enable Acknowledge
    I2C1->CR1 |= (I2C_CR1_ACK);

    //  Clear ADDR flag
    (void)I2C1->SR1;
    (void)I2C1->SR2;
  }

  while (dataSize > 0)
  {
    if (dataSize <= 3)
    {
      /* One byte */
      if (dataSize == 1)
      {
        // Wait until RXNE flag is set
        while (!(I2C1->SR1 & I2C_SR1_RXNE))
        {
          if (++count > timeout)
          {
            uart_printf("wait rx 1 byte fail\r\n");
            return false;
          }
        }
        count = 0;

        // Read data from DR
        pData[dataIndex] = (uint8_t)I2C1->DR;
        dataIndex++;
        dataSize--;
      }
      /* Two bytes */
      else if (dataSize == 2)
      {
        // Wait until BTF flag is set
        while (!(I2C1->SR1 & I2C_SR1_BTF))
        {
          if (++count > timeout)
          {
            uart_printf("btf 2 byte fail\r\n");
            return false;
          }
        }
        count = 0;

        /* Disable all active IRQs around ADDR clearing and STOP programming because the EV6_3
           software sequence must complete before the current byte end of transfer */
        __disable_irq();

        // Generate Stop
        I2C1->CR1 |= I2C_CR1_STOP;

        // Read data from DR
        pData[dataIndex] = (uint8_t)I2C1->DR;
        dataIndex++;
        dataSize--;

        /* Re-enable IRQs */
        __enable_irq();

        // Read data from DR
        pData[dataIndex] = (uint8_t)I2C1->DR;
        dataIndex++;
        dataSize--;
      }
      /* 3 Last bytes */
      else
      {
        // Wait until BTF flag is set
        while (!(I2C1->SR1 & I2C_SR1_BTF))
        {
          if (++count > timeout)
          {
            uart_printf("btf 3 byte fail\r\n");
            return false;
          }
        }
        count = 0;

        // Disable Acknowledge
        I2C1->CR1 &= ~(I2C_CR1_ACK);

        /* Disable all active IRQs around ADDR clearing and STOP programming because the EV6_3
                     software sequence must complete before the current byte end of transfer */
        __disable_irq();

        // Read data from DR
        pData[dataIndex] = (uint8_t)I2C1->DR;
        dataIndex++;
        dataSize--;

        // Wait until BTF flag is set
        while (!(I2C1->SR1 & I2C_SR1_BTF))
        {
          if (++count > timeout)
          {
            uart_printf("btf flag last in 3 byte fail\r\n");
            return false;
          }
        }
        count = 0;

        // Generate Stop
        I2C1->CR1 |= I2C_CR1_STOP;

        // Read data from DR
        pData[dataIndex] = (uint8_t)I2C1->DR;
        dataIndex++;
        dataSize--;

        /* Re-enable IRQs */
        __enable_irq();

        // Read data from DR
        pData[dataIndex] = (uint8_t)I2C1->DR;
        dataIndex++;
        dataSize--;
      }
    }
    else // len>3
    {
      // Wait until RXNE flag is set
      while (!(I2C1->SR1 & I2C_SR1_RXNE))
      {
        if (++count > timeout)
        {
          uart_printf("rxne fail > 3byte\r\n");
          return false;
        }
      }
      count = 0;

      // Read data from DR
      pData[dataIndex] = (uint8_t)I2C1->DR;
      dataIndex++;
      dataSize--;

      if (I2C1->SR1 & I2C_SR1_BTF)
      {
        if (dataSize == 3)
        {
          // Disable Acknowledge
          I2C1->CR1 &= ~(I2C_CR1_ACK);
        }
        // Read data from DR
        pData[dataIndex] = (uint8_t)I2C1->DR;
        dataIndex++;
        dataSize--;
      }
    }
  }
  return true;
}

typedef enum
{
  I2C_READY,
  I2C_BUSY,
  I2C_RECEIVING,
  I2C_ERROR
} I2C_State;

volatile I2C_State i2c1_state = I2C_READY;
volatile uint8_t *i2c1_data;
volatile uint8_t i2c1_length;
volatile uint8_t i2c1_dataIndex;
volatile bool i2c1_transferComplete = false;
uint8_t i2c1_addr;
bool I2C1_masterReceive_IT(uint8_t Addr, uint8_t *pData, uint8_t len, uint32_t timeout)
{

  // Bật ngắt bộ đệm
  I2C1->CR2 |= I2C_CR2_ITBUFEN;
  // Bật ngắt sự kiện
  I2C1->CR2 |= I2C_CR2_ITEVTEN;
  // Bật ngắt phát hiện lỗi
  //  	  I2C1->CR2 |= I2C_CR2_ITERREN;
  if (i2c1_state != I2C_READY)
  {
    return false; // I2C bus is busy
  }

  // Initialize variables
  i2c1_state = I2C_BUSY;
  i2c1_data = pData;
  i2c1_length = len;
  i2c1_dataIndex = 0;
  i2c1_addr = Addr;
  i2c1_transferComplete = false;

  // Clear POS and enable ACK
  I2C1->CR1 &= ~I2C_CR1_POS;
  I2C1->CR1 |= I2C_CR1_ACK;

  // Generate START condition
  I2C1->CR1 |= I2C_CR1_START;

  // Send address with read request (R/W = 1) in ISR

  // Wait for transfer complete using a timeout
  uint32_t count = 0;
  while (!i2c1_transferComplete)
  {
    if (++count > timeout)
    {
      I2C1->CR1 |= I2C_CR1_STOP; // Dừng bus
      i2c1_state = I2C_READY;
      return false; // Timeout
    }
  }

  i2c1_state = I2C_READY;
  return true;
}

void receive_handler()
{
  if (I2C1->SR1 & I2C_SR1_SB)
  { // START condition sent
    // Send address with read request
    I2C1->DR = i2c1_addr << 1 | 0x01;
  }
  else if (I2C1->SR1 & I2C_SR1_ADDR)
  {                  // Address matched
    (void)I2C1->SR1; // Clear ADDR flag
    (void)I2C1->SR2;

    if (i2c1_length == 1)
    {
      // Single-byte reception
      I2C1->CR1 &= ~I2C_CR1_ACK;
      I2C1->CR1 |= I2C_CR1_STOP;
    }
    else if (i2c1_length == 2)
    {
      // Dual-byte reception
      I2C1->CR1 &= ~I2C_CR1_ACK; // Clear ACK before the last two bytes
      I2C1->CR1 |= I2C_CR1_POS;  // Set POS for dual-byte reception
    }
  }
  else if (I2C1->SR1 & I2C_SR1_RXNE)
  { // Data register not empty
    i2c1_data[i2c1_dataIndex++] = (uint8_t)I2C1->DR;
    i2c1_length--;

    if (i2c1_length == 2)
    {
      // Prepare for last two bytes
      I2C1->CR1 &= ~I2C_CR1_ACK; // Clear ACK
    }
    else if (i2c1_length == 1)
    {
      // Prepare STOP condition for the last byte
      I2C1->CR1 |= I2C_CR1_STOP;
    }
    else if (i2c1_length == 0)
    {
      // Transfer complete
      i2c1_transferComplete = true;
    }
  }
  else if (I2C1->SR1 & I2C_SR1_STOPF)
  {                  // STOP condition detected
    (void)I2C1->SR1; // Clear STOP flag
    i2c1_transferComplete = true;
  }
}

void error_i2c_handler()
{
  i2c1_transferComplete = false;
  I2C1->CR1 |= I2C_CR1_STOP; // Force STOP condition
  i2c1_state = I2C_READY;    // Reset state
  uart_printf("error handler\r\n");
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
