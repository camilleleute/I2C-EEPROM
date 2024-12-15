#include "main.h"

#define RANDOM_ADDRESS 0x1EAF
#define RANDOM_DATA 0x3C


void SystemClock_Config(void);

int main(void)
{

  HAL_Init();
  SystemClock_Config();
  SysTick_Init();
  EEPROM_init();
  LED_init();

  // turn LED off
  GPIOA->ODR &= ~(GPIO_ODR_OD5);
  EEPROM_write(RANDOM_ADDRESS, RANDOM_DATA);
  // delay by 5ms to avoid issues
  delay_us(5000);
  uint32_t data = EEPROM_read(RANDOM_ADDRESS);

  // if data at address matches original written data, turn on LED
  if(data == RANDOM_DATA)
  {
	  GPIOA->ODR |= (GPIO_ODR_OD5);
  }

  while (1)
  {

  }

}

uint8_t EEPROM_read(uint16_t addr) {
	// configure to start writing
	I2C1->CR2 = (I2C_CR2_AUTOEND | ADDRESS | (2 << I2C_CR2_NBYTES_Pos));
	// set start to launch communication
	I2C1->CR2 |= I2C_CR2_START;
	// wait for start to finish
	while(!(I2C1->ISR & I2C_ISR_TXIS));
	// send upper byte of address
	I2C1->TXDR = (addr >> BYTE);
	// wait for it to send
	while (!(I2C1->ISR & I2C_ISR_TXIS));
	// send lower byte of address
	I2C1->TXDR = (addr & BYTE_MASK);
	// wait for it to send, finished writing address
	while(!(I2C1->ISR & I2C_ISR_STOPF));
	// configure to read 1 byte ONLY
	I2C1->CR2 = (I2C_CR2_AUTOEND | ADDRESS | (1 << I2C_CR2_NBYTES_Pos) | I2C_CR2_RD_WRN);
	// start reading that byte
	I2C1->CR2 |= I2C_CR2_START;
	// wait for data to be received
	while (!(I2C1->ISR & I2C_ISR_STOPF));
	// read data
	uint8_t data = I2C1->RXDR;
	// return data
	return data;
}

void EEPROM_write(uint16_t addr, uint8_t data)
{
	// configure to start writing
	I2C1->CR2 = (I2C_CR2_AUTOEND | ADDRESS | (3 << I2C_CR2_NBYTES_Pos));
	// set start to launch communication
	I2C1->CR2 |= I2C_CR2_START;
	// wait for start to finish
	while(!(I2C1->ISR & I2C_ISR_TXE));
	// send upper byte of address
	I2C1->TXDR = (addr >> BYTE);
	// wait to finish transmission
	while (!(I2C1->ISR & I2C_ISR_TXIS));
	// send lower byte of address
	I2C1->TXDR = (addr & BYTE_MASK);
	// wait to finish transmission
	while (!(I2C1->ISR & I2C_ISR_TXIS));
	// send data!
	I2C1->TXDR = data;
	// wait for transmission to finish
	while(!(I2C1->ISR & I2C_ISR_STOPF));
}

void EEPROM_init(void)
{
	// GPIO for I2C: PB6 is SCL, PB7 is SDA (in AF4)
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	GPIOB->MODER &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
	GPIOB->MODER |= GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1;
	GPIOB->OTYPER |= GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7;
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7);
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED6 | GPIO_OSPEEDR_OSPEED7;
	// set to AF4 for I2C1 SCL and SDA
	GPIOB->AFR[0] |= (0x4 << GPIO_AFRL_AFSEL6_Pos) | (0x4 << GPIO_AFRL_AFSEL7_Pos);
	// set I2C clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;
	// disable I2C to configure it
	I2C1->CR1 &= ~I2C_CR1_PE;
	// I2C Timing setup
	I2C1->TIMINGR = I2C_TIMING;	//0xE14
	// ENABLE I2C
	I2C1->CR1 |= I2C_CR1_PE;
}

/* Delay function using the SysTick timer to count CPU clock cycles for more
 * precise delay timing. Passing a time of 0 will cause an error and result
 * in the maximum delay. Short delays are limited by the clock speed and will
 * often result in longer delay times than specified. @ 4MHz, a delay of 1us
 * will result in a delay of 10-15 us.
 */
void delay_us(const uint32_t time_us) {
    // set the counts for the specified delay
    SysTick->LOAD = (uint32_t)((time_us * (SystemCoreClock / 1000000)) - 1);
    SysTick->VAL = 0;                                      // clear the timer count
    SysTick->CTRL &= ~(SysTick_CTRL_COUNTFLAG_Msk);        // clear the count flag
    while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)); // wait for the flag
}

/* Configure SysTick Timer for use with delay_us function. This will break
 * break compatibility with HAL_delay() by disabling interrupts to allow for
 * shorter delay timing.
 */
void SysTick_Init(void){
    SysTick->CTRL |= (SysTick_CTRL_ENABLE_Msk |	       // enable SysTick Timer
                      SysTick_CTRL_CLKSOURCE_Msk);     // select CPU clock
    SysTick->CTRL &= ~(SysTick_CTRL_TICKINT_Msk);      // disable interrupt,
                                                       // breaks HAL delay function
}

void LED_init(void){
	  // Configure PA5 output
	  GPIOA->MODER &= ~(GPIO_MODER_MODE5);
	  GPIOA->MODER |= (GPIO_MODER_MODE5_0);
	  // Configure PC4-PC7 as push-pull outputs
	  GPIOA->OTYPER &= ~(GPIO_OTYPER_OT5);
	  //All of them are slow
	  GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED5);
}


// 4MHz clock
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
