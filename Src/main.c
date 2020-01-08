/*******************************************************************************
  * File Name          : main.c
  * Description        : Balance Bot - Inverted Pendulum
	* Author						 : JUSTIN NGO (4/27/2017)
  ******************************************************************************
	Using:
	- Custom motor control board with temperature sensor
	- GY-521 6 axis board (MPU 6050)
			1. This sesnor board has voltage regulator and low-volt drop out. 
			Other boards should use 3.3V to VCC but THIS BOARD needs 5V to VCC.
			2. The board has pull-up resistors on the I2C bus. The value of those pull-up
			are sometimes 10k and sometimes 2k2. The 2k2 is rather low. If it is combined with 
			other sensor board which have also pull-up resistors, the total pull-up impedance might be too low
			3. Some GY-521 modules have the wrong caps (or bad caps) and result in high noise level
			see the Google drive to see the picture. 
			4. AD0 for either i2c address to be 0x68 or 0x69
			5. XDA / XCL provide a master i2c interface for like adding a compass
			6. INT provides interrupt to MCU to say there is fresh sensor data
			7. VCC to 5V (read above)
			8. When starting MUST CALIBRATE THIS CHIP. 
			
			Flow:
			1. input
			2. process input
			3. periodic recalibration of gyro
			4. control of actuator unit. 
			
			Documentation:
			- The InvenSense documents:
			- "MPU-6000 and MPU-6050 Product Specification", PS-MPU-6000A.pdf
			- "MPU-6000 and MPU-6050 Register Map and Descriptions", RM-MPU-6000A.pdf or RS-MPU-6000A.pdf
			- "MPU-6000/MPU-6050 9-Axis Evaluation Board User Guide" AN-MPU-6000EVB.pdf

			- The accuracy is 16-bits.

			- Temperature sensor from -40 to +85 degrees Celsius
			340 per degrees, -512 at 35 degrees.

			- At power-up, all registers are zero, except these two:
			Register 0x6B (PWR_MGMT_2) = 0x40  (I read zero).
			Register 0x75 (WHO_AM_I)   = 0x68.
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "motor.h"
#include "math.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);



/* ___ GLOBAL VARIABLES ___ 
	 variable list goes:
	 1. raw data
	 2. scaled
	 3. angles
*/
short accelX, accelY, accelZ= 0;   									 //for +/- 2g = 16384 LSB/g (see section 4.17 in MPU 6050 reg - basically a conversion factor)
float gforceX, gforceY, gforceZ = 0;								 //[g] 
double angleAX, angleAY, angleAZ = 0;

short gyroX, gyroY, gyroZ = 0;												 //for +/- 250 deg/sec => 131 LSB/deg/sec (see section 4.19 in MPU 6050 reg - basically a conversion factor)
float degsX, degsY, degsZ = 0;											 //[deg/sec] ->for 250 deg/sec /360 deg * 60sec/min = 41.6 RPM (what it can sense up to)
double angleGX, angleGY, angleGZ = 0;

extern volatile double fused_angleX, fused_angleY, fused_angleZ; //[deg]

void  button_init(void) {
    // Initialize PA0 for button input
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;                                          // Enable peripheral clock to GPIOA
    GPIOA->MODER &= ~(GPIO_MODER_MODER0_0 | GPIO_MODER_MODER0_1);               // Set PA0 to input
    GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0_0 | GPIO_OSPEEDR_OSPEEDR0_1);     // Set to low speed
    GPIOC->PUPDR |= GPIO_PUPDR_PUPDR0_1;                                        // Set to pull-down
}

/* ____ Private function prototypes ___ */
void WaitRead(void) {
	while (1) { 
		if (I2C1->ISR & (1<<2)){ //RXNE
			break;
		}			
			
		if (I2C1->ISR & (1<<4)){ //NACKF flag is set
			// means slave didn't respond to address frame
			//do stuff (light on?)
			while (1) {};
		}
	}	
}

void WaitWrite(void) {
	while (1) { 
		if (I2C1->ISR & (1<<1)){ //TXIS flag
			break;
		}			
			
		if (I2C1->ISR & (1<<4)){ //NACKF flag is set
			// means slave didn't respond to address frame
			//do stuff (light on?)
			while (1) {};
		}
	}
}

void to_scale_accel(){											// NOTE: 16384 may change depending on choosen full scale in configuration
	gforceX = accelX / 16384.0; 				//[g]
	gforceY = accelY / 16384.0; 				//[g]
	gforceZ = accelZ / 16384.0; 				//[g]
}

void getAccel(){
	
	I2C1->CR2 &= 0x00000000; 						//reset CR2 register
	I2C1->CR2 |= (0x68<<1) | (1<<16) | (0<<10) | (1<<13);
	WaitWrite();
	I2C1->TXDR = 0x3B;
	while (!(I2C1->ISR & (1<<6))){}; 
	I2C1->CR2 &= 0x00000000; 						//reset CR2 register
	I2C1->CR2 |= (0x68<<1) | (6<<16) | (1<<10) | (1<<13);
	// read in as 2's compement
	WaitRead(); accelY = I2C1->RXDR;
	WaitRead(); accelY = (accelY<<8) | I2C1->RXDR;
	WaitRead(); accelX = I2C1->RXDR;
	WaitRead(); accelX = (accelX<<8) | I2C1->RXDR;
	WaitRead(); accelZ = I2C1->RXDR;
	WaitRead(); accelZ = (accelZ<<8) | I2C1->RXDR;
	while (!(I2C1->ISR & (1<<6))){};
	I2C1->CR2 |= (1<<14); 							// stop bit
	to_scale_accel();
}

void to_scale_gyro(void){										
  /* NOTE: 131 may change depending on choosen full scale in configuration
	degsXYZ = (gyroX / scale ) - calibration (either do a running average then update it or use STMstudio)*/
	degsX = ( gyroX  / 131.0 ) - 45; 							// [deg/sec]
	degsY = gyroY / 131.0;							// [deg/sec]
  degsZ = gyroZ / 131.0;							// [deg/sec]
}

void getGyro(void){
	I2C1->CR2 &= 0x00000000; 						//reset CR2 register
	I2C1->CR2 |= (0x68<<1) | (1<<16) | (0<<10) | (1<<13);
	WaitWrite();
	I2C1->TXDR = 0x43;
	while (!(I2C1->ISR & (1<<6))){}; 
	I2C1->CR2 &= 0x00000000; 						//reset CR2 register
	I2C1->CR2 |= (0x68<<1) | (6<<16) | (1<<10) | (1<<13);
	WaitRead(); gyroX = I2C1->RXDR;
	WaitRead(); gyroX = (gyroX<<8) | I2C1->RXDR;
	WaitRead(); gyroY = I2C1->RXDR;
	WaitRead(); gyroY = (gyroY<<8) | I2C1->RXDR;
	WaitRead(); gyroZ = I2C1->RXDR;
	WaitRead(); gyroZ = (gyroZ<<8) | I2C1->RXDR;
	while (!(I2C1->ISR & (1<<6))){};
	I2C1->CR2 |= (1<<14); 							// stop bit
	to_scale_gyro();														// [deg/sec]
}

void to_deg_gyro(){										//integrate to get degree from degree/sec
	angleGX = angleGX + (0.0375*degsX);
	angleGY = angleGY + (0.0375*degsY);
	angleGZ = angleGZ + (0.0375*degsZ);
}

void to_deg_accel(){
	angleAX = (180/3.141592) * atan(gforceX / sqrt(pow(gforceY, 2) + pow(gforceZ, 2)));  //[deg] 
	angleAY = (180/3.141592) * atan(gforceY / sqrt(pow(gforceX, 2) + pow(gforceZ, 2)));	 //[deg] 
	angleAZ = (180/3.141592) * atan(sqrt(pow(gforceY, 2) + pow(gforceX, 2)) / gforceZ);	 //[deg] 
}
/* ___ Sensor Fusion ___
	 NOTE: The two coef. must add up to 1. 
	 Accelerometer is noisy but stable over long period
	 Gyro is stable but will drift over long term.
*/
void complimentary_filter(){
	to_deg_gyro();
	to_deg_accel();
	fused_angleX = (0.02 * angleGX) + (0.98 * angleAX); 			//[deg] 
	fused_angleY = (0.02 * angleGY) + (0.98 * angleAY); 			//[deg] 
	fused_angleZ = (0.02 * angleGZ) + (0.98 * angleAZ); 			//[deg] 

}

/* ADD calibration? running average over... 100 samples? 1000 samples over a period of time? 
then take that and minus from your raw readings*/
int main(void)
{
  /* ___ MCU Configuration ___ */			
  HAL_Init(); 												//Reset of all peripherals, Initializes the Flash interface and the Systick.
  SystemClock_Config(); 							//Configure the system clock 
	
	button_init(); 											
	
	/* ___ Green LED ___ */
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;  //enable clock GPIO-C block
	GPIOC->MODER |= (1<<18); 						//output mode PC-9

	/* ___ Pin enable to I2C ___ 
	   PB6 - SCL line (i2c 1) - AF1
		 PB7 - SDA line (i2c 1) - AF1 */
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;  //enable clock to pin
	GPIOB->MODER |= (1<<13) | (1<<15);  //Alt Function mode PB6 and PB7 respectively
	GPIOB->OTYPER |= (1<<6) | (1<<7);   //Output type open drain (look at how i2c works)
	GPIOB->AFR[0] |= (1<<24) | (1<<28); //Alt function 1 for PB6 and PB7 respectively
	
	/* ___ I2C ____ */
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; //clock enable to I2C-1
	/* GY-521 standard speed is 100khz - see datasheet (can support 400khz fast mode)
		 set I2C in 100khz: There is a table in Perif. Ref. Manual that gives timing settings
	   our processor is at 8MHz */
	I2C1->TIMINGR |= (1<<28); 					//PRESC: 1
	I2C1->TIMINGR |= 0x13;							//SCLL
	I2C1->TIMINGR |= 0xF00;							//SCLH: 0xF
	I2C1->TIMINGR |= 0x20000; 					//SDADEL: 0x2
	I2C1->TIMINGR |= 0x400000; 					//SCLDEL: 0x4
	I2C1->CR1 |= 1; 										//enable I2C using PE bit in CR1 register
	
	 /* ___ NOTES: How to read I2C ___
	 Reading the register (I2c) 
	 1. set slave address in the SADD bit field (CR2 register)
	 2. set number of byte to be transmitted NBYTES bit field
	 3. Confiugre RD_WRN to indicate read/write operation
	 4. do not set autoend bit, this lab reqires software start/stop operation
	 5. setting START bit to begin address frame 
	 */
	
	/* ___ Checking "WHO_AM_I" Register ____ */
	/* GY-521 slave address is 0x68 (110 1000) *note* can be 0x69 this depends on AD0 if pulled low or high. */
	I2C1->CR2 |= (0x68<<1);              //Slave Address
	I2C1->CR2 |= (1<<16);							// set number of bytes to transmit = 1. NBYTES Fields
	I2C1->CR2 |= (0<<10); 							// set the RD_WRN bit to indicate write operation
	I2C1->CR2 |= (1<<13); 							// Set Start
	WaitWrite(); 												// 2. Wait until either of the TXIS (Transmit Regsiter Empty/Ready) or NACKF (Slave Not-Acknowledge)flag are set
	I2C1->TXDR = 0x75; 									// 3. Write the addresss of the "WHO_AM_I" register (see MPU6050 reg. sheet) into the I2C transmit register (TXDR) should get 110 100 (6 bit, doesn't reflect the AD0 state)
	while (!(I2C1->ISR & (1<<6))){}; 		// 4. wait until TC (Transfer Complete) Flag is set
		
	I2C1->CR2 |= (0x68<<1);							// 5. Reload CR2 register with same parameters but set the RD_WRN bit to READ operation remember to set START bit again to perfrom I2C restart Condition
	I2C1->CR2 |= (1<<16);								// NBYTES Fields
	I2C1->CR2 |= (1<<10); 							// set the RD_WRN bit to indicate read operation
	I2C1->CR2 |= (1<<13);								// Set Start/Restart
	WaitRead();													// 6. Wait until RXNE(recieve Register not empty) or NACKF (slave Not-awknowledge flags are set)
																			
	if (I2C1->RXDR == 0x68){ 						// 7. check the contents of the RXDR register to see if it matches 0x68	which is 110 100 in binary (0110 1000)
		GPIOC->ODR |= GPIO_ODR_9; 
	}						
	while (!(I2C1->ISR & (1<<6))){};    // 8. wait until TC(transfer complete flag is set)
	I2C1->CR2 |= (1<<14); 							// 9. set the STOP bit in the CR2 regsiter to release the I2C bus. // stop bit
	/* NOW LOOK AT THE I2C TRANSATION WITH DIGITAL ANALYZER TO SEE IF IT WORKING PROPERLY		
		 *Note*: that when master reading from slave, the last bit need to be NACK so
		 that the slave will let go of the line so that the master can write STOP */
	
	/* ___ Intializing the GY-521 board (MPU6050) ___ 
	 - Output data rate programmable  (need delay between reads)
	 - Remember, unlike STM32f0 internal peripherals you need to calcuate the desired bit pattern
     and overwite the entire register. (internal peripherals u can read-modify-write/ external u can't easily)
	 -----Initializing the GY-521-----
	 https://www.youtube.com/watch?v=M9lZ5Qy5S2s
	 1. Device power-up into sleep mode.
	 2. all registers besides 0x40(Accel_zout[7:0]) and 0x68 (pwr_mgmt_1) are 0x00 at reset
	 3. MPU6050 is able to auto increment as long as no stop. 
		*/
	/* ___ Overall board ___ */
	I2C1->CR2 &= 0x00000000; 						//reset CR2 register
	I2C1->CR2 |= (0x68<<1) | (2<<16) | (0<<10) | (1<<13);  //format of (slave address, number of packets, write/read, start)
	WaitWrite();
	I2C1->TXDR = 0x6B; 									// write the slave's internal register address you want to change 
  WaitWrite();
  I2C1->TXDR = 0x00;                  // what you want the slave's internal register to be (NOTE) check dif CLKsel for more stability
	while (!(I2C1->ISR & (1<<6))){}; 		// Wait until TC (Transfer Complete) Flag is set
	I2C1->CR2 |= (1<<14); 							// stop bit
	/* ___ Gyroscope config ___ */
	I2C1->CR2 &= 0x00000000; 						//reset CR2 register
	I2C1->CR2 |= (0x68<<1) | (2<<16) | (0<<10) | (1<<13);
	WaitWrite(); 
	I2C1->TXDR = 0x1B; 
	WaitWrite();
	I2C1->TXDR = 0x00; 									//FS_SEL (full scale selection) = +/- 250 deg/sec
	while (!(I2C1->ISR & (1<<6))){}; 
	I2C1->CR2 |= (1<<14); 							// stop bit		
	/* ___ Accelerometer config ___ */
	I2C1->CR2 &= 0x00000000; 						//reset CR2 register
	I2C1->CR2 |= (0x68<<1) | (2<<16) | (0<<10) | (1<<13);
	WaitWrite(); 
	I2C1->TXDR = 0x1C; 
	WaitWrite();
	I2C1->TXDR = 0x00; 									//AFS_SEL = +/- 2g
	while (!(I2C1->ISR & (1<<6))){}; 		
	I2C1->CR2 |= (1<<14); 							// stop bit
	
		
	motor_init();
  while (1)
  {
		
	// read and save value of X and y axis data register every 100 ms.
	// need to assemble the 16bit measured value from 2 data registers for each axis
	HAL_Delay(100); //100ms	
  }// while loop
}//end main

/* Called by SysTick Interrupt
 * Performs button debouncing, changes target speed on button rising edge
 */

// sensor interrupt to calculate motor speed, also manages PI controller
void TIM6_DAC_IRQHandler(void) {
		getAccel();
		getGyro();
		complimentary_filter();	
    // Call the PI update function
    PI_update();
    TIM6->SR &= ~TIM_SR_UIF;        // Acknowledge the interrupt
}

/** System Clock Configuration*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
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



/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
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

