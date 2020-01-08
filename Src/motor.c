
#include "motor.h"
#include "stm32f0xx_hal.h"

/* -------------------------------------------------------------------------------------------------------------
 *  Global Variable and Type Declarations
 *  -------------------------------------------------------------------------------------------------------------
 */
volatile int16_t error_integral;    // Integrated error signal


/* -------------------------------------------------------------------------------------------------------------
 *  Global Variables for STMStudio Debug Viewing (no real purpose to be global otherwise)
 * -------------------------------------------------------------------------------------------------------------
 */
volatile uint8_t duty_cycle;    													// Output PWM duty cycle
//volatile int16_t target_sensor = 0;    									// Desired angle target
volatile double target_sensor = 0;    										// Desired angle target
volatile double fused_angleX, fused_angleY, fused_angleZ; // measured angle [deg]
volatile int8_t adc_value;      													// ADC measured motor current
//volatile int16_t error;         												// Speed error signal
volatile double error;        														// Speed error signal
//volatile uint8_t Kp;            												// Proportional gain
volatile float Kp;           															// Proportional gain
volatile uint8_t Ki;            													// Integral gain

/* -------------------------------------------------------------------------------------------------------------
 *  Motor Control and Initialization Functions
 * -------------------------------------------------------------------------------------------------------------
 */
 
// Sets up the entire motor drive system
void motor_init(void) {
    Kp = 0.05;     // Set default proportional gain
    Ki = 1;     	 // Set default integral gain
    
    pwm_init();
    sensors_init();
}

// Sets up the PWM and direction signals to drive the H-Bridge
void pwm_init(void) {
    
    // Set up a pin for H-bridge PWM output (TIMER 16 CH1)
           //PB8 - This one works
            RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
            GPIOB->MODER |= (1<<17); //AF mode
            GPIOB->AFR[1] |= (1<<1); //AF 2
  
		// Set up a few GPIO output pins for direction control
            // Set PA4 and PA5 to output "(direction for motor)"
            //GPIOA->MODER |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1; 
            RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
            GPIOC->MODER |= (1<<12) | (1<<14); //pc 6 and pc 7
            

		
    /* 			 These pins are processor outputs, inputs to the H-bridge
     *       they can be ordinary 3.3v pins.
     */

    // Set up PWM timer
    RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
    TIM16->CR1 = 0;                         // Clear control register

    // Set output-compare CH1 to PWM1 mode and enable CCR1 preload buffer
    TIM16->CCMR1 = (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE);
    TIM16->CCER = TIM_CCER_CC1E;            // Enable capture-compare channel 1
    TIM16->PSC = 7;                         // Run timer on 1Mhz
    TIM16->ARR = 50;                        // PWM at 20kHz
    TIM16->CCR1 = 0;                        // Start PWM at 0% duty cycle
    TIM16->BDTR |= TIM_BDTR_MOE;  					// Set master output enable (only for timers that have complementary outputs)
    TIM16->CR1 |= TIM_CR1_CEN;              // Enable timer
}

// Set the duty cycle of the PWM, accepts (0-100)
void pwm_setDutyCycle(uint8_t duty) {
    if(duty <= 100) {
        TIM16->CCR1 = ((uint32_t)duty*TIM16->ARR)/100;  // Use linear transform to produce CCR1 value
    }
}

// Sets up interface to read sensors angle
void sensors_init(void) {   
    // Configure a second timer (TIM6 - a 16 bit timer) to fire an ISR on update event
    // Used to periodically check and update speed variable
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    
    // Select PSC and ARR values that give an appropriate interrupt rate
	  // want 0.0375 seconds timer time we need
		// (8mhz/8prescale) = 1mhz. soo ARR needs 37,500 which is witin  16 bit timer
	
	/* Example math: want 0.5 seconds so 0.5/(1/1e4) = 5000 */
        TIM6->PSC = 799; // n -1 so, 800 -1 = 799
        TIM6->ARR = 2500; //  
    
        TIM6->DIER |= TIM_DIER_UIE;             // Enable update event interrupt
        TIM6->CR1 |= TIM_CR1_CEN;               // Enable Timer

        NVIC_EnableIRQ(TIM6_DAC_IRQn);          // Enable interrupt in NVIC
        NVIC_SetPriority(TIM6_DAC_IRQn,2);
}



void PI_update(void) {
    
    /* Run PI control loop
     *
     * Make sure to use the indicated variable names. This allows STMStudio to monitor
     * the condition of the system!
     *
     * target_sensor -> target deg
     * fused_angleX/Y/Z -> measured deg
     * error -> error signal (difference between measured speed and target)
     * error_integral -> integrated error signal
     * Kp -> Proportional Gain
     * Ki -> Integral Gain
     * output -> raw output signal from PI controller
     * duty_cycle -> used to report the duty cycle of the system 
     * adc_value -> raw ADC counts to report current
     *
     */
    
 
		error = target_sensor - fused_angleY; // negative feedback. units in encoder counts
    		

    
   // int16_t output = (Kp * error);// + (Ki * error_integral);
     float output = (Kp * error);
   
     /// Clamp the output value between 0 and 100 
		if (error > -5 && error < 5) {
			output = 0;
		} else {
			  if (error < 0) {
			GPIOC->ODR |= GPIO_ODR_7; 
      GPIOC->ODR &= ~GPIO_ODR_6; 
			output = -output;
			if (output > 100) {
					output = 100;
			}
			else if (output < 85){
				output = 85;
			}
		} 
		if (error > 0) {
			
			GPIOC->ODR |= GPIO_ODR_6; //RED LED - forwards (positive angles) (toward motor terminal on current build)
      GPIOC->ODR &= ~GPIO_ODR_7; //BLUE LED	- backwards (negative angles)
			if (output > 100) {
					output = 100;
			} 
			else if (output < 85){
				output = 85;
			}
		}
		}
  
    pwm_setDutyCycle(output);
    duty_cycle = output;            // For debug viewing
}


