/*
 * This Project aims to generate AC signal with desired frequency and amplitude.
 * To achieve this goal Space Vector PWM method with open loop(V/F Control) implemented using this microcontroller.
 * Conventional Method to calculate dwell times used since microcontroller's processing capabilities enable that kind of operation.
 * Assuming this Inverter going to be used in driving AC Motors, field weakening process(V/f Control) implemented.
 * Amplitude of AC signal decreases linearly with respect to frequency.
 * GPIO PINS B0 B1 E8 E9 E11 and E13 used for gate signals to VSI circuit.
 * GPIO PIN A0 configured as input for enabling PWM output since Discovery Development KIT's onboard user button connected to this pin
 * GPIO PIN C1 configured as ADC input to read Frequency variable being set by user.
 */



#include "main.h"
#include <math.h>
#include "stm32f407xx.h"
#include "stm32f4xx.h"

/******************************************/
/***		FUNCTION PROTOTYPES			***/
/******************************************/
void set_sysclk_to_168(void);
void timer1_init(void);
void gpio_init(void);
void pwm_button_init(void);
void delay_ms(const uint16_t us);
void adc_init(void);
void TIM1_UP_TIM10_IRQHandler(void);
void EXTI0_IRQHandler(void);

/******************************************/
/***		GLOBAL PARAMETERS			***/
/******************************************/
#define PI 3.141592
const double f=10000*2;
unsigned int TS=(168000000/f);
unsigned int T1;
unsigned int T2;
unsigned int Tz;
float M=1;
float theta=0;
int sector=1;
int freq=0;
/******************************************/
/***				MAIN				***/
/******************************************/
int main(void)
{

  set_sysclk_to_168();											// Set main clock to 168MHz
  gpio_init();													// Set GPIO pins for PWM output
  timer1_init();												// Configure Timer1 for PWM generation
  pwm_button_init();											// Configuration for PWM Enable Button
  adc_init();													// Configuration for ADC to obtain PWM Data

  while(1){}
}

/******************************************/
/***			PWM Button Init				***/
/******************************************/
void pwm_button_init(void)
{
	RCC->AHB1ENR |= ((1<<3)|1);									// Enable GPIOD and GPIOA clocks
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;						// Enable EXTI registers
	SYSCFG->EXTICR[0] = 0;										// Select A0 as EXTI source

	EXTI->IMR = 1;												// Enable A0 channel interrupts
	EXTI->RTSR |= 1;											// Rising Edge Detection
	EXTI->FTSR &= ~1;											// Falling Edge Detection cleared

	NVIC_SetPriority(EXTI0_IRQn, 0);							// Set Priority to EXTI Interrupt
	NVIC_EnableIRQ(EXTI0_IRQn);									// Enable EXTI Interrupt

	GPIOD->MODER |= (1<<30);									// D6 BLUE LED SET OUTPUT
	GPIOD->MODER |= (1<<28);									// D5 GREEN LED SET OUTPUT
	GPIOD->MODER |= (1<<26);									// D4 ORANGE LED SET OUTPUT
	GPIOD->MODER |= (1<<24);									// D3 YELLOW LED SET OUTPUT (Color order might be different)
}

/******************************************/
/***			GPIO INIT				***/
/******************************************/
void gpio_init(void)
{
	RCC->AHB1ENR |= (1<<4 | 1<<1);								// Enable GPIOB and GPIOE Clock

	GPIOE->PUPDR  = 0;											// Pin configuration
	GPIOB->PUPDR  = 0;											// Pin configuration
	GPIOE->OTYPER = 0;											// Pin configuration
	GPIOB->OTYPER = 0;											// Pin configuration

	GPIOE->MODER  = ((1<<19)
					|(1<<17)
					|(1<<23)
					|(1<<27));									// GPIOE Pins 8 9 11 13 configured as Alternate Function

	GPIOE->OSPEEDR= ((1<<19)
					|(1<<17)
					|(1<<23)
					|(1<<27));									// Output Speed Set to MAX(6ns rise and fall) for all pins

	GPIOE->AFR[1] = (GPIO_AFRH_AFRH1_0
					|GPIO_AFRH_AFRH0_0
					|GPIO_AFRH_AFRH3_0
					|GPIO_AFRH_AFRH5_0);						// Configuring High Side pins 0001 (TIM1 Peripheral) in alternate function multiplexer

	GPIOB->MODER |= ((1<<1)
					|(1<<3));									// GPIOB Pins 0 1configured as Alternate Function

	GPIOB->OSPEEDR= ((1<<1)
					|(1<<3));									// Output Speed Set to MAX(6ns rise and fall)

	GPIOB->AFR[0]|= (GPIO_AFRL_AFRL0_0
					|GPIO_AFRL_AFRL1_0);						// Configuring Low Side pins for 0001 (TIM1 Peripheral) in alternate function multiplexer
}

/******************************************/
/***			TIMER1 INIT				***/
/******************************************/
void timer1_init(void)
{
	RCC->APB2ENR |= 1;											// Enable TIM1 Clock

	// Output Configurations
	TIM1->CR1   |= (TIM_CR1_CMS_Msk);							// TIMER1 counts in Center Aligned Mod 1
	TIM1->CCMR1 |= 6<<TIM_CCMR1_OC1M_Pos;						// TIMER1 CCMR1 Register configured as PWM TYPE1 output
	TIM1->CCMR1 |= 6<<TIM_CCMR1_OC2M_Pos;						// TIMER1 CCMR2 Register configured as PWM TYPE1 output
	TIM1->CCMR2 |= 6<<TIM_CCMR2_OC3M_Pos;						// TIMER1 CCMR1 Register configured as PWM TYPE1 output

	TIM1->CCMR1 &= ~(TIM_CCMR1_CC1S);							// TIMER1 CC1S bits cleared for output configuration
	TIM1->CCMR1 &= ~(TIM_CCMR1_CC2S);							// TIMER1 CC2S bits cleared for output configuration
	TIM1->CCMR2 &= ~(TIM_CCMR2_CC3S);							// TIMER1 CC3S bits cleared for output configuration

	// Counter Setup
	TIM1->PSC  = 0; 											// 168MHz 5.95nS
	TIM1->CNT  = 0;												// Reset Current Counter
	TIM1->ARR  = TS;											// Center Mode pit to peak is 50uS "8400"
	TIM1->CCR1 = 0;												// %0 Duty Cycle
	TIM1->CCR2 = 0;												// %0 Duty Cycle
	TIM1->CCR3 = 0;												// %0 Duty Cycle


	// Update Configuration
	//TIM1->BDTR |= (0b00001100);								// 70nS deadtime
	//TIM1->BDTR |= (0b00010001);								// 100nS deadtime
	//TIM1->BDTR |= (0b00011010);								// 155nS deadtime
	//TIM1->BDTR |= (0b00100010);								// 202nS deadtime
	//TIM1->BDTR |= (0b01000100);								// 404nS deadtime
	TIM1->BDTR |= (0b10010100);									// 1us deadtime
	//TIM1->BDTR |= (0b11001010);								// 2uS deadtime

	TIM1->CR1 &= ~TIM_CR1_UDIS;									// Update Disabled bit disabled
	TIM1->DIER|= TIM_DIER_UIE;									// Update Interrupt Enabled

	// Enable Outputs
	TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);				// CC CH1 and CH1n enabled for output
	TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE);				// CC CH2 and CH2n enabled for output
	TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE);				// CC CH3 and CH3n enabled for output

	NVIC_SetPriority(TIM1_UP_TIM10_IRQn,2);						// Set NVIC Priority
	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);							// Enable Timer1 Interrupt

	TIM1->CR1  |= (TIM_CR1_CEN_Msk);							// Counter enabled
}

/******************************************/
/***			DELAY mS				***/
/******************************************/
void delay_ms(const uint16_t ms)
{
  uint32_t i = ms * 27778;
  while (i-- > 0) {asm("nop");  }
}

/******************************************/
/***			ADC Init				***/
/******************************************/
void adc_init(void)
{
	RCC->AHB1ENR |= (1<<2); 									// Enable GPIOC Clock
	GPIOC->MODER |= (3<<2); 									// Set PC1 to analog mode
	RCC->APB2ENR |= (1 << 8); 									// Enable ADC1 clock
	ADC->CCR = (1<<16);											// Divide ADC clock by 4 (84/4=21Mh)
	ADC1->CR1 &= ~(1 << 8); 									// SCAN mode disabled (Bit8)
	ADC1->CR1 &=  ~(3 << 24); 									// 12bit resolution (Bit24,25 0b00)
	ADC1->SQR1 |= (1 << 20); 									// Set number of conversions projected (L[3:0] 0b0001) -> 1 conversion
	ADC1->SQR3 &= ~(0x3FFFFFFF); 								// Clear whole 1st 30bits in register
	ADC1->SQR3 |= (0b01011); 									// 1st conversion in regular sequence: SQ1: PC1 as ADC1_IN11
	ADC1->CR2 &= ~(1 << 1); 									// Single conversion
	ADC1->CR2 &= ~(1 << 11); 									// Right alignment of data bits bit12....bit0
	ADC1->SMPR2 |= (2 << 0); 									// Sampling rate 40 cycles. 21MHz bus clock for ADC. 1/84MHz = 12nS. 40*12ns=0.47us
	ADC1->CR2 |= (1 << 0); 										// Switch on ADC1
}

/******************************************/
/***			CLOCK INIT				***/
/******************************************/
void set_sysclk_to_168(void)
{
	/* FPU settings, can be enabled from project makefile */
	#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
	#endif

	/* Reset the RCC clock configuration to the default reset state */
	/* Set HSION bit */
	RCC->CR |= (1U << 0);

	RCC->CFGR = 0x00000000;										/* Reset CFGR register */
	RCC->CR &= ~((1U << 16) | (1U << 19) | (1U << 24));			/* Reset HSEON (16), CSSON (19) and PLLON (24) bits */
	RCC->PLLCFGR = 0x24003010UL;								/* Reset PLLCFGR register to reset value */
	RCC->CR &= ~(1U << 18);										/* Reset HSEBYP bit */
	RCC->CIR = 0x00000000UL;									/* Disable all clock interrupts */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	RCC->CR |= (1U << 16);										/* Enable HSE (CR: bit 16) */
	while(!(RCC->CR & (1 << 17)));								/* Wait till HSE is ready (CR: bit 17) */
	RCC->APB1ENR |= (1 << 28);									/* Enable power interface clock (APB1ENR:bit 28) */

	/* set voltage scale to 1 for max frequency (PWR_CR:bit 14)
	 * (0b0) scale 2 for fCLK <= 144 Mhz
	 * (0b1) scale 1 for 144 Mhz < fCLK <= 168 Mhz
	 */
	PWR->CR |= (1 << 14);
	RCC->CFGR |= (0 << 4);										/* set AHB prescaler to /1 (CFGR:bits 7:4) */
	RCC->CFGR |= (5 << 10);										/* set APB low speed prescaler to /4 (APB1) (CFGR:bits 12:10) */
	RCC->CFGR |= (4 << 13);										/* set APB high speed prescaler to /2 (APB2) (CFGR:bits 15:13) */

	/* Set M, N, P and Q PLL dividers
	 * PLLCFGR: bits 5:0 (M), 14:6 (N), 17:16 (P), 27:24 (Q)
	 * Set PLL source to HSE, PLLCFGR: bit 22, 1:HSE, 0:HSI
	 */
	RCC->PLLCFGR = 8 | (336 << 6) | (((2 >> 1) -1) << 16) |
	               (7 << 24) | (1 << 22);
	RCC->CR |= (1 << 24);										/* Enable the main PLL (CR: bit 24) */
	while(!(RCC->CR & (1 << 25)));								/* Wait till the main PLL is ready (CR: bit 25) */

	/* Configure Flash
	 * prefetch enable (ACR:bit 8)
	 * instruction cache enable (ACR:bit 9)
	 * data cache enable (ACR:bit 10)
	 * set latency to 5 wait states (ARC:bits 2:0)
	 *   see Table 10 on page 80 in RM0090
	 */
	FLASH->ACR = (1 << 8) | (1 << 9) | (1 << 10 ) | (5 << 0);

	/* Select the main PLL as system clock source, (CFGR:bits 1:0)
	 * 0b00 - HSI
	 * 0b01 - HSE
	 * 0b10 - PLL
	 */
	RCC->CFGR &= ~(3U << 0);
	RCC->CFGR |= (2 << 0);
	while (!(RCC->CFGR & (2U << 2)));							/* Wait till the main PLL is used as system clock source (CFGR:bits 3:2) */
	SystemCoreClock = 168000000;								// update SystemCoreClock variable
}

/******************************************/
/***			TIMER1 INTERRUPT		***/
/******************************************/
void TIM1_UP_TIM10_IRQHandler(void)
{
	ADC1->CR2 |= (1<<30);  										// start the ADC conversion
	sector=(theta)/60+1;										// gets Space Vector Sector data from current angle
	TIM1->SR &=~(TIM_SR_UIF);									// Lower Interrupt flag for next interrupts
	T1=TS*M*sin((PI/3)*(sector) - (theta*PI/180));				// Calculates T1 dwell time
	T2=TS*M*sin((-PI/3)*(sector-1) +(theta*PI/180));			// Calculates T2 dwell time
	Tz=TS-T1-T2;												// Calculates Tzero dwell time
	switch (sector)
	{
	case 1:														// Sector 1 outputs for all phases
	TIM1->CCR1=(Tz/2 + T1 +T2);
	TIM1->CCR2=(Tz/2 + T2);
	TIM1->CCR3=(Tz/2);
	break;

	case 2:														// Sector 2 outputs for all phases
	TIM1->CCR1=(Tz/2 +T1);
	TIM1->CCR2=(Tz/2 + T1 +T2);
	TIM1->CCR3=(Tz/2);
	break;

	case 3:														// Sector 3 outputs for all phases
	TIM1->CCR1=(Tz/2);
	TIM1->CCR2=(Tz/2 + T1 +T2);
	TIM1->CCR3=(Tz/2 + T2);
	break;

	case 4:														// Sector 4 outputs for all phases
	TIM1->CCR1=(Tz/2);
	TIM1->CCR2=(Tz/2 +T1);
	TIM1->CCR3=(Tz/2 + T1 +T2);
	break;

	case 5:														// Sector 5 outputs for all phases
	TIM1->CCR1=(Tz/2 + T2);
	TIM1->CCR2=(Tz/2);
	TIM1->CCR3=(Tz/2 + T1 +T2);
	break;

	case 6:														// Sector 6 outputs for all phases
	TIM1->CCR1=(Tz/2 + T1 +T2);
	TIM1->CCR2=(Tz/2);
	TIM1->CCR3=(Tz/2 +T1);
	break;

	default:
	break;
	}

	freq = (ADC1->DR)/50;  										// Write ADC Value to frequency variable and scale it to max 80Hz
	M=freq/50.0;												// V/f Control
	if(M>1) M=1;												// Field Weakening to prevent working on 6 Step PWM mode
	theta+=(freq*360/f);										// Calculating the next angle for given frequency rate (Open Loop Control)
	if(theta>=360) theta=0;										// Theta becomes zero every period
}
/******************************************/
/***			EXTI0 INTERRUPT			***/
/******************************************/
void EXTI0_IRQHandler(void)
{
	GPIOD->ODR ^= (GPIO_ODR_OD15);								// Toggle Leds
	GPIOD->ODR ^= (GPIO_ODR_OD14);								// Toggle Leds
	GPIOD->ODR ^= (GPIO_ODR_OD13);								// Toggle Leds
	GPIOD->ODR ^= (GPIO_ODR_OD12);								// Toggle Leds


	TIM1->BDTR ^= (TIM_BDTR_MOE_Msk);							// Global output toggled for TIM1
	delay_ms(300);												// Delay to Prevent Button Bounce
	EXTI->PR |= (1);											// Lower Interrupt flag
}
