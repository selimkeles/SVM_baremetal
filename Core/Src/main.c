#include "main.h"
#include "Inverter.h"
#include <math.h>
#include "stm32f407xx.h"
#include "stm32f4xx.h"


/******************************************/
/***		CLOCK PREPROCESSORS 		***/
/******************************************/
#if defined (STM32F407xx) || defined (STM32F429xx)
// Main PLL = N * (source_clock / M) / P
// HSE = 8 Mhz
// fCLK =   N * (8Mhz / M) / P
#define PLL_M   8
#define PLL_Q   7
#define PLL_P   2
#endif
/* stm32f407 runs at 168Mhz max */
#if defined (STM32F407xx)
#define PLL_N   336
#endif

/******************************************/
/***	LOOKUP TABLE GLOBAL PARAMETERS 	***/
/******************************************/
#define PI 3.141592
#define RESO 360						// Sinusoidal resolution(Degree)
#define DEC (RESO/360)					// Decrement rate if Reso is higher than 360 degree (Finds the multiplier of 360 degrees)
#define PHASESHIFT (RESO/12)			// Phase Shift for triangular wave
#define TRISIZE (RESO+RESO/12)			// Triangular wave array size is bit higher than sinus arrays since we have to shift it

unsigned int MAX_CNT=8400;				// Amplitude is equal to the Timer Period
double sin_arr[RESO];					// Lookup table arrays
double tri_arr[TRISIZE];
uint32_t u[RESO];
uint32_t v[RESO];
uint32_t w[RESO];

volatile int count=0;					//
volatile int s=0;

/******************************************/
/***		FUNCTION PROTOTYPES			***/
/******************************************/

void create_lookuptable(double M);
void pwm_init(void);
void set_sysclk_to_168(void);
void dma_init(void);


/******************************************/
/***				MAIN				***/
/******************************************/
int main(void)
{

  set_sysclk_to_168();										// Set main clock to 168MHz
  double Mod_Index=1;										// Modulation Index for lookup table

  create_lookuptable(Mod_Index);							// Creates SVM lookup table

  pwm_init();												// Initializes Timer1 for PWM Config
  dma_init();												// Initializes Direct Memory Access

  TIM1->CR1  |= (TIM_CR1_CEN_Msk);							// Counter enabled
  TIM1->BDTR |= (TIM_BDTR_MOE_Msk);							// Global output enable for TIM1

  while(1){}
}

/******************************************/
/***		LOOKUP TABLE FUNCTION		***/
/******************************************/
void create_lookuptable(double M)
{
	int i;																			// Array Indexer
	for(i=0;i<RESO;i++)
		sin_arr[i]=(MAX_CNT*0.5*(sin((2*PI*i/DEC)/360)));							// Creates Sinus

	for(i=0;i<TRISIZE;i++)
		tri_arr[i]=0.288*((MAX_CNT*(60-fabs((i/DEC)%(120)-60))/60)-MAX_CNT/2);		// Creates Triangular Wave

	for(i=0;i<RESO;i++)
		u[i]=M*1.154*sin_arr[i]+tri_arr[i+PHASESHIFT]+MAX_CNT/2;					// Creates SVM waveform (U-Phase)

	for(i=0;i<RESO;i++)
		{if(i<241) v[i+(RESO/3)]=u[i]; else v[i-(2*RESO/3)+1]=u[i];}				// Creates V phase by phase shifting
	for(i=0;i<RESO;i++)
		{if(i<121) w[i+(2*RESO/3)]=u[i]; else w[i-(RESO/3)+1]=u[i];}				// Creates W phase by phase shifting
}

/******************************************/
/***			TIMER1 INIT				***/
/******************************************/
void pwm_init(void)
{
	RCC->AHB1ENR |= (1<<4 | 1<<1);								// Enable GPIOB and GPIOE Clock

	GPIOE->PUPDR  = 0;											// Pin configuration
	GPIOE->OTYPER = 0;											// Pin configuration
	GPIOE->MODER  = (1<<19);									// GPIOE Pin9 configured as Alternate Function
	GPIOE->OSPEEDR= (1<<19);									// Output Speed Set to MAX(6ns rise and fall)
	GPIOE->AFR[1] = (GPIO_AFRH_AFRH1_0);						// Configuring High Side 2nd pin for 0001 (TIM1 Peripheral) in alternate function multiplexer

	GPIOE->MODER |= (1<<17);									// GPIOE Pin8 configured as Alternate Function
	GPIOE->OSPEEDR= (1<<17);									// Output Speed Set to MAX(6ns rise and fall)
	GPIOE->AFR[1]|= (GPIO_AFRH_AFRH0_0);						// Configuring High Side 1st pin for 0001 (TIM1 Peripheral) in alternate function multiplexer

	GPIOE->MODER |= (1<<23);									// GPIOE Pin11 configured as Alternate Function
	GPIOE->OSPEEDR= (1<<23);									// Output Speed Set to MAX(6ns rise and fall)
	GPIOE->AFR[1]|= (GPIO_AFRH_AFRH3_0);						// Configuring High Side 4th pin for 0001 (TIM1 Peripheral) in alternate function multiplexer

	GPIOB->PUPDR  = 0;											// Pin configuration
	GPIOB->OTYPER = 0;											// Pin configuration
	GPIOB->MODER |= (1<<1);										// GPIOB Pin0 configured as Alternate Function
	GPIOB->OSPEEDR= (1<<1);										// Output Speed Set to MAX(6ns rise and fall)
	GPIOB->AFR[0]|= (GPIO_AFRL_AFRL0_0);						// Configuring Low Side 1st pin for 0001 (TIM1 Peripheral) in alternate function multiplexer

	GPIOE->MODER |= (1<<27);									// GPIOE Pin13 configured as Alternate Function
	GPIOE->OSPEEDR= (1<<27);									// Output Speed Set to MAX(6ns rise and fall)
	GPIOE->AFR[1]|= (GPIO_AFRH_AFRH5_0);						// Configuring High Side 6th pin for 0001 (TIM1 Peripheral) in alternate function multiplexer

	GPIOB->MODER |= (1<<3);										// GPIOB Pin1 configured as Alternate Function
	GPIOB->OSPEEDR= (1<<3);										// Output Speed Set to MAX(6ns rise and fall)
	GPIOB->AFR[0]|= (GPIO_AFRL_AFRL1_0);						// Configuring Low Side 1st pin for 0001 (TIM1 Peripheral) in alternate function multiplexer

	/*****************************************************/

	RCC->APB2ENR |= 1;											// Enable TIM1 Clock

	// Output Configurations
	TIM1->CR1   |= (TIM_CR1_CMS_Msk);							// TIMER1 counts in Center Aligned Mod 3
	TIM1->CCMR1 |= 6<<TIM_CCMR1_OC1M_Pos;						// TIMER1 CCMR1 Register configured as PWM TYPE1 output
	TIM1->CCMR1 |= 6<<TIM_CCMR1_OC2M_Pos;						// TIMER1 CCMR2 Register configured as PWM TYPE1 output
	TIM1->CCMR2 |= 6<<TIM_CCMR2_OC3M_Pos;						// TIMER1 CCMR1 Register configured as PWM TYPE1 output
	TIM1->CCMR1 &= ~(TIM_CCMR1_CC1S);							// TIMER1 CC1S bits cleared for output configuration
	TIM1->CCMR1 &= ~(TIM_CCMR1_CC2S);							// TIMER1 CC2S bits cleared for output configuration
	TIM1->CCMR2 &= ~(TIM_CCMR2_CC3S);							// TIMER1 CC3S bits cleared for output configuration

	// Counter Setup
	TIM1->PSC  = 0; 											// 168MHz 5.95nS
	TIM1->CNT  = 0;												// Reset Current Counter
	TIM1->ARR  = 8400;											// Center Mode pit to peak is 50uS
	TIM1->CCR1 = 0;												// %50 Duty Cycle
	TIM1->CCR2 = 0;												// %50 Duty Cycle
	TIM1->CCR3 = 0;												// %50 Duty Cycle


	// Update Configuration
	//TIM1->BDTR |= (0b00001100);								// 70nS deadtime
	//TIM1->BDTR |= (0b00010001);								// 100nS deadtime
	//TIM1->BDTR |= (0b00011010);								// 155nS deadtime
	//TIM1->BDTR |= (0b00100010);								// 202nS deadtime
	//TIM1->BDTR |= (0b01000100);								// 404nS deadtime
	TIM1->BDTR |= (0b10010100);									// 1us deadtime
	//TIM1->BDTR |= (0b11001010);								// 2uS deadtime

	//TIM1->RCR = 0;											// After 50usx200=10ms ;
	TIM1->CR2 |= TIM_CR2_CCDS;
	TIM1->DIER|=(TIM_DIER_UDE|TIM_DIER_CC1DE|TIM_DIER_CC2DE|TIM_DIER_CC3DE);	// Enables all DMA Access
	TIM1->DIER|=TIM_DIER_TDE;													// Enable Trigger DMA


	// Enable Outputs
	TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);				// CC CH1 and CH1n enabled for output
	TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE);				// CC CH2 and CH2n enabled for output
	TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE);				// CC CH3 and CH3n enabled for output
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

	/* Reset CFGR register */
	RCC->CFGR = 0x00000000;

	/* Reset HSEON (16), CSSON (19) and PLLON (24) bits */
	RCC->CR &= ~((1U << 16) | (1U << 19) | (1U << 24));

	/* Reset PLLCFGR register to reset value */
	RCC->PLLCFGR = 0x24003010UL;

	/* Reset HSEBYP bit */
	RCC->CR &= ~(1U << 18);

	/* Disable all clock interrupts */
	RCC->CIR = 0x00000000UL;

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	/* Enable HSE (CR: bit 16) */
	RCC->CR |= (1U << 16);
	/* Wait till HSE is ready (CR: bit 17) */
	while(!(RCC->CR & (1 << 17)));

	/* Enable power interface clock (APB1ENR:bit 28) */
	RCC->APB1ENR |= (1 << 28);

	/* set voltage scale to 1 for max frequency (PWR_CR:bit 14)
	 * (0b0) scale 2 for fCLK <= 144 Mhz
	 * (0b1) scale 1 for 144 Mhz < fCLK <= 168 Mhz
	 */
	PWR->CR |= (1 << 14);

	/* set AHB prescaler to /1 (CFGR:bits 7:4) */
	RCC->CFGR |= (0 << 4);
	/* set APB low speed prescaler to /4 (APB1) (CFGR:bits 12:10) */
	RCC->CFGR |= (5 << 10);
	/* set APB high speed prescaler to /2 (APB2) (CFGR:bits 15:13) */
	RCC->CFGR |= (4 << 13);

	/* Set M, N, P and Q PLL dividers
	 * PLLCFGR: bits 5:0 (M), 14:6 (N), 17:16 (P), 27:24 (Q)
	 * Set PLL source to HSE, PLLCFGR: bit 22, 1:HSE, 0:HSI
	 */
	RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) |
	               (PLL_Q << 24) | (1 << 22);
	/* Enable the main PLL (CR: bit 24) */
	RCC->CR |= (1 << 24);
	/* Wait till the main PLL is ready (CR: bit 25) */
	while(!(RCC->CR & (1 << 25)));
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
	/* Wait till the main PLL is used as system clock source (CFGR:bits 3:2) */
	while (!(RCC->CFGR & (2U << 2)));

	// update SystemCoreClock variable
	SystemCoreClock = 168000000;
}
/******************************************/
/***			DMA INIT				***/
/******************************************/
void dma_init(void)
{
	  RCC->AHB1ENR |= (1 << 22);									// Enable DMA2 clock

	  DMA2_Stream1->CR &= DMA_SxCR_EN;
	  while(DMA2_Stream1->CR & DMA_SxCR_EN){;}
	  DMA2_Stream1->CR |=  ( (6<<25)|(1<<14)|(1 << 12)|(1<<8)|(1<<10)|(1<<6) );

	  DMA2_Stream2->CR &= DMA_SxCR_EN;
	  while(DMA2_Stream2->CR & DMA_SxCR_EN){;}
	  DMA2_Stream2->CR |=  ( (6<<25)|(1<<14)|(1 << 12)|(1<<8)|(1<<10)|(1<<6) );

	  DMA2_Stream6->CR &= DMA_SxCR_EN;
	  while(DMA2_Stream6->CR & DMA_SxCR_EN){;}
	  DMA2_Stream6->CR |=  ( (6<<25)|(1<<14)|(1 << 12)|(1<<8)|(1<<10)|(1<<6) );

	  DMA2_Stream1->NDTR=(uint16_t)360;
	  DMA2_Stream2->NDTR=(uint16_t)360;
	  DMA2_Stream6->NDTR=(uint16_t)360;

	  DMA2_Stream1->PAR=(uint32_t)(&TIM1->CCR1);
	  DMA2_Stream2->PAR=(uint32_t)(&TIM1->CCR2);
	  DMA2_Stream6->PAR=(uint32_t)(&TIM1->CCR3);

	  DMA2_Stream1->M0AR=(uint32_t)(&u);
	  DMA2_Stream2->M0AR=(uint32_t)(&v);
	  DMA2_Stream6->M0AR=(uint32_t)(&w);

	  DMA2_Stream1->CR|=DMA_SxCR_EN;
	  DMA2_Stream2->CR|=DMA_SxCR_EN;
	  DMA2_Stream6->CR|=DMA_SxCR_EN;
}
