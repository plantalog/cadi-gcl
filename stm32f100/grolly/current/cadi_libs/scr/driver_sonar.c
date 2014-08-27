#include "stm32f10x.h"
#include "driver_sonar.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"
#include "misc.h"
#define PLUG_DISABLE	GPIOC->BSRR
#define PLUG_ENABLE		GPIOC->BRR

void sonar_init(void){
#ifdef	SONAR_PINS_PA6_PC4
	// sonar trigger on PC4

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16 | RCC_APB2Periph_TIM17 | RCC_APB2Periph_GPIOA |RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* SONAR1_TIM, 17 remapping pins */
	GPIO_PinRemapConfig(GPIO_Remap_TIM17, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_TIM16, ENABLE);



	  /*

	  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_SONAR1_TIM_IRQn;	// & TIM1_TRG_COM_SONAR1_TIM_IRQn
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

	  TIM_ICInitTypeDef TIM_ICInitStructure;
	   TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	   TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	   TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	   TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	   TIM_ICInitStructure.TIM_ICFilter = 0x0;

	   TIM_ICInit(SONAR1_TIM, &TIM_ICInitStructure); */

	   /* TIM enable counter */
//	   TIM_Cmd(SONAR1_TIM, ENABLE);

	   /* Enable the CC2 Interrupt Request */
//	   TIM_ITConfig(SONAR1_TIM, TIM_IT_CC1, ENABLE);

	  NVIC_InitTypeDef NVIC_InitStructure;

	   /* Enable the TIM15 global Interrupt */

	  NVIC_InitStructure.NVIC_IRQChannel = TIM1_TRG_COM_TIM17_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);


	  // SONAR1_TIM setup
	  RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;    //enable SONAR1_TIM clock
	  SONAR1_TIM->PSC	= 80-1;                //set divider to get ms
//	  SONAR1_TIM->ARR     = 500;                   //2hz = 500ms
	  SONAR1_TIM->DIER    = TIM_DIER_CC1IE;        	 //enable timer interrupt
	  //enable reload and interrupt
	  SONAR1_TIM->CCMR1	&= ~ TIM_CCMR1_CC1S_1; // Select the active input: TIMx_CCR1 must be linked to the TI1 input, so write the CC1S  bits to 01 in the TIMx_CCMR1 register.
	  SONAR1_TIM->CCMR1	|=	TIM_CCMR1_CC1S_0;
	  SONAR1_TIM->CCER &= ~ TIM_CCER_CC1P;	// Select the edge of the active transition on the TI1 channel by writing CC1P bit to 0 in the TIMx_CCER register (rising edge in this case).
	  SONAR1_TIM->CCMR1 &= ~ TIM_CCMR1_IC1PSC_0; // Program the input prescaler. In our example, we wish the capture to be performed at each valid transition, so the prescaler is disabled (write IC1PS bits to ‘00’ in the TIMx_CCMR1 register).
	  SONAR1_TIM->CCMR1 &= ~ TIM_CCMR1_IC1PSC_1;
	  SONAR1_TIM->CCER  |= TIM_CCER_CC1E; // Enable capture from the counter into the capture register by setting the CC1E bit in the TIMx_CCER register.
	  SONAR1_TIM->CR1  = TIM_CR1_UDIS| TIM_CR1_CEN;

#endif

#ifdef SONAR_PINS_PB8_9
	  // sonar trigger on PB7
 //     GPIOC->CRL      &= ~GPIO_CRL_CNF4;		// ... to PC3
 //     GPIOC->CRL   |= GPIO_CRL_MODE4_0;
      GPIOB->CRL      &= ~GPIO_CRL_CNF7;		// ... to PC3
      GPIOB->CRL   |= GPIO_CRL_MODE7_0;


		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16 | RCC_APB2Periph_TIM17 | RCC_APB2Periph_GPIOB, ENABLE);
		GPIO_InitTypeDef GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

		GPIO_Init(SONAR_ECHO_PORT, &GPIO_InitStructure);


		  NVIC_InitTypeDef NVIC_InitStructure;

		   /* Enable the TIM17 global Interrupt */

		  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM16_IRQn;
		  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		  NVIC_Init(&NVIC_InitStructure);

		  NVIC_InitStructure.NVIC_IRQChannel = TIM1_TRG_COM_TIM17_IRQn;
		  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		  NVIC_Init(&NVIC_InitStructure);

		  // SONAR1_TIM setup
		  RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;    //enable SONAR1_TIM clock
		  SONAR1_TIM->PSC	= 800-1;                //set divider
	//	  SONAR1_TIM->ARR     = 500;                   //2hz = 500ms
		  //enable reload and interrupt
		  SONAR1_TIM->CCMR1	&= ~ TIM_CCMR1_CC1S_1; // Select the active input: TIMx_CCR1 must be linked to the TI1 input, so write the CC1S  bits to 01 in the TIMx_CCMR1 register.
		  SONAR1_TIM->CCMR1	|=	TIM_CCMR1_CC1S_0;
		  SONAR1_TIM->CCER |= TIM_CCER_CC1P;	// Select the edge of the active transition on the TI1 channel by writing CC1P bit to 0 in the TIMx_CCER register (rising edge in this case).
		  SONAR1_TIM->CCER |= TIM_CCER_CC1NP;
		  SONAR1_TIM->CCMR1 &= ~ TIM_CCMR1_IC1PSC_0; // Program the input prescaler. In our example, we wish the capture to be performed at each valid transition, so the prescaler is disabled (write IC1PS bits to ‘00’ in the TIMx_CCMR1 register).
		  SONAR1_TIM->CCMR1 &= ~ TIM_CCMR1_IC1PSC_1;
		  SONAR1_TIM->CCER  |= TIM_CCER_CC1E; // Enable capture from the counter into the capture register by setting the CC1E bit in the TIMx_CCER register.
		  SONAR1_TIM->DIER    = TIM_DIER_CC1IE;        	 //enable timer interrupt
		  SONAR1_TIM->CR1  =  TIM_CR1_CEN;

		  // SONAR2_TIM setup
		  RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;    //enable SONAR2_TIM clock
		  SONAR2_TIM->PSC	= 800-1;                //set divider
	//	  SONAR1_TIM->ARR     = 500;                   //2hz = 500ms
		  //enable reload and interrupt
		  SONAR2_TIM->CCMR1	&= ~ TIM_CCMR1_CC1S_1; // Select the active input: TIMx_CCR1 must be linked to the TI1 input, so write the CC1S  bits to 01 in the TIMx_CCMR1 register.
		  SONAR2_TIM->CCMR1	|=	TIM_CCMR1_CC1S_0;
		  SONAR2_TIM->CCER |= TIM_CCER_CC1P;	// Select the edge of the active transition on the TI1 channel by writing CC1P bit to 0 in the TIMx_CCER register (rising edge in this case).
		  SONAR2_TIM->CCER |= TIM_CCER_CC1NP;
		  SONAR2_TIM->CCMR1 &= ~ TIM_CCMR1_IC1PSC_0; // Program the input prescaler. In our example, we wish the capture to be performed at each valid transition, so the prescaler is disabled (write IC1PS bits to ‘00’ in the TIMx_CCMR1 register).
		  SONAR2_TIM->CCMR1 &= ~ TIM_CCMR1_IC1PSC_1;
		  SONAR2_TIM->CCER  |= TIM_CCER_CC1E; // Enable capture from the counter into the capture register by setting the CC1E bit in the TIMx_CCER register.
		  SONAR2_TIM->DIER    = TIM_DIER_CC1IE;        	 //enable timer interrupt
		  SONAR2_TIM->CR1  =  TIM_CR1_CEN;

#endif
}


void sonar_ping(void){
	uint8_t i=2;
	// PB7 logic level "0"
	GPIOB->BSRR = (1<<7);
	vTaskDelay(2);
	// PB7 logic level "1"
	GPIOB->BRR = (1<<7);
	SONAR1_TIM->CNT=0;
	vTaskDelay(2);
	// PB7 logic level "0"
	GPIOB->BSRR = (1<<7);
}
