// DRIVER: Sonar
#define USE_SONAR
#ifdef USE_SONAR
//#define SONAR_PINS_PA6_PC4
#define SONAR_PINS_PB8_9
#ifdef SONAR_PINS_PA6_PC4
#define SONAR_AMOUNT	2
#define	SONAR1_TIM				TIM17
#define SONAR_ECHO_PORT			GPIOA
#endif
#ifdef SONAR_PINS_PB8_9
#define SONAR_AMOUNT	2
#define	SONAR1_TIM				TIM17
#define	SONAR2_TIM				TIM16
#define SONAR_ECHO_PORT			GPIOB
#endif

uint16_t	sonar_read[SONAR_AMOUNT];	// 2 sonars on pins PA6 and PA7 with trig on PC4
uint16_t	sonar_mm[SONAR_AMOUNT];
#endif


void sonar_init(void);
void sonar_ping(void);


