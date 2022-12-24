/**
******************************************************************************
* @author  HeoDabin
******************************************************************************
*/

#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecTIM.h"
#include "ecSysTick.h"
#include "ecUART.h"
#include "ecADC.h"
#include "ecPWM.h"
#include "ecStepper.h"

#define A 0
#define B 1
#define END_CHAR 13
#define MAX_BUF 10
#define FIRE_PIN 7
#define LED_PIN 	5

uint8_t flag = '0';
uint8_t pre_flag = '1';

//ultrasonic
uint32_t ovf_cnt[2] = {0, 0};
float distance_past[2] = {0, 0};
float distance_present[2] = {0, 0};
float timeInterval[2] = {0, 0};
float time1[2] = {0, 0};
float time2[2] = {0, 0};

//usart
uint8_t recvChar = 0;
uint8_t pcData = 0;
uint8_t buffer[MAX_BUF] = {0,};
int idx = 0;
int bReceive = 0;

//belt motor
PWM_t dcPwm;

void setup(void);
	
int main(void) { 

	// Initialiization --------------------------------------------------------
	setup();
	printf("\r\nHello Nucleo\r\n");
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		distance_present[1] = (float) timeInterval[1] * 340.0 / 2.0 / 10.0 * 10; 	// front
		distance_present[0] = (float) timeInterval[0] * 340.0 / 2.0 / 10.0 * 10; 	// back

		if(flag == '0' || flag == '6'){
			if(bReceive == 1 && buffer[0] == 'G'){
				if 			(buffer[1] == 'O')	flag = '1';		
				else 												printf("ERROR : Wrong command\r\n");
				bReceive = 0;
				memset(buffer, 0, sizeof(char) * MAX_BUF);
				flag = '1';
				}
			}
		if (flag == '1'){
			//printf("%d", flag);
			PWM_duty(&dcPwm, 0.7);
			GPIO_write(GPIOC, 6, LOW);
			delay_ms(10000);
			flag = '2';
			//printf("flag: %c\r\n", flag);
			USART_write(USART6, &flag, 1);  // send data to other mcu
		}
		if(flag == '2'){
			//printf("\r\n%d", flag);
			GPIO_write(GPIOC, 6, HIGH);
			PWM_duty(&dcPwm, 1);
			//if (distance_present[0] - distance_past[] > 5000);
			if(distance_present[0] < 8 && distance_present[0] > 2){
				flag = '3';
				//printf("%c\r\n",flag);
				USART_write(USART6, &flag, 1);  // send data to other mcu
			}
		}
		if(flag == '4'){
			//printf("%c", flag);
			if(distance_present[1] < 8 && distance_present[1] > 2){
				flag = '5';
				USART_write(USART6, &flag, 1);  // send data to other mcu
			}
		}
	}
}

// Initialiization 
void setup(void)
{	
	RCC_PLL_init();                         // System Clock = 84MHz
	USART_init(USART2, 9600);
	USART_begin(USART6, GPIOA, 11, GPIOA, 12, 9600); 	// PA11: TXD, PA12: RXD
	SysTick_init();
	
	// input capture
	// PWM configuration ---------------------------------------------------------------------	
	PWM_t trig;													// PWM1 for trig
	PWM_init(&trig, GPIOA, 6);			 		// PA_6: Ultrasonic trig pulse
	PWM_period_us(&trig, 50000);    		// PWM of 50ms period. Use period_us()
	PWM_pulsewidth_us(&trig, 10);   		// PWM pulse width of 10us
	
	// Input Capture configuration -----------------------------------------------------------------------	
	IC_t echo1;													// Input Capture for echo
	ICAP_init(&echo1, GPIOA, 0);    		// PA0 as input caputre
 	ICAP_counter_us(&echo1, 100);   			// ICAP counter step time as 10us
	ICAP_setup(&echo1, 1, IC_RISE);   	// TIM2_CH1 as IC1 , rising edge detect
	ICAP_setup(&echo1, 2, IC_FALL);   	// TIM2_CH1 as IC2 , falling edge detect
	
	IC_t echo2;													// Input Capture for echo
	ICAP_init(&echo2, GPIOB, 10);    		// PB10 as input caputre
 	ICAP_counter_us(&echo2, 100);   			// ICAP counter step time as 10us
	ICAP_setup(&echo2, 3, IC_RISE);   	// TIM2_CH3 as IC3 , rising edge detect
	ICAP_setup(&echo2, 4, IC_FALL);   	// TIM2_CH3 as IC4 , falling edge detect
	
	//belt motor
	PWM_init(&dcPwm, GPIOC, 8);
	PWM_period_us(&dcPwm, 50000);
	PWM_duty(&dcPwm, 0);
	GPIO_write(GPIOC, 6, LOW);
	GPIO_init(GPIOC, 6, OUTPUT);
	GPIO_pupd(GPIOC, 6, EC_PD);
	GPIO_otype(GPIOC, 6, Push_Pull);
	GPIO_ospeed(GPIOC, 6, High);
	/*
	//fire
	GPIO_init(GPIOA, FIRE_PIN, INPUT);  // calls RCC_GPIOC_enable()
	GPIO_pupd(GPIOA, FIRE_PIN, EC_PU);
	EXTI_init(GPIOA, FIRE_PIN, FALL, 2);
	TIM_INT_init(TIM4,1);
	TIM_period_ms(TIM4, 500);
	TIM_INT_enable(TIM4);
	GPIO_init(GPIOA, LED_PIN, OUTPUT);   		   // calls RCC_GPIOA_enable()	
	*/
}



void TIM2_IRQHandler(void){
	if(is_UIF(TIM2)){                     	// Update interrupt
		ovf_cnt[0]++;													// overflow count
		ovf_cnt[1]++;													// overflow count
		clear_UIF(TIM2);  							    	// clear update interrupt flag
	}
	if(is_CCIF(TIM2, 1)){ 									// TIM2_Ch1 (IC1) Capture Flag. Rising Edge Detect
		time1[1] = TIM2->CCR1;								// Capture TimeStart
		clear_CCIF(TIM2, 1);                 	// clear capture/compare interrupt flag 
	}								                      
	else if(is_CCIF(TIM2, 2)){ 							// TIM2_Ch1 (IC2) Capture Flag. Falling Edge Detect
		time2[1] = TIM2->CCR2;								// Capture TimeEnd
		timeInterval[1] = (time2[1]- time1[1] + ovf_cnt[1] * (TIM2->ARR +1)) *10.0 / 1000.0; 					// Total time of echo pulse (10us * counter pulses -> [msec] unit)
		ovf_cnt[1] = 0;                       // overflow reset
		clear_CCIF(TIM2, 2);								  // clear capture/compare interrupt flag 
	}
	if(is_CCIF(TIM2, 3)){ 									// TIM2_Ch3 (IC3) Capture Flag. Rising Edge Detect
		time1[0] = TIM2->CCR3;								// Capture TimeStart
		clear_CCIF(TIM2, 3);                 	// clear capture/compare interrupt flag 
	}								                      
	else if(is_CCIF(TIM2, 4)){ 							// TIM2_Ch3 (IC4) Capture Flag. Falling Edge Detect
		time2[0] = TIM2->CCR4;								// Capture TimeEnd
		timeInterval[0] = (time2[0] - time1[0] + ovf_cnt[0] * (TIM2->ARR +1)) *10.0 / 1000.0; 					// Total time of echo pulse (10us * counter pulses -> [msec] unit)
		ovf_cnt[0] = 0;                       // overflow reset
		clear_CCIF(TIM2, 4);								  // clear capture/compare interrupt flag 
	}
}

void USART6_IRQHandler(){		//USART1 INT 
	if(is_USART_RXNE(USART6)){
		flag = USART_getc(USART6);
	}
}

void USART2_IRQHandler(){		//USART2 INT 
	if(is_USART_RXNE(USART2)){
		pcData = USART_getc(USART2);
		printf("%c",pcData); 							// echo to sender(pc)
		if(pcData == END_CHAR) {
			bReceive = 1;
			idx = 0;
			printf("\r\n"); 							// to change line on PC display
			
		}
		else{
			if(idx > MAX_BUF){
				idx = 0;
				memset(buffer, 0, sizeof(char) * MAX_BUF);
				printf("ERROR : Too long string\r\n");
			}
			buffer[idx] = pcData;
			idx++;
		}
	}
}
/*
void EXTI15_10_IRQHandler(void) {
	if (is_pending_EXTI(FIRE_PIN)) {
		pre_flag = flag;
		flag = '6';
		USART_write(USART6, &flag, 1);  // send data to other mcu
		printf("FIRE\r\n");
		clear_pending_EXTI(FIRE_PIN); 		// cleared by writing '1'
	}
}

void TIM4_IRQHandler(void){
	if(is_UIF(TIM4) && flag == '6'){ // update interrupt flag
		LED_toggle();
		clear_UIF(TIM4);// clear by writing 0	
	}
}
*/