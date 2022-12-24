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
#include "stdlib.h"

#define A 0
#define B 1
#define END_CHAR 13
#define MAX_BUF 10

uint8_t flag = '0';

//IR parameter//
uint32_t IR1, IR2;
uint8_t adc_flag = 0;
int seqCHn[16] = { 8,9, };

//ultrasonic [back, front]
uint32_t ovf_cnt[2] = { 0, 0 };
float distance_past[2] = { 0, 0 };
float distance_present[2] = { 0, 0 };
float timeInterval[2] = { 0, 0 };
float time1[2] = { 0, 0 };
float time2[2] = { 0, 0 };

//usart
uint8_t recvChar = 0;
uint8_t pcData = 0;
uint8_t buffer[MAX_BUF] = { 0, };
int idx = 0;
int bReceive = 0;

//servo motor
int servo_flag = 0;
int count = 0;
float servo_duty = 0;
int duty_count = 0;
PWM_t servo_pwm;

void setup(void);

int main(void) {

	// Initialiization --------------------------------------------------------
	setup();
	float rev = 0.15; 		// stepper
	// Inifinite Loop ----------------------------------------------------------
	while (1) {

		distance_present[0] = (float)timeInterval[0] * 340.0 / 2.0 / 10.0 * 10; 	// [mm] -> [cm]
		distance_present[1] = (float)timeInterval[1] * 340.0 / 2.0 / 10.0 * 10; 	// [mm] -> [cm]

		// Line Tracking
		if (flag == '2') {
			//Collision Prevention, Distance Error Correction
			if (abs(distance_present[1] - distance_past[1]) > 200);

			else if (distance_present[1] < 8) {
				//printf("STOP\r\n");
				Stepper_stop();
			}
			else if (IR1 > 2000) {
				//printf("TURN RIGHT\r\n");
				Stepper_step(2048 * 0.04, 0, 0, FULL);
			}
			else if (IR2 > 3000) {
				//printf("TURN LEFT\r\n");
				Stepper_step(2048 * 0.03, 1, 1, FULL);
			}
			else {
				//printf("GO Front\r\n");
				Stepper_step(2048 * rev, 1, 0, FULL);
			}

			distance_past[0] = distance_present[0];
			distance_past[1] = distance_present[1];
		}

		//Transport Object Box to Belt
		if (flag == '3') {
			//Adjust Distance
			if (duty_count == 0)	Stepper_step(2048 * 0.18, 1, 0, FULL);
			duty_count++;												//0 to 180 degree
			//Transport Object Box to Belt
			servo_duty = 0.00556 * duty_count / 2 + 0.025;
			PWM_duty(&servo_pwm, servo_duty);
			delay_ms(500);
			if (duty_count > 15) {
				duty_count = 30;
				servo_duty = 0.00556 * duty_count / 2 + 0.025;
				PWM_duty(&servo_pwm, servo_duty);
				delay_ms(500);
				duty_count = 0;
				servo_duty = 0.00556 * duty_count / 2 + 0.025;
				PWM_duty(&servo_pwm, servo_duty);
				delay_ms(500);
				//Turn Back
				Stepper_step(2048 * 0.6, 0, 1, FULL);
				Stepper_step(2048 * 1.0, 0, 0, FULL);
				flag = '4';
				USART_write(USART6, &flag, 1);  // send data to other mcu
			}
		}
		// Line Tracking
		if (flag == '4') {
			//Collision Prevention, Distance Error Correction
			if (abs(distance_present[1] - distance_past[1]) > 200);

			else if (distance_present[1] < 8) {
				//printf("STOP\r\n");
				Stepper_stop();
			}
			else if (IR1 > 2000) {
				//printf("TURN RIGHT\r\n");
				Stepper_step(2048 * 0.04, 0, 0, FULL);
			}
			else if (IR2 > 3000) {
				//printf("TURN LEFT\r\n");
				Stepper_step(2048 * 0.03, 1, 1, FULL);
			}
			else {
				//printf("GO Front\r\n");
				Stepper_step(2048 * rev, 1, 0, FULL);
			}

			distance_past[0] = distance_present[0];
			distance_past[1] = distance_present[1];
		}
		//Turn Back, Adjust Distance
		if (flag == '5') {
			Stepper_step(2048 * 0.6, 0, 1, FULL);
			Stepper_step(2048 * 1.07, 0, 0, FULL);
			Stepper_step(2048 * 0.45, 0, 1, FULL);
			flag = '1';
			USART_write(USART6, &flag, 1);  // send data to other mcu
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

	// ADC setting
	ADC_init(GPIOB, 0, TRGO);		//channel 8
	ADC_init(GPIOB, 1, TRGO);		//channel 9

	// ADC channel sequence setting
	ADC_sequence(2, seqCHn);		//2 channel array

	// ADON, SW Trigger enable
	ADC_start();

	// input capture
	// PWM configuration ---------------------------------------------------------------------	
	PWM_t trig;												// PWM1 for trig
	PWM_init(&trig, GPIOA, 6);			 		// PA_6: Ultrasonic trig pulse
	PWM_period_us(&trig, 50000);    	// PWM of 50ms period. Use period_us()
	PWM_pulsewidth_us(&trig, 10);   	// PWM pulse width of 10us

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

	//stepper
	Stepper_init(GPIOC, 6, GPIOC, 8, GPIOB, 8, GPIOC, 9, GPIOC, 0, GPIOC, 1, GPIOC, 2, GPIOC, 3); // Stepper GPIO pin initialization
	Stepper_setSpeed(10);

	//servo motor
	//PWM_t servo_pwm;
	PWM_init(&servo_pwm, GPIOB, 6);						// PWM Initialiization
	PWM_period_ms(&servo_pwm, 20);						// set PWM period
}


//line tracing
void ADC_IRQHandler(void) {
	if ((is_ADC_OVR())) {
		clear_ADC_OVR();
	}

	if (is_ADC_EOC()) {       //after finishing sequence
		if (adc_flag == 0) {
			IR1 = ADC_read();
		}
		else if (adc_flag == 1) {
			IR2 = ADC_read();
		}
		adc_flag = !adc_flag;
	}
}

//ultrasonic distance
void TIM2_IRQHandler(void) {
	if (is_UIF(TIM2)) {                   			// Update interrupt
		ovf_cnt[0]++;								// overflow count
		ovf_cnt[1]++;								// overflow count
		clear_UIF(TIM2);  							// clear update interrupt flag
	}
	if (is_CCIF(TIM2, 1)) { 						// TIM2_Ch1 (IC1) Capture Flag. Rising Edge Detect
		time1[1] = TIM2->CCR1;						// Capture TimeStart
		clear_CCIF(TIM2, 1);                 		// clear capture/compare interrupt flag 
	}
	else if (is_CCIF(TIM2, 2)) { 					// TIM2_Ch1 (IC2) Capture Flag. Falling Edge Detect
		time2[1] = TIM2->CCR2;						// Capture TimeEnd
		// Total time of echo pulse (10us * counter pulses -> [msec] unit)
		timeInterval[1] = (time2[1] - time1[1] + ovf_cnt[1] * (TIM4->ARR + 1)) * 10.0 / 1000.0;
		ovf_cnt[1] = 0;                    		   	// overflow reset
		clear_CCIF(TIM2, 2);						// clear capture/compare interrupt flag 
	}
	if (is_CCIF(TIM2, 3)) { 						// TIM2_Ch3 (IC3) Capture Flag. Rising Edge Detect
		time1[0] = TIM2->CCR3;						// Capture TimeStart
		clear_CCIF(TIM2, 3);                 		// clear capture/compare interrupt flag 
	}
	else if (is_CCIF(TIM2, 4)) { 					// TIM2_Ch3 (IC4) Capture Flag. Falling Edge Detect
		time2[0] = TIM2->CCR4;						// Capture TimeEnd
		// Total time of echo pulse (10us * counter pulses -> [msec] unit)
		timeInterval[0] = (time2[0] - time1[0] + ovf_cnt[0] * (TIM2->ARR + 1)) * 10.0 / 1000.0;
		ovf_cnt[0] = 0;                       		// overflow reset
		clear_CCIF(TIM2, 4);						// clear capture/compare interrupt flag 
	}

}

//setting flag
void USART6_IRQHandler() {							//USART1 INT 
	if (is_USART_RXNE(USART6)) {
		flag = USART_getc(USART6);
	}
}

void USART2_IRQHandler() {							//USART2 INT 
	if (is_USART_RXNE(USART2)) {
		pcData = USART_getc(USART2);
		USART_write(USART6, &pcData, 1);			// transmit char to USART1
		printf("%c", pcData); 						// echo to sender(pc)

		if (pcData == END_CHAR) {
			printf("\r\n"); 						// to change line on PC display
		}
	}
}