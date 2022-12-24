### LAB : Final Project

**Date:** 2022.12.24

**Author/Partner:** Heo Dabin

**Github:** [Github code](https://github.com/DABINHEO/LAB_Final_Project.git)

##            



### Introduction

In this lab, we organized an automatic product transportation system. Two NUCLEO-F401REs were used to control one conveyor belt and one transportation equipment. Ultrasonic sensors were attached to the front of the transportation equipment to prevent it from hitting other objects during transportation, and two IR sensors were used to automatically move along the black line. The wheels were made to work using two stepper motors. A small box was attached to the top of the transportation equipment to receive the goods and move them to the conveyor, which was controlled by the Servomotor.
In the conveyor belt, the belt is operated using a DC motor, and ultrasonic sensors are attached to the front and rear parts of the belt, respectively, to calculate the distance from the transportation equipment to change the state.
The conveyor belt and transportation equipment were connected to Zigbee, respectively, to exchange commands and change the state.
In this lab, we created a simple goods transportation system that receives goods from the conveyor belt and moves them to the opposite side to put them back on the conveyor belt. It is thought that a more useful logistics transportation system will be created by adding various additional functions such as creating an additional warehouse to put goods in and out, or leaving a record of goods transportation on the main server. What we made this time is the basic step.



### Requirement

#### Hardware

* MCU
  
  * NUCLEO-F401RE x 2
  * Xbee ZigBee TH(S2C) x 2
  
* Sensor

   * IR Reflective Sensor (TCRT 5000) x2
   * HC-SR04 x 4
   * Flame Sensor

* Actuator
   * RC Servo Motor (SG90)
   * DC motor, DC motor driver(L9110s),
* 3Stepper Motor 28BYJ-48 x2
   * Motor Driver ULN2003 x2
   
   
   

#### Software

* Keil uVision, CMSIS, EC_HAL library

##          



### Step1: RC Car



#### Procedure

The main functions of transportation equipment are as follows.
1. Line Tracing
2. Collision Prevention
3. Product Transportation
4. Emergency shutdown

The power part made the wheels move using two stepper motors. Line Tracing uses two IR sensors to automatically move along the black line. An ultrasonic sensor was attached to the front to prevent it from colliding with other objects during transportation, and a small box was attached to the top of the transportation equipment to receive goods and move them to the conveyor, which was controlled by a Servomotor. The emergency stop function that stops all functions in the event of a fire was commanded using the Flame Sensor.



#### Configuration

| Type                                                | Port-Pin                               | Timer                  | Configuration                                                |
| --------------------------------------------------- | -------------------------------------- | ---------------------- | ------------------------------------------------------------ |
| System Clock                                        |                                        |                        | PLL 84MHz                                                    |
| USART1 : MCU1 - MCU2 (Zigbee)                       | TXD: PA11<br />RXD: PA12               |                        | No Parity, 8-bit Data, 1-bit Stop bit, 9600 baud-rate        |
| ADC1_CH8 (1st channel) <br />ADC1_CH9 (2nd channel) | PB0 <br />PB1                          | TIM3                   | ADC Clock Prescaler /8 12-bit resolution, right alignment Single Conversion mode Scan mode: Two channels in regular group External Trigger (Timer3 TRGO) @ 1kHz Trigger Detection on Rising Edge<br />Up-Counter, Counter CLK 1kHz OC1M(Output Compare 1 Mode) : PWM mode 1 Master Mode Selection: (TRGO) OC1REF<br />Analog Mode No Pull-up Pull-down |
| PWM1(Ultrasonic)<br />PWM2(Servo Motor)             | PA6<br />PB6                           | TIM3_CH1<br />TIM4_CH1 | AF, Push-Pull, No Pull-up Pull-down, Fast<br />PWM1 period: 50msec, pulse width: 10usec<br />AF, Push-Pull, Pull-up, Fast<br />PWM2 period: 20msec, duty ratio: 0.5~2.5msec |
| Input Capture(Front)<br />Input Capture(Back)       | PA0 <br />PB10                         | TIM2_CH1<br />TIM2_CH3 | AF, No Pull-up Pull-down<br />Counter Clock : 0.01MHz (100us) <br />TI1 -> IC1 (rising edge) <br />TI1 -> IC2 (falling edge)<br />TI3 -> IC3 (rising edge) <br />TI3 -> IC4 (falling edge) |
| Stepper Motor                                       | PC6, PC8, PB8, PC9, PC0, PC1, PC2, PC3 |                        | NO Pull-up Pull-down Push-Pull Fast                          |



#### Flow Chart

![image](https://user-images.githubusercontent.com/113574284/209429897-06d3c619-0c87-4da2-b6bb-3be0379da555.png)

#### Circuit Diagram

![image](https://user-images.githubusercontent.com/113574284/209433316-a7cf5acb-fc0a-4292-80a0-2369a39d557f.png)



#### Description with Code

* LAB_Final_Project_main_1.c description

```c++
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
int seqCHn[16] = {8,9,};

//ultrasonic [back, front]
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
    while(1){

        distance_present[0] = (float) timeInterval[0] * 340.0 / 2.0 / 10.0 * 10; 	// [mm] -> [cm]
        distance_present[1] = (float) timeInterval[1] * 340.0 / 2.0 / 10.0 * 10; 	// [mm] -> [cm]
		
        // Line Tracking
        if(flag == '2'){
			//Collision Prevention, Distance Error Correction
            if (abs(distance_present[1] - distance_past[1]) > 200);

            else if (distance_present[1] < 8){
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
            else{
                //printf("GO Front\r\n");
                Stepper_step(2048 * rev, 1, 0, FULL);
            }

            distance_past[0] = distance_present[0];
            distance_past[1] = distance_present[1];
        }
        
        //Transport Object Box to Belt
        if(flag == '3'){
			//Adjust Distance
            if(duty_count == 0)	Stepper_step(2048 * 0.18, 1, 0, FULL);
            duty_count++;												//0 to 180 degree
            //Transport Object Box to Belt
            servo_duty = 0.00556*duty_count/2 + 0.025;
            PWM_duty(&servo_pwm, servo_duty);
            delay_ms(500);
            if(duty_count > 15){
                duty_count = 30;
                servo_duty = 0.00556*duty_count/2 + 0.025;
                PWM_duty(&servo_pwm, servo_duty);
                delay_ms(500);
                duty_count = 0;
                servo_duty = 0.00556*duty_count/2 + 0.025;
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
        if(flag == '4'){
            //Collision Prevention, Distance Error Correction
            if (abs(distance_present[1] - distance_past[1]) > 200);
            
            else if (distance_present[1] < 8){
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
            else{
                //printf("GO Front\r\n");
                Stepper_step(2048 * rev, 1, 0, FULL);
            }

            distance_past[0] = distance_present[0];
            distance_past[1] = distance_present[1];
        }			
		//Turn Back, Adjust Distance
        if(flag == '5'){
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
	Stepper_init(GPIOC,6,GPIOC,8,GPIOB,8,GPIOC,9,GPIOC,0,GPIOC,1,GPIOC,2,GPIOC,3); // Stepper GPIO pin initialization
	Stepper_setSpeed(10);
	
	//servo motor
	//PWM_t servo_pwm;
	PWM_init(&servo_pwm, GPIOB, 6);						// PWM Initialiization
	PWM_period_ms(&servo_pwm, 20);						// set PWM period
}


//line tracing
void ADC_IRQHandler(void){
	if((is_ADC_OVR())){
		clear_ADC_OVR();
	}
	
	if(is_ADC_EOC()){       //after finishing sequence
			if (adc_flag==0){
				IR1 = ADC_read();
			}  
			else if (adc_flag==1){
				IR2 = ADC_read();
			}
		adc_flag =! adc_flag;
	}
}

//ultrasonic distance
void TIM2_IRQHandler(void){
    if(is_UIF(TIM2)){                   			// Update interrupt
		ovf_cnt[0]++;								// overflow count
		ovf_cnt[1]++;								// overflow count
		clear_UIF(TIM2);  							// clear update interrupt flag
	}
	if(is_CCIF(TIM2, 1)){ 							// TIM2_Ch1 (IC1) Capture Flag. Rising Edge Detect
		time1[1] = TIM2->CCR1;						// Capture TimeStart
		clear_CCIF(TIM2, 1);                 		// clear capture/compare interrupt flag 
	}								                      
	else if(is_CCIF(TIM2, 2)){ 						// TIM2_Ch1 (IC2) Capture Flag. Falling Edge Detect
		time2[1] = TIM2->CCR2;						// Capture TimeEnd
        // Total time of echo pulse (10us * counter pulses -> [msec] unit)
		timeInterval[1] = (time2[1]- time1[1] + ovf_cnt[1] * (TIM4->ARR +1)) *10.0 / 1000.0;
		ovf_cnt[1] = 0;                    		   	// overflow reset
		clear_CCIF(TIM2, 2);						// clear capture/compare interrupt flag 
	}
	if(is_CCIF(TIM2, 3)){ 							// TIM2_Ch3 (IC3) Capture Flag. Rising Edge Detect
		time1[0] = TIM2->CCR3;						// Capture TimeStart
		clear_CCIF(TIM2, 3);                 		// clear capture/compare interrupt flag 
	}								                      
	else if(is_CCIF(TIM2, 4)){ 						// TIM2_Ch3 (IC4) Capture Flag. Falling Edge Detect
		time2[0] = TIM2->CCR4;						// Capture TimeEnd
        // Total time of echo pulse (10us * counter pulses -> [msec] unit)
		timeInterval[0] = (time2[0] - time1[0] + ovf_cnt[0] * (TIM2->ARR +1)) *10.0 / 1000.0;
		ovf_cnt[0] = 0;                       		// overflow reset
		clear_CCIF(TIM2, 4);						// clear capture/compare interrupt flag 
	}
	
}

//setting flag
void USART6_IRQHandler(){							//USART1 INT 
	if(is_USART_RXNE(USART6)){
		flag = USART_getc(USART6);
	}
}

void USART2_IRQHandler(){							//USART2 INT 
	if(is_USART_RXNE(USART2)){
		pcData = USART_getc(USART2);
		USART_write(USART6, &pcData, 1);			// transmit char to USART1
		printf("%c",pcData); 						// echo to sender(pc)
		
		if(pcData == END_CHAR){
			printf("\r\n"); 						// to change line on PC display
		}
	}
}
```

##          



### Step2: Conveyor Belt



#### Procedure

The main functions of the conveyor belt are as follows.
1. Operation of the conveyor belt
2. Give orders to transportation equipment
3. Emergency shutdown

In the conveyor belt, the belt is operated using a DC motor, and ultrasonic sensors are attached to the front and rear parts of the belt, respectively, to calculate the distance from the transportation equipment to change the state.
The conveyor belt and transportation equipment were connected to Zigbee, respectively, to exchange commands and change the state. Similarly, an emergency stop function that stops all functions in the event of a fire was ordered using the Flame Sensor.

#### Configuration

| Type                                          | Port-Pin                 | Timer                  | Configuration                                                |
| --------------------------------------------- | ------------------------ | ---------------------- | ------------------------------------------------------------ |
| System Clock                                  |                          |                        | PLL 84MHz                                                    |
| USART1 : MCU1 - MCU2 (Zigbee)                 | TXD: PA11<br />RXD: PA12 |                        | No Parity, 8-bit Data, 1-bit Stop bit, 9600 baud-rate        |
| USART2 : USB cable (ST-Link)                  |                          |                        | No Parity, 8-bit Data, 1-bit Stop bit, 9600 baud-rate        |
| PWM1(Ultrasonic)<br />PWM2(DC Motor)          | PA6<br />PC8             | TIM3_CH1<br />TIM3_CH3 | AF, Push-Pull, No Pull-up Pull-down, Fast<br />PWM1 period: 50msec pulse width: 10usec<br />PWM2 period: 50msec |
| Input Capture(Front)<br />Input Capture(Back) | PA0 <br />PB10           | TIM2_CH1<br />TIM2_CH3 | AF, No Pull-up Pull-down<br />Counter Clock : 0.01MHz (100us) <br />TI1 -> IC1 (rising edge) <br />TI1 -> IC2 (falling edge)<br />TI3 -> IC3 (rising edge) <br />TI3 -> IC4 (falling edge) |
| Fire Sensor EXTI                              | PA7                      |                        | Pull-up                                                      |



#### Flow Chart

![image](https://user-images.githubusercontent.com/113574284/209429914-df0dc190-50cf-41b1-aea8-0b5437467e51.png)

#### 

#### Circuit Diagram

![image](https://user-images.githubusercontent.com/113574284/209433640-d77e468f-4f50-4ff5-8e01-94a4e98b281d.png)



#### Description with Code

* LAB_Final_Project_main_2.c description

```c++
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

//ultrasonic [Back, Front]
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
		
        //Getting Command from Computer
		if(flag == '0' || flag == '6'){
			if(bReceive == 1 && buffer[0] == 'G'){
				if 			(buffer[1] == 'O')	flag = '1';		
				else 												printf("ERROR : Wrong command\r\n");
				bReceive = 0;
				memset(buffer, 0, sizeof(char) * MAX_BUF);
				flag = '1';
				}
			}
        //Belt Operation
		if (flag == '1'){
			PWM_duty(&dcPwm, 0.7);
			GPIO_write(GPIOC, 6, LOW);
			delay_ms(10000);
			flag = '2';
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
	RCC_PLL_init();                         			// System Clock = 84MHz
	USART_init(USART2, 9600);
	USART_begin(USART6, GPIOA, 11, GPIOA, 12, 9600); 	// PA11: TXD, PA12: RXD
	SysTick_init();
	
	// input capture
	// PWM configuration ---------------------------------------------------------------------	
	PWM_t trig;									// PWM1 for trig
	PWM_init(&trig, GPIOA, 6);					// PA_6: Ultrasonic trig pulse
	PWM_period_us(&trig, 50000);    			// PWM of 50ms period. Use period_us()
	PWM_pulsewidth_us(&trig, 10);   			// PWM pulse width of 10us
	
	// Input Capture configuration -----------------------------------------------------------------------	
	IC_t echo1;									// Input Capture for echo
	ICAP_init(&echo1, GPIOA, 0);    			// PA0 as input caputre
 	ICAP_counter_us(&echo1, 100);   			// ICAP counter step time as 10us
	ICAP_setup(&echo1, 1, IC_RISE);   			// TIM2_CH1 as IC1 , rising edge detect
	ICAP_setup(&echo1, 2, IC_FALL);   			// TIM2_CH1 as IC2 , falling edge detect
	
	IC_t echo2;									// Input Capture for echo
	ICAP_init(&echo2, GPIOB, 10);    			// PB10 as input caputre
 	ICAP_counter_us(&echo2, 100);   			// ICAP counter step time as 10us
	ICAP_setup(&echo2, 3, IC_RISE);   			// TIM2_CH3 as IC3 , rising edge detect
	ICAP_setup(&echo2, 4, IC_FALL);   			// TIM2_CH3 as IC4 , falling edge detect
	
	//Belt DC Motor
	PWM_init(&dcPwm, GPIOC, 8);
	PWM_period_us(&dcPwm, 50000);
	PWM_duty(&dcPwm, 0);
	GPIO_write(GPIOC, 6, LOW);
	GPIO_init(GPIOC, 6, OUTPUT);
	GPIO_pupd(GPIOC, 6, EC_PD);
	GPIO_otype(GPIOC, 6, Push_Pull);
	GPIO_ospeed(GPIOC, 6, High);
	/*
	//Flame Detection
	GPIO_init(GPIOA, FIRE_PIN, INPUT);  		// calls RCC_GPIOC_enable()
	GPIO_pupd(GPIOA, FIRE_PIN, EC_PU);
	EXTI_init(GPIOA, FIRE_PIN, FALL, 2);
	TIM_INT_init(TIM4,1);
	TIM_period_ms(TIM4, 500);
	TIM_INT_enable(TIM4);
	GPIO_init(GPIOA, LED_PIN, OUTPUT);   		// calls RCC_GPIOA_enable()	
	*/
}

void TIM2_IRQHandler(void){
	if(is_UIF(TIM2)){                     		// Update interrupt
		ovf_cnt[0]++;							// overflow count
		ovf_cnt[1]++;							// overflow count
		clear_UIF(TIM2);  						// clear update interrupt flag
	}
	if(is_CCIF(TIM2, 1)){ 						// TIM2_Ch1 (IC1) Capture Flag. Rising Edge Detect
		time1[1] = TIM2->CCR1;					// Capture TimeStart
		clear_CCIF(TIM2, 1);                 	// clear capture/compare interrupt flag 
	}								                      
	else if(is_CCIF(TIM2, 2)){ 					// TIM2_Ch1 (IC2) Capture Flag. Falling Edge Detect
		time2[1] = TIM2->CCR2;					// Capture TimeEnd
        // Total time of echo pulse (10us * counter pulses -> [msec] unit)
		timeInterval[1] = (time2[1]- time1[1] + ovf_cnt[1] * (TIM2->ARR +1)) *10.0 / 1000.0; 				
		ovf_cnt[1] = 0;                       	// overflow reset
		clear_CCIF(TIM2, 2);					// clear capture/compare interrupt flag 
	}
	if(is_CCIF(TIM2, 3)){ 						// TIM2_Ch3 (IC3) Capture Flag. Rising Edge Detect
		time1[0] = TIM2->CCR3;					// Capture TimeStart
		clear_CCIF(TIM2, 3);                 	// clear capture/compare interrupt flag 
	}								                      
	else if(is_CCIF(TIM2, 4)){ 					// TIM2_Ch3 (IC4) Capture Flag. Falling Edge Detect
		time2[0] = TIM2->CCR4;					// Capture TimeEnd
        // Total time of echo pulse (10us * counter pulses -> [msec] unit)
		timeInterval[0] = (time2[0] - time1[0] + ovf_cnt[0] * (TIM2->ARR +1)) *10.0 / 1000.0;
		ovf_cnt[0] = 0;                       	// overflow reset
		clear_CCIF(TIM2, 4);					// clear capture/compare interrupt flag 
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
		printf("%c",pcData); 					// echo to sender(pc)
		if(pcData == END_CHAR) {
			bReceive = 1;
			idx = 0;
			printf("\r\n"); 					// to change line on PC display
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
//Flame detection
void EXTI15_10_IRQHandler(void) {
	if (is_pending_EXTI(FIRE_PIN)) {
		pre_flag = flag;
		flag = '6';
		USART_write(USART6, &flag, 1);  // send data to other mcu
		printf("FIRE\r\n");
		clear_pending_EXTI(FIRE_PIN); 		// cleared by writing '1'
	}
}
//Flame detected
void TIM4_IRQHandler(void){
	if(is_UIF(TIM4) && flag == '6'){ // update interrupt flag
		LED_toggle();
		clear_UIF(TIM4);// clear by writing 0	
	}
}
*/
```

##          



### Reference

[LAB: Timer & PWM](https://ykkim.gitbook.io/ec/course/lab/lab-timer-and-pwm)

[LAB: Stepper Motor](https://ykkim.gitbook.io/ec/course/lab/lab-stepper-motor)

[LAB: Input Capture - Ultrasonic](https://ykkim.gitbook.io/ec/course/lab/lab-input-capture-ultrasonic)

[LAB: USART - LED, Bluetooth](https://ykkim.gitbook.io/ec/course/lab/lab-usart-led-bluetooth)

[LAB: ADC - IR sensor](https://ykkim.gitbook.io/ec/course/lab/lab-adc-irsensor)

[Tutorial: DC motor - Motor driver Connection](https://ykkim.gitbook.io/ec/course/tutorial/tutorial-dcmotor-motor-driver-connection)

[Tutorial: Zigbee with Nucleo Board](https://ykkim.gitbook.io/ec/course/tutorial/zigbee)

Class materials in Embedded Controller by Prof. Kim

RM0383 Reference manual

##          



### Troubleshooting

Using the stepper motor as power, control such as the dc motor was impossible. The dc motor is an on-off concept, so it stops as soon as you give the command to stop. It is impossible to interfere with the previous operation command because the stepper motor is a command that gives a certain number of rotations rather than on-off. This causes a problem when noise is generated in the ultra sonic sensor. In the case of using a DC motor, a method was used to determine the present value as noise if the difference between the previous value of an ultrasonic sensor and the present value was large. It was not a big problem when using this method of ignoring noise. However, the stepper motor shows a problem of moving little by little when there is an obstacle ahead because it cannot be ignored when the noise continues more than twice due to the aforementioned way of moving. As a solution, it is thought that there will be a solution using a method of minimizing noise even if it occurs by using a area average filter. It is judged that there may be a problem with the reaction speed, but it will not move when the obstacle is ahead.

I wanted to use a method of stopping all devices in the event of a fire using a flame detection sensor, saving the current state, and re-running the saved state when entering the keyword to restart. However, the flame detection sensor did not work properly, so I couldn't do it. It seems that it is necessary to find and solve the problem through trial and error and debugging.
