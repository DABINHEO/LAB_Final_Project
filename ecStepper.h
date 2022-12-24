#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecSysTick.h"
			
#ifndef __EC_STEPPER_H
#define __EC_STEPPER_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

//State mode
#define HALF 0
#define FULL 1	 
	 
/* Stepper Motor */
//stepper motor function

typedef struct{
   GPIO_TypeDef *port1;
   int pin1;
	 GPIO_TypeDef *port2;
   int pin2;
	 GPIO_TypeDef *port3;
   int pin3;
	 GPIO_TypeDef *port4;
   int pin4;
	 GPIO_TypeDef *port5;
   int pin5;
	 GPIO_TypeDef *port6;
   int pin6;
	 GPIO_TypeDef *port7;
   int pin7;
	 GPIO_TypeDef *port8;
   int pin8;
	 int _step_num;
} Stepper_t;

	 
void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4, GPIO_TypeDef* port5, int pin5, GPIO_TypeDef* port6, int pin6, GPIO_TypeDef* port7, int pin7, GPIO_TypeDef* port8, int pin8);
void Stepper_setSpeed(long whatSpeed);
void Left_Stepper_setSpeed(long whatSpeed);
void Right_Stepper_setSpeed(long whatSpeed);
void Stepper_step(int steps, int left_direction, int right_direction, int mode); 
void Left_Stepper_step(int steps, int direction, int mode); 
void Right_Stepper_step(int steps, int direction, int mode); 
void Stepper_stop(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
