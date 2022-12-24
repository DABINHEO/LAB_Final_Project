#include "stm32f4xx.h"
#include "ecStepper.h"
#include "ecGPIO.h"
#include "ecSysTick.h"

//State number 
#define S0 0
#define S1 1
#define S2 2
#define S3 3
#define S4 4
#define S5 5
#define S6 6
#define S7 7


// Stepper Motor function
uint32_t direction = 1; 
uint32_t step_delay = 100; 
uint32_t Left_step_delay = 100; 
uint32_t Right_step_delay = 100; 
uint32_t step_per_rev = 64*32;
	 

// Stepper Motor variable
volatile Stepper_t myStepper; 


//FULL stepping sequence  - FSM
typedef struct {
	uint8_t out[4];
  uint32_t next[2];
} State_full_t;

State_full_t FSM_full[4] = {  
 {{1,1,0,0},{S1,S3}},  //S0
 {{0,1,1,0},{S2,S0}},	//S1
 {{0,0,1,1},{S3,S1}},	//S2
 {{1,0,0,1},{S0,S2}}		//S3
};

//HALF stepping sequence
typedef struct {
	uint8_t out[4];
  uint32_t next[2];
} State_half_t;

State_half_t FSM_half[8] = { 
 {{1,0,0,0},{S1,S7}},	//S0
 {{1,1,0,0},{S2,S0}},	//S1
 {{0,1,0,0},{S3,S1}},	//S2
 {{0,1,1,0},{S4,S2}},	//S3
 {{0,0,1,0},{S5,S3}},	//S4
 {{0,0,1,1},{S6,S4}},	//S5
 {{0,0,0,1},{S7,S5}},	//S6
 {{1,0,0,1},{S0,S6}}	//S7
 };



void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4, GPIO_TypeDef* port5, int pin5, GPIO_TypeDef* port6, int pin6, GPIO_TypeDef* port7, int pin7, GPIO_TypeDef* port8, int pin8){
	 
//  GPIO Digital Out Initiation
	 myStepper.port1 = port1;
   myStepper.pin1  = pin1;
	 myStepper.port1 = port2;
   myStepper.pin1  = pin2;
	 myStepper.port1 = port3;
   myStepper.pin1  = pin3;
	 myStepper.port1 = port4;
   myStepper.pin1  = pin4;
	 myStepper.port1 = port5;
   myStepper.pin1  = pin5;
	 myStepper.port1 = port6;
   myStepper.pin1  = pin6;
	 myStepper.port1 = port7;
   myStepper.pin1  = pin7;
	 myStepper.port1 = port8;
   myStepper.pin1  = pin8;
	 	
//  GPIO Digital Out Initiation
	 GPIO_init(port1, pin1, OUTPUT);
	 GPIO_init(port2, pin2, OUTPUT);
	 GPIO_init(port3, pin3, OUTPUT);
	 GPIO_init(port4, pin4, OUTPUT);
	 GPIO_init(port5, pin5, OUTPUT);
	 GPIO_init(port6, pin6, OUTPUT);
	 GPIO_init(port7, pin7, OUTPUT);
	 GPIO_init(port8, pin8, OUTPUT);
		// No pull-up Pull-down , Push-Pull, Fast	
	 GPIO_pupd(port1, pin1, EC_NUD);
	 GPIO_pupd(port2, pin2, EC_NUD);
	 GPIO_pupd(port3, pin3, EC_NUD);
	 GPIO_pupd(port4, pin4, EC_NUD);
	 GPIO_pupd(port5, pin5, EC_NUD);
	 GPIO_pupd(port6, pin6, EC_NUD);
	 GPIO_pupd(port7, pin7, EC_NUD);
	 GPIO_pupd(port8, pin8, EC_NUD);
	 GPIO_otype(port1, pin1, Push_Pull);
	 GPIO_otype(port2, pin2, Push_Pull);
	 GPIO_otype(port3, pin3, Push_Pull);
	 GPIO_otype(port4, pin4, Push_Pull);
	 GPIO_otype(port5, pin5, Push_Pull);
	 GPIO_otype(port6, pin6, Push_Pull);
	 GPIO_otype(port7, pin7, Push_Pull);
	 GPIO_otype(port8, pin8, Push_Pull);
	 GPIO_ospeed(port1, pin1, Fast);
	 GPIO_ospeed(port2, pin2, Fast);
	 GPIO_ospeed(port3, pin3, Fast);
	 GPIO_ospeed(port4, pin4, Fast);
	 GPIO_ospeed(port5, pin5, Fast);
	 GPIO_ospeed(port6, pin6, Fast);
	 GPIO_ospeed(port7, pin7, Fast);
	 GPIO_ospeed(port8, pin8, Fast);
}

void Stepper_pinOut (uint32_t state, int mode){
	   if (mode == FULL){         // FULL mode
			// Right Wheel
			GPIO_write(GPIOC, 6, FSM_full[state].out[0]);
			GPIO_write(GPIOC, 8, FSM_full[state].out[1]);
			GPIO_write(GPIOB, 8, FSM_full[state].out[2]);
			GPIO_write(GPIOC, 9, FSM_full[state].out[3]);
			// Left Wheel
			GPIO_write(GPIOC, 0, FSM_full[state].out[0]);
			GPIO_write(GPIOC, 1, FSM_full[state].out[1]);
			GPIO_write(GPIOC, 2, FSM_full[state].out[2]);
			GPIO_write(GPIOC, 3, FSM_full[state].out[3]);
			}	 
		 else if (mode == HALF){    // HALF mode
			// Right Wheel
			GPIO_write(GPIOC, 6, FSM_half[state].out[0]);
			GPIO_write(GPIOC, 8, FSM_half[state].out[1]);
			GPIO_write(GPIOB, 5, FSM_half[state].out[2]);
			GPIO_write(GPIOC, 9, FSM_half[state].out[3]);
			// Left Wheel
			GPIO_write(GPIOC, 0, FSM_full[state].out[0]);
			GPIO_write(GPIOC, 1, FSM_full[state].out[1]);
			GPIO_write(GPIOC, 2, FSM_full[state].out[2]);
			GPIO_write(GPIOC, 3, FSM_full[state].out[3]);
			}
}
void Right_Stepper_pinOut (uint32_t state, int mode){
	   if (mode == FULL){         // FULL mode
			// Right Wheel
			GPIO_write(GPIOC, 6, FSM_full[state].out[0]);
			GPIO_write(GPIOC, 8, FSM_full[state].out[1]);
			GPIO_write(GPIOB, 8, FSM_full[state].out[2]);
			GPIO_write(GPIOC, 9, FSM_full[state].out[3]);
			}	 
		 else if (mode == HALF){    // HALF mode
			// Right Wheel
			GPIO_write(GPIOC, 6, FSM_half[state].out[0]);
			GPIO_write(GPIOC, 8, FSM_half[state].out[1]);
			GPIO_write(GPIOB, 5, FSM_half[state].out[2]);
			GPIO_write(GPIOC, 9, FSM_half[state].out[3]);
			}
}
void Left_Stepper_pinOut (uint32_t state, int mode){
	   if (mode == FULL){         // FULL mode
			// Left Wheel
			GPIO_write(GPIOC, 0, FSM_full[state].out[0]);
			GPIO_write(GPIOC, 1, FSM_full[state].out[1]);
			GPIO_write(GPIOC, 2, FSM_full[state].out[2]);
			GPIO_write(GPIOC, 3, FSM_full[state].out[3]);
			}	 
		 else if (mode == HALF){    // HALF mode
			// Left Wheel
			GPIO_write(GPIOC, 0, FSM_full[state].out[0]);
			GPIO_write(GPIOC, 1, FSM_full[state].out[1]);
			GPIO_write(GPIOC, 2, FSM_full[state].out[2]);
			GPIO_write(GPIOC, 3, FSM_full[state].out[3]);
			}
}


void Stepper_setSpeed (long whatSpeed){      // rpm
		step_delay = 60000 / 2048 / whatSpeed; //YOUR CODE   // Convert rpm to milli sec
}

void Stepper_step(int steps, int left_direction, int right_direction, int mode){
	 uint32_t Left_state = 0;
	 uint32_t Right_state = 0;

	for(; steps > 0; steps--){ // run for step size
				// YOUR CODE                                  		// delay (step_delay); 
		if (mode == FULL){
				Left_state = FSM_full[Left_state].next[left_direction];// YOUR CODE       // state = next state
				Right_state = FSM_full[Right_state].next[right_direction];// YOUR CODE       // state = next state
				delay_ms(step_delay);
		}
		else if (mode == HALF){
				Left_state = FSM_half[Left_state].next[left_direction];// YOUR CODE       // state = next state
				Right_state = FSM_half[Right_state].next[right_direction];// YOUR CODE       // state = next state
				delay_ms(step_delay / 2);
		}
		Left_Stepper_pinOut(Left_state, mode);
		Right_Stepper_pinOut(Right_state, mode);
   }
}

void Left_Stepper_step(int steps, int direction, int mode){
	 uint32_t Left_state = 0;

	for(; steps > 0; steps--){ // run for step size
				// YOUR CODE                                  		// delay (step_delay); 
		if (mode == FULL){
				Left_state = FSM_full[Left_state].next[direction];// YOUR CODE       // state = next state
				delay_ms(Left_step_delay);
		}
		else if (mode == HALF){
				Left_state = FSM_half[Left_state].next[direction];// YOUR CODE       // state = next state
				delay_ms(Left_step_delay / 2);
		}
		Left_Stepper_pinOut(Left_state, mode);
   }
}

void Right_Stepper_step(int steps, int direction, int mode){
	 uint32_t Right_state = 0;

	for(; steps > 0; steps--){ // run for step size
				// YOUR CODE                                  		// delay (step_delay); 
		if (mode == FULL){
				Right_state = FSM_full[Right_state].next[direction];// YOUR CODE       // state = next state
				delay_ms(Right_step_delay);
		}
		else if (mode == HALF){
				Right_state = FSM_half[Right_state].next[direction];// YOUR CODE       // state = next state
				delay_ms(Right_step_delay / 2);
		}
		Right_Stepper_pinOut(Right_state, mode);
   }
}

void Stepper_stop (void){ 
     
    	myStepper._step_num = 0;    
			// All pins(Port1~4, Pin1~4) set as DigitalOut '0'
			 	// Right Wheel
			GPIO_write(GPIOC, 6, 0);
			GPIO_write(GPIOC, 8, 0);
			GPIO_write(GPIOB, 5, 0);
			GPIO_write(GPIOC, 9, 0);
			// Left Wheel
			GPIO_write(GPIOC, 0, 0);
			GPIO_write(GPIOC, 1, 0);
			GPIO_write(GPIOC, 2, 0);
			GPIO_write(GPIOC, 3, 0);
}

