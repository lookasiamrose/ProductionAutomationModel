/*
 * stepper.h
 *
 * Created: 22.12.2016 23:34:04
 *  Author: lukasz.jamroz
 */
#ifndef STEPPER_H_
#define STEPPER_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include "servo.h"
#define STEPPER_ORDERS_TOTAL_NUMBER 128 //always +1 because index zero is reserved

enum StepperDirection{ CLOCKWISE, COUNTERCLOCKWISE, UNDEFINED };
enum StepperModes{ PHASE_A, PHASE_B, PHASE_C, PHASE_D };
enum StepperPlacement{ MSB, LSB };
enum StepperIndexes{ TURN, UPDOWN, LEFTRIGHT, MOVE, STEPPER_NUM_OF_MOTORS };
#define TOTAL_ORDERS_NUMBER (STEPPER_ORDERS_TOTAL_NUMBER+SERVO_ORDERS_TOTAL_NUMBER)
typedef struct Motor_Order
{
	volatile uint8_t customer_index;
	enum StepperDirection direction;
	uint16_t move;
	volatile uint8_t barrier;
	volatile uint8_t next;
	
} Motor_Order;
typedef struct Motor_Struct
{
	enum StepperIndexes index;
	volatile uint8_t* port; 
	enum StepperPlacement placement;   
	uint8_t speed;
	volatile uint8_t orders_head;
	
	volatile uint8_t timer;
	uint16_t step;
	enum StepperModes mode;
	uint8_t limit; //what is actual speed in between 100-255
} Motor_Struct;

volatile Motor_Struct stepper_motors_table[STEPPER_NUM_OF_MOTORS];
volatile Motor_Order stepper_motors_orders[STEPPER_ORDERS_TOTAL_NUMBER];

void stepper_init();
void stepper_add_motor(enum StepperIndexes index_arg, volatile uint8_t* port_arg, enum StepperPlacement placement_arg);
void stepper_set_speed(enum StepperIndexes index_arg, uint8_t speed_arg);
void stepper_process_motor(uint8_t index_arg);
void stepper_fastest_and_init_motors();
void stepper_pop_front_order(uint8_t index_arg);
void stepper_push_back_order(uint8_t index_arg,uint16_t limit_arg,enum StepperDirection direction_arg, uint8_t customer_barrier_arg, uint8_t customer_index_arg, char isServoBarrier);

#endif /* STEPPER_H_ */