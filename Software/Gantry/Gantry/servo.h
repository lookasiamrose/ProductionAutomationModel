/*
 * servo.h
 *
 * Created: 19.04.2017 18:43:02
 *  Author: lukasz.jamroz
 */ 


#ifndef SERVO_H_
#define SERVO_H_
#include <avr/interrupt.h>
#include <avr/io.h>
#include "stepper.h"
#define SERVO_ORDERS_TOTAL_NUMBER 10 //always +1 because index zero is reserved

enum ServoIndexes{ PUSH, SERVO_NUM_OF_SERVOS };
typedef struct Servo_Struct
{
	enum ServoIndexes index;
	volatile uint8_t* port;
	volatile uint8_t pin;
	volatile uint16_t actual_position;
	volatile uint8_t orders_head;
	volatile uint8_t barrier_delay;
} Servo_Struct;
typedef struct Servo_Orders_Struct
{
	volatile uint8_t customer_index;
	volatile uint16_t position;
	volatile uint8_t barrier;
	volatile uint8_t next;
} Servo_Orders_Struct;

volatile Servo_Struct servo_servos_table[SERVO_NUM_OF_SERVOS];
volatile Servo_Orders_Struct servo_orders_table[SERVO_ORDERS_TOTAL_NUMBER];

void servo_add_servo(enum ServoIndexes index_arg,volatile uint8_t* port_arg, uint8_t pin_arg, uint16_t basic_position_arg);
void servo_init();
void servo_set_servo(enum ServoIndexes index_arg, uint16_t value);
void servo_push_back_order(enum ServoIndexes index_arg,uint16_t position_arg, uint8_t customer_barrier_arg, uint8_t customer_index_arg, char isServoBarrier);
void servo_process_servo(enum ServoIndexes index_arg);

#endif /* SERVO_H_ */