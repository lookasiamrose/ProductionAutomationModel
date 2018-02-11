/*
 * sensors.h
 *
 * Created: 13.07.2017 18:31:46
 *  Author: lukasz.jamroz
 */ 


#ifndef SENSORS_H_
#define SENSORS_H_

#include <avr/io.h>
#include <avr/interrupt.h>

// -- EDITABLE

#define SERVO_ORDERS_TOTAL_NUMBER 13 //always +1 because index zero is reserved
#define STEPPER_ORDERS_TOTAL_NUMBER 24 //always +1 because index zero is reserved
#define ORDERS_TOTAL_SENSORS_ORDERS_NUMBER 24 //always +1 because index zero is reserved

/* ORDER TYPE */
enum Orders_Barrier_Device{ STEPPER_DEVICE, SERVO_DEVICE, SENSOR_DEVICE, NUM_OF_DIFF_DEVICES };
	 
typedef struct Orders_Barrier
{
	enum Orders_Barrier_Device device;
	uint8_t value;
} Orders_Barrier;

uint8_t orders_get_barrier(enum Orders_Barrier_Device barrier_device_arg, uint8_t barrier_index_arg);
uint8_t orders_get_barrier_index(enum Orders_Barrier_Device barrier_device_arg, uint8_t customer_barrier_arg);

// SENSORS ANALOG INPUT
#define SENSORS_IN_PORT   PORTA
#define SENSORS_IN_DDR    DDRA
#define SENSORS_IN_A      PA0
#define SENSORS_IN_B      PA1
#define SENSORS_IN_C      PA2

#define KEY_DDR     DDRB
#define KEY_PORT    PORTB
#define KEY_PIN     PINB
#define KEY0        PB0   //Button on PB0
#define KEY1        PB1   //Button on PB1

// -- END OF EDITABLE

volatile uint16_t sensors_sensors[3];
volatile uint8_t  sensors_decision;

// PRIV
void sensors_read_adc();
// 
void sensors_init();
uint8_t get_key_press( uint8_t key_mask );
void buttons_init_io(void);

uint8_t debounce_cnt;
volatile uint8_t key_press;
uint8_t key_state;

#endif /* SENSORS_H_ */