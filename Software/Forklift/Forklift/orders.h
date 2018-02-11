/*
 * orders.h
 *
 * Created: 13.07.2017 20:27:16
 *  Author: lukasz.jamroz
 */ 
#ifndef ORDERS_H_
#define ORDERS_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include "stepper.h"

enum Orders_Sensors_Index{ GRAYSCALE_LEFT, GRAYSCALE_CENTER, GRAYSCALE_RIGHT, FRONT_SWITCH, NUM_OF_DIFF_SENSORS };

typedef struct Orders_Order
{
	uint8_t index;
	Orders_Barrier barrier;
	uint8_t next;
	
} Orders_Order;

typedef struct Orders_Boolean_Sensor
{
	Orders_Order super;
	uint8_t order;
} Orders_Boolean_Sensor;

/* DEVICE TYPE */

typedef struct Orders_Sensor_Device
{
	uint16_t actual_value;
	volatile uint8_t orders_head;
} Orders_Sensor_Device;

/* VARIABLE */
volatile Orders_Boolean_Sensor orders_sensors_orders[ORDERS_TOTAL_SENSORS_ORDERS_NUMBER];
volatile Orders_Sensor_Device orders_sensors_devices[NUM_OF_DIFF_SENSORS];
volatile uint8_t orders_sensors_orders_head;

/* FUNCTION */
void orders_timer_vector_function();
void orders_refresh_devices_values();
void orders_process_sensor_deprecated(enum Orders_Sensors_Index index_arg);
uint8_t orders_sensors_push_back_order_deprecated( enum Orders_Sensors_Index index_arg, uint8_t order_arg, 
									 uint8_t customer_barrier_arg, uint8_t customer_index_arg, 
									 enum Orders_Barrier_Device barrier_device_arg);
uint8_t orders_sensors_push_back_order( uint8_t order_arg, uint8_t customer_barrier_arg,
									 uint8_t customer_index_arg, enum Orders_Barrier_Device barrier_device_arg);
void orders_sensors_pop_front_order();
void orders_process_sensors();
enum Orders_Barrier_Device orders_get_barrier_from_char(char char_barrier_arg);

#endif /* ORDERS_H_ */