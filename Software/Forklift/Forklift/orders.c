/*
 * orders.c
 *
 * Created: 13.07.2017 20:27:05
 *  Author: lukasz.jamroz
 */ 

#include "orders.h"


volatile uint8_t ticker1;

ISR(TIMER1_COMPB_vect)
{
	orders_refresh_devices_values();
	orders_timer_vector_function();
	orders_process_sensors();
	
	if(ticker1 > 50)
	{
		ticker1 = 0;
		//UDR = sensors_decision;
	}else
		ticker1++;
}

void orders_timer_vector_function()
{
	static uint8_t ct0, ct1;
	uint8_t i;
	
	i = key_state ^ ~KEY_PIN;    // key changed ?
	ct0 = ~( ct0 & i );          // reset or count ct0
	ct1 = ct0 ^ (ct1 & i);       // reset or count ct1
	i &= ct0 & ct1;              // count until roll over ?
	key_state ^= i;              // then toggle debounced state
	key_press |= key_state & i;  // 0->1: key press detect
}
void orders_refresh_devices_values()
{
	volatile uint8_t temp = 0;
	if( get_key_press(1<<KEY0) ) 
	{
		temp = 1;
	}
	orders_sensors_devices[FRONT_SWITCH].actual_value = temp;
	orders_sensors_devices[GRAYSCALE_LEFT].actual_value = sensors_decision & 0b0001;
	orders_sensors_devices[GRAYSCALE_CENTER].actual_value = (sensors_decision & 0b0010) >> 1;
	orders_sensors_devices[GRAYSCALE_RIGHT].actual_value = (sensors_decision & 0b0100) >> 2;
}
uint8_t orders_get_barrier(enum Orders_Barrier_Device barrier_device_arg, uint8_t barrier_index_arg)
{
	if(barrier_index_arg == 0)
		return 0;
		
	if(barrier_device_arg == SENSOR_DEVICE)
	{
		return orders_sensors_orders[barrier_index_arg].order;
	}else
	if(barrier_device_arg == STEPPER_DEVICE)
	{
		return stepper_motors_orders[barrier_index_arg].move;
	}else
	if(barrier_device_arg == SERVO_DEVICE)
	{
		return servo_orders_table[barrier_index_arg].position;
	}
	
	return 0;
}
uint8_t orders_get_barrier_index(enum Orders_Barrier_Device barrier_device_arg, uint8_t customer_barrier_arg)
{
	uint8_t i;
	if(customer_barrier_arg == 0)
	return 0;
	
	if(barrier_device_arg == SENSOR_DEVICE)
	{
		for(i=1;i<ORDERS_TOTAL_SENSORS_ORDERS_NUMBER; i++)
		{
			if(orders_sensors_orders[i].super.index == customer_barrier_arg)
			{
				customer_barrier_arg = i;
				break;
			}
		}
	}else
	if(barrier_device_arg == STEPPER_DEVICE)
	{
		for(i=1;i<STEPPER_ORDERS_TOTAL_NUMBER; i++)
		{
			if(stepper_motors_orders[i].customer_index == customer_barrier_arg)
			{
				customer_barrier_arg = i;
				break;
			}
		}
	}else
	if(barrier_device_arg == SERVO_DEVICE)
	{
		for(i=1;i<SERVO_ORDERS_TOTAL_NUMBER; i++)
		{
			if(servo_orders_table[i].customer_index == customer_barrier_arg)
			{
				customer_barrier_arg = i;
				break;
			}
		}
	}
	return customer_barrier_arg;
}
uint8_t orders_sensors_push_back_order_deprecated( enum Orders_Sensors_Index index_arg, uint8_t order_arg,
									 uint8_t customer_barrier_arg, uint8_t customer_index_arg,
									 enum Orders_Barrier_Device barrier_device_arg)
{
	uint8_t i;
	for(i = 1;i<ORDERS_TOTAL_SENSORS_ORDERS_NUMBER; i++)
	{
		if(orders_sensors_orders[i].order == 0)
			break;
	}
	if(i == ORDERS_TOTAL_SENSORS_ORDERS_NUMBER) // TABLE IS FULFILLED
		return 1;
	
	orders_sensors_orders[i].order = order_arg;
	orders_sensors_orders[i].super.index = customer_index_arg;
	orders_sensors_orders[i].super.next = 0;
	orders_sensors_orders[i].super.barrier.device = barrier_device_arg;
	orders_sensors_orders[i].super.barrier.value = orders_get_barrier_index( barrier_device_arg, customer_barrier_arg);
	
	/* ORDERS LIST PERSPECTIVE ABOVE, DEVICE LIST PERSPECTIVE BELOW */
	
	volatile uint8_t temp = orders_sensors_devices[index_arg].orders_head;
	if(temp != 0) //NOT EMPTY ORDERS LIST
	{
		while(orders_sensors_orders[temp].super.next != 0) temp = orders_sensors_orders[temp].super.next;
		orders_sensors_orders[temp].super.next = i;
	}else //EMPTY ORDERS LIST
	{
		orders_sensors_devices[index_arg].orders_head = i;
	}
	
	return 0;
}
void orders_process_sensor_deprecated(enum Orders_Sensors_Index index_arg)
{	
	uint8_t temp = orders_sensors_orders[ orders_sensors_devices[index_arg].orders_head ].super.barrier.value;
	if (temp != 0)
	{
		temp = orders_sensors_orders[temp].order; //if barrier exists go for value in there
	}
	
	if(temp == 0)
	{
		orders_sensors_orders[ orders_sensors_devices[index_arg].orders_head ].order >>= 1;
		uint8_t requirement = orders_sensors_orders[ orders_sensors_devices[index_arg].orders_head ].order & 1;
		if(requirement == orders_sensors_devices[index_arg].actual_value)
		{
			//orders_sensors_pop_front_order(index_arg);
		}
	}
}
void orders_sensors_pop_front_order()
{
	if(orders_sensors_orders_head != 0)
	{
		uint8_t temp = orders_sensors_orders[orders_sensors_orders_head].super.next;
		
		orders_sensors_orders[orders_sensors_orders_head].order = 0;
		orders_sensors_orders[orders_sensors_orders_head].super.index = 0;
		orders_sensors_orders[orders_sensors_orders_head].super.next = 0;
		orders_sensors_orders[orders_sensors_orders_head].super.barrier.value = 0;
		orders_sensors_orders[orders_sensors_orders_head].super.barrier.device = 0;
		
		orders_sensors_orders_head = temp;
	}
}
void orders_process_sensors()
{
	volatile uint8_t temp = orders_sensors_orders[ orders_sensors_orders_head ].super.barrier.value;
	if (temp != 0)
	{
		temp = orders_get_barrier(orders_sensors_orders[ orders_sensors_orders_head ].super.barrier.device, temp); //if barrier exists go for value in there
	}
	
	if(temp == 0)
	{
		volatile uint8_t requirement = orders_sensors_orders[ orders_sensors_orders_head ].order;
		volatile uint8_t resp = (requirement >> 4) & 0xF;
		requirement &= 0xF;
		
		volatile uint8_t fulfil = 1; 
		if( (resp & 0b0001) == 0b0001)
		{
			if( (requirement & 0b0001) != orders_sensors_devices[GRAYSCALE_LEFT].actual_value) fulfil = 0;
		}
		
		if( (resp & 0b0010) == 0b0010)
		{
			if( ((requirement & 0b0010)>>1) != orders_sensors_devices[GRAYSCALE_CENTER].actual_value) fulfil = 0;
		}
		
		if( (resp & 0b0100) == 0b0100)
		{
			if( ((requirement & 0b0100)>>2) != orders_sensors_devices[GRAYSCALE_RIGHT].actual_value) fulfil = 0;
		}
		
		if( (resp & 0b1000) == 0b1000)
		{
			if( ((requirement & 0b1000)>>3) != orders_sensors_devices[FRONT_SWITCH].actual_value) fulfil = 0;
		}
		if(fulfil == 1)
		{
			orders_sensors_pop_front_order();
		}
	}
	
}
uint8_t orders_sensors_push_back_order( uint8_t order_arg, uint8_t customer_barrier_arg, 
										uint8_t customer_index_arg, enum Orders_Barrier_Device barrier_device_arg)
{
	uint8_t i;
	for(i = 1;i<ORDERS_TOTAL_SENSORS_ORDERS_NUMBER; i++)
	{
		if(orders_sensors_orders[i].order == 0)
		break;
	}
	if(i == ORDERS_TOTAL_SENSORS_ORDERS_NUMBER) // TABLE IS FULFILLED
	return 1;
	
	orders_sensors_orders[i].order = order_arg;
	orders_sensors_orders[i].super.index = customer_index_arg;
	orders_sensors_orders[i].super.next = 0;
	orders_sensors_orders[i].super.barrier.device = barrier_device_arg;
	orders_sensors_orders[i].super.barrier.value = orders_get_barrier_index( barrier_device_arg, customer_barrier_arg);
	
	/* ORDERS LIST PERSPECTIVE ABOVE, ORDERS HEAD PERSPECTIVE BELOW */
	
	volatile uint8_t temp = orders_sensors_orders_head;
	if(temp != 0) //NOT EMPTY ORDERS LIST
	{
		while(orders_sensors_orders[temp].super.next != 0) temp = orders_sensors_orders[temp].super.next;
		orders_sensors_orders[temp].super.next = i;
	}else //EMPTY ORDERS LIST
	{
		orders_sensors_orders_head = i;
	}
	
	return 0;
}
enum Orders_Barrier_Device orders_get_barrier_from_char(char char_barrier_arg)
{
	enum Orders_Barrier_Device enum_barrier;
	switch(char_barrier_arg)
	{
		case 's':
		enum_barrier = STEPPER_DEVICE;
		break;
		case 'o':
		enum_barrier = SERVO_DEVICE;
		break;
		case 'b':
		enum_barrier = SENSOR_DEVICE;
		break;
		
		default:
		enum_barrier = NUM_OF_DIFF_DEVICES;
		break;
	}
	return enum_barrier;
}