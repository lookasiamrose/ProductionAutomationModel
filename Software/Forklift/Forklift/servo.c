/*
 * servo.c
 *
 * Created: 19.04.2017 18:43:15
 *  Author: lukasz.jamroz
 */ 
#include "servo.h"

void servo_init()
{
	for(uint8_t i = 0;i<SERVO_ORDERS_TOTAL_NUMBER; i++)
	{
		servo_orders_table[i].customer_index = 0;
		servo_orders_table[i].barrier.value = 0;
		servo_orders_table[i].next = 0;
		servo_orders_table[i].position = 0;
	}
	
	for(uint8_t index = 0;index<SERVO_NUM_OF_SERVOS;index++)
	{
		if( (servo_servos_table[index].port == &PORTD) && (servo_servos_table[index].pin == PD5) )
		{
			//Set pins as output and no pullup
			DDRD  |= (1<<servo_servos_table[index].pin);
			PORTD &= ~(1<<servo_servos_table[index].pin);
			OCR1A = servo_servos_table[index].actual_position;
			
			TCCR1A|=(1<<COM1A1)|(1<<WGM11);        //NON Inverted PWM
			TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10); //PRESCALER=64 MODE 14(FAST PWM)
		}
	}
	
	ICR1=4999;  //fPWM=50Hz (Period = 20ms Standard).
	TIMSK |= (1<<OCIE1A)|(1<<OCIE1B);
}
ISR(TIMER1_COMPA_vect)
{
	for(uint8_t index = 0;index<SERVO_NUM_OF_SERVOS;index++)
	{
		if(servo_servos_table[index].barrier_delay > 0){
			if( servo_servos_table[index].barrier_delay > 99 )
			{
				uint8_t temp = servo_orders_table[ servo_servos_table[index].orders_head ].next;
				servo_orders_table[ servo_servos_table[index].orders_head ].next = 0;
				servo_orders_table[ servo_servos_table[index].orders_head ].barrier.value = 0; //barrier.device to write next?
				servo_orders_table[ servo_servos_table[index].orders_head ].position = 0;
				
				servo_servos_table[index].orders_head = temp;
				servo_servos_table[index].actual_position = 0;
				servo_servos_table[index].barrier_delay = 0;
			}else
				servo_servos_table[index].barrier_delay++;
		}
	}
}
void servo_set_servo(enum ServoIndexes index_arg, uint16_t value)
{
	if( (servo_servos_table[index_arg].port == &PORTD) && (servo_servos_table[index_arg].pin == PD5) )
	{
		if(value != 0) OCR1A = value;
	}
}
void servo_add_servo(enum ServoIndexes index_arg, volatile uint8_t* port_arg, uint8_t pin_arg, uint16_t basic_position_arg)
{
	servo_servos_table[index_arg].index = index_arg;
	servo_servos_table[index_arg].actual_position = basic_position_arg;
	servo_servos_table[index_arg].pin = pin_arg;
	servo_servos_table[index_arg].port = port_arg;
	servo_servos_table[index_arg].barrier_delay = 0;
	
}
void servo_push_back_order(enum ServoIndexes index_arg,uint16_t position_arg, 
							uint8_t customer_barrier_arg, uint8_t customer_index_arg, enum Orders_Barrier_Device barrier_type_arg)
{
	uint8_t i;
	for(i = 1;i<SERVO_ORDERS_TOTAL_NUMBER; i++)
	{
		if(servo_orders_table[i].position == 0)
		{
			servo_orders_table[i].position = position_arg;
			servo_orders_table[i].customer_index = customer_index_arg;
			servo_orders_table[i].next = 0;
			servo_orders_table[i].barrier.device = barrier_type_arg;
			servo_orders_table[i].barrier.value = orders_get_barrier_index(barrier_type_arg, customer_barrier_arg);
			break;
		}
	}	
	volatile uint8_t temp = servo_servos_table[index_arg].orders_head;
	if(temp != 0)
	{
		while(servo_orders_table[temp].next != 0) temp = servo_orders_table[temp].next;
		servo_orders_table[temp].next = i;
	}else
		{
			servo_servos_table[index_arg].orders_head = i;
		}	
}
void servo_process_servo(enum ServoIndexes index_arg)
{
	uint8_t temp = servo_orders_table[ servo_servos_table[index_arg].orders_head ].barrier.value;
	enum Orders_Barrier_Device dev = servo_orders_table[ servo_servos_table[index_arg].orders_head ].barrier.device;
	
	temp = orders_get_barrier( dev, temp);
	
	if( (temp == 0) && (servo_servos_table[index_arg].barrier_delay == 0) )
	{
		servo_servos_table[index_arg].actual_position = servo_orders_table[ servo_servos_table[index_arg].orders_head ].position;
		servo_set_servo(index_arg,servo_servos_table[index_arg].actual_position);
		
		servo_servos_table[index_arg].barrier_delay = 1;
	}
}