/*
 * stepper.c
 *
 * Created: 22.12.2016 23:33:52
 *  Author: lukasz.jamroz
 */ 
#include "stepper.h"

void stepper_add_motor(enum StepperIndexes index_arg, volatile uint8_t* port_arg, enum StepperPlacement placement_arg)
{
	stepper_motors_table[index_arg].index = index_arg;
	stepper_motors_table[index_arg].port = port_arg;
	stepper_motors_table[index_arg].placement = placement_arg;
	stepper_motors_table[index_arg].step = 0;
	stepper_motors_table[index_arg].speed = 0xFF;
	stepper_motors_table[index_arg].limit = 108; // If faster (lower number), the motor wont move
}
void stepper_set_speed(enum StepperIndexes index_arg, uint8_t speed_arg)
{
	stepper_motors_table[index_arg].limit = speed_arg;
}
void stepper_init()
{	
	TIMSK |= (1<<TOIE0);
	TCNT0 = 1;
	TCCR0 |= (1<<CS00); //interrupts and no prescaler so up to <<< 255 instructions
}
ISR(TIMER0_OVF_vect)
{
	for(uint8_t index = 0; index < STEPPER_NUM_OF_MOTORS; index++)
	{
		if(stepper_motors_table[index].timer == stepper_motors_table[index].speed) 
		{
			if(stepper_motors_table[index].orders_head != 0)
			{
				uint8_t temp = stepper_motors_orders[stepper_motors_table[index].orders_head].barrier.value;
				enum Orders_Barrier_Device dev = stepper_motors_orders[stepper_motors_table[index].orders_head].barrier.device;
				
				temp = orders_get_barrier(dev, temp);
				
				if(temp == 0){
					 stepper_process_motor(index);
				}
			}
			stepper_motors_table[index].timer = 0;
		}else
			stepper_motors_table[index].timer++;
	}
	for(uint8_t index = 0; index < SERVO_NUM_OF_SERVOS; index++)
	{
		if(servo_servos_table[index].orders_head != 0)
		{
			servo_process_servo(index);
		}
	}
}
void stepper_process_motor(uint8_t index_arg)
{
	uint8_t stepper_a, stepper_b, stepper_c, stepper_d;
	if(stepper_motors_table[index_arg].placement == MSB)
	{
		stepper_a = 4;
		stepper_b = 5;
		stepper_c = 6;
		stepper_d = 7;
	}else
		{
			stepper_a = 0;
			stepper_b = 1;
			stepper_c = 2;
			stepper_d = 3;
		}
	
	switch(stepper_motors_table[index_arg].mode)
	{
		case PHASE_A:
		*stepper_motors_table[index_arg].port &= ~(1<<stepper_c);
		*stepper_motors_table[index_arg].port |= (1<<stepper_a);
		break;
		
		case PHASE_B:
		*stepper_motors_table[index_arg].port &= ~(1<<stepper_d);
		*stepper_motors_table[index_arg].port |= (1<<stepper_b);
		break;
		
		case PHASE_C:
		*stepper_motors_table[index_arg].port &= ~(1<<stepper_a);
		*stepper_motors_table[index_arg].port |= (1<<stepper_c);
		break;
		
		case PHASE_D:
		*stepper_motors_table[index_arg].port &= ~(1<<stepper_b);
		*stepper_motors_table[index_arg].port |= (1<<stepper_d);
		break;
		
		default: break;
	}
	
	if(stepper_motors_orders[stepper_motors_table[index_arg].orders_head].direction == CLOCKWISE) //this double if is just mode movement
	{
		if(stepper_motors_table[index_arg].mode == PHASE_D) 
		{
			stepper_motors_table[index_arg].mode = PHASE_A;
		}
		else 
			stepper_motors_table[index_arg].mode++;
	}else
		if(stepper_motors_orders[stepper_motors_table[index_arg].orders_head].direction == COUNTERCLOCKWISE)
		{
			if(stepper_motors_table[index_arg].mode == PHASE_A)
			{
				stepper_motors_table[index_arg].mode = PHASE_D;
			}
			else
				stepper_motors_table[index_arg].mode--;
		}
	if(stepper_motors_table[index_arg].step == (stepper_motors_orders[stepper_motors_table[index_arg].orders_head].move+4)) //to doo
	{
		*stepper_motors_table[index_arg].port &= ~((1<<stepper_a)|(1<<stepper_b)|(1<<stepper_c)|(1<<stepper_d)); //must be here because of a-d shortcuts
		stepper_pop_front_order(index_arg);

	}else
		if(stepper_motors_orders[stepper_motors_table[index_arg].orders_head].move <= 65531)
		{
			stepper_motors_table[index_arg].step++;
		}
	
	if( stepper_motors_table[index_arg].speed > stepper_motors_table[index_arg].limit ) stepper_motors_table[index_arg].speed--; //limit and speed a little inconsistent namespace
}
void stepper_fastest_and_init_motors()
{
	uint8_t fastest_speed = 255;
	for(uint8_t index = 0; index < STEPPER_NUM_OF_MOTORS; index++)
	{
		if( stepper_motors_table[index].speed < fastest_speed )
		{
			fastest_speed = stepper_motors_table[index].speed;
		}
		stepper_motors_table[index].orders_head = 0;
		stepper_motors_table[index].timer = 0;
	}
	fastest_speed = fastest_speed;
	
	for(uint8_t i = 0; i < STEPPER_ORDERS_TOTAL_NUMBER; i++)
	{
		stepper_motors_orders[i].barrier.value = 0;
		stepper_motors_orders[i].direction = UNDEFINED;
		stepper_motors_orders[i].customer_index = 0;
		stepper_motors_orders[i].move = 0;
		stepper_motors_orders[i].next = 0;
	}
}

void stepper_pop_front_order(uint8_t index_arg)
{
	volatile uint8_t temp = stepper_motors_table[index_arg].orders_head;
	stepper_motors_orders[temp].move = 0;
	stepper_motors_orders[temp].barrier.value = 0;
	stepper_motors_orders[temp].direction = UNDEFINED;
	if(temp != 0)
	{
		stepper_motors_table[index_arg].speed = 0xFF;
		if(stepper_motors_orders[temp].next != 0)
		{
			if(stepper_motors_orders[stepper_motors_orders[temp].next].direction == CLOCKWISE)
				stepper_motors_table[index_arg].mode = PHASE_A;
			else
				stepper_motors_table[index_arg].mode = PHASE_D;
		}
		
		stepper_motors_table[index_arg].step = 0;	
		stepper_motors_table[index_arg].timer = 0;
		stepper_motors_table[index_arg].orders_head = stepper_motors_orders[temp].next;
		
		if(stepper_motors_table[index_arg].placement == MSB) //port sequence after last order - no need to take power and heat engines
		{
			*stepper_motors_table[index_arg].port &= ~(0xF0);
		}else
			{
				*stepper_motors_table[index_arg].port &= ~(0xF);
			}  
	}
}
void stepper_push_back_order(uint8_t index_arg,uint16_t limit_arg,enum StepperDirection direction_arg,
							 uint8_t customer_barrier_arg, uint8_t customer_index_arg, enum Orders_Barrier_Device barrier_type_arg)
{	
	volatile uint8_t i;
	for(i=1;i<STEPPER_ORDERS_TOTAL_NUMBER;i++)
	{
		if(stepper_motors_orders[i].move == 0)
		{
			stepper_motors_orders[i].customer_index = customer_index_arg;
			stepper_motors_orders[i].next = 0;
			stepper_motors_orders[i].direction = direction_arg;
			stepper_motors_orders[i].move = limit_arg;
			stepper_motors_orders[i].barrier.device = barrier_type_arg; //here its not customers barrier
			stepper_motors_orders[i].barrier.value = orders_get_barrier_index(barrier_type_arg, customer_barrier_arg);
			break;
		}
	}
	volatile uint8_t temp = stepper_motors_table[index_arg].orders_head;	
	
	if(temp != 0)
	{
		while(stepper_motors_orders[temp].next != 0) temp = stepper_motors_orders[temp].next;
		stepper_motors_orders[temp].next = i;
	}
	else
		{
			if(stepper_motors_orders[i].direction == CLOCKWISE) 
				stepper_motors_table[index_arg].mode = PHASE_A;
			else 
				stepper_motors_table[index_arg].mode = PHASE_D;
			
			stepper_motors_table[index_arg].orders_head = i;
		}
}