#ifndef ONEWIRE_H
#define ONEWIRE_H
  
#include <util/delay.h>
#include "avrlibdefs.h"

// 1-wire connection pin
#define P_DS PA7

#define THERM_CMD_CONVERTTEMP 0x44
#define THERM_CMD_RSCRATCHPAD 0xbe
#define THERM_CMD_WSCRATCHPAD 0x4e
#define THERM_CMD_CPYSCRATCHPAD 0x48
#define THERM_CMD_RECEEPROM 0xb8
#define THERM_CMD_RPWRSUPPLY 0xb4
#define THERM_CMD_SEARCHROM 0xf0
#define THERM_CMD_READROM 0x33
#define THERM_CMD_MATCHROM 0x55
#define THERM_CMD_SKIPROM 0xcc
#define THERM_CMD_ALARMSEARCH 0xec
 
#define MAX_DEVICES 2 

// reset 1-wire network and determine the presence of any 1-wire slave device
u08 ow_reset(void)
{
	u08	pr, sreg;

	sreg = SREG;  cli();

	cbi(PORTA, P_DS);           
	sbi(DDRA, P_DS);

	_delay_us(480);
	cbi(DDRA, P_DS);
	_delay_us(70);

	pr = !(inb(PINA) & (1<<P_DS));
	SREG = sreg;
	_delay_us(410);
	return(pr);	
}						

// write bit to 1-wire network
static inline void write_bit(u08 bitval)
{
	u08 sreg;

	sreg= SREG; cli();
	cbi(PORTA, P_DS);              
	sbi(DDRA, P_DS);
	                       
	_delay_us(10);
	if(bitval) cbi(DDRA, P_DS);
	_delay_us(70);
	
	cbi(DDRA, P_DS);             
	SREG = sreg;
}

// write byte to 1-wire network
void write_byte(u08 val)
{
	for(u08 i = 0; i < 8; i++)	
		write_bit( (val>>i) & 0b01 );	
	sbi(PORTA, P_DS);               // raise the pin level immediately (for parasite power feed)
	sbi(DDRA, P_DS);

	_delay_us(120);
}

// read bit from 1-wire network
static inline u08 read_bit(void)
{
	u08 sreg, r;

	sreg = SREG; cli();

	cbi(PORTA, P_DS);
	sbi(DDRA, P_DS);
	nop();nop();nop();nop();nop();nop();
	cbi(DDRA, P_DS);   

	_delay_us(10);
	r = inb(PINA) & (1<<P_DS);
	SREG = sreg;
	return r;
}

// read byte from 1-wire network
u08 read_byte(void)
{
	u08 value = 0;

	for(u08 i = 0; i < 8; i++)
	{
		if(read_bit()) value |= (0b01 << i);
		_delay_us(120);
	}
	return(value);
}

// CRC8 calculation
u08 calc_crc(u08* buff, u08 num_vals)
{
	u08	shift_reg = 0, data_bit, sr_lsb, fb_bit;

	for(u08 i = 0; i < num_vals; i++)
	{
		for(u08 j = 0; j < 8; j++)
		{
			data_bit = (*buff >> j) & 0b01;
			sr_lsb = shift_reg & 0b01;
			fb_bit = (data_bit ^ sr_lsb) & 0b01;
			shift_reg = shift_reg >> 1;
			if(fb_bit)  shift_reg = shift_reg ^ 0x8C;
		}
		buff++;
	}

	return(shift_reg);
}

#endif
