#define F_CPU 8000000UL
#define BAUD 9600L 
#define UBRRL_value (F_CPU/(BAUD*16))-1
//#define UBRRL_value 25

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#define THERM_PORT PORTD
#define THERM_DDR DDRD
#define THERM_PIN PIND
#define THERM_DQ PD2

#define THERM_INPUT_MODE() THERM_DDR&=~(1<<THERM_DQ)
#define THERM_OUTPUT_MODE() THERM_DDR|=(1<<THERM_DQ)
#define THERM_LOW() THERM_PORT&=~(1<<THERM_DQ)
#define THERM_HIGH() THERM_PORT|=(1<<THERM_DQ)

int a=1;

void lamp_off()
{
		PORTD &= 0b111110111;
}

void lamp_on()
	{
		PORTD |= 0b00001000;	
	}

uint8_t therm_reset(){
	uint8_t i;
	THERM_LOW();
	THERM_OUTPUT_MODE();
	_delay_us(480);
	THERM_INPUT_MODE();
	_delay_us(60);
	i=(THERM_PIN & (1<<THERM_DQ));
	_delay_us(420);
	return i;
}

void therm_write_bit(uint8_t bit){
	THERM_LOW();
	THERM_OUTPUT_MODE();
	_delay_us(2);
	if(bit) THERM_INPUT_MODE();
	_delay_us(60);
	THERM_INPUT_MODE();
}

uint8_t therm_read_bit(void){
	uint8_t bit=0;
	THERM_LOW();
	THERM_OUTPUT_MODE();
	_delay_us(2);
	THERM_INPUT_MODE();
	_delay_us(16);
	if(THERM_PIN&(1<<THERM_DQ)) bit=1;
	_delay_us(45);
	return bit;
}

uint8_t therm_read_byte(void){
	uint8_t i=8, n=0;
	while(i--){
		n>>=1;
		n|=(therm_read_bit()<<7);
	}
	return n;
}

void therm_write_byte(uint8_t byte){
	uint8_t i=8;
	while(i--){
		therm_write_bit(byte&1);
		byte>>=1;
	}
}

#define THERM_CMD_CONVERTTEMP 0x44
#define THERM_CMD_RSCRATCHPAD 0xbe
#define THERM_CMD_SEARCHROM 0xf0
#define THERM_CMD_READROM 0x33
#define THERM_CMD_MATCHROM 0x55
#define THERM_CMD_SKIPROM 0xcc

#define THERM_DECIMAL_STEPS_12BIT 625 //.0625

//int test = 9;
//int test2 = 9;
uint8_t rom[8];
int sign = 1;
char temp;
char temp2;

uint8_t temperature[2];
int8_t digit;
uint16_t decimal;


void therm_read_temperature(char *buffer){
	therm_reset();
	therm_write_byte(THERM_CMD_SKIPROM);
	therm_write_byte(THERM_CMD_CONVERTTEMP);
	while(!therm_read_bit());
	therm_reset();
	therm_write_byte(THERM_CMD_SKIPROM);
	therm_write_byte(THERM_CMD_RSCRATCHPAD);
	temperature[0]=therm_read_byte();
	temperature[1]=therm_read_byte();
	therm_reset();
	
	digit=temperature[0]>>4;
	digit|=(temperature[1]&0x7)<<4;
	temp = temperature[0];
	temp2 = temperature[1];
	
	//digit=temperature[0]>>1;
	decimal=temperature[0]&0xf;
	//digit;
	decimal*=62;
	sign = 1;
	
	if ((temperature[1]&0xF8) == 0xF8)
	{
		digit=127-digit;
		decimal= (1000-decimal)%1000;
		sign = 0;
	}
	//decimal*=THERM_DECIMAL_STEPS_12BIT;
	//sprintf(buffer, "%+d.%04u C", digit, decimal);
}


void init_USART() {
	UBRRL = UBRRL_value;
	UBRRH = (UBRRL_value) >> 8;
	UCSRB = (1<<TXEN);
	UCSRC = (1<< UCSZ0)|(1<< UCSZ1)|(1<< URSEL);
}


void send_UART(char value) {
	while(!( UCSRA & (1 << UDRE))); 
	UDR = value;
}



int main(void)
{
	DDRD |= 0b00111000;
	PORTD |= 0b00010000;
	init_USART();

	//char buffer[15];

	int8_t digit1[8];
	int8_t digit2[8];

	while(1)
	{
		PORTD |= 0b00010000;
		PORTD &= 0b11011111;
		lamp_on();
		_delay_ms(100);
		/*int a1=0;
		int status = 0;
		
		while (status==0)
		{
			a1=0;
			THERM_LOW();
			THERM_OUTPUT_MODE();
			_delay_us(500);
			THERM_INPUT_MODE();
			_delay_us(15);
			
			while ((a1<500)&(status==0))
			{
				a1++;
				if (!(THERM_PIN & (1<<THERM_DQ))) status=1;
				_delay_us(1);
			}
			if (!(THERM_PIN & (1<<THERM_DQ))) status=1;
		}
		
		
		
		lamp_on();
		_delay_us(420);
		send_UART(48+(a1/100));
		send_UART(48+(a1%100/10));
		send_UART(48+(a1%10));
		send_UART(0x2E);*/
		
		while (therm_reset()) {};
		therm_write_byte(THERM_CMD_READROM);
		rom[0]=therm_read_byte();
		rom[1]=therm_read_byte();
		rom[2]=therm_read_byte();
		rom[3]=therm_read_byte();
		rom[4]=therm_read_byte();
		rom[5]=therm_read_byte();
		rom[6]=therm_read_byte();
		rom[7]=therm_read_byte();
		
		for (int u=0;u<8;u++)
		{
			digit2[u]=rom[u]&0b00001111;
			digit1[u]=(rom[u]&0b11110000)>>4;
		}
		
		for (int v=0;v<8;v++)
		{
			if (digit1[v]<10) send_UART(48+digit1[v]); else send_UART(55+digit1[v]);
			if (digit2[v]<10) send_UART(48+digit2[v]); else send_UART(55+digit2[v]);
			//send_UART(68+(digit2[v]));
		}
		
		

		//send_UART(0x2E);
		//send_UART(49);
		//send_UART(48+((decimal%100)/10));
		//send_UART(48+(decimal%10));
		PORTD |= 0b00100000;
		PORTD &= 0b11101111;
		for (int w=0;w<50;w++)
		{
		lamp_off();
		_delay_ms(50);
		lamp_on();
		_delay_ms(50);
		}
		_delay_ms(250);
	}
}