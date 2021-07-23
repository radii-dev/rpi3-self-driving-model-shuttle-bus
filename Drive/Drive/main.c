#define F_CPU 16000000L
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>

void UART1_init(void)
{
	UBRR1H = 0x00;
	UBRR1L = 16;

	UCSR1A |= _BV(U2X1);
	UCSR1C |= 0x06;

	UCSR1B |= _BV(RXEN1);
	UCSR1B |= _BV(TXEN1);
}

unsigned char UART1_receive(void)
{
	while( !(UCSR1A & (1<<RXC1)) );
	return UDR1;
}

void UART1_transmit(char data)
{
	while( !(UCSR1A & (1 << UDRE1)) );
	UDR1 = data;
}

void UART1_print_string(char *str)
{
	for(int i = 0; str[i]; i++)
	UART1_transmit(str[i]);
}

void servo_init()
{
	TCCR1A |= (1<<COM1A1) | (1<<WGM11);
	TCCR1B |= (1<<WGM12) | (1<<WGM13) | (1<<CS11) | (1<<CS10);
	ICR1 = 4999;
}

void DC_init()
{
	TCCR0|=(1<<CS02)|(1<<CS01)|(1<<CS00);//분주비 1024
	TCCR0|=(1<<WGM01)|(1<<WGM00);//고속 PWM 모드
	TCCR0|=(1<<COM01);
	
	OCR0 = 150;
}

void servo_angle(float angle){
	OCR1A = (angle+5)*225/90 + 375 + 0.5 + 25;
}

int main(void)
{
	DDRA = 0xFF;
	DDRB = 0xFF;
	DDRF = 0xFF;
	PORTA = 0xFF;
	PORTB = 0x00;
	char buffer;
	int buffer_int = 0;

	UART1_init();
	servo_init();
	DC_init();
	servo_angle(40);
	
	while(1) {
		buffer = UART1_receive();
//		PORTF = buffer;
		buffer_int = (int)buffer;
		if(buffer_int > 127){
			PORTA = 0b10101010;
			servo_angle(-5*buffer_int+780);
		}
		else if(buffer_int <= 127){
			PORTA = 0b11111111;
		}
	}
	return 0;
}
