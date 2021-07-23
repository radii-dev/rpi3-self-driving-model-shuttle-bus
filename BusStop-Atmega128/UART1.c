#include <avr/io.h>



void UART1_init(void)

{
	UBRR1H = 0x00;			// 9600 보율로 설정
	UBRR1L = 207;

	UCSR1A |= (1 << U2X1);			// 2배속 모드
	// 비동기, 8비트 데이터, 패리티 없음, 1비트 정지 비트 모드
	UCSR1C |= 0x06;

	UCSR1B |= (1 << RXEN1);		// 송수신 가능
	UCSR1B |= (1 << TXEN1);
	UCSR1B |= (1 << RXCIE1); // 송수신 완료 인터럽트 허용
	UCSR1B |= (1 << TXCIE1);
}

void UART1_transmit(char data)
{
	while( !(UCSR1A & (1 << UDRE1)) );	// 송신 가능 대기
	UDR1 = data;				// 데이터 전송
}

unsigned char UART1_receive(void)
{
	while( !(UCSR1A & (1<<RXC1)) );	// 데이터 수신 대기
	return UDR1;
}

void UART1_string_receive(char *str)
{
	int i;
	char data = UART1_receive();
	for(i = 0; data != '\r'; i++)
	{
		str[i] = data;
		data = UART1_receive();
	}
}

void UART1_print_string(char *str)	// 문자열 송신
{
	for(int i = 0; str[i]; i++)			// ‘\0’ 문자를 만날 때까지 반복
	UART1_transmit(str[i]);			// 바이트 단위 출력
}

void UART1_print_1_byte_number(uint8_t n)
{
	char numString[4] = "0";
	int i, index = 0;

	if(n > 0){					// 문자열 변환
		for(i = 0; n != 0 ; i++)
		{
			numString[i] = n % 10 + '0';
			n = n / 10;
		}
		numString[i] = '\0';
		index = i - 1;
	}

	for(i = index; i >= 0; i--)		// 변환된 문자열을 역순으로 출력
	UART1_transmit(numString[i]);
}

void UART0_init(void)

{
	UBRR0H = 0x00;			// 9600 보율로 설정
	UBRR0L = 207;

	UCSR0A |= (1 << U2X0);			// 2배속 모드
	// 비동기, 8비트 데이터, 패리티 없음, 1비트 정지 비트 모드
	UCSR0C |= 0x06;

	UCSR0B |= (1 << RXEN0);		// 송수신 가능
	UCSR0B |= (1 << TXEN0);
	UCSR0B |= (1 << RXCIE0); // 송신 완료 인터럽트 허용
}

void UART0_transmit(char data)
{
	while( !(UCSR0A & (1 << UDRE0)) );	// 송신 가능 대기
	UDR0 = data;				// 데이터 전송
}

unsigned char UART0_receive(void)
{
	while( !(UCSR0A & (1<<RXC0)) );	// 데이터 수신 대기
	return UDR0;
}