#include <iostream>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>

using namespace std;

void uart_ch(char ch)
{
	int fd ;

	if ((fd = serialOpen ("/dev/ttyAMA0", 9600)) < 0)
	{
		fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno));
		return;
	}
	serialPutchar(fd,ch);
	serialClose(fd);
}

void uart_str(char *str)
{
	while(*str) uart_ch(*str++);
}

int main()
{
	while(true)	{
		uart_ch('a');
	}
	return 0;
}

