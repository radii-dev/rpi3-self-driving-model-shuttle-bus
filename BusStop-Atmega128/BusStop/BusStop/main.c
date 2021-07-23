#define F_CPU 16000000L
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "UART1.h"
#include "CLCD.h"

volatile uint8_t gindex = 0;
volatile uint8_t result_index = 0;
volatile uint8_t receive_complete = 0;
char compare[] = "GPGGA,";
volatile char result[100];
ISR(USART0_RX_vect)
{
	char data = UDR0;
	
	if(data == compare[gindex])
		gindex++;
	else if(data == 'E') {
		receive_complete = 1;
		result_index = 0;
		gindex = 0;
	}
	else if(gindex > 5) {
		result[result_index] = data;
		result_index++;
	}
	else
		gindex = 0;
}

void str_clean(char *str)
{
	for(int i = 0; str[i] != '\0'; i++)
	str[i] = '\0';
}

void atCommand(char* command){
	UART1_string_transmit(command);
	_delay_ms(2000);
}

void ESP_init(void){
	atCommand("AT+RST\r\n");
	atCommand("AT+CWMODE=1\r\n");
}

void connectToWiFi(void){
	atCommand("AT+CWJAP=\"Mediv\",\"11112345\"\r\n");
	_delay_ms(10000);
	atCommand("AT+CIFSR\r\n");
}

void startTCPConnection(){
	_delay_ms(5000);
	atCommand("AT+CIPSTART=\"TCP\",\"api.openweathermap.org\",80\r\n");
}

void readJSON(char* tmp){
	int count = 0;
	UART1_string_transmit("AT+CIPSEND=138\r\nGET /data/2.5/weather?APPID=9eb99614296bd4ca54f319f748a35119&id=1897000 HTTP/1.1\r\nHost: api.openweathermap.org\r\nConnection: keep-alive\r\n\r\n");
	
	char data = UART1_receive();
	int i = 0;
	while(i < 1000){
		if(i >= 4){
			tmp[0] = tmp[1];
			tmp[1] = tmp[2];
			tmp[2] = tmp[3];
			tmp[3] = data;
			data = UART1_receive();
		}
		else{
			tmp[i] = data;
			data = UART1_receive();
		}
		if(tmp[0] == 'm' && tmp[1] == 'a' && tmp[2] == 'i' && tmp[3] == 'n'){
			while(1)
			{
				count++;
				tmp[0] = tmp[1];
				tmp[1] = tmp[2];
				tmp[2] = tmp[3];
				tmp[3] = data;
				if(count == 7)	return;
				data = UART1_receive();
			}
		}
		i++;
	}
}

void closeTCPConnection(){
	_delay_ms(5000);
	atCommand("AT+CIPCLOSE\r\n");
}

void parsingGPS(float* coord){
	char gps[100] = {0,};
	memcpy(gps, (char*)result, 100);
	memset((char*)result, 0, 100);
	char *ptrlat = NULL, *ptrlon = NULL;
	char ptrlat_2[8], ptrlon_2[8];
	memset(ptrlat_2, 0, 8);
	memset(ptrlon_2, 0, 8);
	ptrlat = strtok(gps, ","); // 시간
	ptrlat = strtok(NULL, ","); // latitude
	ptrlon = strtok(NULL, ","); // N
	ptrlon = strtok(NULL, ","); // longitude
	memcpy(ptrlat_2, ptrlat + 2, 8);
	memcpy(ptrlon_2, ptrlon + 3, 8);
	for(int i = 2; ptrlat[i] != '\0'; i++) ptrlat[i] = '\0';
	for(int i = 3; ptrlon[i] != '\0'; i++) ptrlon[i] = '\0';
	float lat = atof(ptrlat);
	float lat_2 = atof(ptrlat_2);
	float lon = atof(ptrlon);
	float lon_2 = atof(ptrlon_2);
	coord[0] = lat + lat_2 / 60;
	coord[1] = lon + lon_2 / 60;
}

int main(void)
{
	char weather[5];
	char coordx[5];
	char coordy[6];
	
	UART0_init();
	UART1_init();
	LCD_init();
	// LCD_write_string("Hello World!");
	LCD_goto_XY(0, 5);
	LCD_write_string(", ");
	ESP_init();
	connectToWiFi();
	
	while(1)
	{		
		str_clean(weather);
		startTCPConnection();
		readJSON(weather);
		closeTCPConnection();
		if((weather[0] == 'R' && weather[1] == 'a' && weather[2] == 'i' && weather[3] == 'n') || (weather[0] == 'S' && weather[1] == 'n' && weather[2] == 'o' && weather[3] == 'w')){
			// 주차모드
			UART0_transmit('W');
		}
		else{
			// 주행모드
			UART0_transmit('w');
		}
		
		if(receive_complete == 1){ // gps 파싱
			float coord[2] = {0,};
			parsingGPS(coord);
			sprintf(coordx, "%.6f", coord[0]);
			sprintf(coordy, "%.6f", coord[1]);
			LCD_goto_XY(0, 0);
			LCD_write_string(coordx);
			LCD_goto_XY(0, 7);
			LCD_write_string(coordy);
			
			receive_complete = 0;
			// 서버로 GPS 보내기
		}
	}
	return 0;
}