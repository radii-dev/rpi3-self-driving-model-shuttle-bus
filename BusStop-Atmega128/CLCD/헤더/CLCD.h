#ifndef CLCD_H_

#define CLCD_H_

#define PORT_DATA		PORTD		// 데이터 핀 연결 포트
#define PORT_CONTROL		PORTC		// 제어 핀 연결 포트
#define DDR_DATA		DDRD		// 데이터 핀의 데이터 방향
#define DDR_CONTROL		DDRC		// 제어 핀의 데이터 방향

#define RS_PIN			0		// RS 제어 핀의 비트 번호
#define RW_PIN			1		// R/W 제어 핀의 비트 번호
#define E_PIN			2		// E 제어 핀의 비트 번호

#define COMMAND_CLEAR_DISPLAY	0x01
#define COMMAND_8_BIT_MODE		0x38	// 8비트, 2라인, 5x8 폰트
#define COMMAND_4_BIT_MODE		0x28	// 4비트, 2라인, 5x8 폰트

#define COMMAND_DISPLAY_ON_OFF_BIT		2
#define COMMAND_CURSOR_ON_OFF_BIT		1
#define COMMAND_BLINK_ON_OFF_BIT		0

void LCD_pulse_enable(void);

void LCD_write_data(uint8_t data);

void LCD_write_command(uint8_t command);

void LCD_clear(void);

void LCD_init(void);

void LCD_write_string(char *string);

void LCD_goto_XY(uint8_t row, uint8_t col);

#endif