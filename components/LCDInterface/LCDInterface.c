/**
 * @file LCDInterface.c
 * @author Dmitri Lyalikov
 * @brief LCD Driver Source
 * @version 0.1
 * @date 2022-03-02
 * @copyright Copyright (c) 2022
 * 
 */
#include <LCDinterface.h>

void lcd_command(unsigned char cmd, bool type)
{

	ldata = (cmd & 0xf0);
	ldata = ldata >> 4;
	set_nibble(ldata); //Sending higher nibble of command

	gpio_set_level(RS_GPIO, type);

	gpio_set_level(E_GPIO, 1);
	vTaskDelay(10 / portTICK_PERIOD_MS);
	gpio_set_level(E_GPIO, 0);

	vTaskDelay(1 / portTICK_PERIOD_MS);

	ldata = (cmd & 0x0f);

	set_nibble(ldata); //Sending lower nibble of command

	gpio_set_level(E_GPIO, 1);
	vTaskDelay(10 / portTICK_PERIOD_MS);
	gpio_set_level(E_GPIO, 0);

	vTaskDelay(3 / portTICK_PERIOD_MS);
}

void set_nibble(unsigned char c)
{
	for (int i = 0; i < 4; ++i)
	{
		int b = ((c >> i) & 1);
		gpio_set_level(pins[i + 2], b);
	}
}

void setupLCD(uint8_t setPins[6])
{
	E_GPIO = setPins[0];
	RS_GPIO = setPins[1];
	D4_GPIO = setPins[2];
	D5_GPIO = setPins[3];
	D6_GPIO = setPins[4];
	D7_GPIO = setPins[5];

	for (int i = 0; i < 6; ++i)
	{
		pins[i] = setPins[i];
		/* Setting pins as GPIO */
		gpio_pad_select_gpio(pins[i]);

		/* Set the GPIO as a push/pull output */
		gpio_set_direction(pins[i], GPIO_MODE_OUTPUT);

		gpio_set_level(pins[i], 0);
	}

	vTaskDelay(15 / portTICK_PERIOD_MS); //power on delay

	lcd_command(0x02, false); // 4bit mode on

	lcd_command(0x28, false); // init 5*7 matrix with two rows
	lcd_command(0x01, false); // clear display
	lcd_command(0x0C, false); // cursor off
	lcd_command(0x06, false); // shift cursor to right
}

void printLCD(const char *msg)
{
	while ((*msg) != 0)
	{
		lcd_command(*msg, true);
		msg++;
	}
}

void setCursor(uint8_t x, uint8_t y){
	lcd_command(0x02,false); //Resetting cursor
	for (int8_t i = 0; i < (y*40+x); ++i){  //Cursor moves to second row after 40th digit
		lcd_command(0x14,false);
	}
}

void scrollLCD(uint8_t amount){
	for (uint8_t i = 0; i < amount; ++i){
		lcd_command(0x18,false);
	}
}

void customChar(char *bits ,uint8_t addr){
	lcd_command(0x40+addr*8,false); //Custom character takes 8 CRAM locations

	for (int i = 0; i < 8; ++i ){
		lcd_command(bits[i], true);
	}
	lcd_command(0x80,false);
}
