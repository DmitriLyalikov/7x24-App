/**
 * @file LCDInterface.h
 * @author Dmitri Lyalikov
 * @brief LCD Driver Library
 * @version 0.1
 * @date 2022-03-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef LCDINTERFACE
#define LCDINTERFACE

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

uint8_t E_GPIO;
uint8_t RS_GPIO;
uint8_t D4_GPIO;
uint8_t D5_GPIO;
uint8_t D6_GPIO;
uint8_t D7_GPIO;

uint8_t pins[6];

char ldata;

/**
 * @brief LCD command/data writer
 *
 *        Sends command or data to LCD.
 *
 * @param  cmd  Passed command or data
 *
 * @param  type Defines if the passed char is written to 
 *              command or data register
 *              TRUE=data, FALSE = command
 */
void lcd_command(unsigned char cmd, bool type);

/**
 * @brief LCD command/data writer
 *
 *        Sends  nibble of a command/data to LCD as required
 *        by 4bit mode. 
 *
 * @param  bits Sended nibble with four MSB set as 0
 */
void set_nibble(unsigned char bits);

/**
 * @brief LCD initialization
 *
 *        Sets up LCD to work properly.
 *
 * @param  pins Array of pins that are used to pass data to LCD
 *              [E,RS,D4,D5,D6,D7]
 */
void setupLCD(uint8_t pins[6]);

/**
 * @brief Print on LCD
 *
 *        Prints passed message to LCD.
 *
 * @param  msg Message to be printed.
 */
void printLCD(const char *msg);

/**
 * @brief Change cursor location
 *
 *        Cursor moves to passed location.
 *
 * @param  x x-coordinate of a cursor
 * @param  y y-coordinate of a cursor
 */
void setCursor(uint8_t x, uint8_t y);

/**
 * @brief Scrolls screen
 *
 *        Scrolls view to left and changes cursors location to left.
 *
 * @param  amount How many character are moved
 */
void scrollLCD(uint8_t amount);

/**
 * @brief Custom character definition
 *
 *        Saves up to 8 custom characters to CGRAM and allows to use
 *        them later in code
 *
 * @param  bits Array of chars that defines custom character
 *              e.g https://maxpromer.github.io/LCD-Character-Creator/ can be used to create characters
 *
 * @param  addr number of CGRAM slot that character will be saved to (0-7)
 */
void customChar(char *bits, uint8_t addr);

#endif