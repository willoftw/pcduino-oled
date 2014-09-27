/*
 * gpio.h
 * This file is part of EDAMS
 *
 * Copyright (C) 2012 - Alexandre Dussart
 *
 * EDAMS is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * EDAMS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with EDAMS. If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __GPIO_H
#define __GPIO_H

#include <stdint.h>
#include <stdlib.h>
#include <cstring>

#undef MAX_GPIO_NUM
#define MAX_GPIO_NUM 21

#undef MAX_GPIO_MODE_NUM
#define MAX_GPIO_MODE_NUM 8

#undef MAX_PWM_NUM
#define MAX_PWM_NUM 5

#undef MAX_ADC_NUM
#define MAX_ADC_NUM 11

#define GPIO0  0
#define GPIO1  1
#define GPIO2  2
#define GPIO3  3
#define GPIO4  4
#define GPIO5  5
#define GPIO6  6
#define GPIO7  7
#define GPIO8  8
#define GPIO9  9
#define GPIO10  10
#define GPIO11  11
#define GPIO12  12
#define GPIO13  13
#define GPIO14  14
#define GPIO15  15
#define GPIO16  16
#define GPIO17  17
#define GPIO18  18
#define GPIO19  19
#define GPIO20  20
#define GPIO21  21
#define GPIO22  22
#define GPIO23  23

#define A0 0
#define A1 1
#define A2 2
#define A3 3
#define A4 4
#define A5 5
#define A6 6
#define A7 7
#define A8 8
#define A9 9
#define A10 10
#define A11 11

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

#define HIGH 0x1
#define LOW  0x0


#define IO_SPI_FUNC   2
#define IO_SPIEX_FUNC 3
#define IO_UART_FUNC  3


/*
typedef enum
{
	DIRECTION_ERROR = -1,
	INPUT = 0,
	OUTPUT = 1,
} gpio_mode;


typedef enum
{
	LEVEL_ERROR = -1,
	LOW = 0,
	HIGH = 1,
} gpio_state;
*/


typedef enum
{
	WHITE_LED,		//GPIO2
	BLUE_LED,			//GPIO3
	GREEN_LED,		//GPIO4
	YELLOW_LED,		//GPIO5
	ORANGE_LED,		//GPIO6
	RED_LED				//GPIO7
} Gpio_Led;



extern int gpio_pin_fd[MAX_GPIO_NUM+1];
extern int gpio_mode_fd[MAX_GPIO_NUM+1];

void gpio_init();
void gpio_shutdown();
void gpio_digital_write(uint8_t pin, uint8_t state);
int gpio_digital_read(uint8_t pin);
void gpio_pin_mode(uint8_t pin, uint8_t mode);
void gpio_hw_pin_mode(uint8_t pin, uint8_t mode);

void led_high(int led);
void led_low(int led);

void delay(unsigned long ms);
void delay_us(unsigned int us);

#define LSBFIRST 0
#define MSBFIRST 1

// Progmem is Arduino-specific
#define _BV(x) (1<<(x))
typedef uint16_t prog_uint16_t;
#define printf_P printf
#define strlen_P strlen
#define pgm_read_word(p) (*(p))
#define pgm_read_byte(p) (*(p))
#define PRIPSTR "%s"
#define PSTR(x) (x)
#define PROGMEM

 #endif //__GPIO_H
