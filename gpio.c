/*
 * gpio.c
 * This file is part of EDAMS
 *
 * Copyright (C) 2014 - Alexandre Dussart
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

#include <Ecore_File.h>

#include <fcntl.h>
#include <stdio.h>
#include <stropts.h>
#include <sys/stat.h>
#include <unistd.h>

#include "gpio.h"
#include "utils.h"

#define GPIO_MODE_DIR "/sys/devices/virtual/misc/gpio/mode/"
#define GPIO_PIN_DIR "/sys/devices/virtual/misc/gpio/pin/"
#define GPIO_IF_PREFIX "gpio"

#define ADC_IF "/proc/adc"

int gpio_pin_fd[MAX_GPIO_NUM+1] ={-1};
int gpio_mode_fd[MAX_GPIO_NUM+1] ={-1};

static int  write_to_file(int fd, char *str, int len);

static int
write_to_file(int fd, char *str, int len)
{
    int ret = -1;

	if(!fd) return -1;

    lseek(fd, 0, SEEK_SET);
    ret = write(fd, str, len);
    if ( ret <= 0 )
    {
        debug(MSG_ERROR, _("Impossible d'écrire sur le descripteur fichier '%d'"), fd);
        return -1;
    }
    return ret;
}



void
gpio_hw_pin_mode(uint8_t pin, uint8_t mode)
{
     char buf[4];
     int ret = -1;

	if(!gpio_mode_fd[pin]) return;

     if ( (pin >= 0 && pin <=  MAX_GPIO_NUM) && (mode <= MAX_GPIO_MODE_NUM) )
     {
         memset((void *)buf, 0, sizeof(buf));
         snprintf(buf, sizeof(buf), "%d", mode);
         ret = write_to_file(gpio_mode_fd[pin], buf, sizeof(buf));
         if ( ret <= 0 )
         {
			debug(MSG_ERROR, _("Impossible d'écrire l'état sur la brocher  '%d '"), pin);
			return;
         }
     }
     else
     {
         debug(MSG_ERROR, _("Broche '%d' inaccessible ou mode '%d' inexistant"), pin, mode);
			return;
      }
}


void
gpio_pin_mode(uint8_t pin, uint8_t mode)
{
   switch (mode)
    {
    case INPUT:
    case OUTPUT:
        gpio_hw_pin_mode(pin, mode);
        break;
    case INPUT_PULLUP:
        gpio_hw_pin_mode(pin, 8);
        break;

    default:
        break;
    }
}//gpio_pin_mode

//
//
//
void
gpio_digital_write(uint8_t pin, uint8_t value)
{
    char buf[4];
	int ret = -1;

	if(!gpio_pin_fd[pin]) return;

     if ( (pin >= 0 && pin <=  MAX_GPIO_NUM) && (value == HIGH || value == LOW) )
     {
         memset((void *)buf, 0, sizeof(buf));
         sprintf(buf, "%d", value);
         ret = write_to_file(gpio_pin_fd[pin], buf, sizeof(buf));
         if ( ret <= 0 )
         {
			debug(MSG_ERROR, _("Ecriture de l'état sur la broche '%d' impossible"),  pin);
             return;
         }
     }
     else
     {
		debug(MSG_ERROR, _("Broche '%d' non disponible, ou mode invalide"),  pin);
		return;
      }
}//gpio_digital_write


//
//
//
int
gpio_digital_read(uint8_t pin)
{
    char buf[4];
    int ret = -1;

	if(!gpio_pin_fd[pin]) return -1;

    if ( (pin >= 0) && (pin <= MAX_GPIO_NUM))
    {
        memset((void *)buf, 0, sizeof(buf));
        lseek(gpio_pin_fd[pin], 0, SEEK_SET);
        ret = read(gpio_pin_fd[pin], buf, sizeof(buf));

        if ( ret <= 0 )
        {
			debug(MSG_ERROR, _("Lecture de l'état de la broche '%d' impossible"),  pin);
			ret = -1;
        }

        ret = buf[0] - '0';
        switch( ret )
        {
            case LOW:
            case HIGH:
                break;
            default:
                ret = -1;
                break;
        }
    }
    else
    {
        debug(MSG_ERROR, _("Broche '%d' non disponible"),  pin);
        ret = -1;
    }
    return ret;
}//gpio_digital_read


//
//
//
void
gpio_test()
{
	unsigned int udelay = 2500;

	for(int x = 0; x <= 7 ; x++)
	{
		led_high(x);
		usleep(udelay);
		led_low(x);
		usleep(udelay);
	}
}//gpio_test

//
//
//
void
delay(unsigned long ms)
{
    usleep(ms*1000);
}

//
//
//
void
delay_us(unsigned int us)
{
    usleep(us);
}

//
//
//
void
gpio_init()
{
	int i;
	char path[1024];

	if(ecore_file_is_dir(GPIO_PIN_DIR) == false) 	return;
	if(ecore_file_is_dir(GPIO_MODE_DIR) == false) 	return;

	for( i = 0; i<= MAX_GPIO_NUM; ++i)
	{
		memset(path, 0, sizeof(path));
		snprintf(path, sizeof(path), "%s%s%d", GPIO_PIN_DIR, GPIO_IF_PREFIX, i);
		gpio_pin_fd[i] = open(path, O_RDWR);
		if ( gpio_pin_fd[i] < 0 )
		{
			debug(MSG_ERROR, _("Impossible d'ouvrir le répertoire '%s'"), path);
		}
		else
		{
			memset(path, 0, sizeof(path));
			snprintf(path, sizeof(path), "%s%s%d", GPIO_MODE_DIR, GPIO_IF_PREFIX, i);
			gpio_mode_fd[i] = open(path, O_RDWR);
			if ( gpio_mode_fd[i] < 0 )
			{
				debug(MSG_ERROR, _("Impossible d'ouvrir le répertoire '%s'"), path);
			}
		}
     }

	//Following are GPIO with leds connected:
	gpio_pin_mode(GPIO4, OUTPUT);
    gpio_pin_mode(GPIO5, OUTPUT);
    gpio_pin_mode(GPIO6, OUTPUT);
	gpio_pin_mode(GPIO7, OUTPUT);
    gpio_pin_mode(GPIO8, OUTPUT);
	gpio_pin_mode(GPIO9, OUTPUT);
}//gpio_init


//
//
//
void
gpio_shutdown()
{

}//gpio_shutdown


//
//
//
void
led_high(Gpio_Led led)
{
	if(led == WHITE_LED)
		gpio_digital_write(GPIO4, HIGH);
	else if(led == BLUE_LED)
		gpio_digital_write(GPIO5, HIGH);
	else if(led == GREEN_LED)
		gpio_digital_write(GPIO6, HIGH);
	else if(led == YELLOW_LED)
		gpio_digital_write(GPIO7, HIGH);
	else if(led == ORANGE_LED)
		gpio_digital_write(GPIO8, HIGH);
	else if(led == RED_LED)
		gpio_digital_write(GPIO9, HIGH);
}//led_high


//
//
//
void
led_low(Gpio_Led led)
{
	if(led == WHITE_LED)
		gpio_digital_write(GPIO4, LOW);
	else if(led == BLUE_LED)
		gpio_digital_write(GPIO5, LOW);
	else if(led == GREEN_LED)
		gpio_digital_write(GPIO6, LOW);
	else if(led == YELLOW_LED)
		gpio_digital_write(GPIO7, LOW);
	else if(led == ORANGE_LED)
		gpio_digital_write(GPIO8, LOW);
	else if(led == RED_LED)
		gpio_digital_write(GPIO9, LOW);
}//led_low
