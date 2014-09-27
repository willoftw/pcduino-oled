#ifndef _OLED__
#define _OLED__

#include <stdlib.h>
#include <cstring>

#define BLACK 0
#define WHITE 1

//common parameters
#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64
#define SSD1306_FBSIZE 1024 //128x8
#define SSD1306_MAXROW 8

//command macro
#define SSD1306_CMD_DISPLAY_OFF 0xAE//--turn off the OLED
#define SSD1306_CMD_DISPLAY_ON 0xAF//--turn on oled panel

//initialized the ssd1306 in the setup function
bool oled_init();
void oled_shutdown();

//update the framebuffer to the screen.
 void oled_update();

//totoally 8 rows on this screen in vertical direction.
 void oled_update_row(int rowIndex);
 void oled_update_row_from_start_to_end(int startRow, int endRow);

//draw one pixel on the screen.
 void oled_draw_pixel(int16_t x, int16_t y, uint16_t color);

//clear the screen
void oled_clear(bool isUpdateHW);		//définition C++:clear(bool isUpdateHW=false)


//GFX
void oled_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1,	uint16_t color);
void oled_draw_fast_hline(int16_t x, int16_t y,	int16_t h, uint16_t color);
void oled_draw_fast_vline(int16_t x, int16_t y,	int16_t h, uint16_t color);
void oled_fillrect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void oled_draw_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void oled_fillscreen(uint16_t color);
void oled_draw_triangle(int16_t x0, int16_t y0,	int16_t x1, int16_t y1,	int16_t x2, int16_t y2, uint16_t color);
void oled_draw_char(int16_t x, int16_t y, unsigned char c,   uint16_t color, uint16_t bg, uint8_t size);
size_t oled_write(uint8_t c);
size_t oled_println(const unsigned char c[]);
void oled_set_cursor(int16_t x, int16_t y);
void oled_set_text_color(uint16_t c);
void oled_set_text_size(uint8_t s);
void oled_set_rotation(uint8_t x);
void oled_draw_bmp(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1,const char BMP[]);

 #endif //_OLED_H_
