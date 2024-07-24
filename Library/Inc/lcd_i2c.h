#ifndef LCD_I2C_H
#define LCD_I2C_H

#include <stm32f4xx_hal.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

void lcd_write_nibble(uint8_t nibble, uint8_t rs);
void lcd_send_cmd(uint8_t cmd);
void lcd_send_data(uint8_t data);
void lcd_init();
void lcd_write_string(char *str);
void lcd_set_cursor(uint8_t row, uint8_t column);
void lcd_clear(void);
void lcd_backlight(uint8_t state);

#endif
