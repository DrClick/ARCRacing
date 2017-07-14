#!/usr/bin/python
import time
import Adafruit_CharLCD as LCD


# Raspberry Pi pin configuration:
lcd_rs        = 26  # Note this might need to be changed to 21 for older revision Pi's.
lcd_en        = 19
lcd_d4        = 13
lcd_d5        = 6
lcd_d6        = 5
lcd_d7        = 11
lcd_backlight = 4

# device config
lcd_columns = 20
lcd_rows    = 4

# Initialize the LCD using the pins above.
lcd = LCD.Adafruit_CharLCD(lcd_rs, lcd_en, lcd_d4, lcd_d5, lcd_d6, lcd_d7,
                           lcd_columns, lcd_rows, lcd_backlight)


# Print a two line message
lcd.message('Vector79\nAutonomous RC Racing\nIP: address \nCPU: Temp')

#lcd.message('\n     I love you\n      gorgeous\n')

# Wait 5 seconds
#time.sleep(5.0)

#lcd.clear()

