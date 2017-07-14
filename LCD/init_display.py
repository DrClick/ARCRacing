#!/usr/bin/python
import socket
import Adafruit_CharLCD as LCD
import subprocess
import time

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



def get_ip_address():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setblocking(False)
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except:
    	return ""

def get_temp():
    return subprocess.check_output(['/opt/vc/bin/vcgencmd','measure_temp']).split('=')[1].rstrip()

lcd.clear()
lcd.message("      Vector79\nAutonomous RC Racing")

while True:
    
    current_ip = get_ip_address()
    temp = get_temp()    
    lcd.set_cursor(0, 2)
    lcd.message("IP: {}".format(current_ip))
    lcd.set_cursor(0, 3)
    lcd.message("CPU: {}".format(temp))
    
    # Wait 5 seconds
    time.sleep(1.0)
