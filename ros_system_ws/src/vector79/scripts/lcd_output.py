#!/usr/bin/env python
import rospy
import json
from std_msgs.msg import String
import Adafruit_CharLCD as LCD
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

lcd.clear()
lcd.message("Vector79: ROS")



def callback(data):
    rospy.loginfo(rospy.get_caller_id() + '%s', data.data)
    lcd.set_cursor(0, 1)
    m =  ' '.join(data.data.split(" ")[1:])[:10]

    lcd.message(m)
    time.sleep(10)

def lcd_output():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('lcd_output', anonymous=True)

    rospy.Subscriber('bus_comm', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    lcd_output()

