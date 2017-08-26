import serial
import time
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=.1)
time.sleep(3)
ser.write('LEFT_OFF\r\n')
ser.write('RUNNING_ON\r\n')
ser.write('BRIGHT_ON\r\n')
