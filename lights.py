import serial
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=.1)
ser.write('LEFT_OFF\n')
ser.write('RUNNING_ON\n')
ser.write('BRIGHT_ON\n')
