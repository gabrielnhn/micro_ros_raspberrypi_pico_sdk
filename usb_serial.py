import serial

ser = serial.Serial('/dev/ttyACM0')  # open serial port
print(ser.name)         # check which port was really used

while True:
    
    ser.write(b'200 0\n')     # write a string
    # a = ser.read_until(b'\n')

    # print(a)

ser.close()     