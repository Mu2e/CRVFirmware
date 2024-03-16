# coding: utf-8
import serial
import time
ser = serial.Serial('/dev/ttyUSB0', 460800, timeout=0.1)
ser.write(("FL1\r").encode())
for k in range(100):
    a = ser.readline()
    #print(a)
    if a.decode().find("Begin Load Flash") > -1:
        break
    if a != b'':
        #print("\r                                                       ", end = " ")
        print("\r", a, end=" ")
    time.sleep(0.5)
print("")

import sys
fname = sys.argv[1]
print("sending file: %s" % fname)
print("this will take up to 20s")
with open(fname, 'rb') as file:
    binaryData = file.read()
    ser.write(binaryData)
print("All done, reseting uC and FPGAs")
ser.write(("RESET\r").encode())
