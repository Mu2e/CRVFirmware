# coding: utf-8
import serial
import time
import sys

port = sys.argv[2] if len(sys.argv) > 2 else '/dev/ttyUSB1'

ser = serial.Serial(port, 460800, timeout=0.1)
ser.write(("FL3\r").encode())
print("START")
for k in range(100):
    a = ser.readline()
    #print(a)
    if a.decode().find("Begin Load Flash") > -1:
        break
    if a != b'':
        #print("\r                                                       ", end = " ")
        #print("\r", a, end=" ")
        print("", a)
    time.sleep(0.5)
print("")

import sys
fname = sys.argv[1]
print("sending file: %s" % fname)
print("this will take up to 20s")
with open(fname, 'rb') as file:
    binaryData = file.read()
    print("File length:", len(binaryData))
    ser.write(binaryData)
for k in range(5):
    a = ser.readline()
    if a != b'':
        print("", a)
    time.sleep(0.5)
print("")
print("All done, reseting uC and FPGAs")
ser.write(("RESET\r").encode())
