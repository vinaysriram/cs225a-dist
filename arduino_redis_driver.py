'''
    File:       Arduino_Redis.py
    Contents:   This script is an interface between Redis and Arudino. 
                The script receives keys from redis and sends corresponding commands to Arduino via USB Serial port.
    Created:    Obinna Onyemepu (05/17/17)
'''

import redis
import time
import serial


# _____________________________Variables___________________________
SERVO_KEY = "servo_key"
TRIGGER_KEY = "trigger_key"


# _____________________________Serial connection setup___________________________
ports = ["/dev/cu.usbmodem1411", "/dev/cu.usbmodem1451","/dev/cu.usbmodem1461","/dev/cu.usbmodem1441", "/dev/ttyACM0", "/dev/ttyACM1"]

print "..............................."
print "checking ports"
for port in ports:
    try:
        ser = serial.Serial(port, 115200)
        print "Connected to port:", port
        break
    except:
        print "Could not connect to port:", port

print "Resetting Arduino Uno"
while ser.read() != 'R': # Wait until message is received from Arduino that Uno board is done resetting
    pass
print "Arduino Reset Completed"


# ___________________________________Redis Setup_____________________________________
''' Make sure Redis Server is connected '''
r = redis.StrictRedis(host="localhost", port=6379, db=0)
print "Redis connection verified"
print "..............................."

# _____________________________________Main_________________________________________
print "Running Arduino-Redis Driver"
r.set(SERVO_KEY,"0")  # set servo_key to 0 to ensure servo is in initial postion when script is run

while True:
    count = 0
    while r.get(SERVO_KEY) == "1":
        if count == 0:  # write to Serial port only once
            ser.write("1")
            ser.flush()
            ser.write("0")
            ser.flush()
            count = 1
            
        if ser.read() == 'T':
            r.set(SERVO_KEY,"0")
            r.set(TRIGGER_KEY,"1")

ser.close()


#if(ser.isOpen()):
#    print "Serial connection is still open."
#else:
#    print "Serial connection is closed"
#ser.close()
