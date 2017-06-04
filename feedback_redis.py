'''
    File:       Arduino_Redis.py
    Contents:   This script is an interface between Redis and Arudino. 
                The script receives keys from redis and sends corresponding commands to Arduino via USB Serial port.
    Created:    Obinna Onyemepu (05/17/17)
'''

import redis
import time
import serial


# _____________________________Serial connection setup___________________________
ports = ["/dev/ttyUSB0"]

print "..............................."
print "checking ports"
for port in ports:
    try:
        ser = serial.Serial(port, 115200)
        print "Connected to port:", port
        break
    except:
        print "Could not connect to port:", port

print "Resetting Arduino Mini"
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
r.set("target_key","0")  # set servo_key to 0 to ensure servo is in initial postion when script is run
count = 1

while True:
	#print ser.read()
        if ser.read() == 'T' and count == 1:
		r.set("target_key","1")
		print "HIT"
		count = 0

	elif ser.read() == 'S' and count == 0:
		r.set("target_key","0")
		print "Clear"
		count = 1

ser.close()


#if(ser.isOpen()):
#    print "Serial connection is still open."
#else:
#    print "Serial connection is closed"
#ser.close()
