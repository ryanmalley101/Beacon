import serial
import time
import string
import pynmea2
import RPi.GPIO as GPIO

try:
	while True:
		port="/dev/ttyAMA0"
		ser=serial.Serial(port, baudrate=9600, timeout=0.5)
		dataout = pynmea2.NMEAStreamReader()
		line=ser.readline()
		if line:
			line = line.decode('latin-1')
			#print(line)
			newdata = line
			if newdata[0:6] == "$GNRMC":
				newmsg=pynmea2.parse(newdata)
				lat=newmsg.latitude
				lng=newmsg.longitude
				gps = "Latitude=" + str(lat) + "and Longitude=" + str(lng)
				print(gps)
finally:
	GPIO.cleanup()
