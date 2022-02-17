#Import necessary libraries
from marvelmind import MarvelmindHedge
from time import sleep, strftime, time
from serial import Serial
import sys

#Define GPS Hedgehog object, need to check the port
hedge = MarvelmindHedge(tty = "/dev/ttyACM1", adr = None, debug = False)
hedge.start()

#Define serial object (Arduino)
ser = Serial('/dev/ttyACM0',9600) #need to double check, might change
line = ser.readline()

with open("/home/pi/GPS_data.csv", "a") as log: #Open log file

	while True:
		pos = hedge.position() #get GPS position and parse
		x = pos[1]
		y = pos[2]
		z = pos[3]
		hedge.print_position()
		
		line = ser.readline() #read a byte from the serial monitor of the Arduino
		string = line.decode() #convert the byte to a string
		array = string.split() #split the string into an array
		distance = int(array[0])
		angle = int(array[1])
		print(distance)
		print(angle)
		
		#Write data to a log file
		#Data order:
		#(Date/time, x position, y position, z position, serial data from Arduino)
		log.write("{0},{1},{2},{3},{4},{5}|\n".format(strftime("%Y-%m-%d %H:%M:%S"),str(x),str(y),str(z),str(distance),str(angle)))
		
		sleep(1)
