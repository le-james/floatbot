#Import necessary libraries
from marvelmind import MarvelmindHedge
from time import sleep, strftime, time
from serial import Serial
import sys

#Define GPS Hedgehog object, need to check the port
hedge = MarvelmindHedge(tty = "/dev/ttyACM1", adr = None, debug = False)
hedge.start()

while True:
	# pos = hedge.position() #get GPS position and parse
	pos = hedge.position() #get GPS position and parse
	x = pos[1]
	y = pos[2]
	z = pos[3]
	# hedge.print_position()

	print("pos", pos)

	# print("x position", x)
	# print("y position", y)
	# print("z position", z)
	# print("theta position", z)
	# print("t position", z)

	sleep(1)


