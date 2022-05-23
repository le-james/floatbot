import RPi.GPIO as GPIO
from time import sleep

# Relay_channel = [3, 5, 7, 11, 13, 15, 19, 21]
# Thruster 1: Channel Index 7
# Thruster 2: Channel Index 6
# Thruster 3: Channel Index 5
# Thruster 4: Channel Index 4
# Thruster 5: Channel Index 3
# Thruster 6: Channel Index 2
# Thruster 7: Channel Index 1
# Thruster 8: Channel Index 0

# def setup():
#     GPIO.setmode(GPIO.BOARD);
#     GPIO.setup(Relay_channel, GPIO.OUT, initial = GPIO.HIGH);


# thruster gpio pins
Relay_channel = [17, 27, 22, 10, 9, 11, 0, 5]

# setup thruster gpio pins
def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Relay_channel, GPIO.OUT, initial=GPIO.HIGH)  # relays OFF - reverse logic for some reason


def main():
    boolean = True;

    def moveForward():
            print("forward");
            GPIO.output(Relay_channel[2], GPIO.LOW)
            GPIO.output(Relay_channel[7], GPIO.LOW)
            sleep(0.5);
            GPIO.output(Relay_channel[2], GPIO.HIGH)
            GPIO.output(Relay_channel[7], GPIO.HIGH)
    def moveBackward():
            print("backward");
            GPIO.output(Relay_channel[3], GPIO.LOW)
            GPIO.output(Relay_channel[6], GPIO.LOW)
            sleep(0.5);
            GPIO.output(Relay_channel[3], GPIO.HIGH)
            GPIO.output(Relay_channel[6], GPIO.HIGH)
    def moveRight():
            print("right");
            GPIO.output(Relay_channel[1], GPIO.LOW)
            GPIO.output(Relay_channel[4], GPIO.LOW)
            sleep(0.5);
            GPIO.output(Relay_channel[1], GPIO.HIGH)
            GPIO.output(Relay_channel[4], GPIO.HIGH)
    def moveLeft():
            print("left");
            GPIO.output(Relay_channel[0], GPIO.LOW)
            GPIO.output(Relay_channel[5], GPIO.LOW)
            sleep(0.5);
            GPIO.output(Relay_channel[0], GPIO.HIGH)
            GPIO.output(Relay_channel[5], GPIO.HIGH)
    def rotateCW():
            print("cw");
            GPIO.output(Relay_channel[0], GPIO.LOW)
            GPIO.output(Relay_channel[2], GPIO.LOW)
            GPIO.output(Relay_channel[4], GPIO.LOW)
            GPIO.output(Relay_channel[6], GPIO.LOW)
            sleep(0.25);
            GPIO.output(Relay_channel[0], GPIO.HIGH)
            GPIO.output(Relay_channel[2], GPIO.HIGH)
            GPIO.output(Relay_channel[4], GPIO.HIGH)
            GPIO.output(Relay_channel[6], GPIO.HIGH)
    def rotateCCW():
            print("ccw");
            GPIO.output(Relay_channel[1], GPIO.LOW)
            GPIO.output(Relay_channel[3], GPIO.LOW)
            GPIO.output(Relay_channel[5], GPIO.LOW)
            GPIO.output(Relay_channel[7], GPIO.LOW)
            sleep(0.25);
            GPIO.output(Relay_channel[1], GPIO.HIGH)
            GPIO.output(Relay_channel[3], GPIO.HIGH)
            GPIO.output(Relay_channel[5], GPIO.HIGH)
            GPIO.output(Relay_channel[7], GPIO.HIGH)
    def moveStop():
            print("stop");
            for i in range(7):
                GPIO.output(Relay_channel[i], GPIO.HIGH);
                
    while (True):
        command = input();
        if (command == "w"):
            moveForward();
        elif (command == "s"):
            moveBackward();
        elif (command == "d"):
            moveRight();
        elif (command == "a"):
            moveLeft();
        elif (command == "e"):
            rotateCW();
        elif (command == "q"):
            rotateCCW();
        elif (command == " "):
            break
        
def destroy():
	GPIO.output(Relay_channel, GPIO.LOW)
	GPIO.cleanup()

if __name__ == '__main__':
	setup()
	try:
		main()
	except KeyboardInterrupt:
		destroy()
   