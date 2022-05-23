import RPi.GPIO as GPIO
from time import sleep

Relay_channel = [17, 27, 22, 10, 9, 11, 0, 5]

t1 = Relay_channel[0]   # thruster 1
t2 = Relay_channel[7]   # thruster 2
t3 = Relay_channel[6]   # thruster 3
t4 = Relay_channel[5]   # thruster 4
t5 = Relay_channel[4]   # thruster 5
t6 = Relay_channel[3]   # thruster 6
t7 = Relay_channel[2]   # thruster 7
t8 = Relay_channel[1]   # thruster 8

# setup thruster gpio pins
def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Relay_channel, GPIO.OUT, initial=GPIO.HIGH)  # relays OFF - reverse logic for some reason

t = 0.05
tr = 0.04

def main():
    boolean = True;

    def one():
        print("one");
        GPIO.output(t1, GPIO.LOW)
        sleep(t);
        GPIO.output(t1, GPIO.HIGH)
    def two():
        print("two");
        GPIO.output(t2, GPIO.LOW)
        sleep(t);
        GPIO.output(t2, GPIO.HIGH)
    def three():
        print("three");
        GPIO.output(t3, GPIO.LOW)
        sleep(t);
        GPIO.output(t3, GPIO.HIGH)
    def four():
        print("four");
        GPIO.output(t4, GPIO.LOW)
        sleep(t);
        GPIO.output(t4, GPIO.HIGH)
    def five():
        print("five");
        GPIO.output(t5, GPIO.LOW)
        sleep(t);
        GPIO.output(t5, GPIO.HIGH)
    def six():
        print("six");
        GPIO.output(t6, GPIO.LOW)
        sleep(t);
        GPIO.output(t6, GPIO.HIGH)
    def seven():
        print("seven");
        GPIO.output(t7, GPIO.LOW)
        sleep(t);
        GPIO.output(t7, GPIO.HIGH)
    def eight():
        print("eight");
        GPIO.output(t8, GPIO.LOW)
        sleep(t);
        GPIO.output(t8, GPIO.HIGH)
    def moveForward():
        print("forward");
        GPIO.output(t4, GPIO.LOW)
        GPIO.output(t7, GPIO.LOW)
        sleep(t);
        GPIO.output(t4, GPIO.HIGH)
        GPIO.output(t7, GPIO.HIGH)
    def moveBackward():
        print("backward");
        GPIO.output(t3, GPIO.LOW)
        GPIO.output(t8, GPIO.LOW)
        sleep(t);
        GPIO.output(t3, GPIO.HIGH)
        GPIO.output(t8, GPIO.HIGH)
    def moveRight():
        print("right");
        GPIO.output(t1, GPIO.LOW)
        GPIO.output(t6, GPIO.LOW)
        sleep(t);
        GPIO.output(t1, GPIO.HIGH)
        GPIO.output(t6, GPIO.HIGH)
    def moveLeft():
        print("left");
        GPIO.output(t2, GPIO.LOW)
        GPIO.output(t5, GPIO.LOW)
        sleep(t);
        GPIO.output(t2, GPIO.HIGH)
        GPIO.output(t5, GPIO.HIGH)
    def rotateCW():
        print("cw");
        GPIO.output(t1, GPIO.LOW)
        GPIO.output(t3, GPIO.LOW)
        GPIO.output(t5, GPIO.LOW)
        GPIO.output(t7, GPIO.LOW)
        sleep(tr);
        GPIO.output(t1, GPIO.HIGH)
        GPIO.output(t3, GPIO.HIGH)
        GPIO.output(t5, GPIO.HIGH)
        GPIO.output(t7, GPIO.HIGH)
    def rotateCCW():
        print("ccw");
        GPIO.output(t2, GPIO.LOW)
        GPIO.output(t4, GPIO.LOW)
        GPIO.output(t6, GPIO.LOW)
        GPIO.output(t8, GPIO.LOW)
        sleep(tr);
        GPIO.output(t2, GPIO.HIGH)
        GPIO.output(t4, GPIO.HIGH)
        GPIO.output(t6, GPIO.HIGH)
        GPIO.output(t8, GPIO.HIGH)
    def moveStop():
        print("stop");
        for i in range(7):
            GPIO.output(Relay_channel[i], GPIO.HIGH);
                
    while (True):
        command = input();
        if (command == "1"):
            one();
        elif (command == "2"):
            two();
        elif (command == "3"):
            three();
        elif (command == "4"):
            four();
        elif (command == "5"):
            five();
        elif (command == "6"):
            six();
        elif (command == "7"):
            seven();
        elif (command == "8"):
            eight();
        elif (command == "w"):
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
            moveStop()
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