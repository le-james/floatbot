from turtle import *
import turtle

screen = Screen()
# screen.screensize(canvwidth=100, canvheight=100, bg="yellow") # WTF does this do?
screen.setup(500, 500)

# turtle object
floatbot = turtle.Turtle()

# turtle paramenters
floatbot.shapesize(2, 2, 2)
floatbot.pensize(2)
 
""" # the coordinates
# of each corner
shape = ((-5, -10), (-10, -5), (-10, 5), (-5, 10), (5, 10), (10, 5), (10, -5), (5, -10))
# registering the new shape
turtle.register_shape('octagon', shape)
# changing the shape to 'octagon'
floatbot.shape('octagon') """

# 0-10 speed range
floatbot.speed(1)

# for i in range(4):
#     floatbot.forward(100)
      
#     # get heading value
#     val = floatbot.heading()
      
#     # write it
#     floatbot.write(str(val))
#     floatbot.backward(100)
#     floatbot.left(90)

# for i in range(180):
#     # floatbot.goto(100, 100)
#     # floatbot.setheading(i)

# floatbot.left(45)
# floatbot.setheading(45)
floatbot.forward(100)
floatbot.tilt(90)



turtle.done()