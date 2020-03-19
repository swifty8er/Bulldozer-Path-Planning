import turtle
import time
import math
t = turtle.Pen()
turtle.speed('fastest')
turtle.tracer(False,5)
t.up()
t.left(90)
t.back(300)
t.right(90)
t.back(300)
t.down()
for x in range(3):	
	t.forward(600)
	t.left(90)
	t.forward(100)
	t.left(90)
	t.forward(600)
	t.right(90)
	t.forward(100)
	t.right(90)


t.forward(600)
t.right(90)

for x in range(3):	
	t.forward(600)
	t.right(90)
	t.forward(100)
	t.right(90)
	t.forward(600)
	t.left(90)
	t.forward(100)
	t.left(90)

t.forward(600)
t.left(90)
t.forward(250)
t.left(90)
t.up()
t.forward(250)
t.down()
t.dot(2)
turtle.tracer(True)
#define the four minimum radius turns
t.circle(100,90)
t.circle(100,-180)
t.circle(100,90)
t.left(180)
t.circle(100,90)
t.circle(100,-180)
t.circle(100,90)
t.left(180)
#define the forward and reverse left quadrant other turns
p = 1.025
inc = 0.025
while (p<1.55):
	radius = 100**p
	length = (100*math.pi)/2.0
	theta = (length/radius)*(180.0/math.pi) #convert radians to degrees
	print(radius,theta,length)
	t.circle(radius,theta)
	t.circle(radius,-theta)
	p+=inc
	inc = inc*1.5

turtle.done()