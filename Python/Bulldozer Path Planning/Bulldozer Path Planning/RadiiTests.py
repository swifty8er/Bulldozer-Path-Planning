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

radius = 60
inc = 1.025
add = 1.01
theta = math.pi/4.0
length = radius*theta
theta = theta*(180/math.pi)
i = 0
while i<15:
	if (i%2==0):
		print(radius,theta,length)
		t.circle(radius,theta)
		t.circle(radius,-theta)
	radius = radius**inc
	theta = (length/radius)*(180.0/math.pi)
	inc = inc**add
	add += 0.01
	i+=1
	

t.forward(length)
t.back(length)
t.up()
t.forward(100)
t.down()
print("Hello")

car = (150,150,90)
t.up()
t.goto(car[0],car[1])
t.setheading(car[2])
t.down()

radius = 60
theta = math.pi/4.0
length = radius*theta
theta = theta*(180/math.pi)
t.forward(25)
t.back(25)
localDeltaX = radius*(1-math.cos(math.radians(theta)))
localDeltaY = radius*math.sin(math.radians(theta))
localDeltaTheta = theta

for i in range(2):
	deltaXalpha = localDeltaY*math.sin(math.radians(car[2]))
	deltaYalpha = localDeltaY*math.cos(math.radians(car[2]))

	deltaXbeta = localDeltaX*math.cos(math.radians(car[2]-90))
	deltaYbeta = localDeltaX*math.sin(math.radians(car[2]-90))

	deeX = deltaXalpha + deltaXbeta
	deeY = deltaYalpha - deltaYbeta

	car = (car[0]-deeX,car[1]+deeY,car[2]+localDeltaTheta)

	t.up()
	t.goto(car[0],car[1])
	t.setheading(car[2])
	t.down()
	t.forward(25)
	t.back(25)

# radius = 60
# theta = math.pi/4.0
# length = radius*theta
# theta = theta*(180/math.pi)
# for k in range(2):
# 	t.circle(radius,theta)
# 	t.left(90)
# 	t.forward(radius)
# 	t.back(radius)
# 	t.right(90)

print("Done")
turtle.done()