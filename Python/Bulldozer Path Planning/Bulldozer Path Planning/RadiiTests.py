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
Curves = []
while i<15:
	if (i%2==0):
		print(radius,theta,length)
		Curves.append((radius,theta))
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

for x in range(len(Curves)):

	car = (150,150,90)
	t.up()
	t.goto(car[0],car[1])
	t.setheading(car[2])
	t.down()

	radius = Curves[x][0]
	dtheta = Curves[x][1]
	radTheta = math.radians(car[2])
	deltaTheta = math.radians(dtheta)
	length = radius*deltaTheta
	t.forward(25)
	t.back(25)
	# if (x==1 or x == 2):
	# 	deltaTheta = -1*deltaTheta
	for i in range(2):
		a = radius*(1-math.cos(deltaTheta))
		b = radius*math.sin(deltaTheta)
		# if (x==0):
		# 	a *= -1
		# if (x==3):
		# 	b *= -1
		newX = car[0] + math.cos(radTheta - math.pi/2) * (-1*a) - math.sin(radTheta - math.pi/2) *b
		newY = car[1] + math.sin(radTheta - math.pi/2) * (-1*a) + math.cos(radTheta - math.pi/2) *b
		car = (newX,newY,car[2]+math.degrees(deltaTheta))
		radTheta = math.radians(car[2])
		print("New car is:")
		print(car[0],car[1],car[2])
		t.up()
		t.goto(car[0],car[1])
		t.setheading(car[2])
		t.down()
		t.forward(25)
		t.back(25)

t.up()
t.goto(150,150)
t.setheading(90)
t.down()
for x in range(len(Curves)):
	t.circle(Curves[x][0],Curves[x][1]*2)
	t.circle(Curves[x][0],-Curves[x][1]*2)

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