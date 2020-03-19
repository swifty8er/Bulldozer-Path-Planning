import turtle
t = turtle.Pen()


t.up()
t.back(600)
t.left(90)
t.back(300)
t.right(90)
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
t.circle(100,90)
turtle.done()