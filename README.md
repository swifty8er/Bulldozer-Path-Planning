# Bulldozer-Path-Planning

The task of bulldozing has many aspects that need more research before they can be fully
automated. This code presents a new method for calculating a path for vehicles tasked with pushing
multiple objects to multiple goal locations. Unlike previous approaches which either focus on a single
object, apply both push and pull operations, or operate in discrete domain, this solution works in a
continuous space whilst handling multiple objects in a push-only scenario and doesn’t use computationally
heavy methods to find a potential path planning solution. To achieve the results, two graph structures, one
for the active vehicle and one for the passive pushable objects, is used. These graphs are then combined
with an A* Search Algorithm to find a correct path for the bulldozer to take. The algorithm is tested on a
Microban problem set which results in a 96% success rate.
