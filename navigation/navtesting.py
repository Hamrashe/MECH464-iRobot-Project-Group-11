import Structs
from Structs import XY
import Methods
import matplotlib.pyplot as plt
from numpy.random import randint as rndi




robot_radius = 0.5

points = Structs.make_ptsquare(10,1)
points.extend(Structs.make_circ(XY(3,6),0.7))
points.extend(Structs.make_circ(XY(3,4),0.3))
points.extend(Structs.make_circ(XY(6,3),0.6))

grid = Structs.Grid(points,30,30,robot_radius)

ax = plt.axes()
grid.draw(ax)
for p in points:
    plt.scatter(p.x,p.y,marker = '.',color='black')

start = XY(rndi(0,30),rndi(0,30))
while(grid.points[start.y][start.x].val==1):
    start = XY(rndi(0,30),rndi(0,30))
end = XY(11,25)

s = grid.points[start.y][start.x]
e = grid.points[end.y][end.x]
plt.scatter(s.x,s.y,marker = 'o',color='green')
plt.scatter(e.x,e.y,marker = 'o',color='red')

gp = Methods.WaveFront(grid,start,end)
for row in grid.points:
    for p in row:
        if(p.val!=1):continue
        plt.annotate(f'{p.val}',(p.x,p.y))

path = Methods.WaveFront(grid,start,end)
dv = Methods.ExecutePath(grid,path)
print(dv)
for p in path:
    rp = grid.points[p.y][p.x]
    plt.scatter(rp.x,rp.y,marker = '.', color='blue')

ax.set_aspect(1)
plt.show()
