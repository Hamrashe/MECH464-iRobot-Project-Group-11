import Structs
from Structs import XY
import Methods
import matplotlib.pyplot as plt
from numpy.random import randint as rndi
from math import pi
import sympy as sp
import numpy as np

robot_radius = 0.8

points = Structs.make_ptsquare(10,1)
points.extend(Structs.make_circ(XY(3,6),0.7))
points.extend(Structs.make_circ(XY(3,4),0.3))
points.extend(Structs.make_circ(XY(6,3),0.6))
grid = Structs.Grid(points,30,30,robot_radius)

ax = plt.axes()
grid.draw(ax)
for p in points:
    plt.scatter(p.x,p.y,marker = '.',color='black')

start = XY(5,3)
end = XY(11,25)

s = grid.points[start.y][start.x]
e = grid.points[end.y][end.x]

path,gp = Methods.WaveFront(grid,start,end)


# plt.scatter(s.x,s.y,marker = 'o',color='green')
# plt.scatter(e.x,e.y,marker = 'o',color='red')

# for p in path:
#     rp = grid.points[p.y][p.x]
#     plt.scatter(rp.x,rp.y,marker = '.', color='red')

#maxval = 0
# for row in gp.points:
#     for p in row:
#         maxval = max(maxval,p.val)

# for row in gp.points:
#     for p in row:
#       i = p.val/maxval
#       r = Structs.Rect(p.x,p.y,gp.xlength,gp.ylength)
#       x,y = r.verts()
#       plt.fill(x,y,f'{i}')

bez = Methods.MakeBezier(path)
ts = np.linspace(0.0,1.0,100)
t=sp.Symbol('t')
# for v in ts:
#     p = bez.evalf(subs={t:v})/3
#     plt.scatter(p[0],p[1],color='blue',marker='.')
print(f'x(t): {bez[0]} \n\n\n\n y(t): {bez[1]}')
# ax.set_aspect(1)
# plt.show()

# def evaluate_bez(bez,x):
#     return bez.evalf(subs={t:x})




