import Structs
import math
import copy
from numpy.random import randint as rndi
from scipy.special import binom as bin
import sympy as sp

def WaveFront(grid: Structs.Grid,
               s: Structs.XY, e: Structs.XY):

    #guards
    if((not(0<=s.x<=grid.xn)) or (not(0<=s.y<=grid.yn))):
        print('MAPPING ERROR: start point is outside the grid')
        return [False,[0]]
    if((not(0<=e.y<=grid.xn)) or (not(0<=e.y<=grid.yn))):
        print('MAPPING ERROR: end point is outside the grid')
        return [False,[0]]
    '''
    if(grid.points[s.x][s.y].val == 1):
        print('test3')
        return [False,[0]]
    '''
    
    if(grid.points[s.x][s.y].val == 1):
        print('MAPPING ERROR: Start point is on an obstacle point')
        return [False,[0]]
    

    gp = copy.deepcopy(grid)
    #don't change the actual points

    gp.points[e.y][e.x].val = 2
    d = 3
    neighbors = gp.adjs(e.x,e.y)
    while(neighbors):
        nextneighbors = set()
        for n in neighbors:
            x,y = n[0], n[1]
            if(gp.points[y][x].val != 0): continue
            gp.points[y][x].val = d
            nextneighbors.update(gp.adjs(x,y))
        neighbors = list(nextneighbors)
        d+=1

    # should now have a greedy path
    return MakePath(gp,s,e), gp

def MakePath(gp,s,e):
  curr = s
  path = [s]
  i=0
  
  while(curr != e):
    adjs = gp.adjs(curr.x,curr.y)
    min = adjs[0]
    for adj in adjs:
      if(adj[0]==e.x and adj[1]==e.y):
          min = adj
          break
      if(min[2]<2):
          min = adj
          continue
      if(1<adj[2]< min[2]):
        min = adj
    curr = Structs.XY(min[0],min[1])
    i+=1
    path.append(curr)
  #e = path[-1]
  while(gp.points[e.y][e.x].val==1):
      del path[-1]
      e=path[-1]
  return path
   
def ExecutePath(grid:Structs.Grid,path):
  #path should containing starting point at position 0, end point at position -1
  if(len(path)==0): return False
  disp_vecs=[]
  for i in range(len(path)-1):
    nex = path[i+1]
    curr = path[i]
    nex = grid.points[nex.y][nex.x]
    curr = grid.points[curr.y][curr.x]
    delta = nex-curr
    delta = Structs.XY(round(delta.x,2),round(delta.y,2))*grid.dim
    d_angle = math.atan2(delta.y,delta.x)%(2*math.pi)


    disp_vecs.append((delta,d_angle))

  return disp_vecs

def AvoidObst(side:bool,gp,loc:Structs.XY):
   #side: Left = True, Right = False
   #loc is grid coords
   if(side):
      #turn right
      loc.x+=1
   else:
      #turn left
      loc.x-=1
   #move forward one grid cell (or some number depending on granularity
   path = MakePath(gp,loc)
   return ExecutePath(gp,path)
   


def MakeBezier(path):
  bez=sp.Matrix([0,0])
  t=sp.Symbol('t')
  n=len(path)-1
  for i in range(n+1):
    c = bin(n,i)*(t**i)*((1-t)**(n-i))
    v = sp.Matrix([path[i].x,path[i].y])
    bez += c*v
  return bez

def Curvature(r,t):
  #WARNING: No error checking implemented yet (should be fine for 
  # component polynomials of the Bezier curve)
  n1 = sp.diff(r,t)
  n2 = sp.diff(r,t,t)
  n = n1[0]*n2[1]-n1[1]*n2[0]
  if(n==sp.Zero):
     n = sp.Matrix([0,0])
  n = sp.sqrt(n[0]**2 + n[1]**2)
  d = (sp.sqrt(n1[0]**2+n1[1]**2))**3
  return n/d
  



