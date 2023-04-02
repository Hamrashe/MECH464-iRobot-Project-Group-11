import Structs
import geometry
import math
import copy
from numpy.random import randint as rndi

def WaveFront(grid: Structs.Grid,
               s: Structs.XY, e: Structs.XY):
    
    #guards
    if((not(0<=s.x<grid.xn)) or (not(0<=s.y<grid.yn))):
        return [False,[0]]
    if((not(0<=e.y<grid.xn)) or (not(0<=e.y<grid.yn))):
        return [False,[0]]
    if(grid.points[s.x][s.y].val == 1):
        return [False,[0]]
    if(grid.points[s.x][s.y].val == 1):
        return [False,[0]]
    
    gp = copy.deepcopy(grid)
    #don't change the actual points

    #gp.points[e.y][e.x].val = 2
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
    e = path[-1]
    while(gp.points[e.y][e.x].val==1):
       del path[-1]
       e=path[-1]
    return path
    #return gp

def ExecutePath(grid:Structs.Grid,path,angle=0):
  #path should containing starting point at position 0, end point at position -1
  travel_dist = grid.dim
  if(len(path)==0): return False
  machine_delta = 7/3-4/3-1
  conf = 0.035/4 #reasonable angle delta (+- 0.5 degrees)
  disp_vecs=[]
  for i in range(len(path)-1):
    nex = path[i+1]
    curr = path[i]
    nex = grid.points[nex.y][nex.x]
    curr = grid.points[curr.y][curr.x]
    delta = nex-curr
    delta = Structs.XY(round(delta.x,2),round(delta.y,2))
    d_angle = round(math.atan2(delta.y,delta.x),2)
    
    while(False): #TODO: replace bool with math.fabs(angle - d_angle) > conf
       #TODO: turn robot until within the confidence range for angle
       continue
    #TODO: forward by travel_dist
    disp_vecs.append((delta,d_angle)) #TODO comment this out when done

  return disp_vecs


        
    
        


            
