import Structs
import geometry
import math
import copy

def WaveFront(grid: Structs.Grid,
               s: Structs.XY, e: Structs.XY):
    
    #guards
    if((not(0<=s.x<grid.xn)) or (not(0<=s.y<grid.yn))):
        return [False,[0]]
    if((not(0<=e.y<grid.xn)) or (not(0<=e.y<grid.yn))):
        return [False,[0]]
    if(grid.gridpoints[s.x][s.y].val == 1):
        return [False,[0]]
    if(grid.gridpoints[s.x][s.y].val == 1):
        return [False,[0]]
    
    gp = copy.deepcopy(grid)
    #don't change the actual gridpoints

    gp.gridpoints[e.y][e.x].val = 2
    d = 3
    neighbors = gp.adjs(e.x,e.y)
    while(neighbors):
        nextneighbors = set(tuple)
        for n in neighbors:
            x,y = n[0], n[1]
            if(gp.gridpoints[y][x].val != 0): continue
            gp.gridpoints[y][x].val = d
            nextneighbors.update(gp.adjs(x,y))
        neighbors = list(nextneighbors)

    #should now have a greedy path
    curr = s
    path = [s]
    while(curr != e):
      adjs = grid.adjs(curr.x,curr.y)
      min = adjs[0]
      for adj in adjs:
        if(adj.val< min.val):
          min = adj
      curr = min
      path.append(min)
    return path

def ExecutePath(grid:Structs.Grid,path,angle):
  #path should containing starting point at position 0, end point at position -1
  travel_dist = grid.cell_dim
  if(len(path)==0): return False
  machine_delta = 7/3-4/3-1
  conf = 0.035/4 #reasonable angle delta (+- 0.5 degree)

  for i in range(len(path)-1):
    cell_delta = path(i+1) - path(i)
    d_angle = math.atan2(cell_delta.y,cell_delta.x)
    while(math.abs(angle - d_angle) > conf):
       #turn robot until within the confidence range for angle
       continue
    #forward by travel_dist

  return


        
    
        


            
