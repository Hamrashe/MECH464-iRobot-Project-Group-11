import numpy as np
import math
import matplotlib.pyplot as plt


class XY:
    def __init__(self,x,y,val=0):
        self.x = x
        self.y = y
        self.val = 0
        
    def __repr__(self): 
        return str((self.x,self.y))
    def __str__(self): # print(point) = (x,y) where point is an XY type
        return '({:.2f},{:.2f})'.format(self.x,self.y)
    
    def __eq__(self,other) -> bool: #points p,q p==q
        return self.x == other.x and self.y == other.y
    def __add__(self,other): #p and q are points, p+q = (p.x+q.x, p.y+q.y)
        return XY(self.x+other.x, self.y+other.y)
    def __sub__(self,other):
        return XY(self.x-other.x, self.y-other.y)
    def __mul__(self,other:float): #c is a scalar, q is a point then q*c = (cq.x,cq.y)
        return XY(other*self.x, other*self.y)
    def __xor__(self,other) -> float:
        return self.x*other.y-self.y*other.x
    
    def dist(self,other) -> float:
        return np.hypot(self.x - other.x, self.y - other.y)
    def mag(self) -> float:
        return np.hypot(self.x,self.y)   
    
class Rect:
    def __init__(self, cx, cy, w, h, val = 0):
        self.cx, self.cy = cx, cy
        self.w, self.h = w, h
        self.west_edge, self.east_edge = cx - w/2, cx + w/2
        self.north_edge, self.south_edge = cy - h/2, cy + h/2
        self.val = val

    def __repr__(self) -> str:
        return str((self.west_edge, self.east_edge, self.north_edge,
                self.south_edge))

    def __str__(self) -> str:
        return '({:.2f}, {:.2f}, {:.2f}, {:.2f})'.format(self.west_edge,
                    self.north_edge, self.east_edge, self.south_edge)

    def contains(self, point):
        try:
            point_x, point_y = point.x, point.y
        except AttributeError:
            point_x, point_y = point

        return (point_x >= self.west_edge and
                point_x <  self.east_edge and
                point_y >= self.north_edge and
                point_y < self.south_edge)

    def intersects(self, other):
        return not (other.west_edge > self.east_edge or
                    other.east_edge < self.west_edge or
                    other.north_edge > self.south_edge or
                    other.south_edge < self.north_edge)

    def draw(self, ax, c='k', lw=1, **kwargs):
        x1, y1 = self.west_edge, self.north_edge
        x2, y2 = self.east_edge, self.south_edge
        ax.plot([x1,x2,x2,x1,x1],[y1,y1,y2,y2,y1], c=c, lw=lw, **kwargs)
        
    def area(self):
        return self.w*self.h

class QTNode:
    def __init__(self,bdry,max_points=4,depth=0):
        self.bdry = bdry
        self.max_points = max_points
        self.points = []
        self.depth = depth
        
        self.xrange, self.yrange = [0,0], [0,0]
        self.div = False
        
    def divide(self):
        cx, cy = self.bdry.cx, self.bdry.cy
        w,h = self.bdry.w/2, self.bdry.h/2
        self.nw = QTNode(Rect(cx-w/2,cy-h/2,w,h,self.max_points,self.depth+1))
        self.ne = QTNode(Rect(cx+w/2,cy-h/2,w,h,self.max_points,self.depth+1))
        self.se = QTNode(Rect(cx+w/2,cy+h/2,w,h,self.max_points,self.depth+1))
        self.sw = QTNode(Rect(cx-w/2,cy+h/2,w,h,self.max_points,self.depth+1))
        self.divided = True
    
    def seek(self,bdry,found):
        if not self.bdry.intersects(bdry):
            return False
                        
        for p in self.points:
            if bdry.contains(p):
                found.append(p)
                         
        if self.divided:
            self.nw.seek(bdry,found)
            self.ne.seek(bdry,found)
            self.sw.seek(bdry,found)
            self.se.seek(bdry,found)
                         
        return found
    
    def insert(self,point):
        if not self.bdry.contains(point):
            return False
        if(self.depth == 0):
            # update xrange, yrange for top level QTNode
            self.xrange[0] = min(self.xrange[0],point.x)
            self.xrange[1] = max(self.xrange[1],point.x)
            self.yrange[0] = min(self.yrange[0],point.y)
            self.yrange[1] = max(self.yrange[1],point.y)

        if len(self.points)<self.max_points:
            self.points.append(point)
            return True
        if not self.divided:
            self.divide()
        
        return (self.ne.insert(point) or
                self.nw.insert(point) or
                self.se.insert(point) or
                self.sw.insert(point))
                         
    def draw(self,ax):
        self.boundary.draw(ax)
        if self.divided:
            self.nw.draw(ax)
            self.ne.draw(ax)
            self.se.draw(ax)
            self.sw.draw(ax)
                         
    def reconstruct(self): #reduces size of QT for pathfinding step optimization
        pass
                         
class Grid:
    def __init__(self,points:list[XY],length,cell_dim=1):
        #length is cell length
        self.xrange = [10000000,-10000000]
        self.yrange = [10000000,-10000000]
        self.grid_dim = cell_dim
        for p in points:
            self.xrange[1] = max(self.xrange[1],p.x)
            self.xrange[0] = min(self.xrange[0],p.x)
            self.yrange[1] = max(self.yrange[1],p.y)
            self.yrange[0] = min(self.yrange[0],p.y)

        self.xn = math.ceil((self.xrange[1]-self.xrange[0])/length)
        self.yn = math.ceil((self.yrange[1]-self.yrange[0])/length)
        self.gridpoints = np.array([[0]*self.xn]*self.yn, XY)

        offset = length/2

        for i in range(self.xn):
            for j in range(self.yn):
                x = self.xrange[0] + offset + i*length
                y = self.yrange[0] + offset + j*length
                self.gridpoints[j][i] = XY(x,y)
                gp = self.gridpoints[j][i]
                sqr = Rect(gp.x,gp.y,length/2,length/2)
                for p in points:
                    if(sqr.contains(p)):
                       self.gridpoints[j][i].val = 1
  
    def adjs(self,i,j,vals):
        adjs0 = [XY(i+1,j),XY(i,j+1),XY(i-1,j),XY(i,j-1)]
        adjs = []
        for adj in adjs0:
            if(adj.x<0 or adjs0.x>=self.xn): continue
            if(adj.y<0 or adjs0.y>=self.yn): continue
            adjs.append(adj)
        return adjs



