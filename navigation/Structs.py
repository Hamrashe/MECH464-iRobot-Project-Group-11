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
    # from https://scipython.com/blog/quadtrees-2-implementation-in-python/  
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

        return (1.01*point_x >= self.west_edge and
                0.99*point_x <  self.east_edge and
                1.01*point_y >= self.north_edge and
                0.99*point_y < self.south_edge)

    def intersects(self, other):
        return not (other.west_edge > self.east_edge or
                    other.east_edge < self.west_edge or
                    other.north_edge > self.south_edge or
                    other.south_edge < self.north_edge)

    def draw(self, ax, c='k', lw=1, **kwargs):
        x1, y1 = self.west_edge, self.north_edge
        x2, y2 = self.east_edge, self.south_edge
        ax.plot([x1,x2,x2,x1,x1],[y1,y1,y2,y2,y1], c=c, lw=lw, **kwargs)
    
    def verts(self):
      x1, y1 = self.west_edge, self.north_edge
      x2, y2 = self.east_edge, self.south_edge
      return [x1,x2,x2,x1,x1],[y1,y1,y2,y2,y1]
    def area(self):
        return self.w*self.h

class QTNode:
    # from https://scipython.com/blog/quadtrees-2-implementation-in-python/
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
    def __init__(self,points:list[XY],xn:int,yn:int,rad:float,cell_dim=1):
        #length is cell length
        self.xrange = [10000000,-10000000]
        self.yrange = [10000000,-10000000]
        self.dim = cell_dim
        for p in points:
            self.xrange[1] = max(self.xrange[1],p.x)
            self.xrange[0] = min(self.xrange[0],p.x)
            self.yrange[1] = max(self.yrange[1],p.y)
            self.yrange[0] = min(self.yrange[0],p.y)

        self.xn = xn
        self.yn = yn
        self.points = np.array([[0]*self.xn]*self.yn, XY)
        self.xlength = (self.xrange[1]-self.xrange[0])/xn
        self.ylength = (self.yrange[1]-self.yrange[0])/yn
        
        xoffset = self.xlength/2
        yoffset = self.ylength/2
        for i in range(self.xn):
            for j in range(self.yn):
                x = self.xrange[0] + xoffset + i*self.xlength
                y = self.yrange[0] + yoffset + j*self.ylength
                self.points[j][i] = XY(x,y)
                gp = self.points[j][i]
                sqr = Rect(gp.x,gp.y,self.xlength,self.ylength)
                for p in points:
                    if(sqr.contains(p)):
                       self.points[j][i].val = 1
        
        #account for robot radius; DISGUSTINGLY slow, 
        #optimize if time permits
        for row in self.points:
            for p in row:
                if(p.val != 1): continue
                for row2 in self.points:
                    for q in row2:
                        if(p.dist(q)<=rad and q.val!=1):
                            q.val = -1
        for row in self.points:
            for p in row:
                if(p.val==-1):p.val = 1


  
    def adjs(self,i,j):
        adjs0 = [(i+1,j),(i,j+1),(i-1,j),(i,j-1)]
        adjs0.extend([(i+1,j+1),(i+1,j-1),(i-1,j+1),(i-1,j-1)])
        adjs = []
        for adj in adjs0:
            if(adj[0]<0 or adj[0]>=self.xn): continue
            if(adj[1]<0 or adj[1]>=self.yn): continue
            x,y = adj[0],adj[1]
            adj = (x,y,self.points[y][x].val)
            adjs.append(adj)
        return adjs
    
    def draw(self,ax,fill = 'k'):
        xl=self.xlength
        yl=self.ylength
        for row in self.points:
            for p in row:
                r = Rect(p.x,p.y,xl,yl)
                r.draw(ax,fill)
        

def make_ptsquare(length:float,space:float):
    points = []
    points.append(XY(0,0))
    l=space
    while(l<=length):
        points.append(XY(l,0))
        points.append(XY(l,length))
        points.append(XY(0,l))
        points.append(XY(length,l))
        l+=space
    return points

def make_circ(center:XY,radius:float):
    dtheta = 10
    torad = math.pi/180
    points = []
    vec = XY(1,0)*radius
    theta = 0
    while(theta<360):
        p = center + rotate_vector_ccw(vec,theta*torad)
        points.append(p)
        theta+=dtheta
    return points

def are_collinear(points): #for n points
    #scans non-start, non-end points to determine if ALL n points are collinear
    eps = 10.0**(-12.0)
    for i in range(1,len(points)-1):
        v1 = points[i-1] - points[i]
        v2 = points[i+1]-points[i]
        if(abs(v1^v2)>eps): 
            return False
    return True

def rotate_vector_ccw(vec, angle):
    #angle should be passed in radians
    return XY(vec.x*math.cos(angle)-vec.y*math.sin(angle),\
        vec.x*math.sin(angle)+vec.y*math.cos(angle))

def is_in_boundary(point, boundary):
    if(len(boundary.boundpoints)==0): raise Exception('Non-boundary polygon passed to is_in_boundary().')
    p, r = point, XY(10.0**12.0,0) #point and ray
    n = 0
    vertex = True
    for i in range(len(boundary.boundvectors)):
        q, s = boundary.boundpoints[i], boundary.boundvectors[i]
        if is_intersect(p,q,r,s, vertex): 
            n+=1
            vertex = False
            continue
        vertex = True
    return n%2==1

def is_intersect(p,q,r,s, vertex = False):
    det = r^s
    if(round(abs(r^s),12) == 0): return False #default- the overlap case is handled elsewhere
    c = (q-p) #for convenience
    t = (c^s)/det
    u = (c^r)/det
    if(not vertex):
        return det!=0 and 0<round(t,12)<1 and 0<round(u,12)<1
    return det!=0 and 0<=round(t,13)<=1 and 0<=round(u,13)<=1 
    

