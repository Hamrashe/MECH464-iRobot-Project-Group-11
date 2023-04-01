import math
import Structs

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
    return Structs.XY(vec.x*math.cos(angle)-vec.y*math.sin(angle),\
        vec.x*math.sin(angle)+vec.y*math.cos(angle))

def is_in_boundary(point, boundary):
    if(len(boundary.boundpoints)==0): raise Exception('Non-boundary polygon passed to is_in_boundary().')
    p, r = point, Structs.XY(10.0**12.0,0) #point and ray
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