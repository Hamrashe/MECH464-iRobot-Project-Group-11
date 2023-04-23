#traj.py
#file to hold the functions for all traj related stuff
from sympy import *
from sympy.solvers.ode.systems import dsolve_system
from sympy.plotting import plot
from math import pi
import numpy as np
from matplotlib  import pyplot as plt
from scipy import integrate as integ
from points2bezier import main
import sys
from scipy.interpolate import CubicSpline

sys.path.append(r"navigation")
import Structs
from Structs import XY
import Methods


V_d = 100.0#desired velociy in mm/s 
L = 235.0 #base width in mm


init_printing(use_unicode=True)




####Traj Planner##### 
#input is x(s) and y(s), the parameter, and desired linear velocity
#output is x(t), y(t), and theta(t)
class traj_planning:
    def __init__(self, x_list, y_list,p):
        self.s = Function('s') #s()
        self.t = symbols('t') #time
        self._s = symbols('_s') 
        
        self.x_s = []
        self.y_s = []

        for x_func in x_list:
            self.x_s.append(x_func.replace(p, self.s(self.t))) #parametric curve for x
        
        for y_func in y_list:
            self.y_s.append(y_func.replace(p, self.s(self.t))) #parametric curve for y
       
        #print(f'x_s is {self.x_s[0].subs(s(t), self.t)}')
        
        #self.s = symbols('t ' + s) #symbols for time and parametric parameter
    
    def find_s_t(self):
        print('solving for s(t)')
        s = self.s
        _s = self._s
        t = self.t
        x_s = self.x_s
        y_s = self.y_s
        s_t = []
        self.T_list = []
        T = 0
        #if i'm going to modify this for piecewise, then what i need to do is pass T to the next iteration as the s(0) value, I also need to store T so I can create boundaries for the piecewise function
        #print(f'x is : {x_s[0]}')
        cond = {s(0): 0}
        end_time = 0
        for i in range(len(x_s) -1):

            

            x_dot = diff(x_s[i], t)
            y_dot = diff(y_s[i], t)
            print(f'x_dot is {x_dot}')
            print(f'y_dot is {y_dot}')
            #eqn = Eq((x_dot**2 + y_dot**2)**0.5, V_d)
            eqn = [Eq(x_dot, V_d*cos(self.theta_s[i])),Eq(y_dot, V_d*sin(self.theta_s[i]))]
            print(f'eqn is {eqn}')
            s_solns = dsolve_system(eqn, [s(t), s(t)],t,'default',True,cond)
            #d = dsolv
            print('s(t) has been found')
            print(s_solns)
            for soln in s_solns:
                eqn = soln.subs(s(t), 1)
                print(f'solution of T: {solve(eqn,t)[0]}')
                #print(f'solution of T: {solve(eqn,t)[1]}')
                T = (solve(eqn,t)[0])
                try:
                    
                    
                    #print(T)
                    if T > 0:
                        #print('t greater than 0')
                        #print(soln,s(t))
                        s_t.append(solve(soln,s(t)))
                        end_time = T
                        print(f'end time is : {end_time} \n\n')
                        self.T_list.append(end_time)
                        #print(f's of t :{s_t[i]} \n\n\n t is {T}')
                        cond = {s(0): end_time}
                        
                        #break
                    
                

                except Exception:
                    print('No real soln')
                    pass
            #cond = {s(0): T}
        self.s_t = s_t
        print(self.T_list)
        return self.s_t

     #solve for theta(s)
    def find_theta_s(self):
        theta_s = []
        for i in range(len(self.x_s)):
            s = self.s
            t = self.t
            ds = 0.001 

            x1 = self.x_s[i]
            y1 = self.y_s[i]
            s_next = self.s(t) + ds

            x2 = self.x_s[i].subs(s(t), s_next )
            y2 = self.y_s[i].subs(self.s(t), s_next )
            dx = x2 - x1
            dy = y2- y1

            theta_s.append(atan2(dy,dx))
        #print(theta_s)
        return theta_s
    def find_theta_t(self):
        self.theta_t = []
        for i in range(len(self.x_t)):
            self.theta_t.append(atan2(self.y_t,self.x_t))
        
        print(self.theta_t)


    def pathing(self):
        s = self.s
        t = self.t
        self.theta_s = self.find_theta_s()
        s_t = self.find_s_t()
        #print(f' first s_t is {s_t[0][0]}')

        self.theta_t = [] 
        self.x_t = []
        self.y_t = []


        for i in range(len(s_t)):

            self.theta_t.append(self.theta_s[i].subs(s(t), s_t[i][0]))
            
            self.x_t.append(self.x_s[i].subs(s(t), s_t[i][0]))
            
            self.y_t.append(self.y_s[i].subs(self.s(self.t), s_t[i][0]))
            
        #print(f'x of time: {self.x_t}')
        #print(f'y of time: {self.y_t}')
        #print(f'theta t of time: {self.theta_t}')
        #self.find_theta_t()
        #self.vis_path(self.x_t[0], self.y_t[0], self.t)
        pl = plot_parametric(self.x_t[0], self.y_t[0], (self.t,0, self.T_list[0]), show=False)
        for i in range(len(self.x_t) - 1):
            pl_i = plot_parametric(self.x_t[i+1], self.y_t[i+1], (self.t,0, self.T_list[i+1]),show=False)
            pl.append(pl_i[0])

            
        pl.show()
        return [self.x_t, self.y_t, self.theta_t, self.T_list]
        
   
    def vis_path(self, x_s, y_s, s):
        range = (s, 0,1) 
        plot_parametric(x_s, y_s, range)
     # 
     # solve for x(t), y(t), theta(t)       
            

#####Traj Control##### traj.ctrl()
#input is theta_d(t), time, and theta_a(t) via odometry
#output is motion of the robot
class traj_ctrller:
    def __init__(self, theta_d_t, T):
        self.theta_d_t = theta_d_t
        self.T = T

        self.Kp = 0.5
        self.Kd = 0
        self.Ki = 0

        self.theta_error = []
        self.error_integ = 0

        self.V_d = V_d
    
    def theta_d_profile(self):
        #Find derivative and integral profile
        theta_d_t = self.theta_d_t
        theta_d_t_der = diff(theta_d_t,t)
        theta_d_t_integ = integrate(theta_d_t,t)

        #Initializing time parameters
        self.num_steps = 1000
        self.time_step = self.T/self.num_steps
        self.t_vals = np.linspace(0,self.T, self.num_steps)

        #Creating functions of time
        

        theta_d_func = lambdify(t,theta_d_t, "numpy")
        theta_d_der_func = lambdify(t,theta_d_t_der, "numpy")
        theta_d_integ_func = lambdify(t,theta_d_t_integ, "numpy")

        #Calculating profiles for theta_d(t), theta_d'(t), and the integral of theta_d(t)
        self.theta_d_t_vals = theta_d_func(self.t_vals)
        try:
            print(len(self.theta_d_t_vals))

        except Exception:
            self.theta_d_t_vals = np.full(self.num_steps, theta_d_t ,dtype=float)
            
            #print(self.theta_d_t_vals)
        self.theta_d_t_der_vals = theta_d_der_func(self.t_vals)
        self.theta_d_t_integ_vals = theta_d_integ_func(self.t_vals)

        for index in range(len(self.theta_d_t_vals)):
           if self.theta_d_t_vals[index] < 0:
                self.theta_d_t_vals[index] += 2*pi
    
                

       
        return self.theta_d_t_vals

    def controller(self, index, theta_a):
        theta_d_t_vals = self.theta_d_profile()
        error = theta_d_t_vals[index] - theta_a
        error = atan2(sin(error), cos(error))
        self.theta_error.append(error)
        print(error)

        if index != 0:
            
            e1 = self.theta_error[-2]
            e2 = error

            self.error_integ =+ (e2-e1)*self.time_step
            error_der = (e2-e1)/self.time_step
        else:
            error_integ = 0
            error_der = 0

        



        w = self.Kp*error + self.Kd*error_der + self.Ki*self.error_integ
    
        v_r = V_d + L/2*w
        v_l = V_d - L/2*w
        return float(v_r), float(v_l)

    
class bez_gen:
    def path():
    
        #Generates the bezier curve for a path through a map of obstacles. That map is defined in this funciton.
        #This function replaces the navtesting funcitonality
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
        print(path)
        bez = Methods.MakeBezier(path)
        
        t= symbols('t')
        
        #x_t = lambdify(t,bez[0], "numpy")
        #y_t = lambdify(t,bez[1], "numpy")
        
        x_t = bez[0]
        y_t = bez[1]
        
        #print(f'x(t): {bez[0]} \n\n\n\n y(t): {bez[1]}')
        return x_t, y_t, t        
    def piecewise():
    
        #Generates the bezier curve for a path through a map of obstacles. That map is defined in this funciton.
        #This function replaces the navtesting funcitonality
        robot_radius = 0.8

        points = Structs.make_ptsquare(10,1)
        print(points)
        points.extend(Structs.make_circ(XY(3,6),0.7))
        points.extend(Structs.make_circ(XY(3,4),0.3))
        points.extend(Structs.make_circ(XY(6,3),0.6))
        grid = Structs.Grid(points,10,10,robot_radius)

        ax = plt.axes()
        grid.draw(ax)
        for p in points:
            plt.scatter(p.x,p.y,marker = '.',color='black')
        plt.show()
        start = XY(1,1)
        end = XY(9,9)

        s = grid.points[start.y][start.x]
        e = grid.points[end.y][end.x]

        path,gp = Methods.WaveFront(grid,start,end)
        print(np.array(path))
        for p in path:
            plt.scatter(p.x,p.y,marker = '.',color='black')
        plt.show()
        #plt.plot(path)
        #plt.show()
        

        npoints = 4
        end = 0
        start = 0
        
        x_t = []
        y_t = []
        while True:
            if end + npoints > len(path):
                end - len(path) -1
                x_t.append(Methods.MakeBezier(path[start:len(path) -1])[0])
                y_t.append(Methods.MakeBezier(path[start:len(path) -1])[1])
                break
            else:
                end += npoints
                x_t.append(Methods.MakeBezier(path[start:end])[0])
                y_t.append(Methods.MakeBezier(path[start:end])[1])
                start=end

        
        #bez = Methods.MakeBezier(path)
        #print(bez[0][0])
        
        t= symbols('t')
        
        #x_t = lambdify(t,bez[0], "numpy")
        #y_t = lambdify(t,bez[1], "numpy")
        
        print(f'x_t: {x_t}')
        #print(f'x_t: {y_t}')
        
        #print(f'x(t): {bez[0]} \n\n\n\n y(t): {bez[1]}')
        return x_t, y_t, t   
class quintic_gen:
          
    def piecewise():
        
        #Generates the bezier curve for a path through a map of obstacles. That map is defined in this funciton.
        #This function replaces the navtesting funcitonality
        robot_radius = 0.8

        points = Structs.make_ptsquare(30,1)
        print(points)
        #points.extend(Structs.make_circ(XY(3,6),0.7))
        #points.extend(Structs.make_circ(XY(3,4),0.3))
        #points.extend(Structs.make_circ(XY(6,3),0.6))
        grid = Structs.Grid(points,30,30,robot_radius)

        ax = plt.axes()
        grid.draw(ax)
        for p in points:
            plt.scatter(p.x,p.y,marker = '.',color='black')
        
        start = XY(5,5)
        end = XY(18,25)

        s = grid.points[start.y][start.x]
        e = grid.points[end.y][end.x]

        path,gp = Methods.WaveFront(grid,start,end)
        print(np.array(path))
        x_pts = []
        y_pts = []
        for p in path:
            plt.scatter(p.x,p.y,marker = '.',color='black')
            x_pts.append(p.x)
            y_pts.append(p.y)
            
        plt.show()
        #plt.plot(path)
        #plt.show()
        

        npoints = 4
        end = 0
        start = 0
        
        x_t = []
        y_t = []
        t = symbols('t')
        pl = plot_parametric(show=False)

        print(len(path))
        for i in range(len(path) - len(path)%npoints -2):
            end = npoints*(i+1)+1
            if end >  len(path):
                break

            print(f'i is {i}')
            s = np.linspace(0,1,npoints+1)
            print(s)
            print(x_pts[npoints*(i):end])
            poly_x = np.polyfit(s,x_pts[npoints*(i):end], 2)
            poly_y = np.polyfit(s,y_pts[npoints*(i):end], 2)
            

            '''
            for i in range(len(poly_x)):
                if poly_x[i] <1e-14:
                    poly_x[i] = 0
            for i in range(len(poly_y)):
                if poly_y[i] <1e-14:
                    poly_y[i] = 0      
            '''        
            y_function =  t**2*poly_y[0] + t**1*poly_y[1] + t**0*poly_y[2]
            y_t.append(y_function)
            x_function =  t**2*poly_x[0] + t**1*poly_x[1] + t**0*poly_x[2]
            x_t.append(x_function)
            print(x_function)
            #print(poly)
            print(poly_x)
            #x_t.append(Methods.MakeBezier(path[start:end])[0])
            #y_t.append(Methods.MakeBezier(path[start:end])[1])
            y_t.append(y_function)
            
            pl_i = plot_parametric(x_function, y_function,(t, 0,1), show=False)
            pl.append(pl_i[0])
            start=end-1
            
        
        if start != len(path)-1:
            print(len(path) - start)
            s = np.linspace(0,1,len(path) - start)
            print(s)
            poly_x = np.polyfit(s,x_pts[start:], 3)
            
            poly_y = np.polyfit(s,y_pts[start:], 3)
            for i in range(len(poly_x)):
                if poly_x[i] <1e-14:
                    poly_x[i] = 0
            for i in range(len(poly_y)):
                if poly_y[i] <1e-14:
                    poly_y[i] = 0    
                    
            x_function = t**3*poly_x[0]+ t**2*poly_x[1] + t**1*poly_x[2] + t**0*poly_x[3]
            x_t.append(x_function)
            y_function = t**3*poly_y[0]+ t**2*poly_y[1] + t**1*poly_y[2] + t**0*poly_y[3]
            y_t.append(y_function)
            pl_i = plot_parametric(x_function, y_function,(t, 0,1), show=False)
            pl.append(pl_i[0])
            
        #x_t.append(Methods.MakeBezier(path[start:len(path) -1])[0])
        #y_t.append(Methods.MakeBezier(path[start:len(path) -1])[1])
        
        '''
        while True:
            if start == len(path)-1:
                break
            if end + npoints > len(path):
                #poly = np.polyfit(path[start:len(path) -1])
                s = np.linspace(0,1,len(path) - 1 - start)
                #print(s)
                poly_x = np.polyfit(s,x_pts[start:len(path) -1], 4)
                x_function = t**4*poly_x[4] + t**3*poly_x[3]+ t**2*poly_x[2] + t**1*poly_x[1] + t**0*poly_x[0]
                x_t.append(x_function)
                poly_y = np.polyfit(s,y_pts[start:len(path) -1], 5)
                y_function = t**4*poly_y[4] + t**3*poly_y[3]+ t**2*poly_y[2] + t**1*poly_y[1] + t**0*poly_y[0]
                y_t.append(y_function)
                pl.append(plot_parametric(x_function, y_function, (t, 0,1)))
                
                #x_t.append(Methods.MakeBezier(path[start:len(path) -1])[0])
                #y_t.append(Methods.MakeBezier(path[start:len(path) -1])[1])
                break
            else:
                end += npoints
                s = np.linspace(0,1,end- start)
                #print(s)
                poly_x = np.polyfit(s,x_pts[start:end], 3)
                poly_y = np.polyfit(s,y_pts[start:end], 3)
                y_function = t**3*poly_y[0]+ t**2*poly_y[1] + t**1*poly_y[2] + t**0*poly_y[3]
                y_t.append(y_function)
                x_function =  t**3*poly_x[0]+ t**2*poly_x[1] + t**1*poly_x[2] + t**0*poly_x[3]
                x_t.append(x_function)
                print(x_function)
                #print(poly)
                print(poly_x)
                #x_t.append(Methods.MakeBezier(path[start:end])[0])
                #y_t.append(Methods.MakeBezier(path[start:end])[1])
                y_t.append(y_function)
                
                pl_i = plot_parametric(x_function, y_function,(t, 0,1), show=False)
                pl.append(pl_i[0])
                start=end
        
        
        '''
        pl.show()
        #bez = Methods.MakeBezier(path)
        #print(bez[0][0])
        
        #t= symbols('t')
        
        #x_t = lambdify(t,bez[0], "numpy")
        #y_t = lambdify(t,bez[1], "numpy")
        
        print(f'x_t: {x_t}')
        #print(f'x_t: {y_t}')
        
        #print(f'x(t): {bez[0]} \n\n\n\n y(t): {bez[1]}')
        return x_t, y_t, t   
    

path = quintic_gen.piecewise()

#path = bez_gen.piecewise()

#print(f'path[0] {path[0]}')


#########Testing Code#################
t, r = symbols('t r')
s = Function('s')
x_s = Function('x')
x_s = 1000*cos(r*2*pi)+1000
y_s = 1000*sin(r*2*pi)+1000






#plan = traj_planning(x_s,y_s,r)
plan = traj_planning(path[0], path[1], path[2])


traj_plan = plan.pathing()


traj_ctrl = traj_ctrller(plan.theta_t, plan.T_list)

print(traj_ctrl.theta_d_profile())
plt.plot(traj_ctrl.t_vals,traj_ctrl.theta_d_t_vals)
plt.show()

for i in range(100):   

    print(traj_ctrl.controller(i,0))

'''
'''
trajectory = traj_planning(1, x_s,y_s,r)
traj_plan = trajectory.pathing()
print(f'path is {traj_plan[3]}')
#print(f'theta_s is {trajectory.theta_s}')
#print(trajectory.theta_s())
trajectory.vis_path(x_s, y_s, r)
print(trajectory.s_t)

        










