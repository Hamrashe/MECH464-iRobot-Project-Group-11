#traj.py
#file to hold the functions for all traj related stuff
from sympy import *
from math import pi
import numpy as np
from matplotlib  import pyplot as plt
from scipy import integrate as integ
import time as ti
from tqdm import tqdm

import sys

sys.path.append(r"navigation")
import Structs
from Structs import XY
import Methods

V_d = 50.0 #desired velociy in cm/s 
L = 235.0 #base width in cm




init_printing(use_unicode=True)

####Traj Planner##### 
#input is x(s) and y(s), the parameter, and desired linear velocity
#output is the speed for each wheel (v_r(t), v_l(t))  and desired theta(t)
class traj_planning:
    def __init__(self, x_func, y_func,p):
        self.s = Function('s') #s()
        self.t = symbols('t') #time 
        
        self.x_s = x_func.replace(p, self.s(self.t)) #parametric curve for x
        self.y_s = y_func.replace(p, self.s(self.t)) #parametric curve for y
       
        
        #self.s = symbols('t ' + s) #symbols for time and parametric parameter
    def find_s_t(self,x, y, t, time_var, step):
        x = lambdify(t(time_var), x, "numpy")
        y = lambdify(t(time_var), y, "numpy")
        
        s = np.linspace(0,1,int(1.0/step),)
        #Calculate the discrete points along the curve
        x_pts = x(s)
        y_pts = y(s)
        #print(x_pts, y_pts)
        #plt.scatter(x_pts, y_pts )
        #plt.show()
    
        #initialize the time array
        time = [0]
        #begin populating  it
        for i in range(len(x_pts)-1):
            dx = x_pts[i+1]- x_pts[i]
            dy = y_pts[i+1]- y_pts[i]
            dist = sqrt(dx**2 + dy**2)
            dt = dist/V_d
            time_curr = dt + time[i]
            time.append(time_curr)

        time = np.array(time, dtype=float)
        s = np.array(s, dtype=float)
        #print(f'time points: {time}')
        #plt.plot(time,s)
        #plt.show()
        coeff = np.polyfit(time,s, 5)
        s_t = coeff[0]*time_var**5 + coeff[1]*time_var**4 + coeff[2]*time_var**3 + coeff[3]*time_var**2 + coeff[4]*time_var**1 + coeff[0]*time_var**0
        print(f's_t is: {s_t}')
        p = plot_parametric(time_var,s_t, (time_var, 0, time[-1]), show=False)
        p.xlabel = 'time [s]'
        p.ylabel = 's value'
        p.title = "s(t) vs time"
        p.show()
        
        return s_t, time[-1]
    '''
    #Old method for finding s_t
    def find_s_t(self):
        s = self.s
        t = self.t
        x_s = self.x_s
        y_s = self.y_s
        
        cond = {s(0): 0}

        x_dot = diff(x_s, t)
        y_dot = diff(y_s, t)
        eqn = Eq(x_dot**2 + y_dot**2, V_d**2)
        s_solns = dsolve(eqn, s(t),'default',True,cond)
        
        
        for soln in s_solns:
            eqn = soln.subs(s(t), 1)
           
            try:
                self.T = float(solve(eqn,t)[0].evalf())
                if self.T > 0:
                    self.s_t = solve(soln,s(t))[0]
                    return self.s_t

            except Exception:
                #print('No real soln')
                pass
    '''  

     #solve for theta(s)
    def find_theta_s(self):
        s = self.s
        t = self.t
        ds = 0.001 

        x1 = self.x_s
        y1 = self.y_s
        s_next = self.s(t) + ds
        x2 = self.x_s.subs(s(t), s_next )
        y2 = self.y_s.subs(self.s(t), s_next )
        dx = x2 - x1
        dy = y2- y1

        theta_s = atan2(dy,dx)
        return theta_s
    
    def pathing(self):
        s = self.s
        t = self.t
        s_step = 0.001
        s_t, self.T = self.find_s_t(x=self.x_s, y=self.y_s, t=s, time_var = t, step=s_step)
        #print('s_t is found!')
        
        self.theta_s = self.find_theta_s()
        
        self.theta_t = self.theta_s.subs(s(t), s_t)
        theta_dot_t =  self.theta_t.diff(t)
        #print(f'theta dot test {theta_dot_t(4)}')
        self.theta_t = lambdify( t,self.theta_t, "numpy")
        
        self.x_t = self.x_s.subs(s(t), s_t)
        x_dot_t = self.x_t.diff(t)
        #print(f'theta dot test {theta_dot_t(4)}')
        self.x_t = lambdify( t,self.x_t, "numpy")
        self.y_t = self.y_s.subs(self.s(self.t), s_t)
        y_dot_t =  self.y_t.diff(t)
        #print(f'theta dot test {theta_dot_t(4)}')
        self.y_t = lambdify( t,self.y_t, "numpy")

        v_r, v_l = self.wheel_speed_profiles(theta_dot_t,x_dot_t, y_dot_t, t)

        return [v_r,v_l,self.theta_t, self.T, self.x_t, self.y_t]
        
    def wheel_speed_profiles(self,theta_dot_t, x_dot_t, y_dot_t, time_var):
        #print(theta_t.diff())
        #theta_dot_t = diff(theta_t(time_var), time_var)
        #x_dot_t = diff(x_t(time_var), time_var)
        #y_dot_t = diff(y_t(time_var), time_var)

        V = (x_dot_t**2 + y_dot_t**2)**0.5
        print('V has been found')
        w = theta_dot_t
        v_r = V + L/2*w
        v_l = V - L/2*w
        v_r = lambdify(time_var, v_r)
        v_l = lambdify(time_var, v_l)
        time = np.linspace(0,self.T, 1000)
        print(f'left wheel speed max = {max(v_l(time))}\n right wheel speed max = {max(v_r(time))}\n')
        plt.plot(time, v_r(time))
        plt.plot(time, v_l(time))
        plt.ylim((0,200))
        plt.ylabel('wheel speed [mm/s]')
        plt.xlabel('time [s]')
        plt.legend('left wheel', 'right wheel')
        plt.title('desired wheel velocities')
        plt.show() #plot showing wheel speeds for each wheel vs time
        return v_r,v_l
    def vis_path(self, x_s, y_s, s):
        range = (s, 0,1) 
        plot_parametric(x_s, y_s, range)
     # 
     # solve for x(t), y(t), theta(t)       
            

#####Traj Control##### traj.ctrl()
#input is the desired speed of each wheel, theta_d(t), end time, and theta_a(t) via odometry
#output is left and right speed commands of the robot
class traj_ctrller:
    def __init__(self,v_r_d, v_l_d, theta_d_t, T):
        self.theta_d_t = theta_d_t

        self.T = T
        self.v_r_d = v_r_d
        self.v_l_d = v_l_d
        self.theta_d_t = theta_d_t
        self.Kp = 5
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
        self.num_steps = 100
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

    def controller(self, time, theta_a, t_elapsed):
        #theta_d_t_vals = self.theta_d_profile()
        
        error = self.theta_d_t(time) - theta_a
        error = atan2(sin(error), cos(error))
        self.theta_error.append(error)
        print(f'theta error: {error}')

        if len(self.theta_error) >1:
            
            e1 = self.theta_error[-1]
            e2 = error

            self.error_integ =+ (e2-e1)*t_elapsed
            error_der = (e2-e1)/t_elapsed
        else:
            error_integ = 0
            error_der = 0

        



        w = self.Kp*error + self.Kd*error_der + self.Ki*self.error_integ
        #w = 0
        v_r = self.v_r_d(time) + L/2*w
        v_l = self.v_l_d(time) - L/2*w
        return float(v_r), float(v_l)

    
def bez_gen():
    #Generates the bezier curve for a path through a map of obstacles. That map is defined in this funciton.
    #This function is the behind the scenes of the pathing generation. For a fully realized project, this obstacle maping would come from a camera scanning an environment

    

    room_size = 3800 #mm
    room_pts = 38
    cell_size = room_size/room_pts

    print(f'cell size: {cell_size}mm')


    robot_radius = 400/cell_size

    #######Generating Room Map#################################
    points = Structs.make_ptsquare(room_pts,1) 
    #points.extend(Structs.make_circ(XY(0.4,0.4),0.1)) #right side wall, 325mm to the right of the origin
    #points.extend(Structs.make_square(100/cell_size, 10/cell_size,XY(10,10)))
    points.extend(Structs.make_rect(XY(3250/cell_size,0), XY(room_size/cell_size,room_size/cell_size)))#Right wall
    points.extend(Structs.make_rect(XY(910/cell_size,2080/cell_size), XY((910+910)/cell_size,(2080+1770)/cell_size)))#Table
    points.extend(Structs.make_rect(XY(1100/cell_size,0), XY(room_size/cell_size, 1130/cell_size)))#Carpet
    #points.extend(Structs.make_circ(XY(200/cell_size,400/cell_size),100/cell_size))
    #points.extend(Structs.make_circ(XY(25,25),5))
    grid = Structs.Grid(points,room_pts,room_pts,robot_radius, 1)
    print('grid has been constructed')

    '''
    ax = plt.axes()
    grid.draw(ax)
    for p in tqdm(points):
        plt.scatter(p.x,p.y,marker = '.',color='black')
    plt.title("Obstacle Map")
    plt.show()
    '''
    
    start = XY(int(robot_radius+100/cell_size),int(robot_radius+100/cell_size))
    print(start)
    end = XY(int(2850/cell_size),int((2200+1130)/cell_size))
    print(start, end)

    s = grid.points[start.y][start.x]
    e = grid.points[end.y][end.x]
    
    path,gp = Methods.WaveFront(grid,start,end)


    '''
    ax = plt.axes()
    grid.draw(ax)
    for p in points:
        plt.scatter(p.x,p.y,marker = '.',color='black')
    for p in path:
        plt.scatter(p.x,p.y,marker = '.',color='red')

    plt.show()
    '''
    
    ax = plt.axes()
    grid.draw(ax)
    for p in points:
        plt.scatter(p.x,p.y,marker = '.',color='black')
    for p in path:
        plt.scatter(p.x,p.y,marker = '.',color='red')
    
    path = np.array(path)*cell_size
    #print(path)
    
    bez = Methods.MakeBezier(path)

    t= symbols('t')

    x_t = lambdify(t,bez[0], "numpy")
    y_t = lambdify(t,bez[1], "numpy")

    x_t = bez[0]
    y_t = bez[1]
    x_t_vals = lambdify(t,x_t, "numpy")
    y_t_vals = lambdify(t,y_t, "numpy")
    t_vals = np.linspace(0,1,1000)
    theta_t = atan(y_t/x_t)
    #print(f'x(t): {bez[0]} \n\n\n\n y(t): {bez[1]}')
    plt.plot(x_t_vals(t_vals)/cell_size, y_t_vals(t_vals)/cell_size)
    #plt.legend((points, path),("Obstacles", "Point Path"))
    plt.title("Obstacle Map With Desired Path")
    plt.show()
    #plot_parametric(x_t, y_t, (t, 0,1))

    return x_t, y_t, t, theta_t, path[0]
        







    

    
'''


#########Testing Code#################




x_s, y_s, p, theta = bez_gen()

plan = traj_planning(x_s, y_s, p)
#plan = traj_planning(x_s, y_s, r)

v_r_d, v_l_d, theta_d, endtime, x_time, y_time  = plan.pathing()


traj_ctrl = traj_ctrller(v_r_d, v_l_d, theta_d, endtime)

#print(traj_ctrl.theta_d_profile())
#plt.plot(traj_ctrl.t_vals,traj_ctrl.theta_d_t_vals)
#plt.show()

time = 0
t_step = 0.1
t_start = ti.time()
ctrl_begin = ti.time()
while time < plan.T:
    
    
    print(traj_ctrl.controller(time, plan.theta_t(time),t_elapsed=ti.time()-t_start)) 
    #TODO replace this line with sending speed signal to irobot 
    #TODO replace the second argument with the angle measurement from odometry 
    #TODO note the speed is in m/s, need to convert to mm/s for sending commands to the irobot

    t_start = ti.time()
    ti.sleep(t_step)
    time = ti.time()- ctrl_begin

#visualize results of the run
bez_gen()
time_vals = np.linspace(0,endtime, 1000)
plt.plot(x_time(time_vals),y_time(time_vals))

plt.show()
#TODO Add move visualize to code 
#move.visualize()


'''


