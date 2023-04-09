#traj.py
#file to hold the functions for all traj related stuff
from sympy import *
from math import pi

init_printing(use_unicode=True)

####Traj Planner##### traj.plan()
#input is x(s) and y(s), the parameter, and desired linear velocity
#output is x(t), y(t), and theta(t)
class traj_plan:
    def __init__(self, v_d, x_s, y_s,p):
        self.s = Function('s') #s()
        self.t = symbols('t') #time 
        self.v_d = v_d #desired run velocity
        self.x_s = x_s.replace(p, s(self.t)) #parametric curve for x
        self.y_s = y_s.replace(p, s(self.t)) #parametric curve for y
        self.L = 235.0 #base width in mm
        
        #self.s = symbols('t ' + s) #symbols for time and parametric parameter
    
    def sol_s_func(self):
        s = self.s
        t = self.t
        x_s = self.x_s
        y_s = self.y_s
        
        cond = {s(0): 0}

        x_dot = diff(x_s, t)
        y_dot = diff(y_s, t)
        eqn = Eq(x_dot**2 + y_dot**2, self.v_d)
        self.s_t = dsolve(eqn, s(t),'default',True,cond)


        


    def vis_path(self):
        range = (self.s, 0,1) 
        plotting.plot.plot_parametric(self.x_s, self.y_s, range)



#########Testing Code#################
t, r = symbols('t r')
s = Function('s')
x_s = Function('x')
x_s = r*23
#x_s = x_s.replace(t,s(t))
#x_s = s(t)* 1*pi




trajectory = traj_plan(1, x_s,x_s,r)
trajectory.sol_s_func()
print(trajectory.s_t[1])

        










#####Traj Control##### traj.ctrl()
#input is theta_d(t), time, and theta_a(t) via odometry
#output is motion of the robot