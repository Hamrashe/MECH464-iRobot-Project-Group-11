#traj.py
#file to hold the functions for all traj related stuff
from sympy import *
from math import pi
import numpy as np
from matplotlib  import pyplot as plt
from scipy import integrate as integ

V_d = 50.0 #desired velociy in mm/s 
L = 235.0 #base width in mm


init_printing(use_unicode=True)

####Traj Planner##### 
#input is x(s) and y(s), the parameter, and desired linear velocity
#output is x(t), y(t), and theta(t)
class traj_planning:
    def __init__(self, x_func, y_func,p):
        self.s = Function('s') #s()
        self.t = symbols('t') #time 
        
        self.x_s = x_func.replace(p, self.s(self.t)) #parametric curve for x
        self.y_s = y_func.replace(p, self.s(self.t)) #parametric curve for y
       
        
        #self.s = symbols('t ' + s) #symbols for time and parametric parameter
    
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
        s_t = self.find_s_t()

        self.theta_s = self.find_theta_s()
        
        self.theta_t = self.theta_s.subs(s(t), s_t)
        self.x_t = self.x_s.subs(s(t), s_t)
        self.y_t = self.y_s.subs(self.s(self.t), s_t)


        return [self.x_t,self.y_t,self.theta_t, self.T]
        
   
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

    def controller(self, index, theta_a):
        theta_d_t_vals = self.theta_d_profile()
        error = theta_d_t_vals[index] - theta_a
        error = atan2(sin(error), cos(error))
        self.theta_error.append(error)
        print(error)

        if index != 0:
            
            e1 = self.theta_error[index -1]
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

    
        
        


    



#########Testing Code#################
t, r = symbols('t r')
s = Function('s')
x_s = Function('x')
x_s = 1000*cos(r*2*pi)+1000
y_s = 1000*sin(r*2*pi)+1000



'''
plan = traj_planning(1, x_s,y_s,r)

traj_plan = plan.pathing()


traj_ctrl = traj_ctrller(plan.theta_t, plan.T)

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
'''
        










