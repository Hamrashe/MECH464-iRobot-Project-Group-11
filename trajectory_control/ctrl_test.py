#ctrl_test.py
#file to test controller in
import time
from threading import Timer
import pycreate2 
import sys
from traj import traj_planning, traj_ctrller
from sympy import *
import numpy as np
import matplotlib.pyplot as plt
#sys.path.append(r'/home/pi/Python/Group 11/navigation')
sys.path.append(r"motion")
from movement import movem, RepeatTimer

sys.path.append(r"navigation")
import Structs
from Structs import XY
import Methods



def bez_gen():
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

    bez = Methods.MakeBezier(path)
    
    t= symbols('t')
    
    #x_t = lambdify(t,bez[0], "numpy")
    #y_t = lambdify(t,bez[1], "numpy")
    
    x_t = bez[0]
    y_t = bez[1]
    
    #print(f'x(t): {bez[0]} \n\n\n\n y(t): {bez[1]}')
    return x_t, y_t, t



######################################################working code#############################################
##start the robot and main function begins
if __name__ == "__main__":
    # Create a Create2 Bot
    port = 'COM7'  # this is the serial port on my iMac
    #port = '/dev/ttyUSB0'  # this is the serial port on my raspberry pi
    baud = {
        'default': 115200,
        'alt': 19200  # shouldn't need this unless you accidentally set it to this
    }

    #Assign bot to pycreate2 object and assign serial port and baud rate
    bot = pycreate2.Create2(port=port, baud=baud['default'])

    #Start iRobot in safe mode
    bot.start()
    bot.safe()   

    #Initialize movement class
    move = movem(bot)

    
    #intialize timer
    
    odom = RepeatTimer(0.125, move.odometry)
    #odom.start()
    path = bez_gen()
    x_t = path[0]
    y_t = path[1]
    p = path[2]

    t = symbols('t')
    
    x_t = 100*cos(t*2*pi)+100
    y_t = 100*sin(t*2*pi)+100
    
    #x_t = 100*t
    #y_t = 1*t
    path = bez_gen()
   
    #Initialize Trajecotry Plan
    plan = traj_planning(path[0], path[1], path[2])
    

    
    
    print(plan.pathing())

    ctrl = traj_ctrller(plan.theta_t, plan.T)
    print(ctrl.controller(0,move.theta))

    step_size = plan.T/ctrl.num_steps
    V_d = ctrl.V_d

    plt.plot(ctrl.theta_d_t_vals)
    plt.show()

    move.odometry()
    #t_start = time.time()
    
    for i in  range(ctrl.num_steps) :
        #t_curr = time.time() - t_start
        
        #index = np.absolute((ctrl.t_vals-t_curr)).argmin()
        print(f'index is {i}')
        #print(ctrl.controller(index,np.radians(move.theta)))
        #print(f'theta is : {np.radians(move.theta)}')
        move.odometry()
        v_r, v_l = ctrl.controller(i,np.radians(move.theta))
        #print(f'error is: {ctrl.theta_error}\n\n')
        print(f'vr is {v_r}, vl is {v_l}')
        if abs(v_r) > 300 or abs(v_l) > 300:
            break
        
        
        bot.drive_direct(int(V_d/2.0 + v_r), int(V_d/2.0 +v_l))
        time.sleep(step_size)
    bot.drive_stop()
    move.visualize()



