import serial
import time
import pycreate2
import math
import numpy as np
from threading import Timer
#from machine import Timer
from pycreate2 import Create2
from matplotlib import pyplot as plt

import sys

sys.path.append(r"navigation")
import Structs
from Structs import XY
import Methods


import sys
#sys.path.append(r'/home/pi/Python/Group 11/navigation')
#import navtesting
#import Structs
# Create a Create2 Bot


#Constatnts
WHEEL_DIA_CORRECTION = 0.959
L_WHEEL_DIA_CORRECTION = 1
R_WHEEL_DIA_CORRECTION = 1
#WHEEL_BASE_CORRECTION_CCW = 2.0723
#WHEEL_BASE_CORRECTION_CW =2.0723
WHEEL_BASE_CORRECTION_CCW = 1.125
WHEEL_BASE_CORRECTION_CW = 1.125

#User inputs for constants (for adjusting odometry correction)

#WHEEL_DIA_CORRECTION = float(input(f'Avg wheel diameter correction factor (last was {WHEEL_DIA_CORRECTION}):'))
#L_WHEEL_DIA_CORRECTION = float(input(f'Left wheel diameter correction factor (last was {L_WHEEL_DIA_CORRECTION}):'))
#R_WHEEL_DIA_CORRECTION = float(input(f'Right wheel diameter correction factor (last was {R_WHEEL_DIA_CORRECTION}):'))
#WHEEL_BASE_CORRECTION_CCW = float(input(f'Wheel base correction factor (last was {WHEEL_BASE_CORRECTION_CCW}):'))
#WHEEL_BASE_CORRECTION_CW = float(input(f'Wheel base correction factor (last was {WHEEL_BASE_CORRECTION_CW}):'))

#print(f'Avg wheel diameter correction factor = {WHEEL_DIA_CORRECTION}\nLeft wheel diameter correction factor = {L_WHEEL_DIA_CORRECTION}\nRight wheel diameter correction factor = {R_WHEEL_DIA_CORRECTION}\nWheel base correction factor CCW = {WHEEL_BASE_CORRECTION_CW}\nWheel base correction factor CW = {WHEEL_BASE_CORRECTION_CW}')

#Global odometry variables
x_coord = 0 #Positive x direction is east at the origin
y_coord = 0 #Positive y direction is north at the origin
angle = 0 #Degrees from y-axis at the origin
L = []
R = []
X = []
Y = []

#l_encd_init = 0
#r_encd_init = 0

last_left = 0
last_right = 0


#dv = navtesting.main()

class movem(object):
    def __init__(self, bot):
        self.bot = bot
        self.theta = 0
        self.x = 0
        self.y = 0
        self.left_encod_init = bot.get_sensors().encoder_counts_left
        self.right_encod_init = bot.get_sensors().encoder_counts_right
        print(f'{self.left_encod_init}, {self.right_encod_init}')

    #Max speed of both wheels are 500 mm/s
    def rotate(angle_, ang_speed):
        print('rotating')
        #Initialize angle sensor
        #cur_ang = sensors.angle
        angle_rad = angle_*math.pi/180.0
        #Distance from wheels to center of iRobot in mm (According manual, 235 is the distance between the wheels)
        wheel_rad_nom = 235.0/2 #mm
        if angle <0:

            wheel_rad = float(wheel_rad_nom)*WHEEL_BASE_CORRECTION_CCW #b_nom * E_b
        else:
            wheel_rad = float(wheel_rad_nom)*WHEEL_BASE_CORRECTION_CW

        lin_vel = ang_speed*wheel_rad

        lin_vel = int(lin_vel) #Convert to int so it can be used for drive command
        
        angle_rad = angle_*(math.pi)/180.0
        stopTimeAng = angle_rad/ang_speed

        #(Left velocity, right velocity)
        #while cur_ang < angle:
        movem.drive(-lin_vel, lin_vel)
        time.sleep(stopTimeAng)
        self.bot.drive_stop()
        #cur_ang = cur_ang + sensors.angle

    def forward(dist, speed):
        print('forward')
        #Initialize distance sensor, should be 0 at first
        #cur_dist = sensors.distance

        stopTime = dist/speed

        #while iRobot traversal distance is less than desired distance, drive at *speed* to desired distance
        movem.drive(speed,speed)
        time.sleep(stopTime)
        self.bot.drive_stop()
        print('drive has stopped\n')
        #print(f'distance travelled {sensors.distance}')

    def backward(dist, speed):
        #Initialize distance sensor, should be 0 at first
        #cur_dist = sensors.distance

        stopTime = dist/speed

        #while iRobot traversal distance is less than desired distance, drive at *speed* to desired distance
        movem.drive(-speed,-speed)
        time.sleep(stopTime)
        self.bot.drive_stop()
        print('drive has stopped\n')
        #print(f'distance travelled {sensors.distance}')
       
    def drive(v_L_nom, v_R_nom):
        #this function is for applying corrections before calling drive_direct
        v_L = v_L_nom*WHEEL_DIA_CORRECTION*L_WHEEL_DIA_CORRECTION #nominal speed * distance factor * difference in diameter scaling
        v_R = v_R_nom*WHEEL_DIA_CORRECTION*R_WHEEL_DIA_CORRECTION #nominal speed * distance factor * difference in diameter scaling
        self.bot.drive_direct(int(v_L), int(v_R))
	
	
    def get_sensors_test():
        sensors = self.bot.get_sensors()
        print(sensors)
        print('test\n')
    	
    def square(L, dir):
        #function to drive the robot in a square for UMB calibration
        movem.forward(L,100)
        movem.rotate(dir*90, dir*0.85)
        movem.forward(L,100)
        movem.rotate(dir*90, dir*0.85)
        movem.forward(L,100)
        movem.rotate(dir*90, dir*0.85)
        movem.forward(L,100)
        movem.rotate(dir*90, dir*0.85)

    def sound():
        song = [72, 16] #(note, duration), ...
        song_num = 0
        self.bot.createSong(song_num, song) 
        time.sleep(1)
        self.bot.playSong(song_num)
    
    def odometry(self):
        #update global variables of x, y, angle of roself.bot
        global x_coord
        global y_coord
        global angle
        global l_encd_init
        global r_encd_init
        
        global last_left
        global last_right
        
        global L
        global R
        #print('Interrupted')
        wheel_base = 235.0
        dtheta = 0
        
        
        
        sensors = self.bot.get_sensors()
        
        #print(f'left encoder counts: {sensors.encoder_counts_left} \nright encoder counts: {sensors.encoder_counts_right} \n\n\n\n\n ')

        left_encoder = sensors.encoder_counts_left - self.left_encod_init
        right_encoder = sensors.encoder_counts_right - self.right_encod_init
        
        
        #print(left_encoder)
        #print(round(right_encoder*0.9,0))
        L.append(left_encoder)
        R.append(right_encoder)
        if len(L) >= 2:
            left_encoder = left_encoder - L[len(L) - 2]
            right_encoder = right_encoder - R[len(R) - 2]

        left_distance = left_encoder * math.pi*72.0 / 508.8 #Converts encoder counts into distance in mm
        right_distance = right_encoder * math.pi*72.0 / 508.8 #Converts encoder counts into distance in mm
        #left_distance = left_encoder * 500.0 / 882.0 #Converts encoder counts into distance in mm
        #right_distance = right_encoder * 500.0/ 980.0 #Converts encoder counts into distance in mm

        center_distance = (left_distance+right_distance)/2 #Get new center distance
        #print('centerdistance' + str(center_distance))
        #ang_rad = angle * math.pi / 180.0
        
        
        dtheta = -1*(((left_distance) - (right_distance))/(wheel_base)) #Get change in angle
        if abs(dtheta) < 1:
            dtheta_deg = dtheta*180/math.pi
            
            x_coord += center_distance*math.sin(math.radians(-1*angle)) #Update x_coord from origin
            y_coord += center_distance*math.cos(math.radians(-1*angle)) #Update y_coord from origin
            #print(math.hypot(x_coord,y_coord))
            angle += dtheta_deg #Update total angle away from origin y-axis in degrees
            angle = (angle)%360
        else:
            print('theta error!\n\n\n\n')
        #if angle < 0:

           # angle = angle%(-360)
        #else:
            #angle = angle%(360)
        print('x_coord ' + str(x_coord))
        X.append(x_coord)
        Y.append(y_coord)
        print('y_coord ' + str(y_coord))
        print('angle is: '+ str(angle))
        last_left = left_distance
        last_right = right_distance

        #pass odom data to self
        self.x = x_coord
        self.y = y_coord
        self.theta = angle
    def visualize(self, start):
        print(np.array(X) + start.x,np.array(Y)+start.y)
        plt.plot(np.array(X)+ start.x,np.array(Y)+start.y)
        plt.xlabel('X position')
        plt.ylabel('Y position')
        #plt.xlim(-1000, 1000)
        #plt.ylim(-1000, 1000)
        plt.title("desired trajectory and actual trajectory")
        plt.legend("desired path", "actual path")
        plt.show()
    
    
        
class RepeatTimer(Timer):  
    def run(self):  
        while not self.finished.wait(self.interval):  
            #pycreate2.Create2.get_sensor
            #bot = pycreate2.Create2(port=port, baud=baud['default'])
            self.function(*self.args,**self.kwargs)  
            #print(' ')  


'''
if __name__ == "__main__":
    # Create a Create2 Bot
    #port = '/dev/tty.usbserial-DA01NX3Z'  # this is the serial port on my iMac
    port = '/dev/ttyUSB0'  # this is the serial port on my raspberry pi
    baud = {
        'default': 115200,
        'alt': 19200  # shouldn't need this unless you accidentally set it to this
    }

    #Assign bot to pycreate2 object and assign serial port and baud rate
    bot = pycreate2.Create2(port=port, baud=baud['default'])

    #Start iRobot in safe mode
    bot.start()
    bot.safe()   
    #global l_encd_init
    #global r_encd_init
    
    
    sensors = bot.get_sensors()
    l_encd_init = sensors.encoder_counts_left
   
    r_encd_init = sensors.encoder_counts_right

    #movem updates sensor values every 15 ms, cannot send commands more frequently than that

    start_time = time.time()
    print('Starting...')

    
    #t = RepeatTimer(1, movem.odometry, bot)
    odom = RepeatTimer(0.001, movem.odometry)
    odom.start()
    
    sounding = RepeatTimer(2, movem.sound)
    #sounding.start()
    
    
    
    

    #################################################Test code below
    #sensors =
    #time.sleep(10)
    #movem.rotate(-90, -0.85)
    #movem.square(300, -1)
    #movem.forward(500, 100)
    #movem.backward(500,100)
    
    
    for vec in dv:
        dist = vec[0].mag()
        dist = dist*1000
        print('dist')
        print(dist)
        ang = vec[1]-math.pi/2
        angle - ang*180/math.pi
        #print(angle)
        #print(rot)
        movem.rotate(angle - ang*180/math.pi, 0.85*np.sign(angle - ang*180/math.pi))
        movem.forward(dist, 100)
        #time.sleep(10)
    
    #time.sleep(1)
    #print(dv)
    odom.cancel()
    #sounding.cancel()
    bot.stop()
    #plt.plot(L)
    #plt.show()
    #plt.plot(R)
    #plt.show()
    plt.plot(X,Y)
    plt.xlabel('X position')
    plt.ylabel('Y position')
    plt.xlim(-1000, 1000)
    plt.ylim(-1000, 1000)
    plt.show()
    
'''