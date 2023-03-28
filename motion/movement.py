import serial
import time
import pycreate2
import math
from machine import Timer
#from pycreate2 import Create2

#Constatnts
WHEEL_DIA_CORRECTION = 0.879314135
L_WHEEL_DIA_CORRECTION = 1.001420845
R_WHEEL_DIA_CORRECTION = 0.998579155
WHEEL_BASE_CORRECTION = 1.218447335



#Global odometry variables
x_coord = 0 #Positive x direction is east at the origin
y_coord = 0 #Positive y direction is north at the origin
angle = 0 #Degrees from y-axis at the origin

class roomba(object):

    #Max speed of both wheels are 500 mm/s
    def rotate(angle, ang_speed):
        #Initialize angle sensor
        #cur_ang = sensors.angle
        
        #Distance from wheels to center of iRobot in mm (According manual, 235 is the distance between the wheels)
        wheel_rad_nom = 235/2 #mm
        wheel_rad = wheel_rad_nom*WHEEL_BASE_CORRECTION #b_nom * E_b

        lin_vel = ang_speed*wheel_rad
        t = abs(angle/ang_speed)

        

        #Assume positive angle and ang_speed is CCW
        if ang_speed > 0:
            #(Left velocity, right velocity)
            roomba.drive(-lin_vel, lin_vel)
            time.sleep(t)
            bot.drive_stop


            #while cur_ang < angle:
                #bot.drive_direct(-lin_vel, lin_vel)
                
                #cur_ang = cur_ang + sensors.angle
        else:
            roomba.drive(lin_vel, -lin_vel)
            time.sleep(t)
            bot.drive_stop
            #while cur_ang > angle:
                #bot.drive_direct(lin_vel, -lin_vel)
                
                #cur_ang = cur_ang - sensors.angle

    def forward(dist, speed):
        #Initialize distance sensor, should be 0 at first
        cur_dist = sensors.distance
        
        t = dist/speed 
        print('drive has begun\n')
        roomba.drive(speed,speed)
        #bot.drive_direct(speed, speed)
        roomba.drive(speed, speed)
        time.sleep(t)
        bot.drive_stop
        print('drive has stopped\n')
        #print(f'distance travelled {sensors.distance}')
        '''
        while cur_dist < dist:
            #while iRobot traversal distance is less than desired distance, drive at *speed* to desired distance
            bot.drive_direct(speed,speed)
            
            #Adds distances together until desired distance is traversed
            cur_dist = cur_dist+sensors.distance
        bot.drive_stop
        '''
    def drive(v_L_nom, v_R_nom):
        #this function is for applying corrections before calling drive_direct
        v_L = v_L_nom*WHEEL_DIA_CORRECTION*L_WHEEL_DIA_CORRECTION #nominal speed * distance factor * difference in diameter scaling
        v_R = v_R_nom*WHEEL_BASE_CORRECTION*R_WHEEL_DIA_CORRECTION #nominal speed * distance factor * difference in diameter scaling
        bot.drive_direct(v_L, v_R)

    def square(L):
        #function to drive the robot in a square for UMB calibration

        roomba.forward(L,100)
        roomba.rotate(90, 0.85)
        roomba.forward(L,100)
        roomba.rotate(90, 0.85)
        roomba.forward(L,100)
        roomba.rotate(90, 0.85)
        roomba.forward(L,100)
        roomba.rotate(90, 0.85)

    




#Below if statement is from pycreate2 github, should we initialize the iRobot here?
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

    #Roomba updates sensor values every 15 ms, cannot send commands more frequently than that

    start_time = time.time()
    print('Starting...')

    #Test code below

    while time.time() - start_time < 10:

        sensors = bot.get_sensors()

        odom = Timer(0)
        odom.init(period=1000, mode=Timer.PERIODIC, callback = odometry)
