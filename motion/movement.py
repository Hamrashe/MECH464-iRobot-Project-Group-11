import serial
import time
import pycreate2
import math
#from pycreate2 import Create2

#Constatnts
WHEEL_DIA_CORRECTION = 0.879314135
L_WHEEL_DIA_CORRECTION = 1.7
R_WHEEL_DIA_CORRECTION = 1
WHEEL_BASE_CORRECTION_CCW = 1.8
WHEEL_BASE_CORRECTION_CW = 1.218447335

#User inputs for constants (for adjusting odometry correction)
WHEEL_DIA_CORRECTION = input(f'Avg wheel diameter correction factor (last was {WHEEL_DIA_CORRECTION}):')
L_WHEEL_DIA_CORRECTION = input(f'Left wheel diameter correction factor (last was {L_WHEEL_DIA_CORRECTION}):')
R_WHEEL_DIA_CORRECTION = input(f'Right wheel diameter correction factor (last was {R_WHEEL_DIA_CORRECTION}):')
WHEEL_BASE_CORRECTION_CCW = input(f'Wheel base correction factor (last was {WHEEL_BASE_CORRECTION_CCW}):')
WHEEL_BASE_CORRECTION_CW = input(f'Wheel base correction factor (last was {WHEEL_BASE_CORRECTION_CW}):')

#print(f'Avg wheel diameter correction factor = {WHEEL_DIA_CORRECTION}\nLeft wheel diameter correction factor = {L_WHEEL_DIA_CORRECTION}\nRight wheel diameter correction factor = {R_WHEEL_DIA_CORRECTION}\nWheel base correction factor CCW = {WHEEL_BASE_CORRECTION_CW}\nWheel base correction factor CW = {WHEEL_BASE_CORRECTION_CW}')

#Global odometry variables
x_coord = 0 #Positive x direction is east at the origin
y_coord = 0 #Positive y direction is north at the origin
angle = 0 #Degrees from y-axis at the origin

class roomba(object):

    #Max speed of both wheels are 500 mm/s
    def rotate(angle, ang_speed):
        #Initialize angle sensor
        #cur_ang = sensors.angle
        angle_rad = angle*math.pi/180.0
        #Distance from wheels to center of iRobot in mm (According manual, 235 is the distance between the wheels)
        wheel_rad_nom = 235/2 #mm
        if angle <0:

            wheel_rad = wheel_rad_nom*WHEEL_BASE_CORRECTION_CCW #b_nom * E_b

        lin_vel = ang_speed*wheel_rad
        #t = abs(angle/ang_speed)
        stopTimeAng = abs(angle_rad/ang_speed)
        stopTimeAng = int(stopTimeAng)
        
        roomba.drive(-lin_vel, lin_vel)
        time.sleep(stopTimeAng)
        bot.drive_stop
        #Assume positive angle and ang_speed is CCW
       

    def forward(dist, speed):
        #Initialize distance sensor, should be 0 at first
        ##cur_dist = sensors.distance
        
        t = dist/speed 
        print('drive has begun\n')
        roomba.drive(speed,speed)
        #bot.drive_direct(speed, speed)
        roomba.drive(speed, speed)
        time.sleep(t)
        bot.drive_stop
        print('drive has stopped\n')
        #print(f'distance travelled {sensors.distance}')
       
    def drive(v_L_nom, v_R_nom):
        #this function is for applying corrections before calling drive_direct
        v_L = v_L_nom*WHEEL_DIA_CORRECTION*L_WHEEL_DIA_CORRECTION #nominal speed * distance factor * difference in diameter scaling
        v_R = v_R_nom*WHEEL_BASE_CORRECTION*R_WHEEL_DIA_CORRECTION #nominal speed * distance factor * difference in diameter scaling
        bot.drive_direct(int(v_L), int(v_R))

    def square(L, dir):
        #function to drive the robot in a square for UMB calibration

        roomba.forward(L,100)
        roomba.rotate(dir*90, dir*0.85)
        roomba.forward(L,100)
        roomba.rotate(dir*90, dir*0.85)
        roomba.forward(L,100)
        roomba.rotate(dir*90, dir*0.85)
        roomba.forward(L,100)
        roomba.rotate(dir*90, dir*0.85)

    




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
	
	
    #roomba.forward(500, 100)
    roomba.square(400, 1)
    print(f'Avg wheel diameter correction factor = {WHEEL_DIA_CORRECTION}\nLeft wheel diameter correction factor = {L_WHEEL_DIA_CORRECTION}\nRight wheel diameter correction factor CCW= {R_WHEEL_DIA_CORRECTION}\nWheel base correction factor CCW = {WHEEL_BASE_CORRECTION_CW}\nWheel base correction factor CW =  {WHEEL_BASE_CORRECTION_CW}')

