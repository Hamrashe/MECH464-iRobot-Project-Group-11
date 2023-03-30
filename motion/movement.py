import serial
import time
import pycreate2
import math
import threading
#from machine import Timer
#from pycreate2 import Create2

#Constatnts
WHEEL_DIA_CORRECTION = 0.879314135
L_WHEEL_DIA_CORRECTION = 1
R_WHEEL_DIA_CORRECTION = 1
WHEEL_BASE_CORRECTION_CCW = 2.0723
WHEEL_BASE_CORRECTION_CW =2.0723

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

class roomba(object):

    #Max speed of both wheels are 500 mm/s
    def rotate(angle, ang_speed):
        #Initialize angle sensor
        #cur_ang = sensors.angle
        
        #Distance from wheels to center of iRobot in mm (According manual, 235 is the distance between the wheels)
        wheel_rad_nom = 235.0/2 #mm
        if angle <0:

            wheel_rad = float(wheel_rad_nom)*WHEEL_BASE_CORRECTION_CCW #b_nom * E_b
        else:
            wheel_rad = float(wheel_rad_nom)*WHEEL_BASE_CORRECTION_CW

        lin_vel = ang_speed*wheel_rad

        lin_vel = int(lin_vel) #Convert to int so it can be used for drive command
        
        angle_rad = angle*(math.pi)/180.0
        stopTimeAng = angle_rad/ang_speed

        #(Left velocity, right velocity)
        #while cur_ang < angle:
        bot.drive_direct(-lin_vel, lin_vel)
        time.sleep(stopTimeAng)
        bot.drive_stop
        #cur_ang = cur_ang + sensors.angle

    def forward(dist, speed):
        #Initialize distance sensor, should be 0 at first
        #cur_dist = sensors.distance

        stopTime = dist/speed

        #while iRobot traversal distance is less than desired distance, drive at *speed* to desired distance
        bot.drive_direct(speed,speed)
        time.sleep(stopTime)
        bot.drive_stop
        print('drive has stopped\n')
        #print(f'distance travelled {sensors.distance}')
       
    def drive(v_L_nom, v_R_nom):
        #this function is for applying corrections before calling drive_direct
        v_L = v_L_nom*WHEEL_DIA_CORRECTION*L_WHEEL_DIA_CORRECTION #nominal speed * distance factor * difference in diameter scaling
        v_R = v_R_nom*WHEEL_DIA_CORRECTION*R_WHEEL_DIA_CORRECTION #nominal speed * distance factor * difference in diameter scaling
        bot.drive_direct(int(v_L), int(v_R))

    def square(L, dir):
        #function to drive the robot in a square for UMB calibration
        roomba.forward(L,100)
        roomba.rotate(90, 0.85)
        roomba.forward(L,100)
        roomba.rotate(90, 0.85)
        roomba.forward(L,100)
        roomba.rotate(90, 0.85)
        roomba.forward(L,100)
        roomba.rotate(90, 0.85)

    def sound():
        song = [72, 16] #(note, duration), ...
        song_num = 0
        bot.createSong(song_num, song) 
        time.sleep(1)
        bot.playSong(song_num)
    
    def odometry():
        #update global variables of x, y, angle of robot
        print('Interrupted')

        left_encoder = sensors.encoder_counts_left
        right_encoder = sensors.encoder_counts_right

        left_distance = left_encoder * math.pi * 72.0 / 508.8 #Converts encoder counts into distance in mm
        right_distance = right_encoder * math.pi * 72.0 / 508.8 #Converts encoder counts into distance in mm

        center_distance = (left_distance+right_distance)/2 #Get new center distance

        #ang_rad = angle * math.pi / 180.0
        dtheta = (left_distance - right_distance)/235.0 #Get change in angle
        dtheta_rad = dtheta*math.pi/180.0
        
        x_coord += center_distance*math.sin(dtheta) #Update x_coord from origin
        y_coord += center_distance*math.cos(dtheta) #Update y_coord from origin
        angle += dtheta #Update total angle away from origin y-axis in degrees

        print(x_coord)
        print(y_coord)
        print(angle)
        

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

    poll_time = 2.0 #How often to pull (every x seconds)
    t = threading.Timer(poll_time, roomba.odometry)
    t.start()

    #Test code below
    while time.time() - start_time < 10:

        sensors = bot.get_sensors()

        time.sleep(1)
    
    t.cancel()