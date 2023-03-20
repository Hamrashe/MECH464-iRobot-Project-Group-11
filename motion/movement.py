import serial
import time
import pycreate2
#from pycreate2 import Create2

class bot(object):

    #Max speed of both wheels are 500 mm/s
    def rotate(angle, ang_speed):
        #Initialize angle sensor
        cur_ang = sensors.angle
        
        #Distance from wheels to center of iRobot in mm (According manual, 235 is the distance between the wheels)
        wheel_rad = 235/2
        lin_vel = ang_speed*wheel_rad
        
        #Assume positive angle and ang_speed is CCW
        if ang_speed > 0:
            #(Left velocity, right velocity)
            while cur_ang < angle:
                bot.drive_direct(-lin_vel, lin_vel)
                cur_ang = cur_ang + sensors.angle
        else:
            while cur_ang > angle:
                bot.drive_direct(lin_vel, -lin_vel)
                cur_ang = cur_ang - sensors.angle

    def forward(dist, speed):
        #Initialize distance sensor, should be 0 at first
        cur_dist = sensors.distance

        while cur_dist < dist:
            #while iRobot traversal distance is less than desired distance, drive at *speed* to desired distance
            bot.drive_direct(speed,speed)
            
            #Adds distances together until desired distance is traversed
            cur_dist = cur_dist+sensors.distance
        bot.drive_stop

#Below if statement is from pycreate2 github, should we initialize the iRobot here?
if __name__ == "__main__":
    # Create a Create2 Bot
    #port = '/dev/tty.usbserial-DA01NX3Z'  # this is the serial port on my iMac
    port = '/dev/ttyUSB0'  # this is the serial port on my raspberry pi
    baud = {
        'default': 115200,
        'alt': 19200  # shouldn't need this unless you accidentally set it to this
    }

    bot = pycreate2.Create2(port=port, baud=baud['default'])
    
    #Roomba updates sensor values every 15 ms, cannot send commands more frequently than that
    sensors = bot.get_sensors()

    #Start iRobot in safe mode
    bot.start()
    bot.safe()

    print('Starting...')

    bot.forward(200, 100) #In mm and mm/s