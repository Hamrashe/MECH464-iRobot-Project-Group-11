import serial
import time
import pycreate2

#Below if statement is from pycreate2 github, should we initialize the iRobot here?
if __name__ == "__main__":
    # Create a Create2 Bot
    port = '/dev/tty.usbserial-DA01NX3Z'  # this is the serial port on my iMac
    # port = '/dev/ttyUSB0'  # this is the serial port on my raspberry pi
    baud = {
        'default': 115200,
        'alt': 19200  # shouldn't need this unless you accidentally set it to this
    }

    bot = pycreate2.Create2(port=port, baud=baud['default'])

#Roomba updates sensor values every 15 ms, cannot send commands more frequently than that
sensors = bot.get_sensors()
#sensors.wall == sensors[1] --> Example to get wall sensor, refer to documentation

#Max speed of both wheels are 500 mm/s
def rotate(ang_speed):
    #Distance from wheels to center of iRobot in mm (According manual, 235 is the distance between the wheels)
    wheel_rad = 235/2
    lin_vel = ang_speed*wheel_rad
    
    #Assume positive ang_speed is CCW
    if ang_speed > 0:
        #(Left velocity, right velocity)
        bot.drive_direct(-lin_vel, lin_vel)
    else:
        bot.drive_direct(lin_vel, -lin_vel)

    #Add when to stop?

def forward(dist, speed):
    #Add distance tracking function from iRobot?

    #Initialize distance sensor, should be 0 at first
    cur_dist = sensors.distance

    #The distance that Roomba has traveled in millimeters since the distance it was last requested is sent as a
    #signed 16-bit value, high byte first. This is the same as the sum of the distance traveled by both wheels
    #divided by two. Positive values indicate travel in the forward direction; negative values indicate travel in
    #the reverse direction. If the value is not polled frequently enough, it is capped at its minimum or
    #maximum. 

    while cur_dist < dist:
        #while iRobot traversal distance is less than desired distance, drive at *speed* to desired distance
        bot.drive_direct(speed,speed)
        
        #Adds distances together until desired distance is traversed
        cur_dist = cur_dist+sensors.distance
    bot.drive_stop