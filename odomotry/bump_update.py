#bump_update.py
#the function of the main script of this file is to be given the robots position, direction and the position of the nearest obstacle, and return the translation needed to update the robot's odometry

#A cool idea would be to have this main function run only when it is close to an obstacle (?Maybe? not sure if that would work actually)

#NOTE: this code is pretty messy rn, I may need to restart it, or rework it quite a bit. I'll look back at my proposed algorithm

from numpy import array
from numpy.linalg import norm
import numpy as np
import math


#######Main Function#######################################################################################################################
def position_error(obs_pos, robot_pos, robot_dir):
    #Note, all the above inputs should be passed as np arrays

    obs_dir = obs_pos-robot_pos
    

    #rotate the robot until it's aligned with the direction of the obstacle within 1%
    while angle_between_vectors(obs_dir, robot_dir) > 0.017:
        #rotate(angular speed = pi rad/s)




    return pos_error
    



#calulate the angle between two vectors
def angle_between_vectors(t,s):

    #normalizing vectors
    t = t/norm(t)
    s = s/norm(s)

    #calculating the angle
    ang = 2*math.atan(norm(s - t))/norm(s +t)
    return ang


t = [[1], 
     [2]]

s = [[5],
     [9]]


print(t)
vec = np.array(t)
print(vec+vec)
print(angle_between_vectors(t,s))