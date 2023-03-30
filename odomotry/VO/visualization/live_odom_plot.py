#The purpouse of this script is to house all our functions for visualizing data

from matplotlib import pyplot as plt
import matplotlib.animation as animation
import time


# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
Xs = [0] #X position array w/ starting x position 
Ys = [0] #Y postion array w/ starting y position 

corners_test = [[0,1], [1,2], [3, 1], [0,1]]

def live_odom_plot(X, Y):
    #Function to plot odomotry position while the main function runs
    

    
    

def draw_room(corners):
    #draws the walls and obstacles of the room on any existing plot, takes the set of points which represents the wals of the room as an input
    plt.plot(corners, 'k-', lw=2)


