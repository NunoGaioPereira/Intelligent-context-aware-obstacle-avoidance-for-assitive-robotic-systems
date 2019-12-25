#!/usr/bin/env python

import rospy
import random
import pylab
import matplotlib.pyplot as plt
import numpy as np
from geometry_msgs.msg import Point
from matplotlib import colors



##############################
# This script creates a random distribution of objects for the experiments
##############################

for n in range(25):
    name = ""
    grid = np.array([[0,0,0,0],
                    [0,0,0,0],
                    [0,0,0,0]])


    cols = np.size(grid,1)
    rows = np.size(grid,0)
    ratio = cols*1.0 / rows
    objs = ['Bottle','Orange','Box','Cup','Mug']
    min_obj = 2
    max_obj = 4

    gaze_array = []*(cols*rows)

    # number of objects in the scene
    num_obj = random.randint(min_obj, max_obj)
    print 'Number of objects:  ', num_obj, '\n'

    # order of objects to each position
    obj_order = random.sample(range(0, len(objs)),num_obj)
    # print obj_order, '\n'
    #
    # Generate random positions in the grid for each objects
    obj_position = random.sample(range(0, cols*rows),num_obj)

    # Randomly assign target object
    target_obj = random.choice(obj_order)
    print'Target object: ', objs[target_obj], '\n'

    # Create plot figure
    cmap = colors.ListedColormap(['white','#0060ff','green']) ## '#0060ff','yellow'
    plt.figure(figsize=(6,6/ratio))


    for i in range(num_obj):
        print objs[obj_order[i]], "will go in grid position: ", obj_position[i]

        # Convert 1D position to 2D position
        x_i = obj_position[i]%cols
        y_i = obj_position[i]/cols

        # target object
        if(obj_order[i] == target_obj):
            grid[y_i][x_i] = 1
            plt.text(x_i + 0.3,y_i + 0.5, objs[obj_order[i]], fontsize=12)
        # obstacle
        else:
            grid[y_i][x_i] = 2
            plt.text(x_i + 0.3,y_i + 0.5, objs[obj_order[i]], fontsize=12)

    plt.pcolor(grid[::1],cmap=cmap,edgecolors='k', linewidths=1)

    # print grid number
    for i in range(cols*rows):
        plt.text(i%cols+0.1,i/cols+0.1, i, fontsize=15)

    # grid.shape

    # plt.gca().invert_xaxis()
    name = str(n)
    plt.show(block=False)
    plt.savefig(name)
    rospy.sleep(0.1)
    plt.close('all')
