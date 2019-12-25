#!/usr/bin/env python

# This node receives the array of segmented objects in "/camera_link" frame and the gaze position
# It returns the array of objects transformed to the "/world" link, the target object index and the optimal grasp point

import tf
import rospy
import geometry_msgs.msg
import pylab
import matplotlib.pyplot as plt
import numpy as np
import math
import matplotlib.patches as patches
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2
from main_package.srv import*
from mpl_toolkits.mplot3d import axes3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib import colors

def gaze_optimiser_server():
    # Initialize server proxy for gaze omptimiser service
    s = rospy.Service('gaze_optimiser_service', gazeOptimiser, handle_objects)
    print "Ready to receive objects."
    rospy.spin()
    print("Not get here")

def knn_search(x, D, K):
     # find nearest bounding box to gaze point in order to identify target object
     ndata = D.shape[1]

     K = K if K < ndata else ndata
     # euclidean distances from the other points
     sqd = np.sqrt(((D - x[:,:ndata])**2).sum(axis=0)) # calculating Euclidean distance
     idx = np.argsort(sqd) # sorting the array

     # return the indexes of nearest neighbour
     return idx[:K]

def correct_ik(grasp_point, wrist_co):
    # inverse kinematic calculations to return the position of the elbow joint
    e_co = geometry_msgs.msg.Point() # Elbow joint coordinates
    sh_co = geometry_msgs.msg.Point() # Shoulder joint coordinates
    # Note, shoulder joint is used only for the plots
    e_co.z = grasp_point.z
    sh_co.x = -0.18
    sh_co.y = 0.36
    sh_co.z = 0.54
    u_length = 0.36 # upper arm length
    f_length = 0.38 # forearm length

    # Calculate forearm angle
    e_angle = math.atan((grasp_point.y - wrist_co.y)/(grasp_point.x - wrist_co.x + f_length))
    elbow_angle = math.degrees(e_angle)

    # Calculate elbow joint position
    e_co.x = grasp_point.x - f_length*math.cos(e_angle)
    e_co.y = grasp_point.y - f_length*math.sin(e_angle)

    return (e_co, sh_co)

def handle_objects(req):

    transformer = tf.TransformListener()

    # Iterating through the array from the segmentation node and adding object with the respective center point coordinates and dimensions
    # The array is structurer as follows: [xDim, yDim, zDim, xCenter, yCenter, zCenter]

    objects = req.objects.data
    gaze = req.gaze_point
    print(objects)
    print("--------------")
    print("Gaze point: %s" %gaze)
    num_objects = int(len(objects)/6) # number of object

    world_objects = Float32MultiArray()
    world_objects.data = [0]*len(objects)
    center = np.empty([2, num_objects]) # array that will contain the x,y center point of the objects

    i = 0
    while i < len(objects):
        # Transform the center point of the objects from camera_link frame to world frame
        transformer.waitForTransform("camera_link", "world", rospy.Time(0),rospy.Duration(4.0))
        pointstamp = geometry_msgs.msg.PointStamped()
        pointstamp.header.frame_id = "camera_link"
        pointstamp.header.stamp = rospy.Time()
        pointstamp.point.x = objects[i+3]
        pointstamp.point.y = objects[i+4]
        pointstamp.point.z = objects[i+5]

        # Converting the center point of the object to "/world" frame
        p_tr = transformer.transformPoint("world", pointstamp)

        # Recalculating the z center point and z height of each object so that they are flat on the table
        height_to_table = 0.165 - (p_tr.point.z - (objects[i+2]/2))
        box_z = 0.11 + (objects[i+2]/2)
        box_z = box_z + (p_tr.point.z - box_z)/2

        # Reassign objects to the new array of objects in "/world" frame
        world_objects.data[i] = objects[i]
        world_objects.data[i+1] = objects[i+1]
        world_objects.data[i+2] = objects[i+2] + (p_tr.point.z - box_z)
        world_objects.data[i+3] = p_tr.point.x
        world_objects.data[i+4] = p_tr.point.y
        world_objects.data[i+5] = box_z

        object_id = str(i/6)

        # Print coordinates of each object
        print("x,y,z = ")
        print(world_objects.data[i+3])
        print(world_objects.data[i+4])
        print(world_objects.data[i+5])
        print("------")

        i+=6

    # 1 for plots and animation, 0 for no plots
    plot_request = 0
    if(plot_request==1):
         fig = plt.figure(figsize=(10,10/(1.7/0.8)))
         ax = fig.add_subplot(111)
         ax.axis([-0.2,1.7, -0.2, 0.75])
         plt.gca().invert_xaxis()

    j = 0
    while j < len(objects):
        iteration = int(j/6)
        center[0,iteration] = world_objects.data[j+3]
        center[1,iteration] = world_objects.data[j+4]
        if(plot_request==1):
            # Plotting the 2D objects bounding box
            rect = patches.Rectangle((center[1,iteration]-world_objects.data[j+1]/2,center[0,iteration]-world_objects.data[j]/2),world_objects.data[j+1],world_objects.data[j],linewidth=1,edgecolor='r',facecolor='none')
            ax.add_patch(rect)
        j+=6

    print(center)

    x = np.array([[gaze.x],[gaze.y]]) # Gaze point
    neig_idx = knn_search(x,center,1) # Find target object
	neig_idx = int(neig_idx)
    index = int(neig_idx) # Index of the target object
    print("Index %s" %index)

    print("target object xyz:")
    print(world_objects.data[neig_idx*6 + 3])
    print(world_objects.data[neig_idx*6 + 4])
    print(world_objects.data[neig_idx*6 + 5])

    # X and Y offset for the grasping point
    offset_x = -0.015
    offset_y = -0.015

    # Calculate optimal grasp point (with offset)
    grasp_point = geometry_msgs.msg.Point()
    grasp_point.x = world_objects.data[neig_idx*6 + 3] - offset_x # - world_objects.data[neig_idx*6]/2
    grasp_point.y = world_objects.data[neig_idx*6 + 4] - offset_y #- world_objects.data[neig_idx*6 + 1]/2 - offset_y
    grasp_point.z = world_objects.data[neig_idx*6 + 5]

    print("Grasp point: %s" %grasp_point)

    ########################
    ## The next block is just for plots and information
    # Initial wrist position
    wrist_co = geometry_msgs.msg.Point()
    wrist_co.x = 0.2
    wrist_co.y = 0.36
    wrist_co.z = 0.2

    co_e, co_s = correct_ik(grasp_point, wrist_co)
    print("Coord %s" %co_e)


    if(plot_request==1):
        # Highlighting the target object center
        pylab.plot(center[1,neig_idx],center[0,neig_idx],'o', markerfacecolor='None',markersize=15,markeredgewidth=1)
        # Plot center points of objects
        ax.plot(center[1,:],center[0,:],'ob',x[1,0],x[0,0],'or',grasp_point.y,grasp_point.x,'og')
        # Plot target obejct
        ax.plot(center[1,neig_idx],center[0,neig_idx],'o', markerfacecolor='None',markersize=15,markeredgewidth=1)
        # plot the arm and joints
        plt.plot([co_s.y,co_e.y,grasp_point.y],[co_s.x,co_e.x,grasp_point.x])
        plt.plot([wrist_co.y,grasp_point.y],[wrist_co.x - 0.05,grasp_point.x + offset_x])
        plt.plot([co_s.y,co_e.y,grasp_point.y],[co_s.x,co_e.x,grasp_point.x],'o')
        plt.show(block=False)
        rospy.sleep(5.0)
        plt.close('all')

    print("Grasp point: %s" %grasp_point)
    print("Elbow point: %s" %co_e)
    ############################

    # Sending the service response back to the "Controller" node
    srv_response = gazeOptimiserResponse()
    srv_response.sorted_objects = world_objects
    srv_response.target_id = index
    srv_response.grasp_point = grasp_point

    return srv_response

if __name__ == '__main__':
    rospy.init_node('gaze_optimiser', anonymous=True)
    # Initialize the gaze optimiser service
    gaze_optimiser_server()
    print ("Ready to receive objects.")
