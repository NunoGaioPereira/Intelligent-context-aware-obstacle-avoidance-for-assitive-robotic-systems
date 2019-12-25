#!/usr/bin/env python

import rospy
import sys
from main_package.srv import*

###############
# This script is the gaze service gaze_client
# It sends a gaze point to the controller node
# To use type as follows in the command line:
# rosrun segmentation gaze_client.py 0.48 0.65 0.2
###############

def send_gaze_client(gaze_x, gaze_y, gaze_z):
    rospy.wait_for_service('send_gaze')
    try:
        send_gaze = rospy.ServiceProxy('send_gaze', gazePoint)
        resp = send_gaze(gaze_x, gaze_y, gaze_z)
        return resp.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# In case the service is not called in the correct way
def usage():
    return "%s [x y z]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 4:
        # Get gaze point from terminal
        gaze_x = float(sys.argv[1])
        gaze_y = float(sys.argv[2])
        gaze_z = float(sys.argv[3])
    else:
        print usage()
        sys.exit(1)
    print "Gaze point is x:%s y:%s z:%s"%(gaze_x, gaze_y, gaze_z)
    # True if service got through:
    print "Gaze point sent state: %s"%(send_gaze_client(gaze_x, gaze_y, gaze_z))
