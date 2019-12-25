#!/usr/bin/env python
import tf
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped

# def callback(pose):
# 	# print(pose.pose)
# 	translation = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
# 	q = (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
# 	rotation_euler = tf.transformations.euler_from_quaternion(q) #Convert a quaternion message into an angle in radians.
# 	tf_br.sendTransform((translation[0],translation[1],translation[2]), (q[0], q[1], q[2], q[3]),rospy.Time.now(), "camera_link", "mocap")
# 	tf_br.sendTransform((t[0], t[1], t[2]), (r[0], r[1], r[2], r[3]),rospy.Time.now(), "mocap", "world")
#
#
# if __name__ == '__main__':
# 	# Initialize the node
# 	rospy.init_node('camera_tf_node', anonymous="True")
# 	tf_br = tf.TransformBroadcaster() # Broadcaster for camera_link
#
# 	# Transformation matrix
# 	A = np.array([[1.0, 0, 0, 0.42786135388386903],
# 		    [0, 1.0, 0, 0.97182492778108842],
# 		    [ 0, 0,  1.0, 0.13883507833401487],
# 		    [0.0, 0.0, 0.0, 1.0]])
#
# 	r = tf.transformations.quaternion_from_matrix(A)
# 	t = tf.transformations.translation_from_matrix(A)
#
# 	rospy.Subscriber("/mocap/rigid_bodies/RigidBody_01/pose", PoseStamped, callback, queue_size=1)
# 	rospy.spin()


#####################################
## Use the block below (and comment the block above) for tests without Optitrack, with the camera in a stationary position.

rospy.init_node('camera_tf_node', anonymous="True")
tf_br = tf.TransformBroadcaster() # Broadcaster for camera_link
# pub = rospy.Publisher('gaze', Point, queue_size=10)
rate = rospy.Rate(100) # 3 Hz
A = np.array([[1.0, 0.0090973927011712696, 0.013948279486413945, 0.42786135388386903],
	    [0.00027708163676559705, 1.0, 0.0, 0.97182492778108842],
	    [ -0.015069175090019371, 0.0,  1.0, 0.13883507833401487],
	    [0.0, 0.0, 0.0, 1.0]])

r = tf.transformations.quaternion_from_matrix(A)
t = tf.transformations.translation_from_matrix(A)

while not rospy.is_shutdown():
	# camera on top of box
	## Publish a static transformation
	tf_br.sendTransform((-0.42427238822,-0.339268058538, 0.313522875309), ( -0.085530012846,0.201468780637,  0.0283537395298,  0.975341498852),rospy.Time.now(), "camera_link", "mocap")

	## tf_br.sendTransform((-0.589579701424,-0.279752343893,0.329931378365), (-0.0747203528881, 0.102493800223, 0.0811263248324, 0.988600313663),rospy.Time.now(), "camera_link", "mocap")
	# tf_br.sendTransform(( -0.686933040619,0.355587393045,0.316830247641), (0.0121359312907, 0.0546515248716,  0.227592453361, -0.972145974636),rospy.Time.now(), "camera_link", "mocap")
	tf_br.sendTransform((t[0], t[1], t[2]), (r[0], r[1], r[2], r[3]),rospy.Time.now(), "mocap", "world")
	rate.sleep()

################
