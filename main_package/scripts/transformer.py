#!/usr/bin/env python

# general imports
import sys

# ros specific imports
import rospy
import numpy as np
import tf
import tf2_ros
from tf.transformations import *
from geometry_msgs.msg import TransformStamped, Point
from natnet_msgs.msg import MarkerList

def transform(data):
    if len(data.positions) < 4:
        return
    marker0 = data.positions[0]
    marker1 = data.positions[1]
    marker2 = data.positions[2]
    marker3 = data.positions[3]
    ## In our 3D print, one of the markers is deviated, this is fixed here:
    # marker3.z -= 0.005

    ## Finding the position of the virual centre of mass point:
    body = Point()
    body.x = np.mean([marker0.x,marker1.x,marker2.x,marker3.x])
    body.y = np.mean([marker0.y,marker1.y,marker2.y,marker3.y])
    body.z = np.mean([marker0.z,marker1.z,marker2.z,marker3.z])

    ## Finding the vectors defining the rotated coordinate system of the lens
    v30 = np.array(((marker3.x-marker0.x),(marker3.y-marker0.y),(marker3.z-marker0.z)))
    v01 = np.array(((marker0.x-marker1.x),(marker0.y-marker1.y),(marker0.z-marker1.z)))
    # v10 = np.array(((marker1.x-marker0.x),(marker1.y-marker0.y),(marker1.z-marker0.z)))
    d30 = vector_norm(v30)
    d01 = vector_norm(v01)
    # d10 = vector_norm(v10)
    ## Normalising the vectors, and creating the normal vector
    # ux = v30/d30
    # uz = -v10/d10
    uy = -v01/d01
    # vy = np.cross(uz,ux)
    vz = np.cross(v30/d30,uy)
    dvz = vector_norm(vz)
    uz = vz/dvz
    ux = np.cross(uy,uz)
    ## Rotation matrix can be obtained as follows:
    rot = np.array((ux,uy,uz)).transpose()

    rotation_matrix = np.array([[rot[0][0], rot[0][1], rot[0][2], 0],
                                [rot[1][0], rot[1][1], rot[1][2], 0],
                                [rot[2][0], rot[2][1], rot[2][2], 0],
                                [0, 0, 0, 1]])

    quat = quaternion_from_matrix(rotation_matrix)

    dynamic_broadcaster = tf2_ros.TransformBroadcaster()
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    tf_abs = TransformStamped()
    tf_abs.header.stamp = rospy.Time.now()
    tf_abs.header.frame_id = "mocap"
    tf_abs.child_frame_id = "head"
    tf_abs.transform.translation.x = body.x
    tf_abs.transform.translation.y = body.y
    tf_abs.transform.translation.z = body.z
    tf_abs.transform.rotation.x = quat[0]
    tf_abs.transform.rotation.y = quat[1]
    tf_abs.transform.rotation.z = quat[2]
    tf_abs.transform.rotation.w = quat[3]

    tf_world = TransformStamped()
    tf_world.header.stamp = rospy.Time.now()
    tf_world.header.frame_id = "world"
    tf_world.child_frame_id = "mocap"
    tf_world.transform.translation.x = 0.4876457489909373
    tf_world.transform.translation.y = 1.0021416179753073
    tf_world.transform.translation.z = 0.07989414854903898
    tf_world.transform.rotation.x = -0.004159865089929733
    tf_world.transform.rotation.y = 0.007377069799192935
    tf_world.transform.rotation.z = -0.003486532468107288
    tf_world.transform.rotation.w = 1.0004041253078304

    tf_abs_cam = TransformStamped()
    tf_abs_cam.header.stamp = rospy.Time.now()
    tf_abs_cam.header.frame_id = "head"
    tf_abs_cam.child_frame_id = "camera_link"
    tf_abs_cam.transform.translation.x = 0.0
    tf_abs_cam.transform.translation.y = -0.0325
    tf_abs_cam.transform.translation.z = -0.03120645
    tf_abs_cam.transform.rotation.x = 0
    tf_abs_cam.transform.rotation.y = 0
    tf_abs_cam.transform.rotation.z = 0
    tf_abs_cam.transform.rotation.w = 1

    dynamic_broadcaster.sendTransform(tf_abs)
    static_broadcaster.sendTransform(tf_abs_cam)
    static_broadcaster.sendTransform(tf_world)

def listener():
    rospy.init_node('transformer', anonymous=True)
    rospy.loginfo("Subscribing to MoCap Data")
    rospy.Subscriber('/mocap/rigid_bodies/RigidBody1/markers',MarkerList, transform)
    rospy.spin()


if __name__ == '__main__':
    listener()
