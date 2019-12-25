#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import geometry_msgs.msg
from std_msgs.msg import Float32MultiArray
from main_package.srv import*

class ExecutionManager(object):
  """ExecutionManager"""
  def __init__(self):
    super(ExecutionManager, self).__init__()

    ## Initialize rospy`_ node:
    rospy.init_node('controller_node', anonymous=True)

    # Variable to store the pointcloud
    self.pCloud = PointCloud2()
    self.gaze_point = geometry_msgs.msg.Point()
    self.gaze_sent = 0

  # Storing the pointcloud from the camera depth topic
  def cloud_callback(self, pointcloud):
      self.pCloud = pointcloud


  # Waiting for a gaze point to be sent by the gaze_client
  def gaze_server_setup(self):
      gaze_server = rospy.Service('send_gaze', gazePoint, self.handle_gaze_point)
      print "Ready to send gaze point"
      rospy.spin()

  # Gaze service call back - send pointcloud to segmentation when gaze point is received
  def handle_gaze_point(self, req):
      print "Got gaze point: [x:%s  y:%s  z:%s]"%(req.gaze_x, req.gaze_y, req.gaze_z)
      self.gaze_point.x = req.gaze_x
      self.gaze_point.y = req.gaze_y
      self.gaze_point.z = req.gaze_z
      self.gaze_sent = 1
      objects = self.request_segmentation()
      # After segmentation, send objects to the gaze optimiser
      path_found = self.gaze_optimiser_caller(objects)
      if path_found == True:
          return gazePointResponse(True)
      elif path_found == False:
          return gazePointResponse(False)

  # Segmentation service that sends the obtained pointcloud to the segmentation node
  def request_segmentation(self):
      print("Sending pointcloud to segmentation")
      rospy.wait_for_service('segmentation_service')
      try:
          #print("Sending pointcloud to segmentation")
          segmentation_service = rospy.ServiceProxy('segmentation_service', seg)
          resp = segmentation_service(self.pCloud)
          # Obtain array of objects
          return resp.objects
      except rospy.ServiceException, e:
          print "Service call failed: %s"%e

  # Gaze optimiser service that sends the array of objects and gaze point to the gaze optimiser
  def gaze_optimiser_caller(self, objs):
      print("Sent data to gaze optimiser")
      rospy.wait_for_service('gaze_optimiser_service')
      try:
          gaze_optimiser_service = rospy.ServiceProxy('gaze_optimiser_service', gazeOptimiser)
          data = gaze_optimiser_service(objs,self.gaze_point)
          if(data!=None):
              print(data)
              # Send gaze optimiser output to path planner, if the array of objects is not empty
              self.path_planner_caller(data)
              return True
          else:
              return False
      except rospy.ServiceException, e:
          print "Service call failed: %s"%e


  def path_planner_caller(self, data):
      # Define the path planner request
      planner_req = pathPlannerRequest()
      planner_req.sorted_objects = data.sorted_objects
      planner_req.target_id = data.target_id
      planner_req.grasp_point = data.grasp_point
      print(planner_req)
      rospy.wait_for_service('path_planner_service')

      try:
          # Send data to path planner node
          path_planner_service = rospy.ServiceProxy('path_planner_service', pathPlanner)
          print("Sent data to path planner")
          planner_resp = path_planner_service(data.sorted_objects, data.target_id, data.grasp_point)
          print(planner_resp.path_found)
          # Execute the plan found or not
          if(planner_resp.path_found == True):
              option=raw_input("Execute? (y/n)")
              if option=="y":
                print("\n ")
                self.path_executer(True)
          return planner_resp.path_found

      except rospy.ServiceException, e:
          print "Service call failed: %s"%e

  # Service to execute the plan found
  def path_executer(self,req):
      rospy.wait_for_service('path_executer_service')
      try:
          # Send path execution order
          path_executer_service = rospy.ServiceProxy('path_executer_service', executionOrder)
          print("Sent order to path executer")
          executer_resp = path_executer_service(req)
          print(executer_resp)
          # Obtain True if path was executed
          return executer_resp
      except rospy.ServiceException, e:
          print "Service call failed: %s"%e

if __name__ == '__main__':
    # Declare main class
    controller = ExecutionManager()

    # Declare subscriber to the pointcloud topic
    rospy.Subscriber("/camera/depth_registered/points", PointCloud2, controller.cloud_callback, queue_size=1)
    rospy.sleep(1.0)

    # Setup the service that receives gaze point
    success = controller.gaze_server_setup()
    print(success)

    rospy.spin()
