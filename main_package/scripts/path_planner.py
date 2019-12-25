#!/usr/bin/env python
import tf
import rospy
import math
import sys
import copy
import moveit_commander
import moveit_python
import numpy as np
import moveit_msgs.msg
import geometry_msgs.msg
import pylab
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from moveit_commander.conversions import pose_to_list
from main_package.srv import*
from matplotlib import colors
from heapq import *


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonInterface(object):
  """MoveGroupPythonInterface"""
  def __init__(self):
    super(MoveGroupPythonInterface, self).__init__()

    ## Initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('path_planner', anonymous=True)

    ## Interface to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Interface to one group of joints.
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    # Used to at the objects to the planning scene
    self.objectAdder = moveit_python.PlanningSceneInterface("world")
    self.box_name = ''
    self.scene = scene
    self.group = group

    # Target object id will be stored
    self.target_obj_id = 0

    # This variable flag when the "Top planner" is used
    self.orientation_plan = 0

    self.f_length = 0.38 # Forearm length



  def go_to_initial_state(self):
    # Send robot to initial position
    self.group.set_planning_time(10)

    self.waypoints = []
    wpose = self.group.get_current_pose().pose

    # Initial position waypoint
    wpose.position.x = 0.2
    wpose.position.y = 0.36
    wpose.position.z = 0.2
    wpose.orientation.x = 0.00111054358639
    wpose.orientation.y = 0.70699483645
    wpose.orientation.z = 0.00111089701837
    wpose.orientation.w = 0.707216963763

    self.waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = self.group.compute_cartesian_path(
                                       self.waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0, True)


    self.group.execute(plan, wait = True)
    self.group.clear_pose_targets()

    current_pose = self.group.get_current_pose().pose
    return all_close(wpose, current_pose, 0.01)

  def path_planner_server(self):
      # Initialize server proxy for path_planner_service
      s = rospy.Service('path_planner_service', pathPlanner, self.planner)
      print "Ready to plan."
      rospy.spin()

  def path_executer_server(self):

      no = len(self.scene.get_known_object_names()) # Number of objects in the scene

      #  Loop to remove all the objects
      f = 0
      while f < no:
          box_nam = str(f)
          self.scene.remove_world_object(box_nam)
          # print("Removed object number ")
          # print(f)
          f = f + 1

      # Initialize server proxy for path_planner_service
      s = rospy.Service('path_executer_service', executionOrder, self.path_executer)
      print "Ready to execute path."
      #rospy.spin()
      # print("Not get here")

  def planner(self,request):
      # Main trajectory planner
      plot_request = 0 # 0 for no plots, 1 for plots

      # Handle request data
      objects = request.sorted_objects.data
      optimal_grasp_point = request.grasp_point
      grasp_point = geometry_msgs.msg.Point()
      target_id = request.target_id
      self.target_obj_id = target_id

      # Add object bounding boxes to planning scene
      self.add_box(objects, target_id)

      # Setting up the collision grid
      # The number of rows and columns will dictate the resolution of the grid
      ratio = 1.2/0.7 # Ratio of y/x dimensions of the table
      rows = 240
      cols = int(rows/ratio) + 1
      print("num cols: %s" %cols)
      data = [[0 for _ in range(rows)] for _ in range(cols)] # Array of 0's and 1's. 1 represents a collision state
      Yresolution = 1.2/rows # Table y dimension is approximately 1.2m
      Xresolution = 0.7/cols # Table x dimension is approximately 0.7m


      print("Optimal grasp point: %s" %optimal_grasp_point)
      # Calculate index of the target position in the grid
      target_index = [round(optimal_grasp_point.x/Xresolution),round(optimal_grasp_point.y/Yresolution)]

      # Initial position of the robot end-effector (/wrist)
      wrist_co = geometry_msgs.msg.Point()
      wrist_co = self.group.get_current_pose().pose.position

      elbow_angle = 0 # Variable to store the forearm angle

      # Trajectory initial position (where the wrist starts)
      start_point = [wrist_co.x,wrist_co.y]
      start = (int(round(start_point[0]/Xresolution)-1), int(round(start_point[1]/Yresolution)-1))
      end = (int(target_index[0]), int(target_index[1]))

      # Populate the collision grid:
      # Iterate through the (x,y) points grid along the table and find which states are in collision
      for i in range(rows):
          for j in range(cols):
              grasp_point.x = Xresolution*(j+1)
              grasp_point.y = Yresolution*(i+1)
              # Obtain elbow position and elbow angle
              co_e, co_s, elbow_angle = self.ik_calculator(grasp_point,wrist_co)
              # Calculate if the arm would be in collision in the given hand position
              collision_state = object_collision(co_e,grasp_point, objects, target_id)
              if(collision_state == True):
                  # If the state is in collision, it is reoresented by a '1', '0' otherwise
                  data[j][i] = 1

      # If the target state is not in collision, send collision grid, initial and final positions to A* algorithm
      if data[end[0]][end[1]] != 1:
          print("A star working...")
          path = astar2(np.array(data), start, end)
          print("A star done")

          if(path==False): # If no path was found
              print("The target state is in collision")
              return False

          path_xy =  [[0]*3 for k in range(len(path))] # Initialize path variable

          for j in range(len(path)):
                  r = path[j][0]
                  c = path[j][1]
                  data[r][c]= 4
                  path_xy[j][0]=Xresolution*r
                  path_xy[j][1]=Yresolution*c
                  grasp_point.x = path_xy[j][0]
                  grasp_point.y = path_xy[j][1]
                  co_e, co_s, elbow_angle = self.ik_calculator(grasp_point,wrist_co)
                  path_xy[j][2] = elbow_angle
          data[end[0]][end[1]]=2
          data[start[0]][start[1]]=3
          data = np.array(data)
          data.shape

          # Plot collision state grid
          # fig2 = plt.figure(figsize=(10,10/ratio))
          if(plot_request==1):
              cmap = colors.ListedColormap(['white','red','green','#0060ff','#777777'])
              plt.figure(figsize=(15,15/ratio))
              plt.pcolor(data[::1],cmap=cmap,edgecolors='k', linewidths=1)
              plt.gca().invert_xaxis()
              plt.show(block=False)
              rospy.sleep(5.0)
              plt.close('all')

          grasp_point.x = optimal_grasp_point.x
          grasp_point.y = optimal_grasp_point.y
          # grasp_point.x = 0.15
          # grasp_point.y = 0.38
          co_e, co_s, elbow_angle = self.ik_calculator(grasp_point,wrist_co)
          print("Grasp point: %s" %grasp_point)
          print("Elbow point: %s" %co_e)
          # print(path_xy)


          # Animation of the arm trajectory
          if(plot_request==1):
              fig = plt.figure(figsize=(10,10/(1.7/0.8)))
              ax = fig.add_subplot(111)

          center = np.empty([2, int(len(objects)/6)])
          for i in range(len(path_xy)):
              if(plot_request==1):
                  plt.cla()
                  ax.axis([-0.2,1.7, -0.2, 0.75])
                  plt.gca().invert_xaxis()
              #ax.add_patch(rect)

              j = 0
              while j < len(objects):
                  iteration = int(j/6)
                  center[0,iteration] = objects[j+3]
                  center[1,iteration] = objects[j+4]
                  if(plot_request==1):
                      rect = patches.Rectangle((center[1,iteration]-objects[j+1]/2,center[0,iteration]-objects[j]/2),objects[j+1],objects[j],linewidth=1,edgecolor='r',facecolor='none')
                      ax.add_patch(rect)

                  j+=6

              grasp_point.x = path_xy[i][0]
              grasp_point.y = path_xy[i][1]
              co_e, co_s, elbow_angle = self.ik_calculator(grasp_point,wrist_co)

              if(plot_request==1):
                  ax.plot(center[1,:],center[0,:],'ob',grasp_point.y,grasp_point.x,'og',co_e.y,co_e.x,'ok',0.38,-0.15,'oy')
                  ax.plot(center[1,target_id],center[0,target_id],'o', markerfacecolor='None',markersize=15,markeredgewidth=1)
                  ax.plot([co_s.y,co_e.y,grasp_point.y],[co_s.x,co_e.x,grasp_point.x])
                  plt.draw()
                  plt.pause(0.0001)

              # plt.cla()
              i+=1

          if(plot_request==1): # Close the plots
              rospy.sleep(3.0)
              plt.close('all')

          ##### The next block works as a window averaging filter to smooth the robot trajectory
          path_n = [[0 for col in range(2)] for row in range(len(path_xy))]
          path_x = [[0 for col in range(1)] for row in range(len(path_xy))]
          path_y = [[0 for col in range(1)] for row in range(len(path_xy))]

          for k in range(len(path_xy)):
              path_n[k][0] = path_xy[k][0]
              path_n[k][1] = path_xy[k][1]
              path_x[k] =  path_xy[k][0]
              path_y[k] =  path_xy[k][1]

          # Use convolution to create a window filter
          window_size = 6
          window = np.ones(int(window_size))/float(window_size)
          smooth_y = np.convolve(path_y, window, 'valid')
          smooth_x = np.convolve(path_x, window, 'valid')
          # fig, ax = plt.subplots(figsize=(25,25/(1.2/0.7)))
          # ax.plot(path_y, path_x, 'r.', label= 'Unsmoothed curve')
          # ax.plot(smooth_y, smooth_x, 'b.', label= 'Smoothed curve')

          # Calculating the new forearm orientation of the new path
          new_path = [[0 for col in range(len(smooth_x))] for row in range(len(smooth_y))]
          for j in range(len(new_path)):
              new_path[j][0] = smooth_x[j]
              new_path[j][1] = smooth_y[j]
              grasp_point.x = new_path[j][0]
              grasp_point.y = new_path[j][1]
              co_e, co_s, elbow_angle = self.ik_calculator(grasp_point,wrist_co)
              new_path[j][2] = elbow_angle

          plan_found = self.point_planner(new_path, optimal_grasp_point)

          if(plan_found == False):
              # If the "from the front" reach is not possible, two more strategies are tried
              print("The target state is in collision")

              # First, try to grasp the object from behind
              # Find the forearm angles that don't result in collision
              _, _, collision_angle = self.ik_calculator(optimal_grasp_point,wrist_co)
              collision_angle = math.degrees(collision_angle) + 0.51
              print(collision_angle)
              possible_angles  = []
              for alpha in range(int(collision_angle), 80): # Only allow angles smaller than 80
                  # Calculate elbow position for a given angle
                  alpha_rad = math.radians(alpha)
                  co_e.x = optimal_grasp_point.x - self.f_length*math.cos(alpha_rad)
                  co_e.y = optimal_grasp_point.y - self.f_length*math.sin(alpha_rad)
                  # Calculate if the forearm is in collision or not
                  collision_state = object_collision(co_e,optimal_grasp_point, objects, target_id)
                  if(collision_state == False):
                      if(alpha>62):
                          possible_angles.append(alpha) # Append the possible angles

              print(possible_angles)

              plan_found = self.behind_point_planner(optimal_grasp_point, possible_angles)
              if(plan_found == False):
                  # If the "behind planner" fails, try to avoid the obstacle from the top
                  plan_found = self.orientation_point_planner(optimal_grasp_point)
                  ## TODO: The planner is assuming the target object can be added from the top, disregarding the obstacles height
                  self.orientation_plan = 1
                  if(plan_found == False):
                      return False

              return True

          return True

      elif data[end[0]][end[1]] == 1:
          # If the target position is in collision "from the front", two more strategies are tried
          print("The target state is in collision")

          _, _, collision_angle = self.ik_calculator(optimal_grasp_point,wrist_co)
          collision_angle = math.degrees(collision_angle) + 0.51
          print(collision_angle)
          possible_angles  = []
          for alpha in range(int(collision_angle), 80):
              alpha_rad = math.radians(alpha)
              co_e.x = optimal_grasp_point.x - self.f_length*math.cos(alpha_rad)
              co_e.y = optimal_grasp_point.y - self.f_length*math.sin(alpha_rad)

              collision_state = object_collision(co_e,optimal_grasp_point, objects, target_id)
              if(collision_state == False):
                  if(alpha>60):
                      possible_angles.append(alpha)

          print(possible_angles)

          plan_found = self.behind_point_planner(optimal_grasp_point, possible_angles)
          if(plan_found == False):
              plan_found = self.orientation_point_planner(optimal_grasp_point)
              self.orientation_plan = 1
              if(plan_found == False):
                  return False

          return True

  def ik_calculator(self, grasp_point, wrist_co):
      # Inverse kinematic calculator to obtain the elbow joint position and forearm angle given the end-effector position
      e_co = geometry_msgs.msg.Point()
      sh_co = geometry_msgs.msg.Point()
      e_co.z = grasp_point.z

      sh_co.x = -0.18
      sh_co.y = 0.42
      sh_co.z = 0.54
      u_length = 0.36 # upepr arm length
      f_length = 0.40 # forearm length

      # Calculate forearm angle
      e_angle = math.atan((grasp_point.y - wrist_co.y)/(grasp_point.x - wrist_co.x + self.f_length))
      elbow_angle = math.degrees(e_angle)
      # print("Elbow angle: %s" %elbow_angle)

      # Calculate elbow joint position
      e_co.x = grasp_point.x - (self.f_length + 0.02)*math.cos(e_angle)
      e_co.y = grasp_point.y - (self.f_length + 0.02)*math.sin(e_angle)

      return (e_co, sh_co, e_angle)

  def point_planner(self, path_xy, optimal_grasp_point):
      # This function converts the xy path found from A* to a robot trajectory (waypoints)
      group = self.group
      scene = self.scene
      group.set_planning_time(15)

      self.waypoints = []
      wpose = group.get_current_pose().pose
      self.waypoints.append(copy.deepcopy(wpose))
      print("Initial pose: %s" %wpose)
      waypoints_number = len(path_xy)
      print("Number of waypoints: %s" %waypoints_number)
      print("Object center points: %s" %optimal_grasp_point.z)
      # Z height will be incremented/decremented gradually from the initial to the target position
      z_increment = (wpose.position.z - optimal_grasp_point.z)/waypoints_number
      # This increment is used in case the object height is smaller than a certain threshold
      z_increment_threshold = (wpose.position.z - 0.20)/waypoints_number

      # Distance from starting point to target point
      dist_target = math.sqrt((optimal_grasp_point.y - wpose.position.y)**2 + (optimal_grasp_point.x - wpose.position.x)**2)
      print("Distance to target: %s" %dist_target)
      # Calculate the angle from the initial position to the grasping position according to the target height
      angle_z = math.asin((optimal_grasp_point.z-wpose.position.z)/dist_target)
      # Divide this angle by the number of waypoints to increment the angle gradually
      z_angle_increment = angle_z/waypoints_number
      # Convert angle to degrees
      angle_z = math.degrees(angle_z)
      print("Z angle: %s" %angle_z)

      ## Convert arm trajectory to waypoints:
      z_rot = 1.5707 # Initial z angle
      for i in range(len(path_xy)):
          wpose.position.x = path_xy[i][0]
          wpose.position.y = path_xy[i][1]

          if(optimal_grasp_point.z >=0.20):
              wpose.position.z = wpose.position.z - z_increment
          else:
              # The minimum grasp point height allowed is z = 20cm (relative to the world frame),
              wpose.position.z =  wpose.position.z - z_increment_threshold

          # Create a waypoint for each (x,y) point of the A* trajectory
          euler = [0, z_rot, path_xy[i][2]]
          quat = tf.transformations.quaternion_from_euler(euler[0],euler[1],euler[2])
          wpose.orientation.x = quat[0]
          wpose.orientation.y = quat[1]
          wpose.orientation.z = quat[2]
          wpose.orientation.w = quat[3]
          self.waypoints.append(copy.deepcopy(wpose))

          z_rot = z_rot - z_angle_increment # Gradually increment/decrement the angle

      # Create plan
      (self.plan, fraction) = group.compute_cartesian_path(
                                         self.waypoints,   # waypoints to follow
                                         0.01,        # eef_step
                                         0.0, True)         # jump_threshold


      print("Fraction: %s" %fraction)
      successful_points = int(fraction * waypoints_number) # Number of successful points
      print("Number of successful points: %s" %successful_points)
      if(fraction < 1): # Only execute if fraction =1
          # failure_point = successful_points + 1
          print("Point at which it failed: %s" %(successful_points+1))
      attempt = 0
      while fraction < 1:
          # Iterator that decrements the x value in case the robot is in collision
          # Allow 150 attempts
          if(attempt == 150):
              print("Failed after 150 attempts")
              return False
          print("Attempt number: %s" %attempt)
          successful_points = int(fraction * waypoints_number)
          failure_point_index = successful_points # Find the failure point
          self.waypoints[failure_point_index].position.x = self.waypoints[failure_point_index].position.x - 0.01
          (self.plan, fraction) = group.compute_cartesian_path(
                                             self.waypoints,   # waypoints to follow
                                             0.01,        # eef_step
                                             0.0, True)
          attempt+=1
          print(fraction)

      print(self.waypoints[-1]) # Print the last waypoint, i.e. the target position

      return True

  def orientation_point_planner(self, optimal_grasp_point):
      # This is the "Top planner" that will try to avoid the objects from the top
      group = self.group
      scene = self.scene
      group.set_planning_time(15)

      # Obtain current wrist position
      wrist_co = geometry_msgs.msg.Point()
      wrist_co = self.group.get_current_pose().pose.position
      co_e, co_s, elbow_angle = self.ik_calculator(optimal_grasp_point,wrist_co)

      self.waypoints = []
      wpose = group.get_current_pose().pose
      self.waypoints.append(copy.deepcopy(wpose))
      print("Initial pose: %s" %wpose)

      # First waypoint that rotates the wrist
      wpose.position.z = optimal_grasp_point.z + 0.1
      euler = [-1.5707, 0.505, -1.5707]
      quat = tf.transformations.quaternion_from_euler(euler[0],euler[1],euler[2])
      wpose.orientation.x = quat[0]
      wpose.orientation.y = quat[1]
      wpose.orientation.z = quat[2]
      wpose.orientation.w = quat[3]
      self.waypoints.append(copy.deepcopy(wpose))

      # Second waypoint that takes the hand to the target object
      wpose.position.x = optimal_grasp_point.x - 0.01
      wpose.position.y = optimal_grasp_point.y - 0.01
      euler = [-1.5707, 0.505, -1.15]
      quat = tf.transformations.quaternion_from_euler(euler[0],euler[1],euler[2])
      wpose.orientation.x = quat[0]
      wpose.orientation.y = quat[1]
      wpose.orientation.z = quat[2]
      wpose.orientation.w = quat[3]
      self.waypoints.append(copy.deepcopy(wpose))

      # Create plan
      (self.plan, fraction) = group.compute_cartesian_path(
                                         self.waypoints,   # waypoints to follow
                                         0.01,        # eef_step
                                         0.0, True)         # jump_threshold


      print("Fraction: %s" %fraction)

      # Only want to execute a plan if the fraction is 1
      if fraction == 1:
          return True
      else:
          return False

  def behind_point_planner(self, optimal_grasp_point, possible_angles):
      # This is the "Behind planner" that will try to avoid the objects from behind
      group = self.group
      scene = self.scene
      group.set_planning_time(15)

      wrist_co = geometry_msgs.msg.Point()
      wrist_co = self.group.get_current_pose().pose.position

      length_f = 0.38 # forearm length
      fraction = 0
      i = 0
      while((fraction<1)): # Iterate to the list of possible angles to find a possible trajectory
          if i == len(possible_angles):
              break
          self.waypoints = []
          wpose = group.get_current_pose().pose
          self.waypoints.append(copy.deepcopy(wpose))

          print(i)

          # Calculating the elbow joint position according to the given forearm angle
          hor = self.f_length*math.sin(math.radians(possible_angles[i]))
          ver = self.f_length*math.cos(math.radians(possible_angles[i]))
          wpose.position.y = optimal_grasp_point.y - (hor/2) - 0.05
          wpose.position.x = optimal_grasp_point.x - (ver/2) + 0.03

          # If the z grasping point is below 20cm, it is set to 20cm
          if(optimal_grasp_point.z >=0.20):
              wpose.position.z = optimal_grasp_point.z
          else:
              wpose.position.z = 0.20

          # First waypoint in the middle of the trajectory, to ensuer a smooth path that goes around the obstacles
          euler = [0, 1.5707, math.radians(possible_angles[i]/2)]
          quat = tf.transformations.quaternion_from_euler(euler[0],euler[1],euler[2])
          wpose.orientation.x = quat[0]
          wpose.orientation.y = quat[1]
          wpose.orientation.z = quat[2]
          wpose.orientation.w = quat[3]
          self.waypoints.append(copy.deepcopy(wpose))

          # Grasping waypoint
          wpose.position.x = optimal_grasp_point.x
          wpose.position.y = optimal_grasp_point.y - 0.02
          euler = [0, 1.5707, math.radians(possible_angles[i])]
          quat = tf.transformations.quaternion_from_euler(euler[0],euler[1],euler[2])
          wpose.orientation.x = quat[0]
          wpose.orientation.y = quat[1]
          wpose.orientation.z = quat[2]
          wpose.orientation.w = quat[3]
          self.waypoints.append(copy.deepcopy(wpose))

          # Create plan
          (self.plan, fraction) = group.compute_cartesian_path(
                                             self.waypoints,   # waypoints to follow
                                             0.01,        # eef_step
                                             0.0, True)         # jump_threshold

          print("Fraction: %s" %fraction)
          i+=1
      # Want fraction to be one
      if fraction == 1:
        return True
      else:
        return False

  def path_executer(self,request):
      # Will execute the path if a True execute order is received from the user
      if request.execute_order == True:
          success = self.group.execute(self.plan, wait = True)
          print(success)

          # Remove target object from the scene
          self.scene.remove_world_object(str(self.target_obj_id))
          rospy.sleep(3) # Increase to allow the glove to be closed

          if self.orientation_plan == 1:
              # If the object was avoided from the top, a new reverse trajectory has to be calculated
              reverse_waypoints = []

              # First waypoint is the target point with a z increment, to clear over the obstacles
              wpose = self.waypoints[-1]
              euler = [-1.5707, 0.505, -1.15]
              quat = tf.transformations.quaternion_from_euler(euler[0],euler[1],euler[2])
              wpose.orientation.x = quat[0]
              wpose.orientation.y = quat[1]
              wpose.orientation.z = quat[2]
              wpose.orientation.w = quat[3]
              wpose.position.z += 0.12
              temp_z = wpose.position.z
              reverse_waypoints.append(copy.deepcopy(wpose))

              # Second waypoint: home position
              euler = [-1.5707, 0.505, -1.15]
              quat = tf.transformations.quaternion_from_euler(euler[0],euler[1],euler[2])
              wpose.orientation.x = quat[0]
              wpose.orientation.y = quat[1]
              wpose.orientation.z = quat[2]
              wpose.orientation.w = quat[3]
              wpose.position.x = 0.2
              wpose.position.y = 0.36
              wpose.position.z -= 0.12
              reverse_waypoints.append(copy.deepcopy(wpose))

              # Last waypoint: rotating the wrist to the natural orientation
              reverse_waypoints.append(copy.deepcopy(self.waypoints[0]))

              self.orientation_plan = 0 # Set the orientation flag to 0

          else:
              # In case the objects were not avoided from the top, execute the reverse trajectory to the home position
              reverse_waypoints = self.waypoints[::-1]

          (self.reverse_plan, fraction) = self.group.compute_cartesian_path(
                                             reverse_waypoints,   # waypoints to follow
                                             0.01,        # eef_step
                                             0.0, True)
          rospy.sleep(2)
          self.group.execute(self.reverse_plan, wait = True)

          # Return True if plan was executed with success
          if success == True:
              return True
          elif success == False:
              return False

      elif request.execute_order == False:
          print("Why the hell did I bother planning this?!")

          self.group.clear_pose_targets()

  def execute_plan(self, plan):

    group = self.group
    group.set_planning_time(10)

    # Execute the already planned trajectory
    group.execute(pose_goal, wait=True)

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):

    box_name = self.box_name
    scene = self.scene

    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
    ## call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False

  def add_table(self):
    ## Function to add the tabke to the planning scene

    print("Adding table")

    # Dimensions of the table top
    self.table_x_dim = 0.7
    self.table_y_dim = 1.4
    self.table_z_dim = 0.06
    #  Coordinates of the center point of the table
    self.table_x_center = 0.44
    self.table_y_center = 0.5
    self.table_z_center = 0.08

    # Calculating table height
    self.table_height = self.table_z_center +  self.table_z_dim/2
    # Add table top and set solor
    self.objectAdder.addBox("table", self.table_x_dim, self.table_y_dim, self.table_z_dim, self.table_x_center, self.table_y_center, self.table_z_center)
    self.objectAdder.setColor("table", 0.57, 0.73, 1.0, a=0.9)

    # Dimensions of the legs of the table
    leg_side_length = 0.05
    leg_height = 0.7
    # Calculating the center point of the legs
    leg_center = self.table_z_center - self.table_z_dim/2 - leg_height/2

    # Adding the 4 legs of the table
    self.objectAdder.addBox("leg_top_right", leg_side_length , leg_side_length , leg_height , self.table_x_center + self.table_x_dim/2 - leg_side_length/2 , self.table_y_center - self.table_y_dim/2 + leg_side_length/2 , leg_center)
    self.objectAdder.addBox("leg_top_left", leg_side_length , leg_side_length , leg_height , self.table_x_center + self.table_x_dim/2 - leg_side_length/2 , self.table_y_center + self.table_y_dim/2 - leg_side_length/2 , leg_center)
    self.objectAdder.addBox("leg_bottom_right", leg_side_length , leg_side_length , leg_height , self.table_x_center - self.table_x_dim/2 + leg_side_length/2 , self.table_y_center - self.table_y_dim/2 + leg_side_length/2 , leg_center)
    self.objectAdder.addBox("leg_bottom_left", leg_side_length , leg_side_length , leg_height , self.table_x_center - self.table_x_dim/2 + leg_side_length/2 , self.table_y_center + self.table_y_dim/2 - leg_side_length/2 , leg_center)
    self.objectAdder.setColor("leg_top_right", 0.78, 0.44, 0.2, a=1.0)
    self.objectAdder.setColor("leg_top_left", 0.78, 0.44, 0.2, a=1.0)
    self.objectAdder.setColor("leg_bottom_right", 0.78, 0.44, 0.2, a=1.0)
    self.objectAdder.setColor("leg_bottom_left", 0.78, 0.44, 0.2, a=1.0)

    # Add extra side wall for safety
    self.objectAdder.addBox("side_wall", 2.0, 0.1, 1.5, 0.45, -0.40, 0.20)
    self.objectAdder.setColor("side_wall", 0.1, 1.0, 0.2, a=0.9)
    self.objectAdder.sendColors()

    return self.wait_for_state_update(box_is_known=True, timeout=4)

  def add_box(self, objects, target_id):
    # Function to add the bounding boxes of the objects to the planning scene

    print("Adding boxes...")

    world_objects = objects # Array of the new objects received after segmentation
    old_objs = self.scene.get_objects() # Array of old objects currently in the planning scene

    margin = 0.0 # Margin for the bounding boxes
    roi_margin = 0.04 #
    i = 0
    world_objects = list(world_objects)
    while i < len(world_objects):
        object_id = str(i/6)
        old_objs = self.scene.get_objects()
        is_obj = self.scene.get_known_object_names_in_roi((world_objects[i+3] - (world_objects[i]/2) - roi_margin),(world_objects[i+4] - (world_objects[i+1]/2) - roi_margin),0.11,(world_objects[i+3] + (world_objects[i]/2) + roi_margin),(world_objects[i+4] + (world_objects[i+1]/2) + roi_margin),(world_objects[i+5] + (world_objects[i+2]/2) + 0.5))
        print("i: %s" %i)
        print("Is: %s" %is_obj)
        if is_obj: #list is not empty
            new_area = (world_objects[i] + margin)*(world_objects[i+1] + margin) # area = xDimension * yDimension
            old_area = (old_objs[str(is_obj[0])].primitives[0].dimensions[0])*(old_objs[str(is_obj[0])].primitives[0].dimensions[1])
            new_height = world_objects[i+2]
            old_height = old_objs[str(is_obj[0])].primitives[0].dimensions[2]
            if(new_area > old_area or new_height > old_height):
                print(self.scene.get_known_object_names())
                print("Old objects was removed: %s" %is_obj[0])
                rospy.sleep(1.0)
                self.scene.remove_world_object(str(is_obj[0]))
                print(self.scene.get_known_object_names())
            else:
                print("Else:")
                is_obj = [s for s in is_obj if s.isdigit()]
                old_obj_pose = self.scene.get_object_poses(is_obj)
                # print(i)
                # print(world_objects)
                world_objects.pop(i)
                world_objects.insert(i,old_objs[str(is_obj[0])].primitives[0].dimensions[0])
                world_objects.pop(i+1)
                world_objects.insert(i+1,old_objs[str(is_obj[0])].primitives[0].dimensions[1])
                world_objects.pop(i+2)
                world_objects.insert(i+2,old_objs[str(is_obj[0])].primitives[0].dimensions[2])
                world_objects.pop(i+3)

                if old_obj_pose.get(str(is_obj[0])) == None:
                    print("Key doesn't exist")
                if bool(old_obj_pose) == False:
                    print("Didn't get object pose")
                world_objects.insert(i+3,old_obj_pose[str(is_obj[0])].position.x)
                world_objects.pop(i+4)
                world_objects.insert(i+4,old_obj_pose[str(is_obj[0])].position.y)
                world_objects.pop(i+5)
                world_objects.insert(i+5,old_obj_pose[str(is_obj[0])].position.z)
                self.scene.remove_world_object(str(is_obj[0]))
                print("Old objects was removed: %s" %is_obj[0])
                rospy.sleep(1.0)
        i+=6

    old_objs = self.scene.get_objects()
    print("After")
    print(self.scene.get_known_object_names())
    old_ids = [s for s in self.scene.get_known_object_names() if s.isdigit()]
    print("old_ids: %s" %old_ids)

    # Append old objects that are not in frame
    for k in old_ids:
        print("K: %s" %k)
        old_obj_pose = self.scene.get_object_poses([str(k)])
        if old_obj_pose.get(str(k)) == None:
            print("Key doesn't exist")
        if bool(old_obj_pose) == False:
            print("Didn't get object pose")
            old_obj_pose = self.scene.get_object_poses([str(k)])

        world_objects.append(old_objs[str(k)].primitives[0].dimensions[0])
        world_objects.append(old_objs[str(k)].primitives[0].dimensions[1])
        world_objects.append(old_objs[str(k)].primitives[0].dimensions[2])
        world_objects.append(old_obj_pose[str(k)].position.x)
        world_objects.append(old_obj_pose[str(k)].position.y)
        world_objects.append(old_obj_pose[str(k)].position.z)
        self.scene.remove_world_object(k)
        print("Removed old object: %s" %k)
        rospy.sleep(1.0)

    i = 0

    print("Number of objects: %s" %(len(world_objects)/6))
    print(target_id)

    while i < len(world_objects): # Iterate to the array of objects to add them to the planning scene
        object_id = str(i/6)
        print(object_id)
        if(object_id==str(target_id)): # Add target object bounding box, with a capped z height
            self.objectAdder.addBox(object_id, world_objects[i] + margin, world_objects[i+1] + margin, 0.01, world_objects[i+3], world_objects[i+4], world_objects[i+5]-world_objects[i+2]/2 + 0.005)
            self.objectAdder.setColor(object_id, 0.1, 1.0, 0.2, a=1.0) # Target object is green

        if(object_id!=str(target_id)): # Add obstacles bounding box
            self.objectAdder.addBox(object_id, world_objects[i] + margin, world_objects[i+1] + margin, world_objects[i+2], world_objects[i+3], world_objects[i+4], world_objects[i+5])
            self.objectAdder.setColor(object_id, 1.0, 0.2, 0.2, a=1.0) # Obstacles are red


        i+=6
    self.objectAdder.sendColors()

    return self.wait_for_state_update(box_is_known=True, timeout=4)

def line_collision(x1,y1,x2,y2,x3,y3,x4,y4):
    # Calculates if there is a collision between two lines, given two (x,y) points for each line
    # This is used to infer whether there is a collision or not between the forearm line and the obstacles
    uA = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1))
    uB = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) / ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1))

    if( 0<= uA <= 1 and 0<= uB <= 1):
        # Intersection point
        intersectionX = x1 + (uA*(x2-x1))
        intersectionY = y1 + (uB*(y2-y1))
        return True
    return False

def object_collision(e_co, grasp_point, objects_array, neig_idx):
    i = 0
    num_collisions = 0

    margin = 0.0 # Add margin to avoid the objects with an extra safety distance
    while i < len(objects_array): # Iterate through the obstacles to find collisions
        if(i/6 != neig_idx): # Note that collisions with the target object are allowed
            # Find if the forearm line intersects any of the four sides of each obstacle
            bottom = line_collision(e_co.x, e_co.y, grasp_point.x, grasp_point.y,objects_array[i+3]-objects_array[i+0]/2 - margin,objects_array[i+4]-objects_array[i+1]/2 - margin,objects_array[i+3]-objects_array[i+0]/2 - margin, objects_array[i+4]+objects_array[i+1]/2 + margin)
            top = line_collision(e_co.x, e_co.y, grasp_point.x, grasp_point.y,objects_array[i+3]+objects_array[i+0]/2,objects_array[i+4]-objects_array[i+1]/2,objects_array[i+3]+objects_array[i+0]/2, objects_array[i+4]+objects_array[i+1]/2)
            left = line_collision(e_co.x, e_co.y, grasp_point.x, grasp_point.y,objects_array[i+3]-objects_array[i+0]/2,objects_array[i+4]+objects_array[i+1]/2,objects_array[i+3]+objects_array[i+0]/2, objects_array[i+4]+objects_array[i+1]/2)
            right = line_collision(e_co.x, e_co.y, grasp_point.x, grasp_point.y,objects_array[i+3]-objects_array[i+0]/2,objects_array[i+4]-objects_array[i+1]/2,objects_array[i+3]+objects_array[i+0]/2, objects_array[i+4]-objects_array[i+1]/2)

            if(bottom == True or top == True or left == True or right == True):
                # Number of collisions in a given xy position of the wrist
                num_collisions+=1
        i+=6

    if(num_collisions > 0):
        return True # There is at least one collision, the state is not possible
    else:
        return False # There are no collisions

def heuristic(a, b):
    # heuristic term of the cost-function for the A* algorithm
    return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2

def astar2(array, start, goal):
    # A* pathfinding algorithm function
    # Receive a grid with the non/possible states, the initial and target positions
    # It outputs a trajectory that avoids collisions

    # 8 neighbouring position
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    close_set = set() #
    came_from = {} # Previous node
    gscore = {start:0} # Distance to initial position cost
    fscore = {start:heuristic(start, goal)} # Euclidean distance cost
    oheap = []

    heappush(oheap, (fscore[start], start))

    while oheap:

        current = heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data[::-1] # Return reversed path

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue

            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score# + heuristic(neighbor, goal)
                heappush(oheap, (fscore[neighbor], neighbor))

    return False


if __name__ == '__main__':
    commander = MoveGroupPythonInterface() # Declare main class
    commander.go_to_initial_state() # Move robot to initial position
    commander.add_table() # Add table to the planning scene
    commander.path_executer_server() # Service that receives trajectory execution order
    commander.path_planner_server() # Service that will calculate the collision-free trajectory
