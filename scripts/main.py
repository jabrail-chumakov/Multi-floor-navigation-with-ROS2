#! /usr/bin/env python3

import time  # Time library
import os
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2

from robot_navigator import BasicNavigator, NavigationResult # Helper module
from launch_ros.substitutions import FindPackageShare
'''
Navigates a robot from an initial pose to a goal pose.
'''
def main():
    
  # Start the ROS 2 Python Client Library
  rclpy.init()
  package_name = 'two_wheeled_robot'
  pkg_share = FindPackageShare(package=package_name).find(package_name)
  # Launch the ROS 2 Navigation Stack
  navigator = BasicNavigator()
  map_file_path_warehouse = 'maps/warehouse_world/warehouse_world.yaml'
  map_file_path_office = 'maps/office_world/office_world.yaml'
  map_file_path_hospital = 'maps/hospital_world/hospital_world.yaml'
  
  warehouse_map = os.path.join(pkg_share, map_file_path_warehouse)
  office_map = os.path.join(pkg_share, map_file_path_office)
  hospital_map = os.path.join(pkg_share, map_file_path_hospital)
  
  # Wait for navigation to fully activate. Use this line if autostart is set to true.
  navigator.waitUntilNav2Active()

  # If desired, you can change or load the map as well
  # navigator.changeMap('/path/to/map.yaml')

  # You may use the navigator to clear or obtain costmaps
  # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
  # global_costmap = navigator.getGlobalCostmap()
  # local_costmap = navigator.getLocalCostmap()

  # Set the robot's goal pose
  goals = ["office_worker", "room1_hospital", "room2_hospital", "Package1", "Package2"]
  
  goals_coordinates = [[-1.07, 20.28], [-8.71, -4.1], [8.83, -18.7], [-1.98, -5.04], [0.786, -7.47]]
  
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = 0.0
  goal_pose.pose.position.y = 0.0
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.0
  goal_pose.pose.orientation.w = 1.0
  # 
  # sanity check a valid path exists
  # path = navigator.getPath(initial_pose, goal_pose)

  # Go to the goal pose
  

  i = 0
  have_task = 0
  package1_picked = 0
  package1_delivered = 0
  
  package2_picked = 0
  package2_delivered = 0
  
  done = False 
  # Keep doing stuff as long as the robot is moving towards the goal
  while not done:
    while not navigator.isNavComplete():  
        
        if not have_task: # Do not have task, go to Office worker to get one
            i = 0
            navigator.changeMap(office_map)
            have_task = 1
        else:
            if not package1_picked:
                i = 3
                package1_picked = 1
            elif not package1_delivered:
                i = 1
            
            if (not package2_picked) and (package1_delivered):
                i = 4
                package2_picked = 1
            
            if package1_picked and not package1_delivered:
                i = 1
            
        
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = goals_coordinates[i][0]
        goal_pose.pose.position.x = goals_coordinates[i][1]        
        navigator.goToPose(goal_pose)
        feedback = navigator.getFeedback()
        if feedback:
          if Duration.from_msg(feedback.navigation_time) > Duration(seconds=30.0):
            navigator.cancelNav()

      result = navigator.getResult()
      if result == NavigationResult.SUCCEEDED:
          if i % 7 == 0:
              print('At office worker')
              sleep(2.0)
          elif i % 7 == 1:
              print('Picking the first package') 
              sleep(2.0)
              print('Got the package')
          elif i % 7 == 2:
              print('Picking the second package') 
              sleep(2.0)
              print('Got the package')
          elif i % 7 == 3:
              print('Picking the third package') 
              sleep(2.0)
              print('Got the package')
          elif i % 7 == 4:
              print('Picking the second package') 
              sleep(2.0)
              print('Got the package')
        
      elif result == NavigationResult.FAILED:
          print('Goal failed!')
      else:
          print('Goal has an invalid return status!')

  # Shut down the ROS 2 Navigation Stack
  navigator.lifecycleShutdown()

  exit(0)

if __name__ == '__main__':
  main()
