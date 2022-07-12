#! /usr/bin/env python3

import time  # Time library
import os
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult # Helper module
from launch_ros.substitutions import FindPackageShare


# Start the ROS 2 Python Client Library
rclpy.init()
package_name = 'my_robot'
pkg_share = FindPackageShare(package=package_name).find(package_name)
# Launch the ROS 2 Navigation Stack
navigator = BasicNavigator()
map_file_path_warehouse = 'maps/warehouse_world/warehouse_world.yaml'
map_file_path_office = 'maps/office_world/office_world.yaml'
map_file_path_hospital = 'maps/hospital_world/hospital_world.yaml'

warehouse_map = os.path.join(pkg_share, map_file_path_warehouse)
office_map = os.path.join(pkg_share, map_file_path_office)
hospital_map = os.path.join(pkg_share, map_file_path_hospital)

initial_pose = PoseStamped()

initial_pose.header.frame_id = 'map'
initial_pose.header.stamp = navigator.get_clock().now().to_msg()
initial_pose.pose.position.x = 0.0
initial_pose.pose.position.y = 1.0
initial_pose.pose.position.z = 0.0
# initial_pose.pose
# navigator.setInitialPose(initial_pose)

# Wait for navigation to fully activate. Use this line if autostart is set to true.
# navigator.waitUntilNav2Active()

def moveTo(x, y, name):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0
    done = False
    print('[Robot]   Going to ' + name)
    navigator.goToPose(goal_pose)
    while not navigator.isTaskComplete():  
      feedback = navigator.getFeedback()
      if feedback:
        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
          navigator.cancelTask()

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('[Robot]   Arrived to ' + name)
        done = True
    elif result == TaskResult.FAILED:
        print('[Robot]   Could not get to ' + name)
    else:
        print('Goal has an invalid return status!')
        
    return done

def main():
  goals = ["office_worker", "elevator", "room1_hospital" "Package1"]
  
  goals_coordinates = [[-0.94, 20.01], [18.76, 14.50], [25.19, -25.24], [16.34, 28.42]]
  
  


  i = 0
  have_task = 0
  package1_picked = 0
  package1_delivered = 0
  
  to_elevator = 0
  done = False 
  floor = 1
  current_goal = goals[0]
  print('[Robot]   Okay, I heard I have a task')
  
  # Keep doing stuff as long as the robot is moving towards the goal
  while not done:
      if have_task == 0: # Do not have task, go to Office worker to get one
          floor = 1
          current_goal = goals[0]
          navigator.clearAllCostmaps()
          navigator.changeMap(office_map)
          isDone = moveTo(goals_coordinates[0][0], goals_coordinates[0][1], current_goal)
          if isDone:
              have_task = 1
              print("[Robot]   So, I need to go to warehouse to take package number and deliver it to the patient's room....")
              to_elevator = 1
              floor = 0
          else:
              print("[Robot]   Trying again")
      elif have_task == 1:
          navigator.clearAllCostmaps()
          navigator.changeMap(warehouse_map)
          isDone = moveTo(goals_coordinates[3][0], goals_coordinates[3][1], "Package")
          if isDone:
              time.sleep(2.0)
              print('[Robot]   I have a package, need to go to deliver it...')
              have_task = 2
              floor = 2
              to_elevator = 1
      elif have_task == 2:
          navigator.clearAllCostmaps()
          navigator.changeMap(hospital_map)
          
          isDone = moveTo(goals_coordinates[2][0], goals_coordinates[2][1], "Hospital Room 2")
          print('[Robot]   Here you go, sir')
          done = True
      if to_elevator:
          moveTo(goals_coordinates[1][0], goals_coordinates[1][1], "Elevator")
          print('[Robot]   In elevator moving to ' + str(floor) + ' floor')
          time.sleep(3.0)
          to_elevator = 0
  # Shut down the ROS 2 Navigation Stack
  navigator.lifecycleShutdown()

  exit(0)

if __name__ == '__main__':
  main()
