#!/usr/bin/env python
import roslib
roslib.load_manifest('hello_ros')
 
import sys
import copy
import rospy
import tf_conversions
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from gazebo_msgs.msg import ModelStates
import shape_msgs.msg as shape_msgs
from sensor_msgs.msg import JointState
from numpy import zeros, array, linspace
from math import ceil
import time
 
global cube_positions
cube_positions = {}

currentJointState = JointState()
def jointStatesCallback(msg):
  global currentJointState
  currentJointState = msg

def move_group_python_interface_tutorial(pub):
  global cube_positions, currentJointState


  ## BEGIN_TUTORIAL
  ## First initialize moveit_commander and rospy.
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)


  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group = moveit_commander.MoveGroupCommander("Arm")
 
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)
 
  print "============ Starting tutorial "
  ## We can get the name of the reference frame for this robot
  print "============ Reference frame: %s" % group.get_planning_frame()
  ## We can also print the name of the end-effector link for this group
  print "============ End effector frame: %s" % group.get_end_effector_link()
  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()
  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"
 
  ## Let's setup the planner
  #group.set_planning_time(0.0)
  group.set_goal_orientation_tolerance(0.01)
  group.set_goal_tolerance(0.01)
  group.set_goal_joint_tolerance(0.01)
  group.set_num_planning_attempts(100)
  group.set_max_velocity_scaling_factor(1.0)
  group.set_max_acceleration_scaling_factor(1.0)
 
  ## Planning to a Pose goal
  print "============ Generating plan 1"
  
  open(pub)

  print(cube_positions)

  for cube in cube_positions.keys():
    if cube == "box":
      continue
    pose_goal = group.get_current_pose().pose
    pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0., -math.pi/2, 0.))
    pose_goal.position.x = cube_positions[cube]['x']
    pose_goal.position.y = cube_positions[cube]['y']
    pose_goal.position.z = cube_positions[cube]['z'] + 0.5
    print pose_goal
    group.set_pose_target(pose_goal)
  
  
    ## Now, we call the planner to compute the plan
    plan1 = group.plan()
  
    print "============ Waiting while RVIZ displays plan1..."
    rospy.sleep(0.5)
  
  
    ## You can ask RVIZ to visualize a plan (aka trajectory) for you.
    print "============ Visualizing plan1"
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan1)
    display_trajectory_publisher.publish(display_trajectory);
    print "============ Waiting while plan1 is visualized (again)..."
    rospy.sleep(2.)
  
    
    ## Moving to a pose goal
    group.go(wait=True)

    pose_goal = group.get_current_pose().pose
    pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0., -math.pi/2, 0.))
    pose_goal.position.x = cube_positions[cube]['x']
    pose_goal.position.y = cube_positions[cube]['y']
    pose_goal.position.z = cube_positions[cube]['z'] + 0.2
    print pose_goal
    group.set_pose_target(pose_goal)
  
  
    ## Now, we call the planner to compute the plan
    plan1 = group.plan()
  
    print "============ Waiting while RVIZ displays plan1..."
    rospy.sleep(0.5)
  
  
    ## You can ask RVIZ to visualize a plan (aka trajectory) for you.
    print "============ Visualizing plan1"
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan1)
    display_trajectory_publisher.publish(display_trajectory);
    print "============ Waiting while plan1 is visualized (again)..."
    rospy.sleep(2.)
  
    
    ## Moving to a pose goal
    group.go(wait=True)

    close(pub)
  
    ## second movement
    pose_goal = group.get_current_pose().pose
    pose_goal.position.x = cube_positions['box']['x']
    pose_goal.position.y = cube_positions['box']['y']
    pose_goal.position.z = cube_positions['box']['z'] + 0.5
    group.set_pose_target(pose_goal)

    plan1 = group.plan()
    rospy.sleep(2.)

    ## You can ask RVIZ to visualize a plan (aka trajectory) for you.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan1)
    display_trajectory_publisher.publish(display_trajectory);

    rospy.sleep(2)

    group.go(wait=True)
    
    open(pub)
  rospy.sleep(2.)
 
  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()
 
  ## END_TUTORIAL
  print "============ STOPPING"
  R = rospy.Rate(10)
  while not rospy.is_shutdown():
    R.sleep()

def close(pub):
  global currentJointState
  print("starting to listin")
  currentJointState = rospy.wait_for_message("/joint_states",JointState)
  print 'Received!'
  currentJointState.header.stamp = rospy.get_rostime()
  tmp = 0.7
  #tmp_tuple=tuple([tmp] + list(currentJointState.position[1:]))
  currentJointState.position = tuple(list(currentJointState.position[:6]) + [tmp] + [tmp]+ [tmp])
  for i in range(3):
    pub.publish(currentJointState)
  time.sleep(5)

def open(pub):
  global currentJointState
  print("starting to listin")
  currentJointState = rospy.wait_for_message("/joint_states",JointState)
  print 'Received!'
  currentJointState.header.stamp = rospy.get_rostime()
  tmp = 0.005
  #tmp_tuple=tuple([tmp] + list(currentJointState.position[1:]))
  currentJointState.position = tuple(list(currentJointState.position[:6]) + [tmp] + [tmp]+ [tmp])
  for i in range(3):
    pub.publish(currentJointState)
  time.sleep(5)


print('Start')
#Create callback. This is what happens when a new message is received
def sub_cal(msg):
  global cube_positions
  names = msg.name
  cube_indexes = []
  for i, name in enumerate(names):
    if 'cube' in name:
      cube_indexes.append(i)
  poses = msg.pose
  ii = 0
  for i, pose in enumerate(poses):
    if i not in cube_indexes:
      continue
    position = pose.position
    cube_positions['cube' + str(ii)] = {}
    cube_positions['cube' + str(ii)]['x'] = position.x
    cube_positions['cube' + str(ii)]['y'] = position.y
    cube_positions['cube' + str(ii)]['z'] = position.z
    ii += 1
  cube_positions['box'] = {}
  cube_positions['box']['x'] = pose.position.x
  cube_positions['box']['y'] = pose.position.y
  cube_positions['box']['z'] = pose.position.z
  

#Initialise subscriber
rospy.Subscriber('/gazebo/model_states', ModelStates, sub_cal, queue_size=1000)
pub = rospy.Publisher("/jaco/joint_control", JointState, queue_size=1)
 



if __name__=='__main__':
  try:
    move_group_python_interface_tutorial(pub)
  except rospy.ROSInterruptException:
    pass