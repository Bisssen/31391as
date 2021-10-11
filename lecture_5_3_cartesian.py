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
import shape_msgs.msg as shape_msgs
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import DeleteModel
import math
import time
 
from std_msgs.msg import String

global cube_positions
cube_positions = {}

currentJointState = JointState()
def jointStatesCallback(msg):
    global currentJointState
    currentJointState = msg

def move_to_pos(group, x, y, z):
    print('Moving to position: ' + str(x) + ', ' + str(y) + ', ' + str(z))
    pose_goal = group.get_current_pose().pose
    waypoints = []

    waypoints.append(pose_goal)
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    
    #Create waypoints
    waypoints.append(pose_goal)
    
    #createcartesian  plan
    (plan1, fraction) = group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold
    
    ## Moving to a pose goal
    group.execute(plan1, wait=True)
    time.sleep(10)

def collect_cubes(pub, delete_model):
    ## First initialize moveit_commander and rospy.
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
    
    ## Let's setup the planner
    #group.set_planning_time(0.0)
    group.set_goal_orientation_tolerance(0.01)
    group.set_goal_tolerance(0.01)
    group.set_goal_joint_tolerance(0.01)
    group.set_num_planning_attempts(100)
    group.set_max_velocity_scaling_factor(1.0)
    group.set_max_acceleration_scaling_factor(1.0)
    

    rotation = 0.0
    # Orient the robot to point downwards
    start_pose = group.get_current_pose().pose
    start_pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(rotation, -math.pi/2, 0.))
    group.set_pose_target(start_pose)
    start_plan = group.plan()
    group.go(wait=True)

    # Open the gripper
    open(pub)

    cube_distances = []
    # Sort the cubes in their distance to the bucket
    for cube in cube_positions.keys():
        if cube == "box":
            continue
        cube_distances.append([(cube_positions[cube]['x'] -
                                cube_positions['box']['x'])**2 +\
                               (cube_positions[cube]['y'] -
                                cube_positions['box']['y'])**2,
                                cube])
    cube_distances.sort()

    # loop
    for cube in cube_distances:
        cube = cube[-1]
        print('Moving ' + cube)
        if cube == "box":
            continue

        while True:
            # Move above the the cube
            move_to_pos(group, cube_positions[cube]['x'],
                               cube_positions[cube]['y'],
                               cube_positions[cube]['z'] + 0.5)

            # Touch cube
            move_to_pos(group, cube_positions[cube]['x'],
                               cube_positions[cube]['y'],
                               cube_positions[cube]['z'] + 0.15)

            # Close the gripper
            close(pub)
            
            z_before_lift = cube_positions[cube]['z']

            # Move back above the cube
            move_to_pos(group, cube_positions[cube]['x'],
                               cube_positions[cube]['y'],
                               cube_positions[cube]['z'] + 0.5)

            z_after_lift = cube_positions[cube]['z']

            # Check if the cube was lifted
            if abs(z_before_lift - z_after_lift) < 0.1:
                open(pub)
                # Rotate the hand to hopefully find a valid route down
                rotation += math.pi/4
                # Wrap the rotation
                if rotation >= 2*math.pi:
                    rotation = 0
                start_pose = group.get_current_pose().pose
                start_pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(rotation, -math.pi/2, 0.))
                group.set_pose_target(start_pose)
                start_plan = group.plan()
                group.go(wait=True)
                continue

            break

        # Move above the trash box
        move_to_pos(group, cube_positions['box']['x'],
                           cube_positions['box']['y'],
                           cube_positions['box']['z'] + 0.5)
        
        # Open the gripper
        open(pub)

    ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()
    for cube in cube_positions.keys():
        if cube == "box":
            continue
            
        delete_model(cube)

        
    ## END_TUTORIAL
    print("============ STOPPING")
    R = rospy.Rate(10)
    while not rospy.is_shutdown():
        R.sleep()


def close(pub):
    print('Closing the gripper')
    global currentJointState
    currentJointState = rospy.wait_for_message("/joint_states",JointState)
    currentJointState.header.stamp = rospy.get_rostime()
    tmp = 0.75
    #tmp_tuple=tuple([tmp] + list(currentJointState.position[1:]))
    currentJointState.position = tuple(list(currentJointState.position[:6]) + [tmp] + [tmp]+ [tmp])
    for i in range(3):
        pub.publish(currentJointState)
    time.sleep(9)

def open(pub):
    print('Oppening the gripper')
    global currentJointState
    currentJointState = rospy.wait_for_message("/joint_states",JointState)
    currentJointState.header.stamp = rospy.get_rostime()
    tmp = 0.005
    #tmp_tuple=tuple([tmp] + list(currentJointState.position[1:]))
    currentJointState.position = tuple(list(currentJointState.position[:6]) + [tmp] + [tmp]+ [tmp])
    for i in range(3):
        pub.publish(currentJointState)
    time.sleep(9)


print('Start')
# Function that updates the cubes positions
def sub_cal(msg):
    # Cube position is saved as a global, so ensure it is received
    global cube_positions
    # Wait with updating the cube positions until it is fully filled out
    tmp_cube_positions = {}
    # Get all the names
    names = msg.name
    cube_indexes = []

    # Loop through all the names to figure out which cubes exists
    for i, name in enumerate(names):
        if 'cube' in name:
            cube_indexes.append(i)
        elif 'bucket' in name:
            box_index = i
    # Get all the poses
    poses = msg.pose
    ii = 0
    
    # Only loop through the cubes poses
    for i, pose in enumerate(poses):
        if i in cube_indexes:
            position = pose.position
            tmp_cube_positions['cube' + str(ii)] = {}
            tmp_cube_positions['cube' + str(ii)]['x'] = position.x
            tmp_cube_positions['cube' + str(ii)]['y'] = position.y
            tmp_cube_positions['cube' + str(ii)]['z'] = position.z
            ii += 1
        elif i == box_index:
            tmp_cube_positions['box'] = {}
            tmp_cube_positions['box']['x'] = pose.position.x
            tmp_cube_positions['box']['y'] = pose.position.y
            tmp_cube_positions['box']['z'] = pose.position.z
    cube_positions = tmp_cube_positions

    

# Start listening to the poses of the objects
rospy.Subscriber('/gazebo/model_states', ModelStates, sub_cal, queue_size=1000)
pub = rospy.Publisher("/jaco/joint_control", JointState, queue_size=1)

if __name__=='__main__':
    rospy.wait_for_service('gazebo/delete_model')
    delete_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    try:
        collect_cubes(pub, delete_model)
    except rospy.ROSInterruptException:
        pass