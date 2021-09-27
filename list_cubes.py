#!/usr/bin/env python
# This program publishes randomly-generated velocity
# messages for turtlesim.
import rospy
import sys
 
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
import time
global cube_positions
cube_positions = {}

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
        cube_positions['cube' + str(ii)]['x'] = position.x
        cube_positions['cube' + str(ii)]['y'] = position.y
        cube_positions['cube' + str(ii)]['z'] = position.z
        print(pose.position.x)
        ii += 1

#Initialise subscriber
rospy.Subscriber('/gazebo/model_states', ModelStates, sub_cal, queue_size=1000)

rospy.init_node('listen_to_model_states')
r = rospy.Rate(2) # Set Frequency

print("subbed") 
#loop until someone shutds us down..
while not rospy.is_shutdown():

    r.sleep()
    print('alive')
    # print(cube_positions)