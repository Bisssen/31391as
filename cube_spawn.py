#!/usr/bin/env python

import rospy, tf, random
import tf_conversions
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import *
import sys

pwd = None
for file in sys.path:
    if "catkin" in file:
        pwd = file
        break

if pwd is None:
    sys.exit('No path could be found, edit manually')
else:
    student = pwd.split('home/')[-1].split('/catkin')[0]

if __name__ == '__main__':
    print("Waiting for gazebo services...")
    rospy.init_node("spawn_products_in_bins")
    rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    print("Got it.")
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    with open("/home/" + student + "/catkin_ws/src/hello_ros/urdf/cube.urdf", "r") as f:
        product_xml = f.read()

    orient = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0., 0.0, 0.785398))

    num_of_cubes = random.randint(2,6)
    num_of_cubes = 6
    cube_positions = []
    bucket_x=0.53
    bucket_y=-0.23


    for num in xrange(0,num_of_cubes):
        robot_x = 0.767
        robot_y = 0.056
        r = 0.7
        min_cube_dist = 0.1
        bucket_dist = 0.2

        while True:
       
            bin_y   =   random.uniform(0, 0.5)
            bin_x   =   random.uniform(0, 0.5)
            
            continue_flag = False
            for spawned_cube in cube_positions:
                if (spawned_cube[0] - bin_x)**2 + (spawned_cube[1] - bin_y)**2 < min_cube_dist**2:
                    continue_flag = True
            
            if (bucket_x - bin_x)**2 + (bucket_y - bin_y)**2 < bucket_dist**2:
                continue_flag = True

            if continue_flag:
                continue

            if (robot_x - bin_x)**2 + (robot_y - bin_y)**2 < r**2:
                break

        cube_positions.append([bin_x, bin_y])

        item_name   =   "cube{}".format(num)
        print("Spawning model:%s", item_name)
        item_pose   =   Pose(Point(x=bin_x, y=bin_y,    z=1),   orient)
        #delete_model(item_name)
        spawn_model(item_name, product_xml, "", item_pose, "world")

    with open("/home/" + student + "/catkin_ws/src/hello_ros/urdf/bucket.urdf", "r") as f:
        product_xml = f.read()

    item_pose   =   Pose(Point(x=0.53, y=-0.23,    z=1.0),   orient)
    print("Spawning model:%s", "bucket")
    spawn_model("bucket", product_xml, "", item_pose, "world")