#!/usr/bin/env python

import rospy 

from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

import random

def main():
    rospy.init_node("spawn_balls_node")


    n = 50
    if rospy.has_param("~n"):
        n = rospy.get_param("~n")
    rospy.loginfo(f"Attempting to spawn {n} balls")

    min_x = 0.02
    max_x = 19.8

    min_y = 0.02
    max_y = 35.8

    z = 0.05

    model_path = ""
    if rospy.has_param("~model_path"):
        model_path = rospy.get_param("~model_path")
    else:
        rospy.logerr("model_path param must be specified")
        exit()

    rospy.loginfo(f"Using model file: {model_path}")
    
    f = open(model_path, "r")
    model = f.read()

    model_namespace = "balls"
    model_reference_frame = "world"

    rospy.logdebug("Waiting for /gazebo/spawn_sdf_model service")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    client = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    for i in range(n):
        rand_x = random.uniform(min_x, max_x)
        rand_y = random.uniform(min_y, max_y)
            
        pose = Pose()
        pose.position.x = rand_x 
        pose.position.y = rand_y 
        pose.position.z = z 
        # pose.orientation =

        model_name = "ball_" + str(i)
        
        try:
            resp = client(model_name, model, model_namespace, pose, model_reference_frame)

            if(resp.success):
                rospy.logdebug(f"Spawned {model_name}")
            else:
                rospy.logerr(f"Could not spawn model. Error: {resp.status_message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Ball spawn service failed: {e}")

if __name__ == "__main__":
    main()
