#!/usr/bin/env python3

import rospy 

from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

import random, sys 


class BallSpawner():

    def __init__(self):
        self.min_x = 0.02
        self.max_x = 19.8

        self.min_y = 0.02
        self.max_y = 35.8

        self.z = 0.05

        self.namespace = "balls"
        self.reference = "world"

        rospy.logdebug("Waiting for /gazebo/spawn_sdf_model service")
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        self.client = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.model_path = ""
        
        rospy.loginfo("Waiting for model_path")
        while not rospy.is_shutdown() and not rospy.has_param("~model_path"):
            rospy.sleep(0.1)

        if rospy.has_param("~model_path"):
            self.model_path = rospy.get_param("~model_path")
        else:
            rospy.logerr("model_path param must be specified")
            exit()

        f = open(self.model_path, "r")
        self.model = f.read()
        f.close()

        self.n_balls = 0
    
    def spawn_ball(self):
        rand_x = random.uniform(self.min_x, self.max_x)
        rand_y = random.uniform(self.min_y, self.max_y)
            
        pose = Pose()
        pose.position.x = rand_x 
        pose.position.y = rand_y 
        pose.position.z = self.z 

        model_name = "ball_" + str(self.n_balls)
        self.n_balls += 1
        
        try:
            resp = self.client(model_name, self.model, self.namespace, pose, self.reference)

            if(resp.success):
                rospy.logdebug(f"Spawned {model_name}")
            else:
                rospy.logwarn(f"Could not spawn model. Error: {resp.status_message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Ball spawn service failed: {e}")

    def spawn_n(self, n):
        for i in range(n):
            self.spawn_ball()

if __name__ == "__main__":
    rospy.init_node("spawn_balls_node")

    num_args = len(sys.argv)

    if num_args < 2:
        rospy.logerr("Not enough arguments for spawn_balls_node. Usage: spawn_balls.py <n>")
        exit()

    n = int(sys.argv[1])
    
    spawner = BallSpawner()
    spawner.spawn_n(n)
