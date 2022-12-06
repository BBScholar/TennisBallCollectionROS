#!/usr/bin/env python3

import roslib 
import rospy

from gazebo_msgs.msg import LinkStates, ModelStates
from gazebo_msgs.srv import GetModelState

from geometry_msgs.msg import Pose, PoseArray, PoseStamped

import tf2_ros as tf2
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener


class SimSeeker:

    def __init__(self) -> None:
        self.x_min = 0.3
        self.x_max = 4.0
        self.y_min = -4.0
        self.y_max = 4.0
        
        self.world_frame = "map" #TODO: need to check what this actually is
        self.base_link_frame = "base_root"

        self.model_state_sub = rospy.Subscriber("/gazebo/model_states_throttled", ModelStates, self.model_state_cb, queue_size=4)
        self.link_state_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.link_state_cb, queue_size=32)

        self.model_state_client = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.model_state_client.wait_for_service()

        self.visible_ball_pub = rospy.Publisher("sim_seeker/visible_balls", PoseArray, queue_size=32)
        self.pub_seq = 0

        self.visible_ball_list = []
        self.last_update_time = rospy.Time.now()
        
        self.tf_buf = Buffer()
        self.tf_list = TransformListener(self.tf_buf)
        
        self.update_rate = rospy.Duration(0.1)

        self.update_timer = rospy.Timer(self.update_rate, self.update_cb)
    
    def model_state_cb(self, msg):
        # rospy.loginfo_throttle(1.0, "Receiving model state message")
        n = len(msg.name)
        ret_list = []

        for i in range(n):
            name = msg.name[i]

            if "ball" in name:
                # req = GetModelState()
                # req.model_name = name 
                # req.relative_entity_name = self.base_link_frame
                try:
                    res = self.model_state_client(name, self.base_link_frame)
                except rospy.ServiceException as exc:
                    rospy.logerr("Service did not process request: " + str(exc))
                    continue
                
                if not res.success:
                    rospy.logerr(f"Could not get model_state with message {res.status_message}")
                    continue 

                pose = res.pose

                x = pose.position.x 
                y = pose.position.y

                if x < self.x_max and x > self.x_min and y > self.y_min and y < self.y_max:
                    rospy.logdebug(f"Adding model {name} to visible balls with pose {str(pose)}")
                    # pose.position.z = 0.0 
                    # pose.orientation.x = 0 
                    # pose.orientation.y = 0 
                    # pose.orientation.z = 0 
                    # pose.orientation.w = 1.0
                    ret_list.append(pose)

        self.visible_ball_list = ret_list
        self.last_update_time = rospy.Time.now()

    def link_state_cb(self, msg):
        pass

    def update_cb(self, event):
        rospy.logdebug("Running update")
        msg = PoseArray()
        
        msg.header.stamp = self.last_update_time
        msg.header.frame_id = self.base_link_frame
        msg.header.seq = self.pub_seq 

        self.pub_seq += 1 

        msg.poses = self.visible_ball_list

        self.visible_ball_pub.publish(msg)


def main():
    rospy.init_node("simulated_seeker")
    rospy.loginfo("Starting simulated seeker node")

    seeker = SimSeeker()

    rospy.spin()

if __name__ == "__main__":
    main()
