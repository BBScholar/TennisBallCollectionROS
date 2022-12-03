#!/usr/bin/env python3

import roslib 
import rospy 

import tf2_ros as tf2 
from tf2_ros import TransformListener, Buffer

import tf2_geometry_msgs

from leo_sim.srv import ClosestBall, ClosestBallResponse
from leo_sim.srv import RemoveBall, RemoveBallResponse

from geometry_msgs.msg import Pose, PoseArray, PoseStamped

from std_msgs.msg import Int32

import math

class BallLocations:

    def __init__(self) -> None:
        self.map_frame = "map"
        
        self.tf_buf = Buffer()
        self.tf_list = TransformListener(self.tf_buf)
        
        # TODO: read topic from param
        self.visible_balls_sub = rospy.Subscriber("sim_seeker/visible_balls", PoseArray, self.visible_balls_cb, queue_size=32)

        self.ball_locations_pub = rospy.Publisher("ball_locations/ball_locations", PoseArray, queue_size=32)
        self.ball_count_pub = rospy.Publisher("ball_locations/ball_count", Int32, queue_size=32)
        
        self.closest_ball_service = rospy.Service("ball_locations/closest_ball", ClosestBall, self.handle_closest)
        self.remove_ball_service = rospy.Service("ball_locations/remove_ball", RemoveBall, self.handle_remove)

        self.locations = []
        self.last_update_time = rospy.Time.now()

        self.closest_seq = 0
        self.ball_location_seq = 0

        self.replacement_threshold = 0.1 # 5 centimeters
        self.remove_threshold = self.replacement_threshold

        self.update_rate = rospy.Duration(0.1)
        self.update_timer = rospy.Timer(self.update_rate, self.update_cb)

    # between two pose messages
    def calculate_distance(self, a, b):
        # ignore z difference (for now)
        return math.hypot(a.position.x - b.position.x, a.position.y - b.position.y)

    def find_closest_ball(self, pose):
        idx = 0
        closest_pose = self.locations[0]
        shortest_dist = self.calculate_distance(pose, closest_pose) 

        for i in range(len(self.locations)):
            if i == 0:
                continue 

            dist = self.calculate_distance(pose, self.locations[i])

            if dist < shortest_dist:
                idx = i
                shortest_dist = dist 
                closest_pose = self.locations[i]

        return idx, shortest_dist, closest_pose
    


    def get_transform(self, target, source):
        if not self.tf_buf.can_transform(target, source, rospy.Time(0)):
            rospy.logerr(f"Cannot transform from {target} to {source} frames")
            return None

        try:
            transform = self.tf_buf.lookup_transform(target, source, rospy.Time(0))
        except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationExpection):
            rospy.logerr("Could not lookup transform")
            return None 

        return transform



    def handle_remove(self, req):
        res = RemoveBallResponse()

        if len(self.locations) == 0:
            res.success = False 
            res.text = "No balls in list"
            return res 

        estimated_pose = req.estimated_pose
        
        t = self.get_transform(self.map_frame, estimated_pose.header.frame_id)

        if t is None:
            res.success = False 
            res.text = "Could not get transform between frames"
            return res

        transformed_pose = tf2_geometry_msgs.do_transform_pose(estimated_pose, t)

        tolerance = req.tolerance
        if tolerance == 0.0:
            tolerance = self.remove_threshold 
         
        idx, shortest_dist, closest_pose = self.find_closest_ball(transformed_pose.pose)
        
        if shortest_dist > tolerance:
            res.success = False 
            res.message = "No pose found within tolerance"
            res.distance = shortest_dist
            return res

        rospy.logdebug(f"Removing pose {str(closest_pose)}")
       
        # remove from list
        del self.locations[idx]

        res.success = True 
        res.message = ""
        res.distance = shortest_dist 

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.map_frame 
        pose_stamped.header.stamp = self.last_update_time
        
        res.removed_pose = pose_stamped 

        return res


    def handle_closest(self, req):
        res = ClosestBallResponse()

        if len(self.locations) == 0:
            res.success = False 
            res.text = "No balls visible"
            return res
        
        base_pose = req.current_pose
        max_distance = req.max_distance

        t = self.get_transform(self.map_frame, base_pose.frame_id)

        if t is None:
            res.success = False 
            res.text = "Could not get transform between frames"
            return res
    
        transformed_pose = tf2_geometry_msgs.do_transform_pose(base_pose, t)

        idx, shortest_dist, closest_pose = self.find_closest_ball(transformed_pose.pose)

        rospy.logdebug(f"Point with shortest distance ({shortest_dist} meters) is {str(closest_pose)}")        

        if shortest_dist > max_distance:
            req.success = False 
            req.text = "No balls are within specified maximum range"
            req.distance = shortest_dist
            return req
        
       
        res.success = True 
        res.text = ""
        res.distance = shortest_dist 
        
        pose_stamped = PoseStamped() 
        pose_stamped.pose = closest_pose 
        pose_stamped.header.frame_id = self.map_frame
        pose_stamped.header.stamp = self.last_update_time

        res.ball_pose = pose_stamped

        return res

    def visible_balls_cb(self, msg):
        frame_id = msg.header.frame_id

        transform = self.get_transform(self.map_frame, frame_id)

        if transform is None:
            rospy.logerr_throttle_identical(10.0, "Ball location: Cannot find transform between frames")
            return

        num_replacements = 0 
        num_additions = 0

        for pose in msg.poses:
            temp = PoseStamped()
            temp.pose = pose 
            pose = temp

            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)

            if len(self.locations) == 0:
                num_additions += 1 
                self.locations.append(transformed_pose.pose)
                continue

            idx, shortest_dist, closest_pose = self.find_closest_ball(transformed_pose.pose)

            if shortest_dist < self.replacement_threshold:
                self.locations[idx] = transformed_pose.pose
                num_replacements += 1 
            else:
                rospy.logdebug(f"Appending pose to list {str(transformed_pose.pose)}")
                num_additions += 1
                self.locations.append(transformed_pose.pose)

        rospy.loginfo(f"Update yielded {num_replacements} replacements and {num_additions} additions")
        self.last_update_time = rospy.Time.now()

    def update_cb(self, event):
        rospy.logdebug("Running update")
        ball_locations = PoseArray()
        
        ball_locations.header.seq = self.ball_location_seq 
        self.ball_location_seq += 1
        ball_locations.header.frame_id = self.map_frame 
        ball_locations.header.stamp = self.last_update_time
        ball_locations.poses = self.locations

        self.ball_locations_pub.publish(ball_locations)

        ball_count = Int32() 
        ball_count.data = len(self.locations)
        self.ball_count_pub.publish(ball_count)

def main():
    rospy.init_node("ball_location_node")

    ball_loc = BallLocations()

    rospy.spin()

if __name__ == "__main__":
    main()

