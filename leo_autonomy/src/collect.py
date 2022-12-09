#!/usr/bin/env python3

import rospy 
import roslib

import math

import actionlib 

from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Twist
from std_msgs.msg import Bool, Int32
from std_srvs.srv import SetBool

import tf2_ros as tf2
from tf2_ros import Buffer, TransformListener

import tf2_geometry_msgs

from nav_msgs.msg import Odometry

from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal

from leo_sim.msg import DumpAction, DumpActionGoal

from scipy.spatial.transform import Rotation

class CollectNode:
    
    def __init__(self) -> None:
        rospy.loginfo("Starting collection node")
        self.tf_buf = Buffer()
        self.tf_list = TransformListener(self.tf_buf)

        self.total_balls_collected = 0

        self.sleep_time = rospy.Duration(1.0)
            
        self.home_coords = Pose()
        self.home_coords.position.x = 1.0
        self.home_coords.position.y = 1.0
        self.home_coords.position.z = 0
        self.home_coords.orientation.w = 1.0
        # self.home_coords.orientation.z = 1.0

        # patrol variables
        self.patrol = []
        self.saved_position = None
        self.move_base_seq = 0
        self.dumps = 0

        self.seek_path_done = False

        # callback state 
        self.vac_state = False 
        self.balls_collected = 0 
        self.balls_full = False
        self.visible_balls = None
        self.odom = None

        # publishers
        self.total_balls_pub = rospy.Publisher("collected/total_balls_collected", Int32, queue_size=32)
        self.velocity_pub = rospy.Publisher("diff_drive/cmd_vel", Twist, queue_size=32)
        
        # subscribers
        self.vacuum_state_sub = rospy.Subscriber("vacuum/state", Bool, self.vac_state_cb, queue_size=32)
        self.balls_collected_sub = rospy.Subscriber("vacuum/balls_collected", Int32, self.balls_collected_cb, queue_size=32)
        self.balls_full_sub = rospy.Subscriber("vacuum/full", Bool, self.balls_full_cb, queue_size=32)
        
        self.visible_balls_sub = rospy.Subscriber("sim_seeker/visible_balls", PoseArray, self.visible_balls_cb, queue_size=2)
        self.odom_sub = rospy.Subscriber("odometry/filtered", Odometry, self.odometry_cb, queue_size=32)
        
        # services
        self.vacuum_state_client = rospy.ServiceProxy("vacuum/set_state", SetBool)
        rospy.loginfo("Waiting for vacuum state client")
        # self.vacuum_state_client.wait_for_service()
        rospy.loginfo("Done waiting for vacuum state client")

        # actionservers
        self.move_base_as = actionlib.SimpleActionClient("move_base", MoveBaseAction) # check this
        self.move_base_as.wait_for_server()

        self.dumper_as = actionlib.SimpleActionClient("vacuum/dump", DumpAction) 
        self.dumper_as.wait_for_server()

        # update time 
        self.update_timer = rospy.Timer(rospy.Duration(0.1), self.update)
    
    # def move_done_cb(self,)

    def vac_state_cb(self, msg):
        self.vac_state_cb = msg.data 

    def balls_collected_cb(self, msg):
        self.balls_collected = msg.data

    def balls_full_cb(self, msg):
        self.balls_full = msg.data

    def visible_balls_cb(self, msg):
        self.visible_balls = msg

    def update(self, event):
        self.total_balls_pub.publish(self.total_balls_collected)

    def odometry_cb(self, msg):
        self.odom = msg

    def move_base_done(self, state, result):
        # status = state.status.status 
        rospy.loginfo("Done with path")
        rospy.loginfo(f"Path ended with status {state}")
        if state == 3:
            rospy.loginfo("Patrol successful")
            self.patrol = self.patrol[1:] +  [self.patrol[0]]
            self.schedule_patrol()

    def move_seek_done(self, state, result):
        rospy.loginfo(f"Done with seeking path, with code {state}")
        self.seek_path_done = True
        if state != 3:
            rospy.logwarn("Seeking path unsucessfull")

    def generate_patrol(self):
        p1 = Pose()
        p1.position.x = 0.0 
        p1.position.y = 0.0 
        p1.position.z = 0.0 
        p1.orientation.w = 1.0
        
        p2 = Pose()
        p2.position.x = 1.0
        p2.position.y = 32.0
        p2.position.z = 0.0 
        p2.orientation.w = 1.0

        p3 = Pose()
        p3.position.x = 15.0 
        p3.position.y = 32.0
        p3.position.z = 0.0 
        p3.orientation.w = 1.0

        p4 = Pose() 
        p4.position.x = 15.0
        p4.position.y = 0.0 
        p4.position.z = 0.0 
        p4.orientation.w = 1.0

        self.patrol.append(p1)
        self.patrol.append(p2)
        self.patrol.append(p3)
        self.patrol.append(p4)


    def get_current_position(self):
        odom_pose = self.odom.pose.pose

        odom_map_t = self.get_transform("map", "odom")

        if odom_map_t is None:
            rospy.logerr("Cannot get odom transform")
            return None

        pose_stamped = PoseStamped()
        pose_stamped.pose = odom_pose 

        map_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, odom_map_t)

        return map_pose.pose


    def save_position(self):
        map_pose = self.get_current_position() 
        rospy.loginfo(f"Saved position: {str(map_pose)}")
        self.saved_position = map_pose

    def cancel_all_move_goals(self):
        self.move_base_as.cancel_all_goals()

    def go_to_saved(self):
        if self.saved_position is not None:
            self.add_move_goal(self.saved_position, False)
            self.saved_position = None


    def add_move_goal(self, pose, enqueue_patrol=True):
        goal = MoveBaseActionGoal()
        id = actionlib.GoalID()

        pose_stamped = PoseStamped()
        pose_stamped.pose = pose
        pose_stamped.header.frame_id = "map"

        id.id = f"collect_move_{self.move_base_seq}" 
        id.stamp = rospy.Time.now()

        goal.header.stamp = id.stamp
        goal.header.seq = self.move_base_seq 
        self.move_base_seq += 1 
        goal.header.frame_id = "map" # TODO: check 
        goal.goal_id = id 
        goal.goal.target_pose = pose_stamped
        
        if enqueue_patrol:
            self.move_base_as.send_goal(goal.goal, done_cb=self.move_base_done)
        else:
            self.move_base_as.send_goal(goal.goal, done_cb=self.move_seek_done)

    def pose_dist(self, a, b):
        x = a.position.x - b.position.x 
        y = a.position.y - b.position.y 
        dist = math.hypot(x, y)
        return dist



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

    def schedule_patrol(self):
        self.add_move_goal(self.patrol[0], True)

    def set_vacuum_state(self, state):
        res = self.vacuum_state_client(state)
        return res

    def rotate_pose_radians(self, pose):
        pass

    def generate_ball_pose(self, pose, frame_id):
        map_vac_t = self.get_transform("map", "vacuum_link")
        base_vacuum_t = self.get_transform("vacuum_link", frame_id)

        if map_vac_t is None or base_vacuum_t is None:
            rospy.logerr("Bad transform in ball pose function")
            return False 

        pose_stamped = PoseStamped()
        pose_stamped.pose = pose

        vacuum_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, base_vacuum_t)

        x = pose.position.x
        y = pose.position.y

        at = math.atan2(y, x)

        rot = Rotation.from_euler('xyz', [0, 0, at], degrees=False)
        quat = rot.as_quat()

        vacuum_pose.pose.orientation.w = quat[3]
        vacuum_pose.pose.orientation.x = quat[0]
        vacuum_pose.pose.orientation.y = quat[1]
        vacuum_pose.pose.orientation.z = quat[2]

        map_pose = tf2_geometry_msgs.do_transform_pose(vacuum_pose, map_vac_t)

        # ARCTAN stuff here
        return map_pose.pose

    def get_closest_ball(self):
        # t = self.get_transform("map", self.visible_balls.header.frame_id)
        # if t is None:
        #     rospy.logerr("Cannot get transform for closest ball")
        #     return None
        current_location = self.get_current_position()
        closest_pose = self.generate_ball_pose(self.visible_balls.poses[0], self.visible_balls.header.frame_id)
        shortest_dist = self.pose_dist(current_location, closest_pose)
        for i in range(1, len(self.visible_balls.poses)):
            new_pose = self.generate_ball_pose(self.visible_balls.poses[i], self.visible_balls.header.frame_id)
            new_dist = self.pose_dist(current_location, new_pose)

            if new_dist < shortest_dist:
                shortest_dist = new_dist 
                closest_pose = new_pose

        return closest_pose
            

    def stop(self):
        twist = Twist()
        self.velocity_pub.publish(twist)

    def wait_for_stop(self):
        rospy.loginfo("waiting for robot to stop")
        while not rospy.is_shutdown() and not abs(self.odom.twist.twist.linear.x) < 0.02 and not abs(self.odom.twist.twist.angular.z) < 0.02:
            rospy.sleep(0.1)
        rospy.loginfo("Robot has stopped")
        rospy.sleep(0.5)


    def run(self):
        self.generate_patrol()
        while not rospy.is_shutdown():

            if self.saved_position is not None:
                self.go_to_saved()
                self.move_base_as.wait_for_result()
            self.schedule_patrol()

            while not rospy.is_shutdown() and not self.balls_full:
                if self.visible_balls is not None and len(self.visible_balls.poses) > 0:

                    self.cancel_all_move_goals()
                    self.stop()
                    self.wait_for_stop()
                    self.save_position()

                    rospy.loginfo("Turning vacuum on")
                    self.set_vacuum_state(True)

                    while not rospy.is_shutdown() and not self.balls_full and len(self.visible_balls.poses) > 0:
                        # seeking_pose = self.visible_balls.poses[0]

                        # ball_pose = self.generate_ball_pose(seeking_pose, self.visible_balls.header.frame_id)
                        ball_pose = self.get_closest_ball()
                        rotation_pose = self.get_current_position()
                        rotation_pose.orientation = ball_pose.orientation
                        # rotation_pose.position = self.o

                        self.add_move_goal(rotation_pose, False)
                        self.move_base_as.wait_for_result()
                        self.seek_path_done = False
                        self.wait_for_stop()
                        rospy.loginfo("Adding seek move goal")
                        self.add_move_goal(ball_pose, False)
                        # self.seek_path_done = False

                        balls_before = self.balls_collected
                        rospy.loginfo(f"Balls before: {balls_before}")
                        
                        rospy.loginfo("Waiting for ball or path completion...")
                        while not rospy.is_shutdown() and self.balls_collected == balls_before and not self.seek_path_done:
                            rospy.loginfo(f"Balls now: {self.balls_collected}")
                            rospy.sleep(0.1)
                        self.seek_path_done = False
                        self.cancel_all_move_goals()
                        self.wait_for_stop()

                    rospy.loginfo("Turning vacuum off")
                    self.set_vacuum_state(False)
                    self.go_to_saved()
                    self.move_base_as.wait_for_result()
                    if not self.balls_full:
                        self.schedule_patrol()

                rospy.sleep(0.1) 
            
            self.stop()
            self.wait_for_stop()
            self.save_position()
            self.cancel_all_move_goals() 

            # go home 
            rospy.loginfo("Going to dump")
            self.add_move_goal(self.home_coords, False)
            self.move_base_as.wait_for_result()
            self.seek_path_done = False
            # wait until home

            # dump balls 
            rospy.loginfo("Dumping...")
            self.total_balls_collected += self.balls_collected
            dump_goal = DumpActionGoal()
            dump_goal.goal.dur = rospy.Duration(3.0)
            dump_goal.goal_id.id = f"dump_{self.dumps}"
            self.dumps += 1 
            dump_goal.goal_id.stamp = rospy.Time.now()
            self.dumper_as.send_goal_and_wait(dump_goal.goal)
            rospy.loginfo("Dumping done")
            rospy.loginfo(f"Collected {self.total_balls_collected} in total")
            rospy.sleep(2.0)

def main():
    rospy.init_node("collect_node")
    
    collect = CollectNode()
    collect.run()

    rospy.spin()


if __name__ == "__main__":
    main()
