#!/usr/bin/env python3

import roslib 
# roslib.load_manifest("leo_sim")

import rospy 

import actionlib

from gazebo_msgs.msg import ContactsState
from gazebo_msgs.srv import DeleteModel
from std_srvs.srv import SetBool, SetBoolResponse

from std_msgs.msg import Int32, Bool

# from leo_sim.msg import DumpAction
from leo_sim.msg import DumpAction, DumpActionGoal

class SimDumpServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer("vacuum/dump", DumpAction, self.execute, False)

    def start(self):
        self.server.start()

    def execute(self, goal):
        dur = goal.dur
        # rospy.sleep(self.duration)
        dur.sleep()
        self.num_balls = 0
        self.server.set_succeeded()

class SimVacuum():

    def contact_cb(self, msg):
        if not self.vacuum_on:
            return 

        for state in msg.states:
            s1 = state.collision1_name
            s2 = state.collision2_name

            model_name = s1.split("::")[0]

            if model_name.startswith("ball_"):
                rospy.loginfo(f"Removing model {model_name} from world")

                try:
                    resp = self.remove_model_client(model_name)
                except rospy.ServiceException as e:
                    rospy.logwarn(f"Got error while calling service: {e}")
                    continue
                
                if not resp.success:
                    continue 
                
                self.num_balls += 1 
                if self.num_balls >= self.max_balls:
                    self.vacuum_on = False 
                    return


    def handle_set_state(self, req):
        rospy.loginfo(f"Recieving vacuum request with data: {req.data}")
        if not req.data:
            self.vacuum_on = False 
            return SetBoolResponse(True, "")
        elif req.data and self.num_balls >= self.max_balls:
            return SetBoolResponse(False, "Cannot enable vacuum when bucket is full")
        else:
            self.vacuum_on = True 
            return SetBoolResponse(True, "")

    def __init__(self) -> None:
        self.contact_sub = rospy.Subscriber("vacuum_contacts", ContactsState, self.contact_cb, queue_size=32)

        self.state_pub = rospy.Publisher("vacuum/state", Bool, queue_size=32)
        self.num_balls_pub = rospy.Publisher("vacuum/balls_collected", Int32, queue_size=32)
        self.bucket_full_pub = rospy.Publisher("vacuum/full", Bool, queue_size=32)

        self.state_srv = rospy.Service("vacuum/set_state", SetBool, self.handle_set_state)
        # self.dump_srv = rospy.Service("vacuum/request_dump", SetBool, self.handle_dump)

        self.dump_server = SimDumpServer()
        self.dump_server.start()
        
        rospy.wait_for_service("/gazebo/delete_model")
        self.remove_model_client = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

        self.num_balls = 0
        self.max_balls = 30

        self.vacuum_on = False

        self.get_params()

    def get_params(self):
        if rospy.has_param("~max_balls"):
            self.max_balls = rospy.get_param("~max_balls")

        rospy.loginfo(f"Setting max balls to {self.max_balls}")

    def send_msgs(self):
        self.state_pub.publish(self.vacuum_on)
        self.num_balls_pub.publish(self.num_balls)
        self.bucket_full_pub.publish(self.num_balls >= self.max_balls)

def main():
    rospy.init_node("sim_vacuum")

    sim_vac = SimVacuum()
    sim_vac.get_params()

    loop_rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        sim_vac.send_msgs() 
        loop_rate.sleep()

    


if __name__ == "__main__":
    main()
