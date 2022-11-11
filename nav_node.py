import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

VALID_STATES = {"IDLE", "ZERO", "MAP", "COLLECT", "RETURN"}


state = "IDLE"



pub = rospy.Publisher('diff/cmd_vel', Twist, queue_size=1)

lidar_sub = None
wall_ahead = False


def init():
    global lidar_sub
    rospy.init_node('cmd_srv')
    srv = rospy.Service('cmd_srv', String, cmd_handler)
    lidar_sub = rospy.Subscriber('laser_1', LaserScan, lidar_callback)


def navigate():

    while 1:
        if state == "IDLE":
            pass
        elif state == "ZERO":
            pass
        elif state == "MAP":
            move(1, 0)
        elif state == "COLLECT":
            pass
        elif state == "RETURN":
            pass

        rospy.spin_once()


def cmd_handler(msg):
    global state
    state = msg if msg in VALID_STATES else state


def lidar_callback():
    pass


def move(x, theta):
    move_cmd = Twist()
    move_cmd.linear.x = x
    move_cmd.angluar.z = theta
    pub.publish(move_cmd)


if __name__ == '__main__':
    init()
    navigate()
