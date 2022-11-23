import rospy 

from leo_sim.srv import SpawnBalls, SpawnBallsResponse

from leo_sim.spawn_balls import BallSpawner

spawner = BallSpawner()

def handle_balls(req):
    spawner.spawn_n(req.n_balls)
    return SpawnBallsResponse(True)

if __name__ == "__main__":
    rospy.init_node("spawn_balls_service_node")

    srv = rospy.Service("spawn_balls", SpawnBalls, handle_balls)

    rospy.spin()
