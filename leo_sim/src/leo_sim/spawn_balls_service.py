#!/usr/bin/env python3

import rospy 

from leo_sim.srv import SpawnBalls, SpawnBallsResponse

from leo_sim.spawn_balls import BallSpawner

class BallSpawnerService:
    def __init__(self) -> None:
        self.server = rospy.Service("/spawn_balls", SpawnBalls, self.handle_balls)
        self.spawner = BallSpawner()


    def handle_balls(self, req):
        self.spawner.spawn_n(req.num_balls)

        res = SpawnBallsResponse()
        res.success = True
        return res

if __name__ == "__main__":
    rospy.init_node("spawn_balls_service_node")
    rospy.logwarn("Starting ball spawning service")

    srv = BallSpawnerService()

    rospy.spin()
