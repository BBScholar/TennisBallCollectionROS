#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

int main(int argc, char **argv) {

  ros::init(argc, argv, "go_to_node");

  ros::NodeHandle nh;

  ros::Duration d(10.0);

  Client client("/leo/move_base", true);

  client.waitForServer();

  d.sleep();

  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 5.0;
  goal.target_pose.pose.position.y = 5.0;
  goal.target_pose.pose.orientation.w = 1.0;

  client.sendGoal(goal);

  client.waitForResult();

  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  return 0;
}
