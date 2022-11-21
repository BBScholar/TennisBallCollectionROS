#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

int main(int argc, char **argv) {

  ros::init(argc, argv, "go_to_node");

  ros::NodeHandle nh;

  // ros::Duration d(10.0);

  Client client("/leo/move_base", true);

  client.waitForServer();

  // d.sleep();

  move_base_msgs::MoveBaseGoal goal;

  tf2::Quaternion quat;
  quat.setRPY(0, 0, 0);
  // tf2::convert(quat2, quat);
  // geometry_msgs::Quaternion quat2;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 5.0;
  goal.target_pose.pose.position.y = 5.0;
  // goal.target_pose.pose.orientation.w = 1.0;
  goal.target_pose.pose.orientation = tf2::toMsg(quat);

  client.sendGoal(goal);
  client.waitForResult();

  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 10.0;
  goal.target_pose.pose.position.y = 10.0;

  client.sendGoal(goal);
  client.waitForResult();

  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = -5;
  goal.target_pose.pose.position.y = -5;

  client.sendGoal(goal);
  client.waitForResult();

  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  return 0;
}
