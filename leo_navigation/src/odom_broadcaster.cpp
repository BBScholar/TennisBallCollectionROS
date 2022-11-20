#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"

#include <ros/spinner.h>

nav_msgs::Odometry g_current_odom;

void odom_cb(const nav_msgs::Odometry &msg) { g_current_odom = msg; }

int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry_broadcaster");

  ros::NodeHandle n;
  ros::Subscriber odom_sub = n.subscribe("/leo/diff_drive/odom", 64, odom_cb);

  g_current_odom = nav_msgs::Odometry();
  g_current_odom.pose.pose.orientation.w = 1.0;

  tf::TransformBroadcaster odom_broadcaster;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate r(30);
  while (ros::ok()) {
    ros::Time current_time = ros::Time::now();
    geometry_msgs::TransformStamped odom_trans;

    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_root";

    // double x = g_current_odom.pose.pose.position.x;
    // double y = g_current_odom.pose.pose.position.z;

    odom_trans.transform.translation.x = g_current_odom.pose.pose.position.x;
    odom_trans.transform.translation.y = g_current_odom.pose.pose.position.y;
    odom_trans.transform.translation.z = 0.0;

    // geometry_msgs::Quaternion odom_quat =

    // odom_trans.transform.translation.x = x;
    // odom_trans.transform.translation.y = y;
    // odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = g_current_odom.pose.pose.orientation;

    odom_broadcaster.sendTransform(odom_trans);

    r.sleep();
  }

  return 0;
}
