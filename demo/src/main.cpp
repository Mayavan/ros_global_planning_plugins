/**
 * @file  main.cpp
 * @brief Demo application to run the path planning with different global planners
 *
 * @author Mayavan
 * MIT License
 *
 * Copyright (c) 2020 Mayavan
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 *
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "demo");

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",
                                                                   true);
  // Wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal targetPose;

  // Add the points to the goal vector
  targetPose.target_pose.header.frame_id = "map";
  targetPose.target_pose.header.stamp = ros::Time::now();
  targetPose.target_pose.pose.position.x = 2;
  targetPose.target_pose.pose.position.y = 0.5;
  targetPose.target_pose.pose.orientation.z = 0;
  targetPose.target_pose.pose.orientation.w = 1;

  ac.sendGoal(targetPose);
  ac.waitForResult();
  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO_STREAM("Reached goal \n");   
  } else {
    ROS_WARN_STREAM("Failed to reach goal \n");
  }

  return 0;
}
