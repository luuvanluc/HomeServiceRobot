#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  double PickUpPos[3] = {3, 5, 0};
  double DropOffPos[3] = {11, 4, 0};
  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = PickUpPos[0];
  goal.target_pose.pose.position.y = PickUpPos[1];
  goal.target_pose.pose.orientation.w = 1.57;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending the FIRST goal - PICK-UP POSITION");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the robot reach the FIRST goal");
  else
    ROS_INFO("The robot failed to reach the FIRST goal for some reason");

  // Wait 5 seconds
  ros::Duration(5.0).sleep();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = DropOffPos[0];
  goal.target_pose.pose.position.y = DropOffPos[1];
  goal.target_pose.pose.orientation.w = 1.57;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending the SECOND goal - DROP-OFF POSITION");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the robot reach the SECOND goal");
  else
    ROS_INFO("The robot failed to reach the SECOND goal for some reason");

  // Wait 5 seconds
  ros::Duration(5.0).sleep();

  return 0;
}
