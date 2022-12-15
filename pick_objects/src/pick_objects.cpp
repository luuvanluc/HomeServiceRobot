#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Robot
{
public:
  Robot()
  {
    // Wait 5 sec for move_base action server to come up
    mAc = new MoveBaseClient("move_base", true);
    while(!mAc->waitForServer(ros::Duration(5.0)));
    ROS_INFO("Waiting for the move_base action server to come up");
    // set up the frame parameters
    mGoal.target_pose.header.frame_id = "map";
    mGoal.target_pose.header.stamp = ros::Time::now();
    mGoal.target_pose.pose.position.x = 0;
    mGoal.target_pose.pose.position.y = 0;
    mGoal.target_pose.pose.orientation.w = 0;

  }
  
  ~Robot()
  {
    delete mAc;
  }

  void setGoal(double pos_x, double pos_y, double angle_rad)
  {
    // Define a position and orientation for the robot to reach
    mGoal.target_pose.pose.position.x = pos_x;
    mGoal.target_pose.pose.position.y = pos_y;
    mGoal.target_pose.pose.orientation.w = angle_rad;
  }

  bool startRunningToGoal(void)
  {
    // Send the goal position and orientation for the robot to reach
    mAc->sendGoal(mGoal);

    // Wait an infinite time for the results
    mAc->waitForResult();

    // true if the robot reached its goal
    return (mAc->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
  }

private:
  //MoveBaseClient mAc("move_base", true);
  MoveBaseClient* mAc;
  move_base_msgs::MoveBaseGoal mGoal;

};

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  Robot robot;
  double PickUpPos[3] = {3, 5, 0};
  double DropOffPos[3] = {11, 4, 0};

  ROS_INFO("Set the FIRST goal - PICK-UP POSITION");
  robot.setGoal(PickUpPos[0], PickUpPos[1], 1.57);
  bool state = robot.startRunningToGoal();

  // Check if the robot reached its goal
  if(state)
    ROS_INFO("Hooray, the robot reach the FIRST goal");
  else
    ROS_INFO("The robot failed to reach the FIRST goal for some reason");

  // Wait 5 seconds
  ros::Duration(5.0).sleep();

  ROS_INFO("Set the SECOND goal - DROP-OFF POSITION");
  robot.setGoal(DropOffPos[0], DropOffPos[1], 1.57);
  state = robot.startRunningToGoal();

  // Check if the robot reached its goal
  if(state)
    ROS_INFO("Hooray, the robot reach the SECOND goal");
  else
    ROS_INFO("The robot failed to reach the SECOND goal for some reason");

  // Wait 5 seconds
  ros::Duration(5.0).sleep();

  return 0;
}
