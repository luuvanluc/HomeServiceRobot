/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class CubeMarker
{
public:
  CubeMarker()
  {
    mPub = mNode.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    mSub = mNode.subscribe("/amcl_pose", 10, &CubeMarker::callback, this);
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    mMarker.header.frame_id = "map";
    mMarker.header.stamp = ros::Time::now();
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    mMarker.ns = "basic_shapes";
    mMarker.id = mId++;
    mMarker.type = visualization_msgs::Marker::CUBE;
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    mMarker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    mMarker.pose.position.x = 0;
    mMarker.pose.position.y = 0;
    mMarker.pose.position.z = 0;
    mMarker.pose.orientation.x = 0.0;
    mMarker.pose.orientation.y = 0.0;
    mMarker.pose.orientation.z = 0.0;
    mMarker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    mMarker.scale.x = 0.1;
    mMarker.scale.y = 0.1;
    mMarker.scale.z = 0.1;
    // Set the color -- be sure to set alpha to something non-zero!
    mMarker.color.r = 0.0f;
    mMarker.color.g = 1.0f;
    mMarker.color.b = 0.0f;
    mMarker.color.a = 1.0;
    
    status = VISIBLE;

    mMarker.lifetime = ros::Duration();
  }

  void setPosition(double x, double y, double z)
  {
    mMarker.pose.position.x = x;
    mMarker.pose.position.y = y;
    mMarker.pose.position.z = z;
  }

  void setSize(double x, double y, double z)
  {
    mMarker.scale.x = x;
    mMarker.scale.y = y;
    mMarker.scale.z = z;
  }

  void setColor(float r, float g, float b)
  {
    mMarker.color.r = r;
    mMarker.color.g = g;
    mMarker.color.b = b;
  }

  void hide(void)
  {
    status = INVISIBLE;
    mMarker.color.a = 0.0;
    mPub.publish(mMarker);
  }

  void display(void)
  {
    status = VISIBLE;
    mMarker.color.a = 1.0;
    mPub.publish(mMarker);
  }

  void setPickUpPosition(double x, double y, double z)
  {
    mPickUpPos.x = x;
    mPickUpPos.y = y;
    mPickUpPos.z = z;
  }

  void setDropOffPosition(double x, double y, double z)
  {
    mDropOffPos.x = x;
    mDropOffPos.y = y;
    mDropOffPos.z = z;    
  }


private:
  enum STATUS {INVISIBLE, VISIBLE};
  static int mId;
  visualization_msgs::Marker mMarker;
  ros::NodeHandle mNode;
  ros::Publisher mPub;
  ros::Subscriber mSub;
  geometry_msgs::Point mPickUpPos;
  geometry_msgs::Point mDropOffPos;
  STATUS status;

  void callback(const geometry_msgs::PoseWithCovarianceStamped& data)
  {
    double x = data.pose.pose.position.x;
    double y = data.pose.pose.position.y;
    double z = data.pose.pose.position.z;
    //ROS_INFO("x = %.3lf, y = %.3lf, z = %.3lf",x,y,z);
    double d1 = calculateDistanceToPickUpPos(x, y, z);
    double d2 = calculateDistanceToDropOffPos(x, y, z);

    ROS_INFO("d1 = %.3lf, d2 = %.3lf",d1,d2);

    if ((d1 < 0.2) && (status == VISIBLE))
    {
      ROS_INFO("PICK UP BOX");
      hide();
    }
    
    if ((d2 < 0.2) && (status == INVISIBLE))
    {
      ROS_INFO("DROP OFF BOX");
      setPosition(mDropOffPos.x, mDropOffPos.y, mDropOffPos.z);
      display();
    }
  }

  double calculateDistanceToPickUpPos(double curRobotPosX, double curRobotPosY, double curRobotPosZ)
  {
    double dX = mPickUpPos.x - curRobotPosX;
    double dY = mPickUpPos.y - curRobotPosY;
    double dZ = mPickUpPos.z - curRobotPosZ;
    double d = sqrt(dX*dX + dY*dY + dZ*dZ);
    return d;
  }

  double calculateDistanceToDropOffPos(double curRobotPosX, double curRobotPosY, double curRobotPosZ)
  {
    double dX = mDropOffPos.x - curRobotPosX;
    double dY = mDropOffPos.y - curRobotPosY;
    double dZ = mDropOffPos.z - curRobotPosZ;
    double d = sqrt(dX*dX + dY*dY + dZ*dZ);
    return d;
  }

};
int CubeMarker::mId = 0;

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");

  double PickUpPos[3] = {3, 5, 0};
  double DropOffPos[3] = {11, 4, 0};
  CubeMarker cube;
  ros::Duration(4.0).sleep();
  cube.setSize(0.2,0.2,0.2);
  
  // Set Pick-up and Drop-off position for cube
  cube.setPickUpPosition(PickUpPos[0], PickUpPos[1], PickUpPos[2]);
  cube.setDropOffPosition(DropOffPos[0], DropOffPos[1], DropOffPos[2]);

  // Set Cube position at Pick-up Position
  cube.setPosition(PickUpPos[0], PickUpPos[1], PickUpPos[2]);
  cube.display();

  // Handle ROS communication events
  ros::spin();

  return 0;

}
