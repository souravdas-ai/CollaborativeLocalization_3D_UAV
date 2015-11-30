#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <string>
#include <time.h>

//AED added
#include <tum_ardrone/filter_state.h>

//AED added
void poseLoggerCallback(const tum_ardrone::filter_stateConstPtr statePtr, std::string name)
{
  std::cout << name << ","
            << statePtr->header.stamp << ","
            << statePtr->x << ","
            << statePtr->y << ","
            << statePtr->z << ","
            << statePtr->roll << ","
            << statePtr->pitch << ","
            << statePtr->yaw << "," << std::endl;
}

//AED added
void leaderPoseLoggerCallback(const tum_ardrone::filter_stateConstPtr statePtr)
{
  std::string name = "king";
  poseLoggerCallback(statePtr, name);
}

//AED added
void followerPoseLoggerCallback(const tum_ardrone::filter_stateConstPtr statePtr)
{
  std::string name = "mamba";
  poseLoggerCallback(statePtr, name);
}

int main(int argc, char **argv)
{
    //initialize ros
    ros::init(argc, argv, "pose_logger");
    ros::NodeHandle nh;

    //AED: to output as csv file, just redirect stdout in linux
    //     e.g. 'rosrun logger pose_logger > output.csv'

    //create subscribers
    ros::Subscriber sub_leader_pose = nh.subscribe("/king/ardrone/predictedPose", 100, leaderPoseLoggerCallback);
    ros::Subscriber sub_follower_pose = nh.subscribe("/mamba/ardrone/predictedPose", 100, followerPoseLoggerCallback);

    //output csv format for first line
    std::cout << "name,time,x,y,z,roll,pitch,yaw," << std::endl;

    //let ros run
    ros::spin();
}
