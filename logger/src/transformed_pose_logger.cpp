#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <string>
#include <time.h>

//AED added
#include <tum_ardrone/filter_state.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>

geometry_msgs::PoseStamped prevKingPose;
//ros::Publisher pubTransformedPose;

//AED added
void poseLoggerCallback(const tum_ardrone::filter_stateConstPtr statePtr, std::string name)
{
  double x = statePtr->x;
  double y = statePtr->y;
  double z = statePtr->z;
  double roll = statePtr->roll;
  double pitch = statePtr->pitch;
  double yaw = statePtr->yaw;

  if(name == "king")
  {
    prevKingPose.header.stamp = statePtr->header.stamp;
    prevKingPose.pose.position.x = x;
    prevKingPose.pose.position.y = y;
    prevKingPose.pose.position.z = z;
    tf::Quaternion q;
    q.setRPY(pitch*M_PI/180,roll*M_PI/180,  -yaw*M_PI/180);
    prevKingPose.pose.orientation.x = q.x();
    prevKingPose.pose.orientation.y = q.y();
    prevKingPose.pose.orientation.z = q.z();
    prevKingPose.pose.orientation.w = q.w();
  }
  else //name == "mamba"
  {
    //since tf convention is positive x being forward while positive y is left
    //but ours is positive x to the right and positive y forward, swap for transform
    //calculations, then swap back after
    geometry_msgs::TransformStamped transform;
    transform.transform.translation.x = prevKingPose.pose.position.y;
    transform.transform.translation.y = -prevKingPose.pose.position.x;
    transform.transform.translation.z = prevKingPose.pose.position.z;
    transform.transform.rotation.x = prevKingPose.pose.orientation.x;
    transform.transform.rotation.y = prevKingPose.pose.orientation.y;
    transform.transform.rotation.z = prevKingPose.pose.orientation.z;
    transform.transform.rotation.w = prevKingPose.pose.orientation.w;
    
    geometry_msgs::PoseStamped mambaPose;
    mambaPose.header.stamp = statePtr->header.stamp;
    mambaPose.header.frame_id = "king_world";
    mambaPose.pose.position.x = y;
    mambaPose.pose.position.y = -x;
    mambaPose.pose.position.z = z;
    
    tf::Quaternion q;
    q.setRPY(pitch*M_PI/180,roll*M_PI/180,  -yaw*M_PI/180);
    mambaPose.pose.orientation.x = q.x();
    mambaPose.pose.orientation.y = q.y();
    mambaPose.pose.orientation.z = q.z();
    mambaPose.pose.orientation.w = q.w();
    
    geometry_msgs::PoseStamped mambaTransformedPose;
    
    tf2::doTransform(mambaPose, mambaTransformedPose, transform);
    
    x = -mambaTransformedPose.pose.position.y;
    y = mambaTransformedPose.pose.position.x;
    z = mambaTransformedPose.pose.position.z;
    q = tf::Quaternion(mambaTransformedPose.pose.orientation.x,
                       mambaTransformedPose.pose.orientation.y,
                       mambaTransformedPose.pose.orientation.z,
                       mambaTransformedPose.pose.orientation.w);
    tf::Matrix3x3 rotationMatrix(q);
    rotationMatrix.getRPY(pitch, roll, yaw);
    roll = roll*180/M_PI;
    pitch = pitch*180/M_PI;
    yaw = -yaw*180/M_PI;
    
    //mambaTransformedPose.header.frame_id = "king_world";
    //mambaTransformedPose.pose.position.x = x;
    //mambaTransformedPose.pose.position.y = y;
    //pubTransformedPose.publish(mambaTransformedPose);
  }

  //output the csv line
  std::cout << name << ","
            << statePtr->header.stamp << ","
            << x << ","
            << y << ","
            << z << ","
            << roll << ","
            << pitch << ","
            << yaw;
  if(name == "mamba")
  {
    std::cout << ","
              << statePtr->x << ","
              << statePtr->y << ","
              << statePtr->z << ","
              << statePtr->roll << ","
              << statePtr->pitch << ","
              << statePtr->yaw;
  }
  std::cout << std::endl;
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

    //publish into rviz to compare with rviz-transformed mamba coordinates
    //pubTransformedPose = nh.advertise<geometry_msgs::PoseStamped>("/mamba/poseTransformed", 1);

    //output csv format for first line
    //std::cout << "name,time,x,y,z,roll,pitch,yaw," << std::endl;

    prevKingPose.header.frame_id = "world";
    prevKingPose.header.stamp = ros::Time(0);
    prevKingPose.pose.position.x = 0;
    prevKingPose.pose.position.y = 0;
    prevKingPose.pose.position.z = 0;
    prevKingPose.pose.orientation.x = 0;
    prevKingPose.pose.orientation.y = 0;
    prevKingPose.pose.orientation.z = 0;
    prevKingPose.pose.orientation.w = 1;

    //let ros run
    ros::spin();
}
