#ifndef PARTICLE_FILTER_NODE_H_
#define PARTICLE_FILTER_NODE_H_

#include "ros/ros.h"

#include <iostream>
#include <fstream>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>


#include <dynamic_reconfigure/server.h>
#include <particle_filter/ParticleFilterConfig.h>
#include <dot_finder/DuoDot.h>



#include <particle_filter/visualization.h>
#include <particle_filter/region_of_interest.h>
#include <particle_filter/mutual_pose_estimation.h>

//AED added to adapt to new controller
#include <tum_ardrone/filter_state.h>
#include <ardrone_autonomy/Navdata.h>

//AED added to view path in rviz
#include <nav_msgs/Path.h>

//AED added to deal with pose transforms
#include <tf/transform_broadcaster.h>

namespace particle_filter
{

typedef  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > ListVector2d;

class ParticleFilter{
public:
	ParticleFilter(ros::NodeHandle n);
    void leaderDotsCallback(const dot_finder::DuoDot::ConstPtr& dots_msg);
    void followerDotsCallback(const dot_finder::DuoDot::ConstPtr& dots_msg);
    void leaderImuCallback(const sensor_msgs::Imu::ConstPtr& pLeader_imu_msg);
    void followerImuCallback(const sensor_msgs::Imu::ConstPtr& pFollower_imu_msg);
//AED added to adapt for new controller
    void followerNavdataCallback(const ardrone_autonomy::NavdataConstPtr navdataPtr);
	void dynamicParametersCallback(particle_filter::ParticleFilterConfig &config, uint32_t level);

	void visualizationCallbackLeader(const sensor_msgs::Image::ConstPtr& image_msg);
	void visualizationCallbackFollower(const sensor_msgs::Image::ConstPtr& image_msg);
	void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
	
	void ptamLeaderCallback(const tum_ardrone::filter_stateConstPtr statePtr);

    void generateCSVLog();
// PB added to correct the pose with IMU
    geometry_msgs::PoseStamped generatePoseMessage(const Eigen::Vector3d &position, Eigen::Matrix3d rotation);
//AED added to adapt for new controller
    tum_ardrone::filter_state generatePredictedPoseMessage(const Eigen::Vector3d &position, Eigen::Matrix3d rotation);

private:
    void createPublishers(const std::string& topic_leader, const std::string& topic_follower);
    void createSubscribers(const std::string& topic_leader, const std::string& topic_follower);
    ListVector2d fromROSPoseArrayToVector2d(std::vector<geometry_msgs::Pose2D> ros_msg);
    std::vector<cv::Point2f> fromROSPoseArrayToCvPoint(std::vector<geometry_msgs::Pose2D> ros_msg);
    Eigen::Vector3d fromRelativePositionToGlobalPositionViaIMU(const Eigen::Vector3d &position);
    void runParticleFilter();
    bool isAllMessageInitiated();

    bool leaderDotsInitiation, followerDotsInitiation;
    bool leaderImuInitiation, followerImuInitiation;
    dot_finder::DuoDot leaderLastMsg, followerLastMsg;
    sensor_msgs::Imu leaderImuMsg, followerImuMsg;
    RegionOfInterest regionOfInterest[2];

    cv::Mat lastLeaderImgRaw, lastFollowerImgRaw;
    Visualization visualization;


    bool haveCameraInfo;
    sensor_msgs::CameraInfo camInfo;
    cv::Mat cameraMatrixK;
    Eigen::Matrix<double, 3, 4> cameraProjectionMatrix;

    geometry_msgs::PoseArray candidatesPoseMsgs;

    MutualPoseEstimation poseEvaluator;

    ros::Subscriber subDotsLeader;
    ros::Subscriber subDotsFollower;
    ros::Subscriber subImuLeader;
    ros::Subscriber subImuFollower;
    ros::Subscriber subVisualizationLeader;
    ros::Subscriber subVisualizationFollower;
    ros::Subscriber subCameraInfo;

//AED added to adapt for new controller
    ros::Subscriber subNavdataFollower;
    ros::Publisher pubPredictedPose; 
    ardrone_autonomy::Navdata lastNavdataReceived;
    int prevZpackageID;
    double prevNavTimeMicros, timespan;
    double prevZmsg, prevZnavdata;//, prevZdiff;
    double prevYawNavdata, prevYawdiff;
    bool initdZ, initdYaw;

//AED added for rviz viewing of leader pose
    ros::Publisher pubLeaderPose;
//AED added to view path in rviz
    ros::Publisher pubPath;
    nav_msgs::Path pathMsg;
    int tempSeq;// = 0;
    
//AED added to deal with pose transforms
    bool ptamInitialized;
    ros::Subscriber subPtamLeader;

    ros::Publisher pubPose;//, pubPoseCorrect;
    ros::Publisher pubMarker;
    ros::Publisher pubMarkerCandidates;
    ros::Publisher pubPoseCandidates;
    ros::Publisher pubImuLeader;
    image_transport::Publisher pubROILeader;
    image_transport::Publisher pubROIFollower;

    ros::NodeHandle nodeHandler;

    dynamic_reconfigure::Server<particle_filter::ParticleFilterConfig> dynamicReconfigServer; //!< The dynamic reconfigure server
    dynamic_reconfigure::Server<particle_filter::ParticleFilterConfig>::CallbackType dynamicReconfigCallback; //!< The dynamic reconfigure callback type

    bool position_correction_with_imu_enable;

    //tf::Matrix3x3 imuPitchRot, imuGlobalRot;
};

}


#endif /* PARTICLE_FILTER_NODE_H_ */

