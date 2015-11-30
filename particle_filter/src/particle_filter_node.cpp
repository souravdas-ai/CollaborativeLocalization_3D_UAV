#include "particle_filter/particle_filter_node.h"

using namespace std;

namespace particle_filter
{


ParticleFilter::ParticleFilter(ros::NodeHandle n) : 
    nodeHandler(n),
    haveCameraInfo(false),
    leaderDotsInitiation(false),
    followerDotsInitiation(false),
    leaderImuInitiation(false),
    followerImuInitiation(false),
    initdZ(false), //AED added to adapt for new controller
    initdYaw(false), //AED added to adapt for new controller
    tempSeq(0),
    position_correction_with_imu_enable(true),
    ptamInitialized(false)
  {

  string topic_leader, topic_follower;
  ros::param::get("~leader", topic_leader);
  ros::param::get("~follower", topic_follower);

  ROS_INFO("Subscribing to %s and %s", topic_leader.c_str(), topic_follower.c_str());

  // Set up a dynamic reconfigure server.
  // This should be done before reading parameter server values.
  dynamic_reconfigure::Server<particle_filter::ParticleFilterConfig>::CallbackType dynamicReconfigCallback;
  dynamicReconfigCallback = boost::bind(&ParticleFilter::dynamicParametersCallback, this, _1, _2);
  dynamicReconfigServer.setCallback(dynamicReconfigCallback);

  this->createPublishers(topic_leader, topic_follower);
  this->createSubscribers(topic_leader, topic_follower);

}

void ParticleFilter::createPublishers(const string& topic_leader, const string& topic_follower){
//AED added for rviz viewing of leader pose
    this->pubLeaderPose = this->nodeHandler.advertise<geometry_msgs::PoseStamped>(topic_leader + "/pose", 1);

    this->pubPose = this->nodeHandler.advertise<geometry_msgs::PoseStamped>(topic_follower + "/pose", 1);
    //this->pubPoseCorrect = this->nodeHandler.advertise<geometry_msgs::PoseStamped>(topic_follower + "/poseCorrect", 1);
//AED added to adapt for new controller
    this->pubPredictedPose = this->nodeHandler.advertise<tum_ardrone::filter_state>(topic_follower + "/ardrone/predictedPose", 1);
    this->pubMarker = this->nodeHandler.advertise<visualization_msgs::Marker>(topic_follower + "/marker", 1);
    this->pubMarkerCandidates = this->nodeHandler.advertise<visualization_msgs::MarkerArray>(topic_follower + "/pose_candidates", 1);
    this->pubPoseCandidates = this->nodeHandler.advertise<geometry_msgs::PoseArray>(topic_follower + "/pose_array_candidates", 1);

    //PB added to see the imu vector in rviz
    this->pubImuLeader = this->nodeHandler.advertise<sensor_msgs::Imu>(topic_leader + "/imu_tf_fix", 1);
    
//AED added to view path in rviz
    //this->pubPath = this->nodeHandler.advertise<nav_msgs::Path>("/path", 1);

    image_transport::ImageTransport image_transport(this->nodeHandler);
    this->pubROILeader = image_transport.advertise(topic_leader + "/image_ROI", 1);
    this->pubROIFollower = image_transport.advertise(topic_follower + "/image_ROI", 1);
}

void ParticleFilter::createSubscribers(const string& topic_leader, const string& topic_follower){
    this->subDotsLeader = this->nodeHandler.subscribe(topic_leader + "/dots", 1, &ParticleFilter::leaderDotsCallback, this);
    this->subDotsFollower = this->nodeHandler.subscribe(topic_follower + "/dots", 1, &ParticleFilter::followerDotsCallback, this);

    this->subImuLeader = this->nodeHandler.subscribe(topic_leader + "/ardrone/imu", 1, &ParticleFilter::leaderImuCallback, this);
    this->subImuFollower = this->nodeHandler.subscribe(topic_follower + "/ardrone/imu", 1, &ParticleFilter::followerImuCallback, this);

//AED added to adapt for new controller
    this->subNavdataFollower = this->nodeHandler.subscribe(topic_follower + "/ardrone/navdata", 1, &ParticleFilter::followerNavdataCallback, this);

    this->subVisualizationLeader = this->nodeHandler.subscribe(topic_leader + "/ardrone/image_raw", 1, &ParticleFilter::visualizationCallbackLeader, this);
    this->subVisualizationFollower = this->nodeHandler.subscribe(topic_follower + "/ardrone/image_raw", 1, &ParticleFilter::visualizationCallbackFollower, this);

    this->subCameraInfo = this->nodeHandler.subscribe(topic_leader + "/ardrone/camera_info", 1, &ParticleFilter::cameraInfoCallback, this);
    
    //AED added to deal with pose transforms
    this->subPtamLeader = this->nodeHandler.subscribe(topic_leader + "/ardrone/predictedPose", 1, &ParticleFilter::ptamLeaderCallback, this);

}

//AED added to adapt for new controller
void ParticleFilter::followerNavdataCallback(const ardrone_autonomy::NavdataConstPtr navdataPtr)
{
  lastNavdataReceived = *navdataPtr;

  //changes copied from EstimationNode.cpp
  lastNavdataReceived.rotZ *= -1; // yaw inverted
  lastNavdataReceived.rotY *= -1; // pitch inverted
  lastNavdataReceived.vy *= -1;   // yaw inverted
  lastNavdataReceived.vz *= -1;   // pitch inverted
  lastNavdataReceived.ay *= -1;   // yaw inverted
  lastNavdataReceived.az *= -1;   // pitch inverted
}

void ParticleFilter::ptamLeaderCallback(const tum_ardrone::filter_stateConstPtr statePtr)
{
    if(!ptamInitialized)
        ptamInitialized = true;
      
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    geometry_msgs::PoseStamped leader_pose;
    leader_pose.header.frame_id = "king_world";
    leader_pose.pose.position.x = statePtr->x;
    leader_pose.pose.position.y = statePtr->y;
    leader_pose.pose.position.z = statePtr->z;
	    
    double roll = statePtr->roll;
    double pitch = statePtr->pitch;
    double yaw = statePtr->yaw;
    
    //AED change from degrees to radians
    roll = roll*M_PI/180;
    pitch = pitch*M_PI/180;
    yaw = yaw*M_PI/180;
    
    //AED change RPY later to correct orientation
    
    tf::Quaternion q;
    q.setRPY(pitch,roll,-yaw); //in the order rviz expects
    
    transform.setOrigin(tf::Vector3(statePtr->x,statePtr->y,statePtr->z));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "king_world", "king_pose"));

 	leader_pose.pose.orientation.x = q.getX();
    leader_pose.pose.orientation.y = q.getY();
   	leader_pose.pose.orientation.z = q.getZ();
    leader_pose.pose.orientation.w = q.getW();
    this->pubLeaderPose.publish(leader_pose);
}

void ParticleFilter::leaderImuCallback(const sensor_msgs::Imu::ConstPtr& leaderImuMsg){
    if(!this->leaderImuInitiation)
        this->leaderImuInitiation = true;

    this->leaderImuMsg = *leaderImuMsg;

    // PB added to see the imu vector in rviz
    // The imu has a tf on the ardrone_base_link notardrone_base_frontcam
    //this->leaderImuMsg.header.frame_id = "ardrone_base_frontcam";
    this->leaderImuMsg.header.frame_id = "king_pose"; //AED changed to where our pose will be
    this->pubImuLeader.publish(this->leaderImuMsg);


//AED added to deal with pose transforms
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    if(!ptamInitialized)
    {
//AED added for rviz viewing of leader pose
        geometry_msgs::PoseStamped leader_pose;
        leader_pose.header.frame_id = "king_world";
        leader_pose.pose.position.x = 0;
	    leader_pose.pose.position.y = 0;
	    leader_pose.pose.position.z = 0;
	    


    //convert to Eigen calculation? (I was more comfortable with tf::Quaternion)
	    tf::Quaternion q(this->leaderImuMsg.orientation.x,
	                     this->leaderImuMsg.orientation.y,
	                     this->leaderImuMsg.orientation.z,
	                     this->leaderImuMsg.orientation.w);                   

//AED changed for new controller
    //convert from IMU frame of reference to world
    //tf::Matrix3x3 rotationMatrix(q);
    //double roll, pitch, yaw;
    //rotationMatrix.getRPY(roll, pitch, yaw);
    //q.setRPY(pitch, 0, yaw); //0 because leader defines "forward"
	    tf::Matrix3x3 rotationMatrix(q);

    	double roll, pitch, yaw;
	    rotationMatrix.getRPY(roll, pitch, yaw);
    /*
    //"roll" = real world yaw
    //"pitch" = real world pitch
    //"yaw" = real world roll
    //setRPY -> R=pitch
    //setRPY -> P=roll
    //setRPY -> Y=yaw
    cout << "Roll: " << roll*180/M_PI << " Pitch: " << pitch*180/M_PI << " Yaw: " << yaw*180/M_PI << endl;
    this->imuGlobalRot = rotationMatrix;
    this->imuPitchRot.setRPY(0, pitch, 0);
    cout << this->imuPitchRot[0][0] << " " << this->imuPitchRot[0][1] << " " << this->imuPitchRot[0][2] << endl;
    cout << this->imuPitchRot[1][0] << " " << this->imuPitchRot[1][1] << " " << this->imuPitchRot[1][2] << endl;
    cout << this->imuPitchRot[2][0] << " " << this->imuPitchRot[2][1] << " " << this->imuPitchRot[2][2] << endl;//*/
	    q.setRPY(-pitch, roll, 0); //0 because leader defines "forward"

    //cout << "leader roll: " << (roll*180/M_PI) << " pitch: " << (pitch*180/M_PI) << endl;

        //AED added to deal with pose transforms
	    transform.setOrigin(tf::Vector3(0,0,0));	                     
		transform.setRotation(q);	 
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "king_world", "king_pose")); 

    	leader_pose.pose.orientation.x = q.getX();
	    leader_pose.pose.orientation.y = q.getY();
    	leader_pose.pose.orientation.z = q.getZ();
	    leader_pose.pose.orientation.w = q.getW();
        this->pubLeaderPose.publish(leader_pose);
	}//end else
}
void ParticleFilter::followerImuCallback(const sensor_msgs::Imu::ConstPtr& follower_imu_msg){
    if(!this->followerImuInitiation)
        this->followerImuInitiation = true;

    this->followerImuMsg = *follower_imu_msg;
}

void ParticleFilter::followerDotsCallback(const dot_finder::DuoDot::ConstPtr& follower_msg){
    if(!this->followerDotsInitiation)
        this->followerDotsInitiation = true;
    this->followerLastMsg = *follower_msg;


    if(this->isAllMessageInitiated()){
        this->runParticleFilter();
    }

    /*
    sensor_msgs::ImagePtr msg = this->visualization.generateROIVisualization(this->followerLastMsg.leftDistortDot,
                                                                             this->followerLastMsg.rightDistortDot,
                                                                             this->regionOfInterest[1].getCvRect(),
                                                                             this->lastFollowerImgRaw);
    this->pubROIFollower.publish(msg);//*/
}

void ParticleFilter::leaderDotsCallback(const dot_finder::DuoDot::ConstPtr& leader_msg){
    if(!this->leaderDotsInitiation)
        this->leaderDotsInitiation = true;
    this->leaderLastMsg = *leader_msg;

    if(this->isAllMessageInitiated()){
        this->runParticleFilter();
       // this->generateCSVLog();
    }
    /*
    sensor_msgs::ImagePtr msg = this->visualization.generateROIVisualization(this->leaderLastMsg.leftDistortDot,
                                                                             this->leaderLastMsg.rightDistortDot,
                                                                             this->regionOfInterest[0].getCvRect(),
                                                                             this->lastLeaderImgRaw);
    this->pubROILeader.publish(msg);//*/
}


bool ParticleFilter::isAllMessageInitiated(){
    return this->leaderDotsInitiation &&
           this->followerDotsInitiation &&
           this->leaderImuInitiation &&
           this->followerImuInitiation;
}

void ParticleFilter::runParticleFilter(){
    if(!this->haveCameraInfo){
        ROS_WARN_ONCE("No camera_info");
        return;
    }

    ListVector2d leaderLeftDot     = fromROSPoseArrayToVector2d(this->leaderLastMsg.leftDot);
    ListVector2d leaderRightDot    = fromROSPoseArrayToVector2d(this->leaderLastMsg.rightDot);
    ListVector2d followerLeftDot   = fromROSPoseArrayToVector2d(this->followerLastMsg.leftDot);
    ListVector2d followerRightDot  = fromROSPoseArrayToVector2d(this->followerLastMsg.rightDot);

    //this->regionOfInterest[0].filterCandidate(leaderLeftDot, leaderRightDot);
    //this->regionOfInterest[1].filterCandidate(followerLeftDot, followerRightDot);

    double weight;
    double best = -1;
    Eigen::Vector3d position, bestPosition;
    Eigen::Matrix3d rotation, bestRotation;
    visualization_msgs::MarkerArray candidatesMarkerMsgs;
    //geometry_msgs::PoseArray candidatesPoseMsgs;
    this->candidatesPoseMsgs.poses.clear();
    for(int i = 0; i < leaderLeftDot.size(); i++){
        for(int j = 0; j < followerLeftDot.size(); j++){
            weight = this->poseEvaluator.comparePoseABtoBA(leaderLeftDot[i], leaderRightDot[i],
                                                           followerLeftDot[j], followerRightDot[j],
                                                           position, rotation);
            //cout << i << " on " << j << " Weight: " << weight << endl << "Pose: "<< position.transpose() << endl;

            // Create for a rviz Marker for each combination of dots, with a tranparency factor of 0.3
            candidatesMarkerMsgs.markers.push_back(MutualPoseEstimation::generateMarkerMessage(position, rotation, 0.3));
            candidatesPoseMsgs.poses.push_back(MutualPoseEstimation::generatePoseMessage(position, rotation).pose);
            if((best < 0 && abs(weight) > 0.00001) || (abs(weight) < best && abs(weight) > 0.00001)){
                best = abs(weight);
                bestPosition = position;
                bestRotation = rotation;
            }
        }
    }
    if(best > 0){
        /*
        printf("=> Best: %6.4f \nPose: ", best);
        cout << bestPosition.transpose() << endl<<"Distance: "<< bestPosition.norm() << endl;
        cout << "Position:" << endl << bestPosition << endl;
        cout << "Rotation:" << endl << bestRotation << endl << endl;*/

        //PB fix the pitch error
        bestPosition = fromRelativePositionToGlobalPositionViaIMU(bestPosition);
        //cout << "Distance: "<< bestPosition.norm() << endl;
        
        //AED added to deal with pose transforms
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(-bestPosition[0],bestPosition[2],bestPosition[1]));
        Eigen::Quaterniond q2 = Eigen::Quaterniond(rotation);
        tf::Quaternion q(q2.x(), q2.y(), q2.z(), q2.w());
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "king_pose", "mamba_pose"));
        
    //AED added to adapt to new controller
        this->pubPredictedPose.publish(ParticleFilter::generatePredictedPoseMessage(bestPosition, bestRotation));
        this->pubPose.publish(MutualPoseEstimation::generatePoseMessage(bestPosition, bestRotation));
        this->pubMarker.publish(MutualPoseEstimation::generateMarkerMessage(bestPosition, bestRotation, 1.0));
        this->pubMarkerCandidates.publish(candidatesMarkerMsgs);
        candidatesPoseMsgs.header.frame_id = "king_pose";
        this->pubPoseCandidates.publish(candidatesPoseMsgs);
        
//AED added to view path in rviz
/*
        pathMsg.header.stamp = ros::Time::now();
        pathMsg.header.frame_id = "ardrone_base_frontcam";//ardrone_base_link

        geometry_msgs::PoseStamped tempMsg = MutualPoseEstimation::generatePoseMessage(bestPosition, bestRotation);
        tempMsg.header.stamp = ros::Time::now();
        tempMsg.header.seq = tempSeq;
        tempSeq++;
        pathMsg.poses.push_back(tempMsg);
        this->pubPath.publish(pathMsg);
*/
    }
}

Eigen::Vector3d ParticleFilter::fromRelativePositionToGlobalPositionViaIMU(const Eigen::Vector3d &position){
    // Imu pitch extraction
    tf::Quaternion imu_quat(this->leaderImuMsg.orientation.x,
                            this->leaderImuMsg.orientation.y,
                            this->leaderImuMsg.orientation.z,
                            this->leaderImuMsg.orientation.w);
    tf::Matrix3x3 imuGlobalRot(imu_quat);

    double roll, pitch, yaw;
    imuGlobalRot.getRPY(yaw, pitch, roll);


    tf::Matrix3x3 imuPitchRot;

    if(position_correction_with_imu_enable)
        // The matrix orientation (z being optical axis) is not the standard orientation for
        // pitch-roll-yaw convention. We thus found the proper mapping of pitch from IMU (z axis
        // vertical) to the correction matrix imuPitchRot. Visually, this made sense.
        // P. Babin + Giguere 16 feb 2014.
        imuPitchRot.setRPY(pitch, 0, 0);
    else
        imuPitchRot.setRPY(0, 0, 0);

    // Convert to tf:Matrix3x3 to Eigen
     Eigen::Matrix3d pitchRot;
     pitchRot << imuPitchRot[0][0], imuPitchRot[0][1], imuPitchRot[0][2],
                 imuPitchRot[1][0], imuPitchRot[1][1], imuPitchRot[1][2],
                 imuPitchRot[2][0], imuPitchRot[2][1], imuPitchRot[2][2];
     return pitchRot * position;
}

/*
geometry_msgs::PoseStamped ParticleFilter::generatePoseMessage(const Eigen::Vector3d &position, Eigen::Matrix3d rotation){

    // Imu pitch extraction
    tf::Quaternion imu_quat(this->leaderImuMsg.orientation.x,
                            this->leaderImuMsg.orientation.y,
                            this->leaderImuMsg.orientation.z,
                            this->leaderImuMsg.orientation.w);
    tf::Matrix3x3 imuGlobalRot(imu_quat);

    double roll, pitch, yaw;
    imuGlobalRot.getRPY(yaw, pitch, roll);


    tf::Matrix3x3 imuPitchRot;

    if(position_correction_with_imu_enable)
        // The matrix orientation (z being optical axis) is not the standard orientation for
        // pitch-roll-yaw convention. We thus found the proper mapping of pitch from IMU (z axis
        // vertical) to the correction matrix imuPitchRot. Visually, this made sense.
        // P. Babin + Giguere 16 feb 2014.
        imuPitchRot.setRPY(pitch, 0, 0);
    else
        imuPitchRot.setRPY(0, 0, 0);

    // Convert to tf:Matrix3x3 to Eigen
     Eigen::Matrix3d pitchRot;
     pitchRot << imuPitchRot[0][0], imuPitchRot[0][1], imuPitchRot[0][2],
                 imuPitchRot[1][0], imuPitchRot[1][1], imuPitchRot[1][2],
                 imuPitchRot[2][0], imuPitchRot[2][1], imuPitchRot[2][2];

     cout << "Pitch: " << pitch*180/M_PI << " Roll: " << roll*180/M_PI << " Yaw: " << yaw*180/M_PI << endl;
     cout << "Pitch Matrix rot:" << endl;
     cout << pitchRot << endl;
     cout << "CL    Matrix rot:" << endl;
     cout << rotation << endl;

     cout << "Position          :" << position.transpose() << endl;
     Eigen::Vector3d posCorrect = pitchRot * position;
     cout << "Position corrected:" << posCorrect.transpose() << endl;

    geometry_msgs::PoseStamped estimated_position;
    estimated_position.header.frame_id = "ardrone_base_frontcam";//ardrone_base_link

    estimated_position.pose.position.x = -posCorrect[0];// + 0.21;
    estimated_position.pose.position.y = posCorrect[2];
    estimated_position.pose.position.z = posCorrect[1];

    Eigen::Quaterniond q = Eigen::Quaterniond(rotation);

//AED change to the axes our controller expects
    tf::Quaternion q2(q.x(), q.y(), q.z(), q.w());
    tf::Matrix3x3 rotationMatrix(q2);

    rotationMatrix.getRPY(roll, pitch, yaw);
    q2.setRPY(-roll, yaw, -pitch); //0 because leader defines "forward"
    estimated_position.pose.orientation.x = q2.x();
    estimated_position.pose.orientation.y = q2.y();
    estimated_position.pose.orientation.z = q2.z();
    estimated_position.pose.orientation.w = q2.w();

    //this->pubPoseCorrect.publish(estimated_position);

    estimated_position.pose.position.x = -position[0];// + 0.21;
    estimated_position.pose.position.y = position[2];
    estimated_position.pose.position.z = position[1];


    return estimated_position;
}*/


//AED added to adapt to new controller
tum_ardrone::filter_state ParticleFilter::generatePredictedPoseMessage(const Eigen::Vector3d &position, Eigen::Matrix3d rotation){

    //message to be published
    tum_ardrone::filter_state msg;

    //metadata
    msg.header.stamp = ros::Time::now();
    msg.ptamState = 4; //"best" dummy value
    msg.scale = 1.0; //dummy value
    msg.scaleAccuracy = 0.5; //dummy value
    msg.droneState = lastNavdataReceived.state;
    msg.batteryPercent = lastNavdataReceived.batteryPercent;

    //pose: position
    msg.x = -position[0];
    msg.y = position[2];
    msg.z = position[1];

    //pose: orientation
    //TODO: convert to Eigen calculation? (I was more comfortable with tf::Quaternion)
    Eigen::Quaterniond q = Eigen::Quaterniond(rotation);
    tf::Quaternion q2(q.x(), q.y(), q.z(), q.w());
    tf::Matrix3x3 rotationMatrix(q2);

    double roll, pitch, yaw;
    //return value -> real-world value: roll -> pitch, pitch -> yaw, yaw -> roll
    rotationMatrix.getRPY(pitch, yaw, roll);
    //all are off by a rotation of ~pi radians
    roll += M_PI; //rotate
    if(roll > M_PI) roll -= 2*M_PI; //adjust into [-pi,pi] range
    pitch += M_PI;
    if(pitch > M_PI) pitch -= 2*M_PI;
    yaw += M_PI;
    if(yaw > M_PI) yaw -= 2*M_PI;

    //Convert from radians to -180 -> 180 degrees range
    msg.roll = (-roll)*180/M_PI; //currently pi or -pi
    msg.pitch = (-pitch)*180/M_PI; //currently ~0
    msg.yaw = (yaw)*180/M_PI; //currently pi or -pi

    //velocity values in "global" i.e. leader's frame of reference
    //by rotating velocity values by -yaw
    msg.dx = (std::sin(yaw)*lastNavdataReceived.vx + std::cos(yaw)*lastNavdataReceived.vy) / 1000.0;
    msg.dy = (std::cos(yaw)*lastNavdataReceived.vx - std::sin(yaw)*lastNavdataReceived.vy) / 1000.0;

//dz went from ~-1 to ~1 (not bounded)
//maybe (nav->altd - prev.nav->altd)*.001 = heightDiff
//      dz = heightDiff/timespan;
//      only update every 8 packets?
    if(initdZ){
      if((prevZnavdata != lastNavdataReceived.altd) || ((lastNavdataReceived.header.seq - prevZpackageID) > 8)){
        double zDiff = (lastNavdataReceived.altd - prevZnavdata)*0.001;
        timespan = (lastNavdataReceived.tm - prevNavTimeMicros)/1000000000;
        if(std::abs(zDiff) < 0.150){
          msg.dz =  zDiff / timespan;
        }
        prevZnavdata = lastNavdataReceived.altd;
      }
    }else{
      initdZ = true;
      prevZpackageID = lastNavdataReceived.header.seq;
      msg.dz = 0;
      prevZnavdata = lastNavdataReceived.altd;
    }

//dyaw goes from ~+90 to ~-90
//maybe (nav->rotZ - prev.nav->rotZ) = yawDiff
//      dyaw = yawDiff/timespan;
    if(initdYaw){
      double yawDiff = (lastNavdataReceived.rotZ - prevYawNavdata);
      timespan = (lastNavdataReceived.tm - prevNavTimeMicros)/1000000000;
      msg.dyaw = yawDiff/timespan;
      prevYawNavdata = lastNavdataReceived.rotZ;
    }else{
      initdYaw = true;
      msg.dyaw = 0;
      prevYawNavdata = lastNavdataReceived.rotZ;
      prevNavTimeMicros = lastNavdataReceived.tm;
    }

    if(msg.roll*msg.roll < 0.001) msg.roll = 0;
    if(msg.pitch*msg.pitch< 0.001) msg.pitch = 0;
    if(msg.yaw*msg.yaw < 0.001) msg.yaw = 0;
    if(msg.x*msg.x < 0.001) msg.x = 0;
    if(msg.y*msg.y < 0.001) msg.y = 0;
    if(msg.z*msg.z < 0.001) msg.z = 0;
    if(msg.dx*msg.dx < 0.001) msg.dx = 0;
    if(msg.dy*msg.dy < 0.001) msg.dy = 0;
    if(msg.dz*msg.dz < 0.001) msg.dz = 0;
    if(msg.dyaw*msg.dyaw < 0.001) msg.dyaw = 0;

    return msg;
}

ListVector2d ParticleFilter::fromROSPoseArrayToVector2d(vector<geometry_msgs::Pose2D> ros_msg){
    ListVector2d eigenVectorArray;
    for(int i = 0; i < ros_msg.size(); i++){
        eigenVectorArray.push_back(Eigen::Vector2d(ros_msg[i].x, ros_msg[i].y));
    }
    return eigenVectorArray;
}

/**
 * Visualization
 */
void ParticleFilter::visualizationCallbackLeader(const sensor_msgs::Image::ConstPtr& image_msg){
    if(!this->haveCameraInfo)
        return;

    this->lastLeaderImgRaw = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
}

void ParticleFilter::visualizationCallbackFollower(const sensor_msgs::Image::ConstPtr& image_msg){
    if(!this->haveCameraInfo)
        return;

    this->lastFollowerImgRaw = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
}




void ParticleFilter::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg){
    if (!this->haveCameraInfo){
        sensor_msgs::CameraInfo camInfo = *msg;

        // Calibrated camera
        Eigen::Matrix<double, 3, 4> cameraProjectionMatrix;
        cameraProjectionMatrix(0, 0) = camInfo.P[0];
        cameraProjectionMatrix(0, 2) = camInfo.P[2];
        cameraProjectionMatrix(1, 1) = camInfo.P[5];
        cameraProjectionMatrix(1, 2) = camInfo.P[6];
        cameraProjectionMatrix(2, 2) = 1.0;


        cv::Mat cameraMatrixK = cv::Mat(3, 3, CV_64F);
        cv::Mat cameraMatrixP = cv::Mat(3, 4, CV_64F);

        cameraMatrixK.at<double>(0, 0) = camInfo.K[0];
        cameraMatrixK.at<double>(0, 1) = camInfo.K[1];
        cameraMatrixK.at<double>(0, 2) = camInfo.K[2];
        cameraMatrixK.at<double>(1, 0) = camInfo.K[3];
        cameraMatrixK.at<double>(1, 1) = camInfo.K[4];
        cameraMatrixK.at<double>(1, 2) = camInfo.K[5];
        cameraMatrixK.at<double>(2, 0) = camInfo.K[6];
        cameraMatrixK.at<double>(2, 1) = camInfo.K[7];
        cameraMatrixK.at<double>(2, 2) = camInfo.K[8];
        std::vector<double> cameraDistortionCoeffs = camInfo.D;
        cameraMatrixP.at<double>(0, 0) = camInfo.P[0];
        cameraMatrixP.at<double>(0, 1) = camInfo.P[1];
        cameraMatrixP.at<double>(0, 2) = camInfo.P[2];
        cameraMatrixP.at<double>(0, 3) = camInfo.P[3];
        cameraMatrixP.at<double>(1, 0) = camInfo.P[4];
        cameraMatrixP.at<double>(1, 1) = camInfo.P[5];
        cameraMatrixP.at<double>(1, 2) = camInfo.P[6];
        cameraMatrixP.at<double>(1, 3) = camInfo.P[7];
        cameraMatrixP.at<double>(2, 0) = camInfo.P[8];
        cameraMatrixP.at<double>(2, 1) = camInfo.P[9];
        cameraMatrixP.at<double>(2, 2) = camInfo.P[10];
        cameraMatrixP.at<double>(2, 3) = camInfo.P[11];

        this->haveCameraInfo = true;
        ROS_INFO("Camera calibration information obtained.");

        Eigen::Vector2d focal, center;
        focal[0] = cameraMatrixK.at<double>(0, 0);
        focal[1] = cameraMatrixK.at<double>(1, 1);

        center[0] = cameraMatrixK.at<double>(0, 2);
        center[1] = cameraMatrixK.at<double>(1, 2);
        this->poseEvaluator.setCameraParameters(focal, center, camInfo.width, camInfo.height);

        this->visualization.setCameraParameter(cameraMatrixK,
                                               cameraMatrixP,
                                               cameraProjectionMatrix,
                                               cameraDistortionCoeffs);
    }
}

/**
 * The dynamic reconfigure callback function. This function updates the variable within the program whenever they are changed using dynamic reconfigure.
 */
void ParticleFilter::dynamicParametersCallback(particle_filter::ParticleFilterConfig &config, uint32_t level){
  this->poseEvaluator.setMarkersParameters(config.pos_right_led_cam_a,
                                           config.pos_left_led_cam_a,
                                           config.pos_right_led_cam_b,
                                           config.pos_left_led_cam_b);
  ROS_INFO("Parameters changed");
  
}

void ParticleFilter::generateCSVLog(){
    string filename_imu, filename_dot, filename_poses;
    filename_imu  = "_imu.csv";
    filename_dot  = "_dot.csv";
    filename_poses  = "pose_candidates.csv";
    const char *type[] = {"leader", "follower"};
    //double offset = 1406320000;

    ofstream myfile;

    // imu.csv
    sensor_msgs::Imu imu_log;
    for(int t = 0; t < 2; t++){
        myfile.open((string(type[t])+ filename_imu).c_str(), ios::app);

        if(t == 0)
            imu_log = this->leaderImuMsg;
        else
            imu_log = this->followerImuMsg;
        myfile << imu_log.header.stamp.toNSec() << ","
               << imu_log.orientation.x << ","
               << imu_log.orientation.y << ","
               << imu_log.orientation.z << ","
               << imu_log.orientation.w << ","
               << imu_log.angular_velocity.x << ","
               << imu_log.angular_velocity.y << ","
               << imu_log.angular_velocity.z << ","
               << imu_log.linear_acceleration.x << ","
               << imu_log.linear_acceleration.y << ","
               << imu_log.linear_acceleration.z << "\n";
        myfile.close();
    }

    // dot.csv
    dot_finder::DuoDot dot_log;
    for(int t = 0; t < 2; t++){
        myfile.open((string(type[t])+ filename_dot).c_str(), ios::app);
        if(t == 0)
            dot_log = this->leaderLastMsg;
        else
            dot_log = this->followerLastMsg;

        for(int i = 0;  i < dot_log.leftDot.size(); i++){
            myfile << dot_log.header.stamp.toNSec() << ","
                   << i << ","
                   << dot_log.leftDot.at(i).x << ","
                   << dot_log.leftDot.at(i).y << ","
                   << dot_log.rightDot.at(i).x << ","
                   << dot_log.rightDot.at(i).y << "\n";
        }
        myfile.close();
    }

    // Pose candidate
    ros::Time timeNow = ros::Time::now();
    myfile.open(filename_poses.c_str(), ios::app);
    for(int i = 0;  i < this->candidatesPoseMsgs.poses.size(); i++){
        myfile  << timeNow.toNSec() << ","
               << (i - i % this->followerLastMsg.leftDot.size())/ this->followerLastMsg.leftDot.size() << ","
               << i % this->followerLastMsg.leftDot.size() << ","
               << this->candidatesPoseMsgs.poses[i].orientation.x << ","
               << this->candidatesPoseMsgs.poses[i].orientation.y << ","
               << this->candidatesPoseMsgs.poses[i].orientation.z << ","
               << this->candidatesPoseMsgs.poses[i].orientation.w << ","
               << this->candidatesPoseMsgs.poses[i].position.x -0.21 << ","
               << this->candidatesPoseMsgs.poses[i].position.y << ","
               << this->candidatesPoseMsgs.poses[i].position.z << "\n";
        // z -x -y => x=z , y = -x, z = -y
    }

    myfile.close();

    ROS_INFO("All files saved!!!");

}


} // namespace particle_filter


int main(int argc, char* argv[]){

    
	ROS_INFO("Main start...\n");
	ros::init(argc, argv,  "particle_filter_node");

	ros::NodeHandle n;
	particle_filter::ParticleFilter particle_filter(n);

    ros::spin();
}

