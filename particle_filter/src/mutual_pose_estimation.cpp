#include "particle_filter/mutual_pose_estimation.h"



using namespace std;

namespace particle_filter
{

void MutualPoseEstimation::setMarkersParameters(const double distanceRightLedRobotA, const double distanceLeftLedRobotA,
                                 const double distanceRightLedRobotB, const double distanceLeftLedRobotB){
    this->rdA = distanceRightLedRobotA; this->ldA = distanceLeftLedRobotA;
    this->rdB = distanceRightLedRobotB; this->ldB = distanceLeftLedRobotB;
}

void MutualPoseEstimation::setCameraParameters(const Eigen::Vector2d pFocalCam, const Eigen::Vector2d pCenterCam, int pWidth, int pHeight){
    this->focalCam = pFocalCam;
    this->centerCam = pCenterCam;

//    Eigen::Matrix2d switcheXandY;
//    switcheXandY(0,0) = 0; switcheXandY(0,1) = 1;
//    switcheXandY(1,0) = 1; switcheXandY(1,1) = 0;
//    this->focalCam = switcheXandY * pFocalCam;
//    this->centerCam = switcheXandY * pCenterCam;

    this->width = pWidth;
    this->height = pHeight;
}


visualization_msgs::Marker MutualPoseEstimation::generateMarkerMessage(const Eigen::Vector3d &position, Eigen::Matrix3d rotation, const double alpha){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "king_pose";
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = alpha;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    //marker.mesh_resource = "package://particle_filter/mesh/quadrotor_3.stl";
    marker.mesh_resource = "package://particle_filter/mesh/quadrotor_2_move_cam.stl";


    // The camera is 21cm in front of the center of the drone
    marker.pose.position.x = position[0] + 0.21;
    marker.pose.position.y = position[1];
    marker.pose.position.z = position[2];

    /*rotation = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())
               * rotation;*/

    Eigen::Quaterniond q = Eigen::Quaterniond(rotation);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    return marker;
}

geometry_msgs::PoseStamped MutualPoseEstimation::generatePoseMessage(const Eigen::Vector3d &position, Eigen::Matrix3d rotation){

    geometry_msgs::PoseStamped estimated_position;
    estimated_position.header.frame_id = "king_pose";

//AED change to the axes our controller expects
    //estimated_position.pose.position.x = position[0];// + 0.21;
    //estimated_position.pose.position.y = position[1];
    //estimated_position.pose.position.z = position[2];
    estimated_position.pose.position.x = -position[0];// + 0.21;
    estimated_position.pose.position.y = position[2];
    estimated_position.pose.position.z = position[1];

    /*rotation = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())
               * rotation;*/

    Eigen::Quaterniond q = Eigen::Quaterniond(rotation);

//AED change to the axes our controller expects
    //estimated_position.pose.orientation.x = q.x();
    //estimated_position.pose.orientation.y = q.y();
    //estimated_position.pose.orientation.z = q.z();
    //estimated_position.pose.orientation.w = q.w();
    tf::Quaternion q2(q.x(), q.y(), q.z(), q.w());
    tf::Matrix3x3 rotationMatrix(q2);
    double roll, pitch, yaw;
    rotationMatrix.getRPY(pitch, yaw, roll);
    q2.setRPY(-pitch, roll, -yaw); //0 because leader defines "forward"
    estimated_position.pose.orientation.x = q2.x();
    estimated_position.pose.orientation.y = q2.y();
    estimated_position.pose.orientation.z = q2.z();
    estimated_position.pose.orientation.w = q2.w();

    return estimated_position;
}


double MutualPoseEstimation::comparePoseABtoBA(const Eigen::Vector2d &pixelA1, const Eigen::Vector2d &pixelA2,
                                               const Eigen::Vector2d &pixelB1, const Eigen::Vector2d &pixelB2,
                                               Eigen::Vector3d &positionAB,  Eigen::Matrix3d &rotationAB){

    Eigen::Matrix3d rotationBA;
    Eigen::Vector3d positionBA;

    // We inverse the x and y, 'cause of the image on the cube
    Eigen::Vector2d lPixelA1;
    Eigen::Vector2d lPixelA2;
    Eigen::Vector2d lPixelB1;
    Eigen::Vector2d lPixelB2;
    Eigen::Matrix2d switcheXandY;
//    switcheXandY(0,0) = 0; switcheXandY(0,1) = 1;
//    switcheXandY(1,0) = 1; switcheXandY(1,1) = 0;
//    lPixelA1 = switcheXandY * pixelA1;
//    lPixelA2 = switcheXandY * pixelA2;
//    lPixelB1 = switcheXandY * pixelB1;
//    lPixelB2 = switcheXandY * pixelB2;

    lPixelA1 = pixelA1;
    lPixelA2 = pixelA2;
    lPixelB1 = pixelB1;
    lPixelB2 = pixelB2;

/*
    cout<<"pixelA1:"<<pixelA1<<endl;
    cout<<"pixelA2:"<<pixelA2<<endl;
    cout<<"pixelB1:"<<pixelB1<<endl;
    cout<<"pixelB2:"<<pixelB2<<endl;*/


    this->compute3DMutualLocalisation(lPixelA1, lPixelA2,
                                      lPixelB1, lPixelB2,
                                      positionAB, rotationAB);


    this->compute3DMutualLocalisation(lPixelB1, lPixelB2,
                                      lPixelA1, lPixelA2,
                                      positionBA, rotationBA);

    //this->centerCam = notDOTHAT;

    double distanceError = positionAB.norm() - positionBA.norm();
//AED attempt to filter based on distance discrepancy
/*
    double distanceError;
    distanceError = positionAB.norm() - positionBA.norm();
    if(abs(distanceError) > 1.0)
    	distanceError = 0;
    
    if(positionAB.norm() > positionBA.norm())
    {
    	if(positionAB.norm() < 2*positionBA.norm())
     		distanceError = positionAB.norm() - positionBA.norm();
    	else
    		distanceError = 0;
    }
    else
    {
    	if(positionBA.norm() < 2*positionAB.norm())
     		distanceError = positionAB.norm() - positionBA.norm();
    	else
    		distanceError = 0;
    }
    
    cout << "PositionAB: " << positionAB.norm() << endl;
    cout << "PositionBA: " << positionBA.norm() << endl;
    cout << "Distance Error: " << distanceError << endl;
*/

    return distanceError;
}

Eigen::Vector2d MutualPoseEstimation::fromXYCoordinateToUVCoordinate(const Eigen::Vector2d &vXY){
    Eigen::Vector2d vUV;
    vUV[0] = this->width - vXY[0];
    vUV[1] = this->height - vXY[1];
    return vUV;
}

/**
   * Compute the 3D pose in 6DOF using to camera for mutual localization
   *
   * \param pixelA1 Position of the left LED on robot A
   * \param pixelA2 Position of the right LED on robot A
   * \param pixelB1 Position of the left LED on robot B
   * \param pixelB2 Position of the right LED on robot
   * \param position (Output) the position vector
   * \param rotation (Output) the rotation matrix
   *
   * \return the rotation matrix of the relative pose
   *
   */
void MutualPoseEstimation::compute3DMutualLocalisation(const Eigen::Vector2d &lPixelA1, const Eigen::Vector2d &lPixelA2,
                                                       const Eigen::Vector2d &lPixelB1,const  Eigen::Vector2d &lPixelB2,
                                                       Eigen::Vector3d &position, Eigen::Matrix3d &rotation){

  Eigen::Vector2d fCamA = this->focalCam;
  Eigen::Vector2d fCamB = this->focalCam;
  Eigen::Vector2d ppA, pixelA1, pixelA2, pixelB1, pixelB2;
  if(ConventionUV){ // Oliver's code use the uv convention ( u point left v point up)
    ppA = this->centerCam;
    pixelA1 = lPixelA1;
    pixelA2 = lPixelA2;
    pixelB1 = lPixelB1;
    pixelB2 = lPixelB2;
  }
  else{
    ppA = fromXYCoordinateToUVCoordinate(this->centerCam);
    pixelA1 = fromXYCoordinateToUVCoordinate(lPixelA1);
    pixelA2 = fromXYCoordinateToUVCoordinate(lPixelA2);
    pixelB1 = fromXYCoordinateToUVCoordinate(lPixelB1);
    pixelB2 = fromXYCoordinateToUVCoordinate(lPixelB2);
  }
    Eigen::Vector2d ppB = ppA;
    //Eigen::Vector2d ppA = 0;
    //Eigen::Vector2d ppB = 0;

  /*cout<<"-Parameters-"<<endl;
  cout<<"pixelA1:"<<pixelA1<<endl;
  cout<<"pixelA2:"<<pixelA2<<endl;
  cout<<"pixelB1:"<<pixelB1<<endl;
  cout<<"pixelB2:"<<pixelB2<<endl;
  cout<<"ppA:"<<ppA<<endl;
  cout<<"ppB:"<<ppB<<endl;
  cout<<"fCamA:"<<fCamA<<endl;
  cout<<"fCamB:"<<fCamB<<endl;
  cout<<"rdA:"<<rdA<<endl;
  cout<<"ldA:"<<ldA<<endl;
  cout<<"rdB:"<<rdB<<endl;
  cout<<"ldB:"<<ldB<<endl;*/

  //Eigen::Vector3d PAM1, PAM2;
  //if (ConventionUV) {
    Eigen::Vector3d PAM1((pixelB1[0]-ppB[0])/fCamB[0], (pixelB1[1]-ppB[1])/fCamB[1], 1);
    Eigen::Vector3d PAM2((pixelB2[0]-ppB[0])/fCamB[0], (pixelB2[1]-ppB[1])/fCamB[1], 1);
//  }
//  else {
//    // Convention x-y
//    PAM1((ppB[0]-pixelB1[0])/fCamB[0], (ppB[1]-pixelB1[1])/fCamB[1], 1);
//    PAM2((ppB[0]-pixelB2[0])/fCamB[0], (ppB[1]-pixelB2[1])/fCamB[1], 1);
//  }

  PAM1.normalize();
  PAM2.normalize();
  double alpha = acos(PAM1.dot(PAM2));
  //printf("Alpha: %f\n",alpha);

  double d = this->rdA + this->ldA;

  Eigen::Vector2d BLeftMarker = pixelA2;
  Eigen::Vector2d BRightMarker = pixelA1;
  
  Eigen::Vector2d PB1(BLeftMarker[0] + (this->ldB/(rdB+ldB)) * (BRightMarker[0] - BLeftMarker[0]),
                      BLeftMarker[1] + (this->ldB/(rdB+ldB)) * (BRightMarker[1] - BLeftMarker[1]));

  Eigen::Vector3d PB12((PB1[0]-ppA[0])/fCamA[0], (PB1[1]-ppA[1])/fCamA[1], 1);
  PB12.normalize();
  double phi = acos(PB12[0]);
  double beta = 0.5f * M_PI - phi;
  //printf("Beta: %f\n",beta* 180.f/M_PI);

  Eigen::Vector2d plane = MutualPoseEstimation::computePositionMutual(alpha, beta, d);

  double EstimatedDistance = plane.norm();

  position =  PB12 * EstimatedDistance;
    //====================================================================
    //=========================Axis Angle Part============================
    //Create the two plans
    //Plan in B Refs
  Eigen::Vector2d ALeftMarker = pixelB2;
  Eigen::Vector2d ARightMarker = pixelB1;

  Eigen::Vector3d ALM((ALeftMarker[0]-ppB[0])/fCamB[0], (ALeftMarker[1]-ppB[1])/fCamB[1], 1);
  ALM.normalize();

  Eigen::Vector3d ARM((ARightMarker[0]-ppB[0])/fCamB[0], (ARightMarker[1]-ppB[1])/fCamB[1], 1);
  ARM.normalize();
  //Plan in A Refs
  Eigen::Vector3d AToB = PB12;

  Eigen::Vector3d LeftMarker(1, 0, 0);

  //Align the two plans
  Eigen::Vector3d NormalPlanInB = ALM.cross(ARM);
  Eigen::Vector3d NormalPlanInA = AToB.cross(LeftMarker);
  Eigen::Vector3d AxisAlignPlans = NormalPlanInB.cross(NormalPlanInA);
  NormalPlanInB.normalize();
  NormalPlanInA.normalize();
  AxisAlignPlans.normalize();
  double AngleAlignPlans = acos(NormalPlanInB.dot(NormalPlanInA));

  Eigen::MatrixXd AlignPlans = vrrotvec2mat(AngleAlignPlans, AxisAlignPlans);

  //Align the vector of the cameraA seen from B with the plan
  Eigen::Vector3d CameraASeenFromB(
                      ((ALeftMarker[0] + (this->ldA/(this->rdA+this->ldA))*(ARightMarker[0] - ALeftMarker[0]))-ppB[0])/fCamB[0],
                      ((ALeftMarker[1] + (this->ldA/(this->rdA+this->ldA))*(ARightMarker[1] - ALeftMarker[1]))-ppB[1])/fCamB[1],
                      1);
  CameraASeenFromB.normalize();
  Eigen::Vector3d alignedBToA = AlignPlans * CameraASeenFromB;
  //Turn the vector BToA to make it align with AToB
  Eigen::Vector3d AxisAlignABwBA = alignedBToA.cross(AToB);
  AxisAlignABwBA.normalize();
  //Since we know that cameras are facing each other, we rotate pi
  double AngleAlignABwBA = acos(alignedBToA.dot(AToB)) - M_PI;
  
  Eigen::Matrix3d AlignVectors = vrrotvec2mat(AngleAlignABwBA, AxisAlignABwBA);

  rotation =  AlignVectors * AlignPlans;

}

Eigen::Vector2d MutualPoseEstimation::computePositionMutual(double alpha, double beta, double d){
  double r = 0.5*d/sin(alpha);
  
  // Position of the center
  double Cy = 0.5*d/tan(alpha);

  // From http://mathworld.wolfram.com/Circle-LineIntersection.html
  // Intersection of a line and of a circle, with new beta
  double OtherAngle = 0.5 * M_PI - beta;
  double x1 = 0; 
  double x2 = 50 * cos(OtherAngle);
  double y1 = -Cy; 
  double y2 = 50 * sin(OtherAngle) - Cy;

  double dx = x2-x1;
  double dy = y2-y1;

  double dr = sqrt(dx*dx + dy*dy);
  double D  = x1*y2 - x2*y1;
    
  double X = (D*dy+abs(dy)/dy*dx*sqrt(r*r*dr*dr-D*D))/dr/dr;
  double Y = (-D*dx + abs(dy)*sqrt(r*r*dr*dr-D*D))/dr/dr;
    
  Y = Y + Cy;

  Eigen::Vector2d p(X, Y);
  return  p;
}

Eigen::Matrix3d MutualPoseEstimation::vrrotvec2mat(double p, Eigen::Vector3d r){
  double s = sin(p);
  double c = cos(p);
  double t = 1 - c;
  r.normalize();
  Eigen::Vector3d n = r;

  double x = n[0];
  double y = n[1];
  double z = n[2];
  Eigen::Matrix3d m(3,3);
  m(0,0) = t*x*x + c; m(0,1) = t*x*y - s*z; m(0,2) = t*x*z + s*y;
  m(1,0) = t*x*y + s*z; m(1,1) = t*y*y + c; m(1,2) = t*y*z - s*x;
  m(2,0) = t*x*z - s*y; m(2,1) = t*y*z + s*x; m(2,2) = t*z*z + c;
  return m;
}





} // namespace particle_filter

