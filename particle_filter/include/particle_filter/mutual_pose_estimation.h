#ifndef POSE_ESTIMATION_H
#define POSE_ESTIMATION_H

#include <iostream>
#include <fstream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace particle_filter
{
class MutualPoseEstimation
{
public:
    void setMarkersParameters(const double distanceRightLedRobotA, const double distanceLeftLedRobotA,
                             const double distanceRightLedRobotB, const double distanceLeftLedRobotB);
    void setCameraParameters(const Eigen::Vector2d pFocalCam, const Eigen::Vector2d pCenterCam, int pWidth, int pHeight);

    double comparePoseABtoBA(const Eigen::Vector2d &pixelA1, const Eigen::Vector2d &pixelA2,
                             const Eigen::Vector2d &pixelB1, const Eigen::Vector2d &pixelB2,
                             Eigen::Vector3d &positionAB,  Eigen::Matrix3d &rotationAB);

    static visualization_msgs::Marker generateMarkerMessage(const Eigen::Vector3d &position, Eigen::Matrix3d rotation, const double alpha);
    static geometry_msgs::PoseStamped generatePoseMessage(const Eigen::Vector3d &position, Eigen::Matrix3d rotation);
    void compute3DMutualLocalisation(const Eigen::Vector2d &pixelA1, const Eigen::Vector2d &pixelA2,
                                     const Eigen::Vector2d &pixelB1,const  Eigen::Vector2d &pixelB2,
                                     Eigen::Vector3d & position, Eigen::Matrix3d & rotation);

private:
    Eigen::Vector2d fromXYCoordinateToUVCoordinate(const Eigen::Vector2d &vXY);
    Eigen::Vector2d computePositionMutual(double alpha, double beta, double d);
    Eigen::Matrix3d vrrotvec2mat(double p, Eigen::Vector3d r);


    Eigen::Vector2d focalCam, centerCam;
    double rdA, ldA, rdB, ldB;
    int width, height;

//AED changed to static to compile
    static const bool ConventionUV = false;
};
}
#endif // POSE_ESTIMATION_H
