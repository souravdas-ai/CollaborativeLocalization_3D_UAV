/**
 * \file visualization.cpp
 * \brief File containing the function definitions for all visualization tasks
 *
 */

#include "particle_filter/visualization.h"
#include <stdio.h>
#include "ros/ros.h"

using namespace std;
namespace particle_filter
{


void Visualization::setCameraParameter(cv::Mat pCameraMatrixK,
                                       cv::Mat pCameraMatrixP,
                                       Eigen::Matrix<double, 3, 4> pCameraProjectionMatrix,
                                       std::vector<double> pCameraDistortionCoeffs){
    this->cameraMatrixK = pCameraMatrixK;
    this->cameraMatrixP = pCameraMatrixP;
    this->cameraProjectionMatrix = pCameraProjectionMatrix;
    this->cameraDistortionCoeffs = pCameraDistortionCoeffs;
}

sensor_msgs::ImagePtr Visualization::generateROIVisualization(const vector<geometry_msgs::Pose2D> &left,
                                                                        const vector<geometry_msgs::Pose2D> &right,
                                                                        const cv::Rect ROI,
                                                                        cv::Mat &img){
    this->drawDistortedMarker(left, right, img);
    this->drawRegionOfInterest(ROI, img);

    cv_bridge::CvImage visualized_image_msg;
    visualized_image_msg.encoding = sensor_msgs::image_encodings::BGR8;
    visualized_image_msg.image = img;

    return visualized_image_msg.toImageMsg();
}

void Visualization::drawDistortedMarker(const vector<geometry_msgs::Pose2D> &left,
                                        const vector<geometry_msgs::Pose2D> &right,
                                        cv::Mat &img){
    vector<cv::Point2f> leftDot, rightDot, leftDistortDot, rightDistortDot;
    leftDot  = fromROSPoseArrayToCvPoint(left);
    rightDot = fromROSPoseArrayToCvPoint(right);

    // We distort the dots
    for(int i = 0; i < leftDot.size(); i++){
        cv::circle(img, leftDot[i], 2, CV_RGB(173, 33, 96), 2);
        cv::circle(img, rightDot[i], 2, CV_RGB(173, 33, 96), 2);

        cv::line(img, leftDot[i], rightDot[i], CV_RGB(255, 0, 0), 2);
    }
}
void Visualization::drawRegionOfInterest(const cv::Rect ROI, cv::Mat &img){
    cv::rectangle(img, ROI, CV_RGB(0, 0, 255), 2);
}

std::vector<cv::Point2f> Visualization::fromROSPoseArrayToCvPoint(std::vector<geometry_msgs::Pose2D> ros_msg){
    vector<cv::Point2f> openCvVectorArray;
    for(int i = 0; i < ros_msg.size(); i++){
        openCvVectorArray.push_back(cv::Point2f(ros_msg[i].x, ros_msg[i].y));
    }
    return openCvVectorArray;
}



void Visualization::distortPoints(const std::vector<cv::Point2f> & src, std::vector<cv::Point2f> & dst){
    dst.clear();
    double fx_K = this->cameraMatrixK.at<double>(0, 0);
    double fy_K = this->cameraMatrixK.at<double>(1, 1);
    double cx_K = this->cameraMatrixK.at<double>(0, 2);
    double cy_K = this->cameraMatrixK.at<double>(1, 2);
    double fx_P = this->cameraMatrixP.at<double>(0, 0);
    double fy_P = this->cameraMatrixP.at<double>(1, 1);
    double cx_P = this->cameraMatrixP.at<double>(0, 2);
    double cy_P = this->cameraMatrixP.at<double>(1, 2);
    double k1 = this->cameraDistortionCoeffs[0];
    double k2 = this->cameraDistortionCoeffs[1];
    double p1 = this->cameraDistortionCoeffs[2];
    double p2 = this->cameraDistortionCoeffs[3];
    double k3 = this->cameraDistortionCoeffs[4];
    for (unsigned int i = 0; i < src.size(); i++)
    {
        // Project the points into the world
        const cv::Point2d &p = src[i];
        double x = (p.x - cx_P) / fx_P;
        double y = (p.y - cy_P) / fy_P;
        double xCorrected, yCorrected;
        // Correct distortion
        {
            double r2 = x * x + y * y;
            // Radial distortion
            xCorrected = x * (1. + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
            yCorrected = y * (1. + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
            // Tangential distortion
            xCorrected = xCorrected + (2. * p1 * x * y + p2 * (r2 + 2. * x * x));
            yCorrected = yCorrected + (p1 * (r2 + 2. * y * y) + 2. * p2 * x * y);
        }
        // Project coordinates onto image plane
        {
            xCorrected = xCorrected * fx_K + cx_K;
            yCorrected = yCorrected * fy_K + cy_K;
        }
        dst.push_back(cv::Point2f(xCorrected, yCorrected));
    }
}

}

