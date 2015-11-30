/*
 * visualization.h
 *
 *  Created on: Mar 21, 2014
 *      Author: Matthias Faessler
 */

/**
 * \file visualization.h
 * \brief File containing the includes and the function prototypes for the visualization.cpp file.
 *
 */

#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <sstream>

namespace particle_filter
{
class Visualization
{
public:
    void setCameraParameter(cv::Mat pCameraMatrixK,
                            cv::Mat pCameraMatrixP,
                            Eigen::Matrix<double, 3, 4> pCameraProjectionMatrix,
                            std::vector<double> pCameraDistortionCoeffs);
    sensor_msgs::ImagePtr generateROIVisualization(const std::vector<geometry_msgs::Pose2D> &left,
                                                             const std::vector<geometry_msgs::Pose2D> &right,
                                                             const cv::Rect ROI,
                                                             cv::Mat &img);

private:
    void drawDistortedMarker(const std::vector<geometry_msgs::Pose2D> &left,
                             const std::vector<geometry_msgs::Pose2D> &right,
                             cv::Mat &img);
    void drawRegionOfInterest(const cv::Rect ROI, cv::Mat &img);

    void distortPoints(const std::vector<cv::Point2f> & src, std::vector<cv::Point2f> & dst);
    std::vector<cv::Point2f> fromROSPoseArrayToCvPoint(std::vector<geometry_msgs::Pose2D> ros_msg);

    cv::Mat cameraMatrixK;
    cv::Mat cameraMatrixP;
    Eigen::Matrix<double, 3, 4> cameraProjectionMatrix;
    std::vector<double> cameraDistortionCoeffs;

};

} // namespace

#endif /* VISUALIZATION_H_ */
