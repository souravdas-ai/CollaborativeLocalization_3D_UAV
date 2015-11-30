#ifndef DOT_FINDER_NODE_H_
#define DOT_FINDER_NODE_H_

#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <dot_finder/DuoDot.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

#include <dot_finder/DotFinderConfig.h>
#include <dynamic_reconfigure/server.h>

namespace dot_finder
{
  class DotFinder
  {
    public:
      DotFinder(ros::NodeHandle n);
      void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& pMsg);
      void imageCallback(const sensor_msgs::Image::ConstPtr& pImageMsg);
      void dynamicParametersCallback(dot_finder::DotFinderConfig &config, uint32_t level);

    private:
      ros::NodeHandle nodeHandler;
      ros::Subscriber image_sub; //subscribes to camera image
      ros::Subscriber subCameraInfo; //gets camera calibration
      ros::Publisher dots_pub; //publishes detected dots
      image_transport::Publisher detection_image_pub; //outputs image with detections for humans

      bool haveCameraInfo;
      sensor_msgs::CameraInfo camInfo;
      cv::Mat cameraMatrixK, cameraMatrixP;
      std::vector<double> cameraDistortionCoeffs;

      int thresh_low;
      int thresh_high;// = 255;
      int dilation_size;
      int erosion_size;

      dynamic_reconfigure::Server<dot_finder::DotFinderConfig> dynamicReconfigServer;
      dynamic_reconfigure::Server<dot_finder::DotFinderConfig>::CallbackType dynamicReconfigCallback;

  };//class DotfinderNode

} //namespace dot_finder


#endif /* DOTFINDER_NODE_H_ */

