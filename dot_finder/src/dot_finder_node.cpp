#include "dot_finder/dot_finder_node.h"

using namespace std;
using namespace cv;

namespace dot_finder
{
  DotFinder::DotFinder(ros::NodeHandle n):
  haveCameraInfo(false)
  {
    //set up publishers/subscribers
    string topic = "";
    //ros::param::get("~topic", topic);

    n.param<string>("topic", topic, "");
    n.param<int>("thresh_low", thresh_low, 70);
    n.param<int>("thresh_high", thresh_high, 255);
    n.param<int>("dilation", dilation_size, 20);
    n.param<int>("erosion", erosion_size, 10);

    ROS_INFO("===== Dot_finder node started =====");
    ROS_INFO_STREAM("Topic: " << topic);
    ROS_INFO_STREAM("Thresholding lower value: " << thresh_low);
    ROS_INFO_STREAM("Thresholding higher value: " << thresh_high);
    ROS_INFO_STREAM("Dilation: " << dilation_size);
    ROS_INFO_STREAM("Erosion: " << erosion_size);

    //Dynamic reconfiguration
    this->dynamicReconfigCallback = boost::bind(&DotFinder::dynamicParametersCallback, this, _1, _2);
    this->dynamicReconfigServer.setCallback(this->dynamicReconfigCallback);

    image_transport::ImageTransport image_transport(n);
    detection_image_pub = image_transport.advertise(topic + "/ardrone/image_with_detections", 1);
    dots_pub = n.advertise<dot_finder::DuoDot>(topic + "/dots", 1);
    image_sub = n.subscribe(topic + "/ardrone/image_raw", 1, &DotFinder::imageCallback, this);
    subCameraInfo = n.subscribe(topic + "/ardrone/camera_info", 1, &DotFinder::cameraInfoCallback, this);
  }

  void DotFinder::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& pMsg)
  {
    if (!this->haveCameraInfo)
    {
      this->camInfo = *pMsg;
   
      this->cameraMatrixK = cv::Mat(3, 3, CV_64F);
      this->cameraMatrixP = cv::Mat(3, 4, CV_64F);

      this->cameraMatrixK.at<double>(0, 0) = this->camInfo.K[0];
      this->cameraMatrixK.at<double>(0, 1) = this->camInfo.K[1];
      this->cameraMatrixK.at<double>(0, 2) = this->camInfo.K[2];
      this->cameraMatrixK.at<double>(1, 0) = this->camInfo.K[3];
      this->cameraMatrixK.at<double>(1, 1) = this->camInfo.K[4];
      this->cameraMatrixK.at<double>(1, 2) = this->camInfo.K[5];
      this->cameraMatrixK.at<double>(2, 0) = this->camInfo.K[6];
      this->cameraMatrixK.at<double>(2, 1) = this->camInfo.K[7];
      this->cameraMatrixK.at<double>(2, 2) = this->camInfo.K[8];
      cameraDistortionCoeffs = this->camInfo.D;
      cameraMatrixP.at<double>(0, 0) = this->camInfo.P[0];
      cameraMatrixP.at<double>(0, 1) = this->camInfo.P[1];
      cameraMatrixP.at<double>(0, 2) = this->camInfo.P[2];
      cameraMatrixP.at<double>(0, 3) = this->camInfo.P[3];
      cameraMatrixP.at<double>(1, 0) = this->camInfo.P[4];
      cameraMatrixP.at<double>(1, 1) = this->camInfo.P[5];
      cameraMatrixP.at<double>(1, 2) = this->camInfo.P[6];
      cameraMatrixP.at<double>(1, 3) = this->camInfo.P[7];
      cameraMatrixP.at<double>(2, 0) = this->camInfo.P[8];
      cameraMatrixP.at<double>(2, 1) = this->camInfo.P[9];
      cameraMatrixP.at<double>(2, 2) = this->camInfo.P[10];
      cameraMatrixP.at<double>(2, 3) = this->camInfo.P[11];

      this->haveCameraInfo = true;
      ROS_INFO("Camera calibration information obtained.");
    }

  }

  void DotFinder::imageCallback(const sensor_msgs::Image::ConstPtr& pImageMsg)
  {
    if(!this->haveCameraInfo)
    {
      ROS_WARN("No calibration information yet");
      return;
    }

    //convert from image message to cv image
    cv_bridge::CvImagePtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(pImageMsg, sensor_msgs::image_encodings::BGR16);// MONO
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //begin dot detection
    //int dilation_size = 20;
    //int erosion_size = 10;
    Mat gray, red, imdiff_red, output;
    Mat bw_red;
    Mat channelBGR[3];
    Mat element_d, element_e; 

    //split colorspace channels
    cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
    split(cv_ptr->image, channelBGR);

    //highlight red regions
    imdiff_red = channelBGR[2] - gray;

    //dilation and erosion
    element_d = getStructuringElement(MORPH_RECT, 
                                      Size(2*dilation_size + 1, 2*dilation_size + 1),
                                      Point(dilation_size, dilation_size));
    element_e = getStructuringElement(MORPH_RECT, 
                                      Size(2*erosion_size + 1, 2*erosion_size + 1), 
                                      Point(erosion_size, erosion_size));
    dilate(imdiff_red, imdiff_red, element_d);
    erode(imdiff_red, imdiff_red, element_e);

    //black & white version, where "detections" are white
    threshold(imdiff_red, bw_red, thresh_low, thresh_high, THRESH_BINARY);

    //remove blobs that are too big
    int i;
    vector<vector<Point> > contours;
    findContours(bw_red.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    for (i = 0; i < contours.size(); i++)
    {
      double area = contourArea(contours[i]);
    
      if (area >= 10000 || area <= 0)
        drawContours(bw_red, contours, i, CV_RGB(0,0,0), -1);
    }

    //find the remaining contours, and their centers
    findContours(bw_red.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    vector<vector<Point> > contours_poly( contours.size() );  
    vector<Point2f>center( contours.size() );
    vector<float>radius( contours.size() );
    for( i = 0; i < contours.size(); i++ )
    { 
      approxPolyDP( Mat(contours[i]), contours_poly[i], 2, true );
      minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
    }

    //convert to a color image to display a red line connecting centers
    cvtColor(bw_red, output, CV_GRAY2BGR);

    //ros message for detection image
    cv_bridge::CvImage detection_image_msg;
    detection_image_msg.header.stamp = ros::Time::now();
    detection_image_msg.encoding = sensor_msgs::image_encodings::BGR8;

    //if there were no contours, there are no dot pairs to publish
    if(center.size() < 2)
    {
      detection_image_msg.image = output;
      detection_image_pub.publish(detection_image_msg.toImageMsg());
      return;
    }

    //undo camera distortion on dots
    vector<Point2f> center_undistorted;
    cv::undistortPoints(center, center_undistorted, cameraMatrixK, cameraDistortionCoeffs, cv::noArray(), cameraMatrixP);

    //generate and publish dots message
    dot_finder::DuoDot msg;

    //find every pair of dots
    geometry_msgs::Pose2D position_in_image_left, position_in_image_right;
    geometry_msgs::Pose2D undistort_left, undistort_right;
    for(i = 0; i < center.size()-1; i++)
    {
      for(int j=i+1; j < center.size(); j++)
      {
        double diffX = center[j].x-center[i].x;
        double diffY = center[j].y-center[i].y;

        //make sure they are more horizontally aligned than vertically
        if(abs(diffX) > abs(diffY))
        {
          if(diffX > 0) //j is right dot
          {
            //distorted/raw image centers
            position_in_image_left.x = center[i].x;
            position_in_image_left.y = center[i].y;
            position_in_image_right.x = center[j].x;
            position_in_image_right.y = center[j].y;

            //undistorted centers
            undistort_left.x = center_undistorted[i].x;
            undistort_left.y = center_undistorted[i].y;
            undistort_right.x = center_undistorted[j].x;
            undistort_right.y = center_undistorted[j].y;
          }
          else //j would be left dot
          {
            //distorted/raw image centers
            position_in_image_left.x = center[j].x;
            position_in_image_left.y = center[j].y;
            position_in_image_right.x = center[i].x;
            position_in_image_right.y = center[i].y;

            //undistorted centers
            undistort_left.x = center_undistorted[j].x;
            undistort_left.y = center_undistorted[j].y;
            undistort_right.x = center_undistorted[i].x;
            undistort_right.y = center_undistorted[i].y;
          }

          //draw a line connecting the two dots
          line(output, center[i], center[j], Scalar(0,0,255), 3);

          //add the pair to the dots message
          msg.leftDot.push_back(undistort_left);
          msg.rightDot.push_back(undistort_right);
          msg.leftDistortDot.push_back(position_in_image_left);
          msg.rightDistortDot.push_back(position_in_image_right);
        }
      }
    }

    //publish our image of detections
    detection_image_msg.image = output;
    detection_image_pub.publish(detection_image_msg.toImageMsg());

    //publish our dots message
    msg.header.stamp = ros::Time::now();
    dots_pub.publish(msg);
  }

  /**
   * The dynamic reconfigure callback function. This function updates the variable within 
   * the program whenever they are changed using dynamic reconfigure.
   */
  void DotFinder::dynamicParametersCallback(dot_finder::DotFinderConfig &config, uint32_t level)
  {
    thresh_low = config.thresh_low;
    dilation_size = config.dilatation_size;
    thresh_high = config.thresh_high;
    erosion_size = config.erosion_size;

    ROS_INFO("Parameters changed");
  }

}//namespace dot_finder

int main(int argc, char** argv)
{
  //set up node
  ros::init(argc, argv, "dot_finder");

  ros::NodeHandle n("~");
  dot_finder::DotFinder dot_finder(n);

  //let ros run
  ros::spin();

  return 0;
}
