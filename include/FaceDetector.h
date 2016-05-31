#include <iostream>
#include <string>
#include <stdint.h>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "ComputationMode.h"

class FaceDetector {
 private:
  /** opencv properties */
  const std::string cascade_name;
  const std::string cascade_name_gpu;
  cv::CascadeClassifier face_cascade;
  cv::gpu::CascadeClassifier_GPU face_cascade_gpu;
  const std::string window_name;
  ComputationMode computation_mode; 

  /** image transport properties */
  image_transport::ImageTransport* it;
  image_transport::Subscriber image_sub;
  image_transport::Publisher image_pub;

  ros::Publisher faces_pub;
  ros::NodeHandle nh;

 public:  
  explicit FaceDetector(ros::NodeHandle& n);
  void detector(const sensor_msgs::ImageConstPtr& msg);
};
