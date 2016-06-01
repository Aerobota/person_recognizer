#include "FaceDetector.h"

FaceDetector::FaceDetector(ros::NodeHandle& n) :
  nh(n),
  cascade_name("/home/kendemu/catkin_ws/src/person_recognizer/cfg/haarcascade_frontalface_alt.xml"),
  cascade_name_gpu("/home/kendemu/catkin_ws/src/person_recognizer/cfg/gpu/haarcascade_frontalface_alt.xml"),
  window_name("Face Detection"){
  
  /** image transport and topic subscribe/publish */
  it = new image_transport::ImageTransport(nh);
  image_sub = it->subscribe("/kodak/image_raw", 1, 
			    &FaceDetector::detector, this);
  face_pub = nh.advertise<person_recognizer::Face>("/faces", 1);

  /** Detect GPU */
  if(cv::gpu::getCudaEnabledDeviceCount() == 0){
    ROS_ERROR("No GPU Support. falling back to cpu mode");
    computation_mode = CPU;
  } else {
    ROS_INFO("GPU Loading");
    computation_mode = GPU;
  }  

  /** load cascade files */
  ROS_INFO("Loading cascade file. Takes about 10 seconds......");

  if(face_cascade_gpu.load(cascade_name_gpu) ) {
    ROS_INFO("cascade file loading finished");
  } else {
    ROS_ERROR("cascade file not found");
    abort();
  }

};

void FaceDetector::detector(const sensor_msgs::ImageConstPtr& msg){

  /** convert to CV image format */
  cv_bridge::CvImagePtr image;

  try {
    image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception : %s", e.what() );
    return;
  }
  
  /** get grayscale images */
  cv::gpu::GpuMat faces;
  int faces_number;
  cv::Mat gray_img;
  
  cv::cvtColor(image->image, gray_img, CV_BGR2GRAY );
  cv::equalizeHist(gray_img, gray_img );

  /** copy gray image to GPU Memory */
  cv::gpu::GpuMat gray_img_gpu(gray_img);

  /** do haarcascade with gpu */
  faces_number = face_cascade_gpu.detectMultiScale ( gray_img_gpu,
						     faces,
						     1.2,
						     4,
						     cv::Size(20, 20) 
						     );

  /** transfer to host memory */
  cv::Mat obj_host;
  faces
    .colRange(0,faces_number)
    .download(obj_host);

  /** take Region of Interest of face image */
  cv::Rect* cfaces = obj_host.ptr<cv::Rect>();
  std::vector<cv::Mat> faces_roi;
  cv::Mat camera_img = image->image;

  /** make Face message */
  person_recognizer::Face face;
  
  /** store faces number */
  face.number = (uint8_t)faces_number;
  
  /** do image convertion and stack face images
      CvImagePtr -> 
      Image::ConstPtr -> 
      boost::shared_ptr<Image*>() -> 
      Image*
  */
  for( int i = 0; i < faces_number; ++i ){
    faces_roi.push_back(camera_img(cfaces[i]));
    image->image = faces_roi[i];
    sensor_msgs::Image* img_msg = image->toImageMsg().get();
    face.faces.push_back(*img_msg);
    //image_pub.publish(image->toImageMsg() );
  }

  /** publish face topic */
  face_pub.publish(face);

}
