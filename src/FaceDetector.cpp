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
  image_pub = it->advertise("/faces", 1);
  faces_pub = nh.advertise<std_msgs::String>("/faces_number",1000);

  if(cv::gpu::getCudaEnabledDeviceCount() == 0){
    ROS_ERROR("No GPU Support. falling back to cpu mode");
    computation_mode = CPU;
  } else {
    ROS_INFO("GPU Loading");
    computation_mode = GPU;
  }  

  ROS_INFO("Loading cascade file. Takes about 10 seconds......");

  /** load cascade files */
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
  //std::vector<cv::Rect> faces;
  cv::gpu::GpuMat faces;
  int faces_number;
  cv::Mat gray_img;
  
  cv::cvtColor(image->image, gray_img, CV_BGR2GRAY );
  cv::equalizeHist(gray_img, gray_img );
  /** GPU Memory of gray image */
  cv::gpu::GpuMat gray_img_gpu(gray_img);

  /** do haarcascade cpu */
  /*
  face_cascade.detectMultiScale ( gray_img,
				  faces,
				  1.1,
				  2,
				  0 | CV_HAAR_SCALE_IMAGE,
				  cv::Size(30, 30) 
				  );
  */
  faces_number = face_cascade_gpu.detectMultiScale ( gray_img_gpu,
						     faces,
						     1.2,
						     4,
						     cv::Size(20, 20) 
						     );
  cv::Mat obj_host;
  faces
    .colRange(0,faces_number)
    .download(obj_host);

  cv::Rect* cfaces = obj_host.ptr<cv::Rect>();
  
  /** draw faces gpu */
  for( int i = 0; i < faces_number; ++i ){
    cv::Point pt1 = cfaces[i].tl();
    cv::Size sz = cfaces[i].size();
    cv::Point pt2(pt1.x + sz.width,
		  pt1.y + sz.height );
    cv::rectangle(image->image, pt1, pt2, cv::Scalar(255));
  }

  /** draw faces cpu */
  /*
  for( size_t i = 0; i < faces.size(); i++ ){
    cv::Point center( faces[i].x + faces[i].width * 0.5,
		      faces[i].y + faces[i].height * 0.5
		      );
    
    cv::ellipse( image->image,
		 center,
		 cv::Size( faces[i].width * 0.5,
			   faces[i].height * 0.5
			   ),
		 0,0,360,
		 cv::Scalar( 255,0,255),
		 4,8,0
		 );
  }
  */
  /** publish face images */
  image_pub.publish(image->toImageMsg() );
  std_msgs::String f_msg;
  f_msg.data = std::to_string(faces_number);
  faces_pub.publish(f_msg);
}
