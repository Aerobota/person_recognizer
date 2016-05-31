#include "FaceDetector.cpp"

int main(int argc, char** argv){
  ros::init(argc, argv, "face_detector");
  
  ros::NodeHandle nh("~");
  FaceDetector* facedetector = new FaceDetector(nh);
  ros::spin();
}
