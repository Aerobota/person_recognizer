#include "FaceDetector.cpp"
#include <nodelet/nodelet.h>
#include <boost/thread.hpp>

#include <pluginlib/class_list_macros.h>

namespace facedetector{
  class FaceDetectorNodelet : public nodelet::Nodelet{
  private:
    bool is_running;
    bool is_init;
    boost::shared_ptr<boost::thread> thread;
    boost::shared_ptr<FaceDetector> face_detector;

    virtual void onInit(){
      thread = boost::shared_ptr<boost::thread>
	(new boost::thread(boost::bind(&FaceDetectorNodelet::main, this)));
    }

    void main() {
      while(is_running){
	if(is_init){
	  boost::shared_ptr<FaceDetector> 
	    facedetector_ptr(
			     new FaceDetector(getPrivateNodeHandle())
			     );
	  is_init = false;
	}	
      }
    }
    
  public:
    explicit FaceDetectorNodelet() : 
      is_running(false),
      is_init(true) {
    }
    
    ~FaceDetectorNodelet(){
      if (is_running) {
	is_running = false;
	thread->join();
      }
    }
  };
}

PLUGINLIB_DECLARE_CLASS(facedetector,FaceDetectorNodelet,facedetector::FaceDetectorNodelet,nodelet::Nodelet);
