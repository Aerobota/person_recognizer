B1;2802;0c# person_recognizer  
```git clone --recursive https://github.com/demulab/person_recognizer.git```  
## dependencies  
CUDA 7.5 runtime  
opencv  
caffe  
ROS  
sqlite3  
apsw(python sqlite library, get it from pip)  
## face_detector  
works awesome when using gpu and full hd camera mode.  
## ROS Msg  
###Face  
Header header  
uint8 number  
sensor_msgs/Image faces  
## gender_recognizer.py  
gender recognition with caffe.  
## TODO  
extract just face image using opencv ROI. : clear  
make person remembering and person classification  
make logging & database functionality  
make data extraction ablity  
