# person_recognizer  
```git clone --recursive https://github.com/demulab/person_recognizer.git```  
## dependencies  
CUDA 7.5 runtime  
opencv(GPU support build)  
caffe/pycaffe  
ROS  
sqlite3  
h5py  
openface  
## face_detector  
works awesome when using gpu and full hd camera mode.  
## ROS Msg  
###Face  
Header header  
uint8 number  
sensor_msgs/Image[] faces  
int32[] xangle  
int32[] yangle  
## gender_recognizer.py  
gender recognition with caffe.  
## person_recognizer_server.py  
person memorizing and matching with openface.  
## name_extractor.py  
name extraction from voice using kaldi and nltk.  
## person_recognizer_client.py  
person memorizing, matching and describing features of people.  
## TODO  
make logging  
