#ifndef CAMERA_H_
#define CAMERA_H_
#include <iostream>
#include <opencv2/opencv.hpp>

class Camera
{
private:
    std::string _pipeline;
    cv::VideoCapture _cap;

public:
    Camera(std::string pipeline = "nvarguscamerasrc sensor-mode=5 wbmode=0 ! "
                                  "video/x-raw(memory:NVMM),width=1080,height=720,framerate=60/1 ! "
                                  "nvvidconv ! "
                                  "video/x-raw,format=BGRx ! "
                                  "videoconvert ! "
                                  "video/x-raw,width=480,height=288,format=BGR! "
                                  "appsink drop=true max-buffers=1") : _pipeline(pipeline){}
    ~Camera();
    bool Init();
    void GetFrame(cv::Mat &frame);
};

#endif
