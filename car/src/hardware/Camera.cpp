#include "hardware/Camera.h"

std::string TAG = "[My_Camera]";

Camera::~Camera()
{
    if (_cap.isOpened())
    {
        _cap.release();
        std::cout << TAG << "Destructor!\n";
    }
}

bool Camera::Init()
{
    if (!_cap.isOpened()) _cap.open(_pipeline, cv::CAP_GSTREAMER);
    if (!_cap.isOpened())
    {
        std::cout << TAG << " Camera open failed\n";
        return false;
    }
    return true;
}

void Camera::GetFrame(cv::Mat &frame)
{
    _cap >> frame;
}
