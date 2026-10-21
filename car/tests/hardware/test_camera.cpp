#include "hardware/Camera.h"

#include <iostream>
#include <opencv2/highgui.hpp>

int main()
{
    Camera camera;
    if (!camera.Init())
    {
        std::cerr << "Camera init failed\n";
        return 1;
    }

    while (true)
    {
        cv::Mat frame;
        camera.GetFrame(frame);
        if (frame.empty())
        {
            std::cerr << "Empty camera frame\n";
            continue;
        }

        cv::imshow("Camera Test", frame);
        const int key = cv::waitKey(1);
        if (key == static_cast<int>('q') || key == 27)
        {
            break;
        }
    }

    cv::destroyAllWindows();
    return 0;
}
