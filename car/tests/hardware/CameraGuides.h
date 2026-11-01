#ifndef UCR_CAMERA_GUIDES_H_
#define UCR_CAMERA_GUIDES_H_

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

inline std::vector<int> GuideRows(int height)
{
    if (height < 6) return {};

    std::vector<int> rows;
    for (int k = 1; k <= 3; ++k)
        rows.push_back(height - static_cast<int>((1LL * k * height) / 6));
    return rows;
}

inline void DrawGuides(cv::Mat& frame)
{
    if (frame.empty() || frame.type() != CV_8UC3 || frame.rows < 6) return;

    const cv::Scalar colors[] = {{0, 255, 255}, {0, 255, 0}, {0, 0, 255}};
    const char* labels[] = {"1/6", "2/6", "3/6"};
    const auto rows = GuideRows(frame.rows);
    for (int i = 0; i < 3; ++i)
    {
        cv::line(frame, cv::Point(0, rows[i]), cv::Point(frame.cols - 1, rows[i]), colors[i], 1);
        cv::putText(frame, labels[i], cv::Point(4, rows[i] - 4),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, colors[i], 1);
    }
}

#endif
