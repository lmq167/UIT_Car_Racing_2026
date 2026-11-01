#include "hardware/CameraGuides.h"

#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
void Check(bool ok, const std::string& message)
{
    if (!ok) throw std::runtime_error(message);
}

void CheckPixel(const cv::Mat& frame, int y, int x, const cv::Vec3b& expected)
{
    Check(frame.at<cv::Vec3b>(y, x) == expected,
          "Sai màu tại (x=" + std::to_string(x) + ", y=" + std::to_string(y) + ")");
}
}

int main()
{
    const char* current = "T1: GuideRows";
    try
    {
        Check(GuideRows(720) == std::vector<int>({600, 480, 360}), "Chiều cao 720");
        Check(GuideRows(288) == std::vector<int>({240, 192, 144}), "Chiều cao 288");
        Check(GuideRows(384) == std::vector<int>({320, 256, 192}), "Chiều cao 384");
        Check(GuideRows(6) == std::vector<int>({5, 4, 3}), "Chiều cao 6");
        Check(GuideRows(7) == std::vector<int>({6, 5, 4}), "Chia nguyên khi cao 7");
        Check(GuideRows(11) == std::vector<int>({10, 8, 6}), "Nhân trước chia khi cao 11");
        Check(GuideRows(std::numeric_limits<int>::max()) ==
                  std::vector<int>({1789569706, 1431655765, 1073741824}),
              "Không tràn số nguyên ở chiều cao lớn nhất");
        for (int height : {5, 1, 0, -1, std::numeric_limits<int>::min()})
            Check(GuideRows(height).empty(), "Chiều cao dưới 6 phải trả rỗng");

        current = "T2: Màu và bề rộng ảnh 1280x720";
        cv::Mat frame(720, 1280, CV_8UC3, cv::Scalar(0, 0, 0));
        DrawGuides(frame);
        for (int x : {0, 640, 1279})
        {
            CheckPixel(frame, 600, x, cv::Vec3b(0, 255, 255));
            CheckPixel(frame, 480, x, cv::Vec3b(0, 255, 0));
            CheckPixel(frame, 360, x, cv::Vec3b(0, 0, 255));
        }

        current = "T3: Nét 1 px và vùng phía trên không đổi";
        for (int y : {599, 601, 479, 481, 359, 361})
            CheckPixel(frame, y, 640, cv::Vec3b(0, 0, 0));
        Check(cv::norm(frame.rowRange(0, 300), cv::NORM_INF) == 0,
              "Mọi pixel ở các hàng 0..299 phải đen");

        current = "T4: Kích thước, kiểu và ảnh không hợp lệ";
        Check(frame.rows == 720 && frame.cols == 1280 && frame.type() == CV_8UC3,
              "Không được đổi kích thước hoặc kiểu ảnh");
        cv::Mat empty;
        const int empty_type = empty.type();
        DrawGuides(empty);
        Check(empty.empty() && empty.rows == 0 && empty.cols == 0 && empty.type() == empty_type,
              "Ảnh rỗng phải giữ nguyên");
        cv::Mat gray(720, 1280, CV_8UC1, cv::Scalar(37));
        cv::Mat short_frame(5, 480, CV_8UC3, cv::Scalar(11, 22, 33));
        for (cv::Mat* image : {&gray, &short_frame})
        {
            const cv::Mat before = image->clone();
            DrawGuides(*image);
            Check(image->size() == before.size() && image->type() == before.type(),
                  "Ảnh bị bỏ qua phải giữ kích thước và kiểu");
            Check(cv::norm(*image, before, cv::NORM_INF) == 0,
                  "Ảnh bị bỏ qua phải giữ mọi pixel");
        }

        current = "T5: Màu trên ảnh 480x288";
        cv::Mat small(288, 480, CV_8UC3, cv::Scalar(0, 0, 0));
        DrawGuides(small);
        for (int x : {0, 240, 479})
        {
            CheckPixel(small, 240, x, cv::Vec3b(0, 255, 255));
            CheckPixel(small, 192, x, cv::Vec3b(0, 255, 0));
            CheckPixel(small, 144, x, cv::Vec3b(0, 0, 255));
        }
    }
    catch (const std::exception& error)
    {
        std::cerr << "HỎNG " << current << ": " << error.what() << '\n';
        return 1;
    }
    std::cout << "ĐẠT T1–T5: đường hướng dẫn camera\n";
    return 0;
}
