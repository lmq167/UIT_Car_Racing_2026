// Công cụ phụ cho test chấp nhận: tạo chuỗi ảnh giả frame_000000.jpg ... để làm camera giả hết nguồn
// (mô phỏng "mất ảnh"). Dùng: acc_make_frames <thư mục> <số ảnh>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <cstdio>
#include <cstdlib>
#include <string>

int main(int argc, char **argv) {
    if (argc != 3) { std::fprintf(stderr, "Dung: acc_make_frames <thu muc> <so anh>\n"); return 2; }
    const std::string dir = argv[1];
    const int n = std::atoi(argv[2]);
    for (int i = 0; i < n; ++i) {
        cv::Mat img(720, 1280, CV_8UC3, cv::Scalar(30 + 20 * (i % 10), 90, 160));
        cv::circle(img, cv::Point(60 + 50 * i, 190), 40, cv::Scalar(255, 255, 255), -1);
        cv::putText(img, std::to_string(i), cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 0, 0), 3);
        char name[64];
        std::snprintf(name, sizeof name, "/frame_%06d.jpg", i);
        if (!cv::imwrite(dir + name, img)) { std::fprintf(stderr, "Khong ghi duoc %s%s\n", dir.c_str(), name); return 1; }
    }
    return 0;
}
