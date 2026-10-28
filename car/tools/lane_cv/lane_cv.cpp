#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <unistd.h>
#include <vector>
#if defined(__GNUC__) && __GNUC__ < 8
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace fs = std::filesystem;
#endif

namespace {
constexpr int W = 480, H = 288;
constexpr int OK = 0, ARG_ERROR = 1, READ_ERROR = 2, NO_EVIDENCE = 3, WRITE_ERROR = 4;
const cv::Scalar LEFT(255, 180, 0), RIGHT(0, 145, 255), CENTER(75, 240, 100);

struct Options {
    fs::path input, output;
    bool directory = false;
    std::string fit = "crop";
    bool steps = true;
    double smax = 75, vmin = 205, kernel = 3;
    double leftArea = 12, leftHeight = 3, leftX = .55;
    double rightAreaRatio = .005, rightHeight = 1, rightX = .65;
    double roiTLX = .30, roiTLY = .38, roiTRX = 1, roiTRY = .38;
    double roiBRX = 1, roiBRY = 1, roiBLX = .06, roiBLY = 1;
    double minLeft = 4, minRows = 4, minSpan = 3;
    double iterations = 3, residualFloor = 2.5, residualScale = 3, margin = 20;
};

struct Parameter {
    const char* name;
    double Options::*value;
    double low, high;
    bool integer;
    const char* description;
};

// shortcut: các mặc định chọn riêng cho ảnh đối chiếu, cần hiệu chỉnh trên nhiều ảnh xe.
const Parameter PARAMETERS[] = {
    {"smax", &Options::smax, 0, 255, true, "S lon nhat (HSV)"},
    {"vmin", &Options::vmin, 0, 255, true, "V nho nhat (HSV)"},
    {"morph-kernel", &Options::kernel, 1, 31, true, "Co elip mo hinh thai, so le"},
    {"left-min-area", &Options::leftArea, 1, W*H, true, "Dien tich cum trai, px"},
    {"left-min-height", &Options::leftHeight, 1, H, true, "Chieu cao cum trai, px"},
    {"left-x-max", &Options::leftX, 0, 1, false, "Tam cum trai < ti le W"},
    {"right-min-area-ratio", &Options::rightAreaRatio, 0, 1, false, "Dien tich cum phai / (WH)"},
    {"right-min-height", &Options::rightHeight, 1, H, true, "Chieu cao cum phai, px"},
    {"right-x-min", &Options::rightX, 0, 1, false, "Tam cum phai > ti le W"},
    {"roi-tl-x", &Options::roiTLX, 0, 1, false, "ROI tren trai x/W"},
    {"roi-tl-y", &Options::roiTLY, 0, 1, false, "ROI tren trai y/H"},
    {"roi-tr-x", &Options::roiTRX, 0, 1, false, "ROI tren phai x/W"},
    {"roi-tr-y", &Options::roiTRY, 0, 1, false, "ROI tren phai y/H"},
    {"roi-br-x", &Options::roiBRX, 0, 1, false, "ROI duoi phai x/W"},
    {"roi-br-y", &Options::roiBRY, 0, 1, false, "ROI duoi phai y/H"},
    {"roi-bl-x", &Options::roiBLX, 0, 1, false, "ROI duoi trai x/W"},
    {"roi-bl-y", &Options::roiBLY, 0, 1, false, "ROI duoi trai y/H"},
    {"min-left-components", &Options::minLeft, 1, W*H, true, "So cum trai toi thieu"},
    {"fit-min-rows", &Options::minRows, 4, H, true, "So hang toi thieu de khop bac ba"},
    {"fit-min-y-span", &Options::minSpan, 3, H-1, true, "Khoang y toi thieu, px"},
    {"fit-iterations", &Options::iterations, 0, 100, true, "So luot loai diem lech"},
    {"fit-residual-floor", &Options::residualFloor, .001, W, false, "Nguong lech san, px"},
    {"fit-residual-scale", &Options::residualScale, .001, 100, false, "He so nhan trung vi do lech"},
    {"boundary-margin", &Options::margin, 0, W, false, "Khoang cach hai bien > nguong, px"}
};

std::vector<cv::Point> polygon(const Options& o, int width = W, int height = H) {
    auto point = [width, height](double x, double y) {
        return cv::Point(std::min(width-1, int(std::lround(x*width))),
                         std::min(height-1, int(std::lround(y*height))));
    };
    return {point(o.roiTLX, o.roiTLY), point(o.roiTRX, o.roiTRY),
            point(o.roiBRX, o.roiBRY), point(o.roiBLX, o.roiBLY)};
}

void validate(const Options& o) {
    for (const auto& p : PARAMETERS) {
        const double v = o.*(p.value);
        if (!std::isfinite(v) || v < p.low || v > p.high || (p.integer && v != std::floor(v)))
            throw std::runtime_error(std::string("Tham so ngoai mien: --") + p.name);
    }
    if (int(o.kernel)%2 != 1) throw std::runtime_error("--morph-kernel phai la so le");
    if (o.fit != "crop" && o.fit != "stretch") throw std::runtime_error("--fit phai la crop hoac stretch");
    const auto poly = polygon(o);
    if (!cv::isContourConvex(poly) || std::abs(cv::contourArea(poly)) < 1)
        throw std::runtime_error("ROI phai la tu giac loi, khong suy bien");
}

void help() {
    std::cout << "ucr_lane_cv --in <anh|thu_muc> --out-dir <thu_muc_moi> [--fit crop|stretch] [--no-steps]\n";
    std::cout << "ucr_lane_cv --in-dir <thu_muc_anh> --out-dir <thu_muc_moi_hoac_rong>\n"
                 "  --in-dir: giu nguyen co anh; ghi <stem>_mask.png, <stem>_overlay.jpg, summary.csv\n"
                 "  Trung stem da ghi trong luot nay: dung ten day du gom duoi anh.\n"
                 "  Anh ti le 5:3 (ke ca 640x384) giu nguyen co trong ca hai che do.\n"
                 "  Nguong px duoi day tinh theo anh moc 480x288, tu doi theo ti le W/H.\n";
    Options defaults;
    for (const auto& p : PARAMETERS)
        std::cout << "  --" << p.name << " <so> = " << defaults.*(p.value) << " : " << p.description << '\n';
    std::cout << "Ma thoat: 0 du bang chung; 1 doi so; 2 doc anh; 3 thieu bang chung; 4 ghi du lieu.\n";
    std::cout << "Voi --in-dir: 0 xu ly xong (ke ca khong co lan); 1 doi so; "
                 "2 khong doc duoc anh nao; 4 loi xu ly/ghi (ke ca thu muc ra khong rong). "
                 "time_ms chi do phan tich, khong tinh doc/ghi.\n";
}

Options parse(int argc, char** argv) {
    Options o;
    bool singleInput = false;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        if (key == "--no-steps") { o.steps = false; continue; }
        if (i+1 == argc) throw std::runtime_error("Thieu gia tri: " + key);
        const std::string value = argv[++i];
        if (value.compare(0,2,"--") == 0) throw std::runtime_error("Thieu gia tri: " + key);
        if (key == "--in") {
            if (o.directory) throw std::runtime_error("Khong ket hop --in va --in-dir");
            singleInput = true; o.input = fs::path(value);
        }
        else if (key == "--in-dir") {
            if (singleInput) throw std::runtime_error("Khong ket hop --in va --in-dir");
            o.directory = true; o.input = fs::path(value);
        }
        else if (key == "--out-dir") o.output = fs::path(value);
        else if (key == "--fit") o.fit = value;
        else {
            bool found = false;
            for (const auto& p : PARAMETERS) {
                if (key != std::string("--") + p.name) continue;
                std::size_t used = 0;
                const double v = std::stod(value, &used);
                if (used != value.size()) throw std::runtime_error("Gia tri khong phai so: " + key);
                o.*(p.value) = v;
                found = true;
                break;
            }
            if (!found) throw std::runtime_error("Doi so khong biet: " + key);
        }
    }
    if (o.input.empty() || o.output.empty()) throw std::runtime_error("Can --in hoac --in-dir, va --out-dir");
    validate(o);
    return o;
}

double median(std::vector<double> values) {
    std::sort(values.begin(), values.end());
    const std::size_t mid = values.size()/2;
    return values.size()%2 ? values[mid] : (values[mid-1]+values[mid])/2;
}

struct Curve {
    bool valid = false;
    int y0 = -1, y1 = -1, rows = 0, inliers = 0;
    double rmse = 0;
    cv::Mat coefficients;
    int height = H;
    std::string reason = "chua_khop";
    double at(int y) const {
        const double t = double(y)/height;
        return ((coefficients.at<double>(0)*t + coefficients.at<double>(1))*t
               + coefficients.at<double>(2))*t + coefficients.at<double>(3);
    }
};

Curve fitRows(const cv::Mat& mask, const Options& o) {
    Curve f;
    f.height = mask.rows;
    std::vector<int> ys;
    std::vector<double> xs;
    for (int y = 0; y < mask.rows; ++y) {
        std::vector<double> row;
        for (int x = 0; x < mask.cols; ++x) if (mask.at<uchar>(y,x)) row.push_back(x);
        if (!row.empty()) { ys.push_back(y); xs.push_back(median(row)); }
    }
    f.rows = int(ys.size());
    if (!ys.empty()) { f.y0 = ys.front(); f.y1 = ys.back(); }
    std::vector<bool> good(ys.size(), true);
    for (int iteration = 0; iteration <= int(o.iterations); ++iteration) {
        const int count = int(std::count(good.begin(), good.end(), true));
        int first = mask.rows, last = -1;
        for (std::size_t i = 0; i < ys.size(); ++i) if (good[i]) {
            first = std::min(first, ys[i]); last = std::max(last, ys[i]);
        }
        if (count < o.minRows || last-first < o.minSpan) {
            f.reason = "khong_du_hang_hoac_mien_y"; return f;
        }
        cv::Mat a(count, 4, CV_64F), b(count, 1, CV_64F);
        int row = 0;
        for (std::size_t i = 0; i < ys.size(); ++i) if (good[i]) {
            const double t = double(ys[i])/mask.rows;
            a.at<double>(row,0) = t*t*t; a.at<double>(row,1) = t*t;
            a.at<double>(row,2) = t; a.at<double>(row,3) = 1;
            b.at<double>(row++,0) = xs[i];
        }
        if (!cv::solve(a, b, f.coefficients, cv::DECOMP_SVD) || !cv::checkRange(f.coefficients)) {
            f.reason = "giai_he_that_bai"; return f;
        }
        std::vector<double> residual;
        double sum = 0;
        for (std::size_t i = 0; i < ys.size(); ++i) {
            const double error = std::abs(xs[i] - f.at(ys[i]));
            residual.push_back(error);
            if (good[i]) sum += error*error;
        }
        if (iteration == int(o.iterations)) {
            f.inliers = count; f.rmse = std::sqrt(sum/count); break;
        }
        const double threshold = std::max(o.residualFloor, o.residualScale*median(residual));
        for (std::size_t i = 0; i < good.size(); ++i) good[i] = residual[i] <= threshold;
    }
    f.valid = true; f.reason = "du_bang_chung";
    return f;
}

cv::Mat normalize(const cv::Mat& image, const Options& o, cv::Rect& crop) {
    crop = cv::Rect(0, 0, image.cols, image.rows);
    if (o.directory || static_cast<long long>(image.cols)*3 == static_cast<long long>(image.rows)*5)
        return image;
    if (o.fit == "crop") {
        if (static_cast<long long>(image.cols)*3 > static_cast<long long>(image.rows)*5) {
            crop.width = std::max(1, int(static_cast<long long>(image.rows)*5/3));
            crop.x = (image.cols-crop.width)/2;
        } else {
            crop.height = std::max(1, int(static_cast<long long>(image.cols)*3/5));
            crop.y = (image.rows-crop.height)/2;
        }
    }
    cv::Mat result;
    cv::resize(image(crop), result, cv::Size(W,H), 0, 0, cv::INTER_AREA);
    return result;
}

struct Result {
    cv::Mat image, roi, white, cropped, clean, left, right, overlay;
    cv::Rect crop;
    Curve lc, rc;
    int components = 0, leftComponents = 0, rightCandidates = 0, rightComponents = 0;
    int common0 = -1, common1 = -1;
    bool valid = false, disjoint = false, ordered = false;
    std::string reason;
    double timeMs = 0;
};

bool checkPair(Result& r, const Options& o) {
    cv::Mat overlap;
    cv::bitwise_and(r.left, r.right, overlap);
    r.disjoint = cv::countNonZero(overlap) == 0;
    if (!r.disjoint) { r.reason = "mat_na_chong_nhau"; return false; }
    if (r.leftComponents < o.minLeft || !r.rightComponents) {
        r.reason = "khong_du_cum_hai_bien"; return false;
    }
    if (!r.lc.valid || !r.rc.valid) { r.reason = "khong_du_bang_chung_khop"; return false; }
    r.common0 = std::max(r.lc.y0, r.rc.y0);
    r.common1 = std::min(r.lc.y1, r.rc.y1);
    if (r.common0 > r.common1) { r.reason = "khong_co_mien_y_chung"; return false; }
    // Không vẽ một đường khớp rời khỏi ảnh như thể vẫn có bằng chứng hợp lệ.
    for (const Curve* f : {&r.lc, &r.rc}) for (int y = f->y0; y <= f->y1; ++y) {
        const double x = f->at(y);
        if (!std::isfinite(x) || x < 0 || x > r.image.cols-1) {
            r.reason = "duong_khop_ra_ngoai_anh"; return false;
        }
    }
    for (int y = r.common0; y <= r.common1; ++y) if (r.lc.at(y)+o.margin >= r.rc.at(y)) {
        r.reason = "hai_bien_sai_thu_tu_hoac_qua_gan"; return false;
    }
    r.ordered = true; r.reason = "du_bang_chung";
    return true;
}

// Giữ đơn vị tham số dòng lệnh cũ; đổi ngưỡng hình học theo kích thước ảnh đang xử lý.
Options scaledOptions(const Options& o, const cv::Size& size) {
    Options scaled = o;
    const double sx = double(size.width)/W, sy = double(size.height)/H;
    scaled.leftArea *= sx*sy;
    scaled.leftHeight *= sy; scaled.rightHeight *= sy;
    scaled.minRows = std::max(4., o.minRows*sy); scaled.minSpan *= sy;
    scaled.residualFloor *= sx; scaled.margin *= sx;
    scaled.kernel = 2*std::lround((o.kernel-1)/2*std::min(sx,sy))+1;
    return scaled;
}

Result analyze(const cv::Mat& original, const Options& options) {
    const auto start = std::chrono::steady_clock::now();
    Result r;
    r.image = normalize(original, options, r.crop);
    const int width = r.image.cols, height = r.image.rows;
    const Options o = scaledOptions(options, r.image.size());
    r.roi = cv::Mat::zeros(height,width,CV_8U);
    cv::fillPoly(r.roi, std::vector<std::vector<cv::Point>>{polygon(o,width,height)}, cv::Scalar(255));
    cv::Mat hsv;
    cv::cvtColor(r.image, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0,0,o.vmin), cv::Scalar(179,o.smax,255), r.white);
    cv::bitwise_and(r.white, r.roi, r.cropped);
    cv::morphologyEx(r.cropped, r.clean, cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(int(o.kernel),int(o.kernel))));
    cv::Mat labels, stats, centroids;
    const int n = cv::connectedComponentsWithStats(r.clean, labels, stats, centroids, 8, CV_32S);
    r.components = n-1;
    r.left = cv::Mat::zeros(height,width,CV_8U); r.right = r.left.clone();
    int rightId = -1, rightArea = -1;
    for (int id = 1; id < n; ++id) {
        const int area = stats.at<int>(id,cv::CC_STAT_AREA);
        const int componentHeight = stats.at<int>(id,cv::CC_STAT_HEIGHT);
        const double x = centroids.at<double>(id,0);
        if (area >= o.leftArea && componentHeight >= o.leftHeight && x < o.leftX*width) {
            r.left.setTo(255, labels == id); ++r.leftComponents;
        }
        if (area >= o.rightAreaRatio*width*height && componentHeight >= o.rightHeight && x > o.rightX*width) {
            ++r.rightCandidates;
            if (area > rightArea) { rightArea = area; rightId = id; }
        }
    }
    if (rightId >= 0) { r.right.setTo(255, labels == rightId); r.rightComponents = 1; }
    r.lc = fitRows(r.left, o); r.rc = fitRows(r.right, o);
    r.valid = checkPair(r, o);
    r.overlay = r.image.clone();
    for (const auto& item : std::vector<std::pair<cv::Mat,cv::Scalar>>{{r.left,LEFT},{r.right,RIGHT}}) {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(item.first.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::drawContours(r.overlay, contours, -1, item.second, 1, cv::LINE_AA);
    }
    const double scale = double(width)/W;
    const int step = std::max(1, int(std::lround(5.*height/H)));
    const int radius = std::max(1, int(std::lround(scale)));
    if (r.valid) {
        for (const auto& item : std::vector<std::pair<const Curve*,cv::Scalar>>{{&r.lc,LEFT},{&r.rc,RIGHT}})
            for (int y = item.first->y0; y <= item.first->y1; y += step)
                cv::circle(r.overlay, cv::Point(int(std::lround(item.first->at(y))),y), radius, item.second, -1, cv::LINE_AA);
        for (int y = r.common0; y <= r.common1; y += step)
            cv::circle(r.overlay, cv::Point(int(std::lround((r.lc.at(y)+r.rc.at(y))/2)),y), radius, CENTER, -1, cv::LINE_AA);
    }
    const std::string label = r.valid ? "TRAI: xanh | PHAI: cam | TAM: la" : "THIEU BANG CHUNG - KHONG CO TAM";
    const cv::Point origin(int(std::lround(6*scale)), int(std::lround(18.*height/H)));
    cv::putText(r.overlay, label, origin, cv::FONT_HERSHEY_SIMPLEX, .42*scale, cv::Scalar(0,0,0), 3, cv::LINE_AA);
    cv::putText(r.overlay, label, origin, cv::FONT_HERSHEY_SIMPLEX, .42*scale, cv::Scalar(255,255,255), 1, cv::LINE_AA);
    r.timeMs = std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    return r;
}

std::string jsonString(const std::string& s) {
    std::ostringstream out;
    out << '"';
    for (unsigned char c : s) {
        if (c == '"' || c == '\\') out << '\\' << char(c);
        else if (c < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;
        else out << char(c);
    }
    out << '"';
    return out.str();
}

void writeText(const fs::path& path, const std::string& text) {
    std::ofstream out(path.string(), std::ios::binary);
    out << text;
    out.close();
    if (!out) throw std::runtime_error("Khong ghi duoc: " + path.string());
}

cv::Mat readImage(const fs::path& path) {
    std::ifstream in(path.string(), std::ios::binary);
    if (!in) return {};
    const std::vector<uchar> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (in.bad() || bytes.empty()) return {};
    return cv::imdecode(bytes, cv::IMREAD_COLOR);
}

void writeImage(const fs::path& path, const cv::Mat& image) {
    std::vector<uchar> bytes;
    if (!cv::imencode(path.extension().string(), image, bytes)) throw std::runtime_error("Khong ma hoa duoc anh");
    const fs::path temp = path.string() + ".tmp";
    std::ofstream out(temp.string(), std::ios::binary);
    out.write(reinterpret_cast<const char*>(bytes.data()), std::streamsize(bytes.size()));
    out.close();
    if (!out) throw std::runtime_error("Khong ghi duoc: " + path.string());
    fs::rename(temp, path);
}

void curveJson(std::ostream& out, const Curve& f) {
    out << "{\"fit_ok\":" << (f.valid ? "true" : "false") << ",\"reason\":" << jsonString(f.reason)
        << ",\"observed_y\":";
    if (f.y0 < 0) out << "null"; else out << '[' << f.y0 << ',' << f.y1 << ']';
    out << ",\"rows\":" << f.rows << ",\"inlier_rows\":" << f.inliers << ",\"row_coverage\":";
    if (f.y0 < 0) out << "null"; else out << double(f.rows)/(f.y1-f.y0+1);
    out << ",\"fit_residual_px\":";
    if (f.valid) out << f.rmse; else out << "null";
    out << '}';
}

std::string metrics(const fs::path& input, const cv::Mat& original, const Result* r, const Options& o) {
    std::ostringstream out;
    out << std::setprecision(15) << "{\n  \"input\":" << jsonString(input.u8string())
        << ",\n  \"status\":\"" << (!r ? "read_error" : r->valid ? "ok" : "insufficient_evidence")
        << "\",\n  \"reason\":" << jsonString(r ? r->reason : "khong_doc_duoc_anh")
        << ",\n  \"opencv\":" << jsonString(CV_VERSION) << ",\n  \"original_size\":";
    if (r) out << '[' << original.cols << ',' << original.rows << ']'; else out << "null";
    out << ",\n  \"normalized_size\":";
    if (r) out << '[' << r->image.cols << ',' << r->image.rows << ']'; else out << "null";
    out << ",\n  \"fit\":" << jsonString(o.fit)
        << ",\n  \"steps\":" << (o.steps ? "true" : "false") << ",\n  \"thresholds\":{";
    bool first = true;
    for (const auto& p : PARAMETERS) {
        if (!first) out << ',';
        first = false;
        out << '\n' << "    " << jsonString(p.name) << ':' << o.*(p.value);
    }
    out << "\n  }";
    if (r) {
        out << ",\n  \"crop_xywh\":[" << r->crop.x << ',' << r->crop.y << ',' << r->crop.width << ',' << r->crop.height
            << "],\n  \"roi_polygon\":[";
        first = true;
        for (const auto& p : polygon(o,r->image.cols,r->image.rows)) {
            if (!first) out << ',';
            first = false; out << '[' << p.x << ',' << p.y << ']';
        }
        out << "],\n  \"components_total\":" << r->components << ",\n  \"left_components\":" << r->leftComponents
            << ",\n  \"right_candidates\":" << r->rightCandidates << ",\n  \"right_components\":" << r->rightComponents
            << ",\n  \"left_pixels\":" << cv::countNonZero(r->left) << ",\n  \"right_pixels\":" << cv::countNonZero(r->right)
            << ",\n  \"time_ms\":" << r->timeMs << ",\n  \"left\":";
        curveJson(out, r->lc); out << ",\n  \"right\":"; curveJson(out, r->rc);
        out << ",\n  \"masks_disjoint\":" << (r->disjoint ? "true" : "false")
            << ",\n  \"boundaries_ordered\":" << (r->ordered ? "true" : "false") << ",\n  \"midpoint_y\":";
        if (r->valid) out << '[' << r->common0 << ',' << r->common1 << ']'; else out << "null";
    }
    out << ",\n  \"note\":\"Sai so khop do do lech voi diem anh da tach, khong phai do chinh xac nhan dien. "
           "Trung diem trong anh chua phai quy dao mat duong da hieu chuan.\"\n}\n";
    return out.str();
}

cv::Mat steps(const Result& r) {
    const int width = r.image.cols, height = r.image.rows;
    cv::Mat roiView, observed = cv::Mat::zeros(height,width,CV_8UC3);
    cv::bitwise_and(r.image, r.image, roiView, r.roi);
    cv::addWeighted(r.image, .55, roiView, .45, 0, roiView);
    observed.setTo(LEFT, r.left); observed.setTo(RIGHT, r.right);
    const std::vector<cv::Mat> panels{r.image, roiView, r.white, r.clean, observed, r.overlay};
    const char* titles[]{"1. Anh chuan hoa", "2. Vung quan tam", "3. Mat na trang HSV",
                         "4. ROI va mo hinh thai", "5. Diem anh giu lai", "6. Duong khop va trung diem"};
    cv::Mat sheet(3*(height+32),2*width,CV_8UC3,cv::Scalar(20,25,34));
    for (int i = 0; i < 6; ++i) {
        const int x = (i%2)*width, y = (i/2)*(height+32);
        cv::Mat panel;
        if (panels[i].channels() == 1) cv::cvtColor(panels[i], panel, cv::COLOR_GRAY2BGR);
        else panel = panels[i];
        panel.copyTo(sheet(cv::Rect(x,y+32,width,height)));
        cv::putText(sheet, titles[i], cv::Point(x+8,y+22), cv::FONT_HERSHEY_SIMPLEX, .5,
                    cv::Scalar(255,255,255), 1, cv::LINE_AA);
    }
    return sheet;
}

// Đưa mặt nạ về đúng vị trí ảnh gốc; phần ngoài vùng cắt giữ bằng 0.
cv::Mat originalMask(const cv::Mat& mask, const cv::Size& size, const cv::Rect& crop) {
    if (mask.size() == size && crop == cv::Rect(0,0,size.width,size.height)) return mask;
    cv::Mat out = cv::Mat::zeros(size,CV_8U), scaled;
    cv::resize(mask, scaled, crop.size(), 0, 0, cv::INTER_NEAREST);
    scaled.copyTo(out(crop));
    return out;
}

int processFile(const fs::path& input, const fs::path& dir, const Options& o) {
    cv::Mat original;
    try { original = readImage(input); } catch (const cv::Exception&) { /* Ảnh hỏng được báo bằng mã đọc ảnh. */ }
    if (original.empty()) {
        writeText(dir/"metrics.json", metrics(input, original, nullptr, o));
        return READ_ERROR;
    }
    const Result r = analyze(original, o);
    std::cout << input.string() << " : " << r.timeMs << " ms\n";
    writeImage(dir/"normalized.png", r.image);
    for (const auto& item : std::vector<std::pair<std::string,cv::Mat>>{
            {"white_mask",r.white},{"roi_mask",r.cropped},{"clean_mask",r.clean}}) {
        writeImage(dir/(item.first+".png"), item.second);
        writeImage(dir/(item.first+"_original.png"), originalMask(item.second, original.size(), r.crop));
    }
    writeImage(dir/"overlay.png", r.overlay);
    if (o.steps) writeImage(dir/"steps.png", steps(r));
    writeText(dir/"metrics.json", metrics(input, original, &r, o));
    return r.valid ? OK : NO_EVIDENCE;
}

std::vector<fs::path> inputFiles(const fs::path& input, bool directoryMode = false) {
    if (!fs::is_directory(input)) return {input};
    std::vector<fs::path> files;
    for (const auto& item : fs::directory_iterator(input)) {
        if (!fs::is_regular_file(item.path())) continue;
        std::string ext = item.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return char(std::tolower(c)); });
        if (ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".bmp" || (!directoryMode &&
                (ext == ".tif" || ext == ".tiff" || ext == ".ppm" || ext == ".pgm" || ext == ".webp")))
            files.push_back(item.path());
    }
    std::sort(files.begin(), files.end());
    return files;
}

std::string csvString(const std::string& value) {
    std::string out = "\"";
    for (char c : value) { if (c == '"') out += '"'; out += c; }
    return out + '"';
}

int runDirectory(const Options& o) {
    try {
        if (fs::exists(o.output) && (!fs::is_directory(o.output) || !fs::is_empty(o.output))) {
            std::cerr << "--out-dir da co ket qua/dang duoc su dung hoac khong phai thu muc; can thu muc moi/rong\n"; return WRITE_ERROR;
        }
        std::vector<fs::path> files;
        try {
            if (!fs::is_directory(o.input)) { std::cerr << "--in-dir khong phai thu muc doc duoc\n"; return READ_ERROR; }
            files = inputFiles(o.input, true);
        } catch (const fs::filesystem_error& e) {
            std::cerr << "Loi doc thu muc: " << e.what() << '\n'; return READ_ERROR;
        }
        if (!fs::create_directories(o.output) && (!fs::is_directory(o.output) || !fs::is_empty(o.output)))
            throw std::runtime_error("--out-dir khong tao duoc hoac da co ket qua/dang duoc su dung");
        const int fd = ::open((o.output/"summary.csv").string().c_str(), O_CREAT | O_EXCL | O_WRONLY, 0666);
        if (fd < 0) throw std::system_error(errno, std::generic_category(), "Khong tao doc quyen summary.csv; dich co the dang duoc su dung");
        FILE* csvRaw = ::fdopen(fd, "w");
        struct Closer { void operator()(FILE* f) const { if (f) std::fclose(f); } };
        std::unique_ptr<FILE, Closer> csv(csvRaw);
        if (!csv) { ::close(fd); throw std::runtime_error("Khong mo duoc summary.csv de ghi"); }
        if (std::fputs("image,left_clusters,right_clusters,has_center,time_ms\n", csv.get()) == EOF || std::fflush(csv.get()) != 0)
            throw std::runtime_error("Khong ghi duoc summary.csv");
        std::size_t count = 0;
        double totalMs = 0;
        for (const auto& file : files) {
            cv::Mat image;
            try { image = readImage(file); } catch (const cv::Exception&) { /* Báo ảnh hỏng và tiếp tục. */ }
            if (image.empty()) { std::cerr << "Khong doc duoc anh: " << file.string() << '\n'; continue; }
            const Result r = analyze(image, o);
            std::string name = file.stem().string();
            if (fs::exists(o.output/(name+"_mask.png"))) name = file.filename().string();
            writeImage(o.output/(name+"_mask.png"), r.clean);
            writeImage(o.output/(name+"_overlay.jpg"), r.overlay);
            std::ostringstream row;
            row << std::setprecision(10) << csvString(file.filename().u8string()) << ',' << r.leftComponents << ',' << r.rightComponents
                << ',' << (r.valid ? 1 : 0) << ',' << r.timeMs << '\n';
            if (std::fputs(row.str().c_str(), csv.get()) == EOF || std::fflush(csv.get()) != 0)
                throw std::runtime_error("Khong ghi duoc summary.csv");
            ++count; totalMs += r.timeMs;
            std::cout << file.string() << " : " << r.timeMs << " ms\n";
        }
        if (std::fclose(csv.release()) != 0) throw std::runtime_error("Khong dong duoc summary.csv");
        std::cout << "Tong: " << count << " anh, trung binh " << (count ? totalMs/count : 0) << " ms\n";
        if (!count) { std::cerr << "Thu muc khong co anh doc duoc\n"; return READ_ERROR; }
        return OK;
    } catch (const std::exception& e) {
        std::cerr << "Loi xu ly/ghi ket qua: " << e.what() << '\n'; return WRITE_ERROR;
    }
}

int run(const Options& o) {
    if (o.directory) return runDirectory(o);
    std::vector<fs::path> files;
    try { files = inputFiles(o.input); }
    catch (const fs::filesystem_error& e) { std::cerr << "Loi doc dau vao: " << e.what() << '\n'; return READ_ERROR; }
    try {
        if (fs::exists(o.output)) { std::cerr << "--out-dir da ton tai; hay chon thu muc moi\n"; return WRITE_ERROR; }
        fs::create_directories(o.output.parent_path().empty() ? fs::path(".") : o.output.parent_path());
        if (!fs::create_directory(o.output)) return WRITE_ERROR;
        if (files.empty()) {
            writeText(o.output/"metrics.json", metrics(o.input, cv::Mat(), nullptr, o));
            std::cerr << "Thu muc khong co anh ho tro\n"; return READ_ERROR;
        }
        int status = OK;
        std::size_t index = 0;
        for (const auto& file : files) {
            std::ostringstream name;
            name << std::setw(6) << std::setfill('0') << ++index << '_' << file.filename().string();
            const fs::path dir = o.output/name.str();
            if (!fs::create_directory(dir)) throw std::runtime_error("Khong tao duoc thu muc ket qua");
            const int code = processFile(file, dir, o);
            std::cout << file.string() << " -> " << dir.string() << " : " << code << '\n';
            // Lỗi đọc có ưu tiên cao hơn thiếu bằng chứng khi xử lý nhiều ảnh.
            if (code == READ_ERROR || status == OK) status = code;
        }
        return status;
    } catch (const std::exception& e) {
        std::cerr << "Loi xu ly/ghi ket qua: " << e.what() << '\n'; return WRITE_ERROR;
    }
}

void require(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}

cv::Mat synthetic() {
    cv::Mat image(H,W,CV_8UC3,cv::Scalar(35,35,35));
    for (int y = 125; y <= 230; y += 35)
        cv::rectangle(image, cv::Rect(190,y,9,14), cv::Scalar(255,255,255), -1);
    cv::rectangle(image, cv::Rect(355,115,11,160), cv::Scalar(255,255,255), -1);
    return image;
}

int selfTest(const fs::path& dir) {
    try {
        require(!fs::exists(dir), "Thu muc kiem thu da ton tai");
        fs::create_directories(dir);
        Options o;
        require(median({1,4,2,3}) == 2.5 && median({1,3,2}) == 2, "Trung vi sai");
        require(jsonString("\"\\\n") == "\"\\\"\\\\\\u000a\"", "Ma hoa chuoi JSON sai");
        require(csvString("a,\"b\n.png") == "\"a,\"\"b\n.png\"", "Ma hoa cot CSV sai");
        const cv::Mat image = synthetic();
        const Result r = analyze(image,o);
        require(r.valid && r.leftComponents == 4 && r.rightComponents == 1, "Anh tong hop phai co du hai bien");
        require(r.common0 == 125 && r.common1 == 243, "Trung diem vuot mien quan sat chung");
        require(r.lc.rmse < 1e-6 && r.rc.rmse < 1e-6, "Duong thang khop sai");
        const double scale = 640./W;
        cv::Mat enlarged, direct(384,640,CV_8UC3,cv::Scalar(35,35,35));
        cv::resize(image, enlarged, direct.size(), 0, 0, cv::INTER_LINEAR);
        auto rectangle = [&direct, scale](int x, int y, int width, int height) {
            cv::rectangle(direct, cv::Rect(int(std::lround(x*scale)), int(std::lround(y*scale)),
                          int(std::lround(width*scale)), int(std::lround(height*scale))), cv::Scalar(255,255,255), -1);
        };
        for (int y = 125; y <= 230; y += 35) rectangle(190,y,9,14);
        rectangle(355,115,11,160);
        for (const cv::Mat& large : {enlarged, direct}) {
            const Result scaled = analyze(large,o);
            require(scaled.image.size() == large.size() && scaled.crop == cv::Rect(0,0,640,384)
                    && cv::norm(large,scaled.image,cv::NORM_INF) == 0, "Anh 640 bi cat/doi co/noi dung");
            require(scaled.leftComponents == r.leftComponents && scaled.rightComponents == r.rightComponents
                    && scaled.valid == r.valid, "Ket qua 640 khong tuong duong 480");
            for (int y = r.common0; y <= r.common1; ++y) {
                const int ly = int(std::lround(y*scale));
                if (ly < scaled.common0 || ly > scaled.common1) continue;
                const double oldCenter = (r.lc.at(y)+r.rc.at(y))/2;
                const double newCenter = (scaled.lc.at(ly)+scaled.rc.at(ly))/2;
                require(std::abs(newCenter/640-oldCenter/W) <= .03, "Tam 640 lech qua 3% W");
            }
        }
        require(!analyze(cv::Mat::zeros(H,W,CV_8UC3),o).valid, "Anh den phai thieu bang chung");
        cv::Mat oneSide = image.clone(); oneSide(cv::Rect(300,0,180,H)).setTo(0);
        require(!analyze(oneSide,o).valid, "Mot bien khong du de tinh tam");
        Options overlap = o; overlap.leftX = .9; overlap.rightX = .7;
        require(analyze(image,overlap).reason == "mat_na_chong_nhau", "Khong chan mat na chong nhau");
        Result crossed = r;
        crossed.lc.coefficients = r.lc.coefficients.clone();
        crossed.lc.coefficients.at<double>(3) = 400;
        require(!checkPair(crossed,o) && crossed.reason == "hai_bien_sai_thu_tu_hoac_qua_gan", "Khong chan bien dao thu tu");
        Result separate = r; separate.rc.y0 = 250;
        require(!checkPair(separate,o) && separate.reason == "khong_co_mien_y_chung", "Khong chan mien y roi nhau");
        cv::Mat shortMask = cv::Mat::zeros(H,W,CV_8U);
        shortMask(cv::Rect(100,120,10,3)).setTo(255);
        require(!fitRows(shortMask,o).valid, "Ba hang khong du khop bac ba");
        cv::Mat cubic = cv::Mat::zeros(H,W,CV_8U);
        for (int y = 70; y < 270; ++y) {
            const double t = double(y)/H;
            int x = int(std::lround(120+35*t-20*t*t+30*t*t*t));
            if (y%13 == 0) x += 50;
            cubic.at<uchar>(y,x) = 255;
        }
        const Curve f = fitRows(cubic,o);
        const double t = 200./H;
        require(f.valid && f.inliers < f.rows && f.rmse < 1 && std::abs(f.at(200)-(120+35*t-20*t*t+30*t*t*t)) < 1,
                "Khop bac ba/loai diem lech sai");
        cv::Rect crop;
        normalize(cv::Mat(300,800,CV_8UC3),o,crop);
        require(crop == cv::Rect(150,0,500,300), "Cat giua ngang sai");
        normalize(cv::Mat(600,500,CV_8UC3),o,crop);
        require(crop == cv::Rect(0,150,500,300), "Cat giua doc sai");
        o.fit = "stretch";
        normalize(cv::Mat(600,500,CV_8UC3),o,crop);
        require(crop == cv::Rect(0,0,500,600), "Stretch khong giu toan anh");
        const cv::Mat restored = originalMask(r.white, cv::Size(800,300), cv::Rect(150,0,500,300));
        require(restored.size() == cv::Size(800,300) && cv::countNonZero(restored(cv::Rect(0,0,150,300))) == 0,
                "Mat na goc khong bu tru vung cat");
        o = Options();
        writeImage(dir/"lanes.png",image);
        writeImage(dir/"blank.png",cv::Mat::zeros(H,W,CV_8UC3));
        writeText(dir/"broken.png","khong phai anh");
        fs::create_directory(dir/"output");
        require(processFile(dir/"lanes.png",dir/"output",o) == OK, "Xu ly tep tong hop that bai");
        require(readImage(dir/"output"/"normalized.png").size() == cv::Size(W,H), "Sai co anh dau ra");
        for (const char* name : {"white_mask.png", "roi_mask.png", "clean_mask.png", "white_mask_original.png",
                                 "roi_mask_original.png", "clean_mask_original.png", "overlay.png"})
            require(readImage(dir/"output"/name).size() == cv::Size(W,H), "Thieu anh ket qua hoac sai kich thuoc");
        require(fs::exists(dir/"output"/"steps.png"), "Thieu bang cac buoc mac dinh");
        fs::create_directory(dir/"blank-output");
        require(processFile(dir/"blank.png",dir/"blank-output",o) == NO_EVIDENCE, "Sai ma thieu bang chung");
        fs::create_directory(dir/"broken-output");
        require(processFile(dir/"broken.png",dir/"broken-output",o) == READ_ERROR, "Sai ma doc anh");
        Options batch;
        batch.directory = true; batch.input = dir/"batch-input"; batch.output = dir/"batch-output";
        fs::create_directory(batch.input); fs::create_directory(batch.output);
        writeImage(batch.input/"a.png", direct);
        writeImage(batch.input/"b.png", cv::Mat::zeros(384,640,CV_8UC3));
        writeText(batch.input/"c.jpg", "anh hong");
        require(runDirectory(batch) == OK, "Xu ly thu muc co anh den/hong sai");
        require(readImage(batch.output/"a_mask.png").size() == direct.size()
                && readImage(batch.output/"a_overlay.jpg").size() == direct.size(), "Sai dau ra thu muc");
        std::ifstream summary((batch.output/"summary.csv").string(), std::ios::binary);
        const std::string csv((std::istreambuf_iterator<char>(summary)), std::istreambuf_iterator<char>());
        require(csv.find("image,left_clusters,right_clusters,has_center,time_ms\n\"a.png\",4,1,1,") == 0
                && csv.find("\n\"b.png\",0,0,0,") != std::string::npos, "Sai CSV/thu tu/anh thieu lan");
        require(runDirectory(batch) == WRITE_ERROR, "Khong chan thu muc ra da co ket qua");
        batch.output = dir/"collision-output";
        writeImage(batch.input/"same.jpg", direct);
        writeImage(batch.input/"same.png", cv::Mat::zeros(384,640,CV_8UC3));
        require(runDirectory(batch) == OK, "Xu ly trung stem that bai");
        for (const char* name : {"same_mask.png", "same_overlay.jpg", "same.png_mask.png", "same.png_overlay.jpg"})
            require(readImage(batch.output/name).size() == direct.size(), "Trung stem lam mat ket qua");
        require(cv::countNonZero(readImage(batch.output/"same_mask.png").reshape(1)) > 0
                && cv::countNonZero(readImage(batch.output/"same.png_mask.png").reshape(1)) == 0,
                "Trung stem ghi de mat na hoac sai thu tu");
        batch.input = dir/"empty-input"; batch.output = dir/"empty-output";
        fs::create_directory(batch.input);
        require(runDirectory(batch) == READ_ERROR, "Thu muc rong phai tra ma 2");
        writeText(batch.input/"broken.png", "anh hong"); batch.output = dir/"unreadable-output";
        require(runDirectory(batch) == READ_ERROR, "Thu muc toan anh hong phai tra ma 2");
        batch.input = dir/"missing-input"; batch.output = dir/"missing-output";
        require(runDirectory(batch) == READ_ERROR, "Thu muc khong ton tai phai tra ma 2");
        for (std::vector<std::string> args : {
                std::vector<std::string>{"ucr_lane_cv", "--in-dir"},
                {"ucr_lane_cv", "--in-dir", "input"},
                {"ucr_lane_cv", "--in-dir", "--out-dir", "output"},
                {"ucr_lane_cv", "--in-dir", "input", "--out-dir"},
                {"ucr_lane_cv", "--in-dir", "input", "--out-dir", "output", "--unknown", "1"},
                {"ucr_lane_cv", "--in-dir", "input", "--in", "image", "--out-dir", "output"},
                {"ucr_lane_cv", "--in", "image", "--in-dir", "input", "--out-dir", "output"}}) {
            std::vector<char*> argv;
            for (auto& arg : args) argv.push_back(&arg[0]);
            bool rejected = false;
            try { parse(int(argv.size()), argv.data()); } catch (const std::exception&) { rejected = true; }
            require(rejected, "Khong chan doi so sai/thieu gia tri");
        }
        std::cout << "Kiem thu tong hop: DAT\n";
        return OK;
    } catch (const std::exception& e) { std::cerr << "Kiem thu: " << e.what() << '\n'; return ARG_ERROR; }
}
} // namespace

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") { help(); return OK; }
    if (argc == 3 && std::string(argv[1]) == "--self-test") return selfTest(fs::path(argv[2]));
    try { return run(parse(argc,argv)); }
    catch (const std::exception& e) { std::cerr << "Doi so: " << e.what() << '\n'; return ARG_ERROR; }
}
