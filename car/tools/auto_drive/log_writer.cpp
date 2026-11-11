#include "control/LaneController.h"
#include "log_writer.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <csignal>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <spawn.h>
#include <sys/statvfs.h>
#include <sys/wait.h>

extern char** environ;
namespace auto_drive {
void warningNonBlocking(const char* text) noexcept {
    pollfd out{STDERR_FILENO, POLLOUT, 0};
    if (poll(&out, 1, 0) > 0 && (out.revents & POLLOUT)) {
        const auto ignored = ::write(STDERR_FILENO, text, std::strlen(text)); (void)ignored;
    }
}
namespace {
int64_t timeMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
}
// Coreutils có sẵn trên Ubuntu/Jetson; không shell, không nạp toàn engine vào RAM.
std::string sha256(const std::string& path) try {
    int pipefd[2];
    if (pipe2(pipefd,O_CLOEXEC|O_NONBLOCK)!=0) throw std::runtime_error("Không tạo được ống SHA-256");
    posix_spawn_file_actions_t actions;
    if (posix_spawn_file_actions_init(&actions)!=0) {
        close(pipefd[0]); close(pipefd[1]); throw std::runtime_error("Không chuẩn bị được SHA-256");
    }
    int error=posix_spawn_file_actions_adddup2(&actions,pipefd[1],STDOUT_FILENO);
    if (!error) error=posix_spawn_file_actions_addclose(&actions,pipefd[0]);
    char* argv[]={const_cast<char*>("sha256sum"),const_cast<char*>("--"),const_cast<char*>(path.c_str()),nullptr};
    pid_t pid=-1;
    if (!error) error=posix_spawnp(&pid,"sha256sum",&actions,nullptr,argv,environ);
    posix_spawn_file_actions_destroy(&actions); close(pipefd[1]);
    if (error) { close(pipefd[0]); throw std::runtime_error("Không chạy được sha256sum"); }
    int status=0; bool done=false;
    const auto deadline=timeMs()+5000;
    while (timeMs()<deadline) {
        const auto r=waitpid(pid,&status,WNOHANG);
        if (r==pid) { done=true; break; }
        if (r<0 && errno!=EINTR) break;
        poll(nullptr,0,10);
    }
    if (!done) { kill(pid,SIGKILL); while (waitpid(pid,&status,0)<0 && errno==EINTR) {} }
    std::array<char,4096> buf{};
    const auto n=read(pipefd[0],buf.data(),buf.size()); close(pipefd[0]);
    if (!done || !WIFEXITED(status) || WEXITSTATUS(status)!=0 || n<64)
        throw std::runtime_error("SHA-256 lỗi/quá hạn");
    std::string result(buf.data(),64);
    if (result.find_first_not_of("0123456789abcdef")!=std::string::npos) throw std::runtime_error("SHA-256 sai định dạng");
    return result;
} catch (...) {
    return "loi";
}
namespace {
std::string modelSession(const std::string& name,const std::string& info,const std::string& hash) {
    cv::FileStorage file;
    if (!info.empty()) file.open(info,cv::FileStorage::READ | cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_JSON);
    std::string fields;
    for (const char* key:{"variant","seed","data_tag"}) {
        const auto node=file.isOpened() ? file[key] : cv::FileNode();
        std::ostringstream value; value<<std::setprecision(17);
        if (node.isInt()) value<<int(node);
        else if (node.isReal()) value<<double(node);
        else value<<jsonString(node.isString() ? std::string(node) : "");
        fields+=",\n"+jsonString(name+"_"+key)+":"+value.str();
    }
    return fields+",\n"+jsonString(name+"_onnx_sha256")+":"+jsonString(hash);
}
void writeFile(const std::string& path,const std::string& bytes) {
    std::ofstream f; f.exceptions(std::ios::badbit|std::ios::failbit);
    f.open(path,std::ios::binary); f<<bytes; f.close();
}
double diskMb(const std::string& path) {
#if defined(UCR_AUTO_TEST_HOOKS) && UCR_AUTO_TEST_HOOKS
    if (hooks.freeMb>=0) return hooks.freeMb;
    if (const char* v=std::getenv("UCR_AUTO_FAKE_FREE_MB")) {
        size_t used=0; const double n=std::stod(v,&used);
        if (used!=std::string(v).size() || !std::isfinite(n) || n<0) throw std::runtime_error("FREE_MB sai");
        return n;
    }
#endif
    struct statvfs s{};
    if (statvfs(path.c_str(),&s)!=0) throw std::runtime_error("Không đọc được đĩa trống");
    return double(s.f_bavail)*s.f_frsize/(1024.0*1024.0);
}
double processRssMb() {
    std::ifstream f("/proc/self/status"); std::string line;
    while (std::getline(f,line)) {
        if (line.compare(0,6,"VmRSS:")==0) {
            std::istringstream in(line.substr(6)); double kb=0; in>>kb; return kb/1024;
        }
    }
    return 0;
}
}
LogWriter::LogWriter(const Options& o,const std::string& yaml,
                     std::function<void(const char*,const std::string&)> trace)
    : root_(o.outRoot),yaml_(yaml),info_(o.info),engine_(o.engine),yoloInfo_(o.yoloInfo),yoloEngine_(o.yoloEngine),
      unetOnnxSha256_(o.unetOnnxSha256),yoloOnnxSha256_(o.yoloOnnxSha256),yoloEnabled_(!o.noYolo),trace_(std::move(trace)) {
    worker_=std::thread(&LogWriter::run,this);
}
bool LogWriter::tryImage(const ModelResult& image) noexcept {
    if (!imagesEnabled) return false;
    try {
        std::unique_lock<std::mutex> lock(mutex_,std::try_to_lock);
        if (!lock.owns_lock() || images_.size()>=8) { ++dropped; return false; }
        images_.push_back(image); available_.notify_one(); return true;
    } catch (...) { ++dropped; return false; }
}
void LogWriter::offerDetections(const control::Detections& batch) noexcept {
    try {
        std::unique_lock<std::mutex> lock(mutex_,std::try_to_lock);
        if (!lock.owns_lock() || quit_ || finished || detections_.size()>=64) { ++detectionsDropped; return; }
        detections_.push_back(batch); available_.notify_one();
    } catch (...) { ++detectionsDropped; }
}
void LogWriter::offerControl(const std::string& row) noexcept {
    try {
        std::unique_lock<std::mutex> lock(mutex_,std::try_to_lock);
        if (!lock.owns_lock() || csv_.size()>=256) { ++csvDropped; return; }
        csv_.push_back(row); available_.notify_one();
    } catch (...) { ++csvDropped; }
}
void LogWriter::offerStatistics(const Statistics& value,int every,bool p95) {
    statistics_.offer(value); recordEvery_=every;
    if (p95) recordP95_=true;
}
void LogWriter::requestStop() noexcept { quit_=true; available_.notify_one(); }
LogWriter::~LogWriter() { requestStop(); if (worker_.joinable()) worker_.join(); }
void LogWriter::run() noexcept {
    try {
        makeDirs(root_);
        const auto base=root_+"/"+std::to_string(std::time(nullptr))+"_"+std::to_string(getpid());
        folder=base;
        if (mkdir(folder.c_str(),0700)!=0) throw std::runtime_error("Không tạo được phiên log mới");
        makeDirs(folder+"/images"); makeDirs(folder+"/masks");
        writeFile(folder+"/control.yaml",yaml_);
        const auto info=readBytes(info_);
        writeFile(folder+"/road_info.json",info);
        const auto engineHash=engine_.empty() ? "" : sha256(engine_);
        const auto controlHash=sha256(folder+"/control.yaml");
        const auto infoHash=sha256(folder+"/road_info.json");
        const auto yoloInfo=yoloEnabled_ ? readBytes(yoloInfo_) : "null";
        if (yoloEnabled_) writeFile(folder+"/yolo_info.json",yoloInfo);
        const auto yoloHash=yoloEnabled_ && !yoloEngine_.empty() ? sha256(yoloEngine_) : "";
        const auto yoloInfoHash=yoloEnabled_ ? sha256(folder+"/yolo_info.json") : "";
        if (engineHash=="loi" || controlHash=="loi" || infoHash=="loi" || yoloHash=="loi" || yoloInfoHash=="loi")
            warningNonBlocking("CANH BAO: bam SHA-256 that bai, van ghi log phien\n");
        writeFile(folder+"/session.json","{\n\"git_hash\":"+jsonString(UCR_AUTO_GIT_HASH)+
            ",\n\"git_branch\":"+jsonString(UCR_AUTO_GIT_BRANCH)+
            ",\n\"control_yaml\":"+jsonString(yaml_)+",\n\"control_sha256\":"+jsonString(controlHash)+
            ",\n\"info_json\":"+info+",\n\"info_sha256\":"+jsonString(infoHash)+
            ",\n\"engine_name\":"+jsonString(engine_)+",\n\"engine_sha256\":"+jsonString(engineHash)+
            ",\n\"yolo_enabled\":"+std::to_string(int(yoloEnabled_))+",\n\"yolo_info_json\":"+yoloInfo+
            ",\n\"yolo_info_sha256\":"+jsonString(yoloInfoHash)+
            ",\n\"yolo_engine_name\":"+jsonString(yoloEngine_)+",\n\"yolo_engine_sha256\":"+jsonString(yoloHash)+
            modelSession("unet",info,unetOnnxSha256_)+
            modelSession("yolo",yoloEnabled_ ? yoloInfo : "",yoloEnabled_ ? yoloOnnxSha256_ : "")+"\n}\n");
        const auto csvPath=folder+"/control_log.csv";
        writeFile(csvPath,"t_ms,frame_id,mask_file,cmd_cms,cmd_deg,mask_ms,image_ms,go,stop,lane_ref,planner_state,"
            "image_age_ms,mask_age_ms,det_age_ms,scan_rows,e,p,d,requested_steer_deg,requested_speed_cms,"
            "sent_steer_deg,sent_speed_cms,reason,road_ms,yolo_ms,loop_ms,dropped,csv_dropped,"
            "camera_fps,loop_p95_ms,rss_mb,free_mb,command_id,command_ms,mask_id,det_frame_id,det_captured_ms,detections_dropped,yaw_deg,odo_cm,"
            "route_step,route_note,turn_dir,traj_mode,look_x,look_y,fit_curv\n");
        const auto detectionsPath=folder+"/detections.csv";
        writeFile(detectionsPath,"frame_id,captured_ms,class,conf,x1,y1,x2,y2\n");
        auto nextDisk=int64_t(0),nextStats=timeMs()+2000;
        Statistics stats;
        bool stallNotice=false;
        while (true) {
            if (timeMs()>=nextDisk) {
                try {
                    freeMb=diskMb(root_);
                    if (freeMb<kMinFreeMb) {
                        if (imagesEnabled.exchange(false)) std::cerr<<"Cảnh báo: đĩa dưới 1024 MiB, chỉ ghi CSV\n";
                    }
                } catch (const std::exception& e) {
                    imagesEnabled=false; std::cerr<<"Cảnh báo: "<<e.what()<<'\n';
                }
                nextDisk=timeMs()+10000;
            }
            ready=true;
            statistics_.latest(stats);
            if (recordP95_.exchange(false)) {
                std::cout<<"record_reason=p95 record_every_n="<<recordEvery_<<'\n'<<std::flush;
                trace_("record",std::to_string(recordEvery_.load())+" "+std::to_string(dropped.load())+" "+std::to_string(freeMb.load()));
            }
            if (timeMs()>=nextStats) {
                stats.rss_mb=processRssMb(); rssMb=stats.rss_mb; stats.free_mb=freeMb;
                std::cout<<"camera_fps="<<stats.camera_fps<<" road_ms="<<stats.road_ms<<" yolo_ms="<<stats.yolo_ms<<" loop_p95_ms="
                         <<stats.loop_p95_ms<<" rss_mb="<<stats.rss_mb<<" dropped="<<dropped
                         <<" free_mb="<<stats.free_mb<<" csv_dropped="<<csvDropped
                         <<" detections_dropped="<<detectionsDropped<<" jpeg_fail="<<jpegFail<<" csv_fail="<<csvFail<<'\n'<<std::flush;
                trace_("record",std::to_string(recordEvery_.load())+" "+std::to_string(dropped.load())+" "+std::to_string(stats.free_mb));
                nextStats=timeMs()+2000;
            }
            if (hooks.writerStall && !quit_) {
                if (!stallNotice) { trace_("writer_stalled",""); stallNotice=true; }
                poll(nullptr,0,10); continue;
            }
            stallNotice=false;
            std::string row; ModelResult image; control::Detections batch;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                if (csv_.empty() && images_.empty() && detections_.empty() && !quit_)
                    available_.wait_for(lock,std::chrono::milliseconds(10));
                if (!csv_.empty()) { row=std::move(csv_.front()); csv_.pop_front(); }
                if (!images_.empty()) { image=std::move(images_.front()); images_.pop_front(); }
                if (!detections_.empty()) { batch=std::move(detections_.front()); detections_.pop_front(); }
                if (quit_ && row.empty() && image.mask.empty() && !batch.frame_id) break;
            }
            if (batch.frame_id) {
                try {
                    std::ofstream f; f.exceptions(std::ios::badbit|std::ios::failbit);
                    f.open(detectionsPath,std::ios::app); f<<std::setprecision(12);
                    if (batch.items.empty()) f<<batch.frame_id<<','<<batch.captured_ms<<",,,,,,\n";
                    for (const auto& d:batch.items) f<<batch.frame_id<<','<<batch.captured_ms<<','<<csvString(d.class_name)<<','
                        <<d.confidence<<','<<d.box.x<<','<<d.box.y<<','<<double(d.box.x)+d.box.width<<','<<double(d.box.y)+d.box.height<<'\n';
                    f.close();
                } catch (...) { ++detectionsDropped; ++csvFail; }
            }
            if (!image.mask.empty() && imagesEnabled) {
                const auto stem=std::to_string(image.frame.id);
                const auto jpg=folder+"/images/"+stem+".jpg", png=folder+"/masks/"+stem+".png";
                try {
                    if (hooks.jpegFail.exchange(false)) throw std::runtime_error("Tiêm lỗi JPEG");
                    if (!cv::imwrite(jpg,image.frame.bgr,{cv::IMWRITE_JPEG_QUALITY,kJpegQuality}) ||
                        !cv::imwrite(png,image.mask)) throw std::runtime_error("Không ghi được ảnh/mask");
                } catch (...) {
                    ::unlink(jpg.c_str()); ::unlink(png.c_str());
                    ++jpegFail; std::cerr<<"Cảnh báo: jpeg_fail="<<jpegFail<<'\n';
                }
            }
            if (!row.empty()) {
                try {
                    if (hooks.csvFail.exchange(false)) throw std::runtime_error("Tiêm lỗi CSV");
                    // Chỉ công bố đường dẫn ảnh đã hoàn tất; lỗi/đầy hàng đợi không phá replay.
                    const auto a=row.find(','),b=row.find(',',a+1),c=row.find(',',b+1);
                    const auto mask=row.substr(b+1,c-b-1);
                    if (!mask.empty() && !regularFile(folder+"/"+mask)) row.erase(b+1,c-b-1);
                    std::ofstream f; f.exceptions(std::ios::badbit|std::ios::failbit);
                    f.open(csvPath,std::ios::app); f<<row; f.close();
                } catch (...) { ++csvFail; std::cerr<<"Cảnh báo: csv_fail="<<csvFail<<'\n'; }
            }
        }
    } catch (const std::exception& e) { std::cerr<<"Cảnh báo: ghi log: "<<e.what()<<'\n'; }
    catch (...) { std::cerr<<"Cảnh báo: ghi log thất bại\n"; }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        finished=true;
        detectionsDropped+=detections_.size(); detections_.clear();
    }
}
// Cột scan_rows (giữ tên cũ): luôn đủ corridor_rows hàng như hợp đồng log cũ; center_frac = target (điểm mục tiêu hàng).
std::string scanJson(const control::ControlDebug& debug,const control::ControlParams& params,int width,int height) {
    std::ostringstream out; out<<'[';
    const auto frac=[&](double v,int size) {
        if (v<0 || size<=0) return std::string("null");
        std::ostringstream s; s<<std::max(0.0,std::min(1.0,v/size)); return s.str();
    };
    const int n=params.corridor_rows;
    for (int i=0;i<n;++i) {
        if (i) out<<',';
        control::RowDebug row=size_t(i)<debug.rows.size() ? debug.rows[i] : control::RowDebug{};
        const double y=params.corridor_bottom_frac+(params.corridor_top_frac-params.corridor_bottom_frac)*i/std::max(1,n-1);
        out<<"{\"y_frac\":"<<(size_t(i)<debug.rows.size() && height>0 ? double(row.y)/height : y)
           <<",\"left_frac\":"<<frac(row.left,width)
           <<",\"right_frac\":"<<frac(row.right,width)<<",\"center_frac\":"<<frac(row.target,width)
           <<",\"conf\":"<<row.conf<<",\"valid\":"<<(row.valid ? "true" : "false")<<'}';
    }
    return out.str()+']';
}
}
