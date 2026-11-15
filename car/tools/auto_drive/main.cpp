#include "vehicle_runtime.h"
#include "model_worker.h"
#include "log_writer.h"
#include "debug_fill.h"
#include "tui_runtime.h"
#include "control/Planner.h"
#include "telemetry_state.h"
#include <iomanip>
#include <locale>
#include <sstream>
#include <opencv2/imgproc.hpp>
#if !defined(UCR_AUTO_TEST_HOOKS) || !UCR_AUTO_TEST_HOOKS
#include "common/TelemetryClient.h"
#endif

namespace auto_drive { HookState hooks; }
namespace {
// Telemetry dashboard BTC (giá trị theo Config.h mẫu BTC; ảnh camera gốc + mã nhãn YOLO, 10 Hz).
// shortcut: hằng số trong mã, chuyển sang control.yaml nếu BTC đổi IP/port.
// IP này chỉ dùng trong mạng LAN tại sân thi, để gửi telemetry tới dashboard phục vụ ban giám khảo chấm thi.
constexpr const char* kTelemetryIp="192.168.1.100";
constexpr int kTelemetryPort=9999;
constexpr const char* kTelemetryCarId="HDQT_No1";
constexpr int kTelemetryPeriodMs=100;
// Quân 13/11: video telemetry vẽ CHỈ hộp YOLO (khung, lớp, conf) trên BẢN SAO; hộp Decode ở hệ 1280x720; chỉ vẽ khi lô hộp đúng khung này.
inline cv::Mat telemetryYoloOverlay(const cv::Mat& image,uint64_t frameId,int64_t capturedMs,const control::Detections& detections) {
    if (image.empty() || capturedMs<0 || detections.frame_id!=frameId ||
        detections.captured_ms!=capturedMs || detections.items.empty()) return image;
    cv::Mat overlay=image.clone();
    const float sx=image.cols/1280.0f,sy=image.rows/720.0f;
    for (const auto& item:detections.items) {
        const auto& b=item.box;
        if (item.captured_ms!=capturedMs || !std::isfinite(item.confidence) || item.confidence<0 ||
            !std::isfinite(b.x) || !std::isfinite(b.y) || !std::isfinite(b.width) || !std::isfinite(b.height) ||
            b.width<=0 || b.height<=0) continue;
        const auto clipped=b & cv::Rect2f(0,0,1280,720);
        if (clipped.empty()) continue;
        const cv::Point first(cvRound(clipped.x*sx),cvRound(clipped.y*sy));
        const cv::Point last(std::min(image.cols-1,cvRound((clipped.x+clipped.width)*sx)),
                             std::min(image.rows-1,cvRound((clipped.y+clipped.height)*sy)));
        // Server nhận ảnh thu còn 480x288 (≈0,375 lần): nét/chữ to để còn đọc được sau khi thu nhỏ.
        cv::rectangle(overlay,first,last,cv::Scalar(0,255,0),5);
        std::ostringstream label;
        label.imbue(std::locale::classic());
        label<<item.class_name<<' '<<std::fixed<<std::setprecision(2)<<item.confidence;
        cv::putText(overlay,label.str(),cv::Point(first.x,std::max(35,first.y-8)),
                    cv::FONT_HERSHEY_SIMPLEX,1.2,cv::Scalar(0,255,0),3,cv::LINE_AA);
    }
    return overlay;
}
struct ControlInput {
    ModelResult model;
    int64_t image_ms=-1;
    bool enabled=false;
    uint64_t epoch=0;
    std::shared_ptr<const control::ControlParams> params;
    YoloResult yolo;
    SensorSample sensor;
};
struct CommandStamped {
    control::Plan plan;
    ModelResult model;
    std::shared_ptr<const control::ControlParams> params;
    uint64_t id=0, epoch=0;
    int64_t created_ms=-1, image_ms=-1;
    double loop_ms=0;
    YoloResult yolo;
    SensorSample sensor;
};
std::string checkModel(const std::string& dir,const std::string& name) {
    const auto base=dir+"/"+name;
    const auto hint="; chạy lại build_engine.sh "+name;
    try {
        for (const char* suffix:{".onnx","_info.json",".engine",".engine.src"})
            if (!regularFile(base+suffix)) throw std::runtime_error("Thiếu file model: "+base+suffix);
        struct stat engineStat{};
        if (stat((base+".engine").c_str(),&engineStat)!=0 || !S_ISREG(engineStat.st_mode) || engineStat.st_size<=0)
            throw std::runtime_error("Engine không phải file thường có dữ liệu: "+base+".engine");
        auto source=readBytes(base+".engine.src");
        if (source.size()==65 && source.back()=='\n') source.pop_back();
        if (source.size()!=64 || source.find_first_not_of("0123456789abcdef")!=std::string::npos)
            throw std::runtime_error("Sai định dạng: "+base+".engine.src");
        const auto hash=sha256(base+".onnx");
        if (hash=="loi" || hash!=source) throw std::runtime_error("ONNX không khớp nguồn engine: "+base);
        return hash;
    } catch (const std::exception& e) { throw Failure(2,std::string(e.what())+hint); }
}
double percentile95(std::vector<double> samples) {
    if (samples.empty()) return 0;
    std::sort(samples.begin(),samples.end());
    return samples[static_cast<size_t>(std::ceil(samples.size()*0.95))-1];
}
bool stopVehicle(DriveMotor& motor,DriveServo& servo) noexcept {
    bool motorOk=false,servoOk=false;
    try { motorOk=motor.SetSpeedCms(0); } catch (...) {}
    if (!motorOk) motorOk=motor.EmergencyStop();
    try { servoOk=servo.SetAngle(0); } catch (...) {}
    return motorOk && servoOk;
}
std::string csvRow(const CommandStamped& command,const ModelResult& model,
                   const control::ControlParams& params,int64_t now,int64_t imageMs,
                   double sentSpeed,int sentAngle,const std::string& state,const std::string& reason,
                   bool go,bool stop,const std::string& maskFile,const Statistics& stats,const LogWriter& writer) {
    std::ostringstream out; out<<std::setprecision(12);
    out<<now<<','<<model.frame.id<<','<<maskFile<<','<<sentSpeed<<','<<sentAngle<<',';
    if (model.frame.captured_ms>=0) out<<model.frame.captured_ms;
    out<<',';
    if (imageMs>=0) out<<imageMs;
    out<<','<<go<<','<<stop<<",RIGHT,"<<state<<','
       <<(imageMs<0 ? -1 : now-imageMs)<<','
       <<(model.frame.captured_ms<0 ? -1 : now-model.frame.captured_ms)<<','
       <<(command.yolo.detections.captured_ms<0 ? -1 : now-command.yolo.detections.captured_ms)<<','
       <<csvString(scanJson(command.plan.debug,params,model.mask.cols,model.mask.rows))<<','
       <<command.plan.debug.e<<','<<command.plan.debug.p<<','<<command.plan.debug.d<<','
       <<command.plan.cmd.deg<<','<<command.plan.cmd.cms<<','<<sentAngle<<','<<sentSpeed<<','
       <<csvString(reason)<<','<<model.road_ms<<','<<command.yolo.yolo_ms<<','<<command.loop_ms<<','
       <<writer.dropped<<','<<writer.csvDropped<<','<<stats.camera_fps<<','<<stats.loop_p95_ms
       <<','<<stats.rss_mb<<','<<writer.freeMb<<','<<command.id<<',';
    if (command.created_ms>=0) out<<command.created_ms;
    out<<','<<model.frame.id<<','<<command.yolo.detections.frame_id<<',';
    if (command.yolo.detections.captured_ms>=0) out<<command.yolo.detections.captured_ms;
    out<<','<<writer.detectionsDropped<<',';
    const bool measured=params.turn_enable;
    if (measured && std::isfinite(command.sensor.yaw_deg)) out<<command.sensor.yaw_deg;
    out<<',';
    if (measured && std::isfinite(command.sensor.odo_cm)) out<<command.sensor.odo_cm;
    // Cột chẩn đoán: bước lộ trình, ghi chú, hướng rẽ, chế độ quỹ đạo, điểm nhìn trước (pixel) và độ cong fit[2].
    const auto& dbg=command.plan.debug;
    out<<','<<dbg.route_step<<','<<csvString(dbg.route_note)<<','<<dbg.turn_dir<<','<<csvString(dbg.mode)<<','
       <<dbg.look_x<<','<<dbg.look_y<<',';
    if (dbg.fit.size()>=3) out<<dbg.fit[2];
    // Cột cua IMU: đang rẽ, yaw bắt đầu, yaw đích, sai lệch còn lại, lý do kết thúc/hủy gần nhất.
    out<<','<<dbg.turn_active<<','<<dbg.turn_yaw0<<','<<dbg.turn_yaw_tgt<<','<<dbg.turn_yaw_err<<','<<csvString(dbg.turn_end);
    out<<'\n';
    return out.str();
}

int driveAuto(const Options& options,const Paths& paths,int lockFd,
              std::shared_ptr<const control::ControlParams> params,const std::string& yaml,
              const perception::InfoJson& info,std::unique_ptr<perception::IInferEngine>& engine,
              const perception::YoloInfo* yoloInfo,std::unique_ptr<perception::IInferEngine>& yoloEngine) {
    DriveMotor motor(kReverseMotor); DriveServo servo;
    DriveImu imu;
    // Quân 13/11: TẮT HẲN ToF trong ucr_auto (đỗ ô cố định sau biển P): không khởi tạo, không đọc, không in; ô ToF gửi Planner luôn "chưa khởi tạo".
    terminateMotor=&motor; terminateServo=&servo;
    std::unique_ptr<Camera> camera;
    drive::PreviewOptions previewOptions;
    previewOptions.debug_windows=options.debugWindows;
    drive::PreviewHost preview(previewOptions);
    std::unique_ptr<Watchdog> watchdog;
    std::unique_ptr<LogWriter> writer;
    TuiRuntime tui;  // P5b: chỉ bật khi stdout/stderr là terminal; dừng SAU stopVehicle
    std::atomic<bool> quit{false},cameraDone{true},modelDone{true},controllerDone{true},telemetryDone{true};
    std::atomic<bool> controllerFault{false};
    std::atomic<int> cameraError{0};
    std::atomic<int64_t> imageMs{-1};
    std::atomic<uint64_t> frameCount{0};
    Mailbox<Frame> frames;
    Mailbox<ModelResult> models;
    Mailbox<YoloResult> detections;
    Mailbox<Frame> yoloFrames;  // khung YOLO vừa chạy (chung dữ liệu, không chép ảnh), chỉ để telemetry vẽ hộp đúng khung
    std::atomic<int> yoloEvery{params->yolo_every_n};
    Mailbox<ControlInput> inputs;
    Mailbox<CommandStamped> commands;
    static_assert(std::atomic<uint64_t>::is_always_lock_free,"Lenh telemetry phai khong khoa");
    std::atomic<uint64_t> sentTelemetry{0};
    std::atomic<bool> telemetryPlanActive{false};
    std::thread cameraThread,modelThread,controllerThread,telemetryThread;
    int code=0; std::string error;
    try {
        // Camera chỉ fork, chưa mở thiết bị; preview cũng fork trước mọi luồng điều khiển.
        camera.reset(new Camera(lockFd));
        if (preview.start()!=drive::PreviewStartResult::Started)
            throw Failure(1,"Không chạy được cửa sổ ảnh: "+preview.diagnostic());
#if !defined(UCR_AUTO_TEST_HOOKS) || !UCR_AUTO_TEST_HOOKS
        // CUDA có thể tạo luồng nội bộ: nạp sau các fork, vẫn trước Motor::Init.
        engine.reset(new perception::TrtEngine(options.engine,&info));
        if (yoloInfo) {
            validateYoloEngine(options.yoloEngine,*yoloInfo);
            yoloEngine.reset(new perception::TrtEngine(options.yoloEngine));
            std::cout<<"YOLO: nap OK "<<options.yoloEngine<<' '<<yoloInfo->class_names.size()<<" lop\n"<<std::flush;
        }
#endif
        if (stopping) throw SignalStop{};
        InitSignalMask initMask;
        watchdog.reset(new Watchdog(motor,servo));
        if (params->turn_enable) {
            try { imu.init(paths); }
            catch (const std::exception& e) {
                std::cerr<<"IMU LOI: KHONG RE\n"<<e.what()<<'\n';
                trace("imu_error",e.what());
            }
        }
        trace("state","CHO_SAN_SANG");
        if (!motor.init()) throw Failure(1,"Motor::Init thất bại");
        if (!motor.SetSpeedCms(0)) throw Failure(1,"Không xác nhận motor 0 sau Init");
        if (!servo.open()) throw Failure(1,"Không mở được servo/PWM");
        if (!servo.SetAngle(0)) throw Failure(1,"Không trả được servo 0 sau motor 0");
        // Các luồng con thừa hưởng chặn tín hiệu; chỉ vòng an toàn nhận tín hiệu dừng.
        writer.reset(new LogWriter(options,yaml,trace));
        if (params->turn_enable && !imu.initialized)
            writer->offerControl(csvRow(CommandStamped{},ModelResult{},*params,nowMs(),-1,0,0,
                "CHO_SAN_SANG","LOI_CAM_BIEN",false,false,"",Statistics{},*writer));
        cameraDone=false;
        cameraThread=std::thread([&] {
            try {
                uint64_t id=0;
                while (!quit) {
                    cv::Mat bgr; Header h;
                    if (!camera->read(bgr,h,[&] { return !quit.load(); })) break;
                    if (h.status!=1 || bgr.empty()) continue;
                    if (hooks.cameraStall || nowMs()<hooks.cameraPauseUntil) continue;
                    const auto captured=h.ns/1000000;
                    if (nowMs()-captured>kStaleFrameMs) continue;
                    Frame frame{bgr,++id,captured};
                    trace("frame",std::to_string(id)+" "+std::to_string(captured));
                    imageMs=captured; ++frameCount;
                    frames.offer(frame); preview.publishFrame(bgr,captured);
                }
            } catch (...) { cameraError=1; }
            cameraDone=true;
        });
        modelDone=false;
        modelThread=std::thread([&] {
            try {
                perception::RoadSegmenter segmenter(info);
                uint64_t last=0,processed=0;
                std::unique_ptr<perception::YoloDetector> detector;
                if (yoloInfo) detector.reset(new perception::YoloDetector(*yoloInfo));
                while (!quit) {
                    if (hooks.roadStall) { ::poll(nullptr,0,2); continue; }
                    Frame frame;
                    if (!frames.latest(frame) || !frame.id || frame.id==last) { ::poll(nullptr,0,2); continue; }
                    last=frame.id;
                    try {
                        if (hooks.roadThrow.exchange(false)) throw std::runtime_error("Tiêm lỗi road");
#if defined(UCR_AUTO_TEST_HOOKS) && UCR_AUTO_TEST_HOOKS
                        static_cast<FakeRoadEngine&>(*engine).select(frame.id);
#endif
                        const auto start=Clock::now();
                        auto mask=segmenter.Infer(frame.bgr,*engine);
                        const double ms=std::chrono::duration<double,std::milli>(Clock::now()-start).count();
                        trace("mask",std::to_string(frame.id)+" "+std::to_string(frame.captured_ms));
                        models.offer(ModelResult{frame,mask,ms});
                        if (detector && ++processed%yoloEvery.load()==0) {
                            while (!quit && hooks.yoloStall) ::poll(nullptr,0,2);
                            if (quit) break;
                            try {
                                if (hooks.yoloThrow.exchange(false)) throw std::runtime_error("Tiêm lỗi YOLO");
#if defined(UCR_AUTO_TEST_HOOKS) && UCR_AUTO_TEST_HOOKS
                                if (hooks.yoloThrowOn.load()) throw std::runtime_error("Tiêm lỗi YOLO liên tục");
                                static_cast<FakeYoloEngine&>(*yoloEngine).select(frame.id);
#endif
                                const auto yoloStart=Clock::now();
                                const auto found=detector->Infer(frame.bgr,*yoloEngine);
                                YoloResult result;
                                result.detections.frame_id=frame.id; result.detections.captured_ms=frame.captured_ms;
                                for (const auto& d:found) result.detections.items.push_back({d.class_name,d.box,d.confidence,frame.captured_ms});
                                result.yolo_ms=std::chrono::duration<double,std::milli>(Clock::now()-yoloStart).count();
                                yoloFrames.offer(frame);  // khung trước hộp: telemetry ít gặp hộp mới với khung cũ
                                if (detections.offer(result)) {
                                    writer->offerDetections(result.detections);
                                    trace("detections",std::to_string(frame.id)+" "+std::to_string(frame.captured_ms));
                                }
                            } catch (...) { trace("model_notice","yolo_throw"); }
                        }
                    } catch (...) { trace("model_notice","road_throw"); }
                }
            } catch (...) {
                warningNonBlocking("CANH BAO: luong model da dung, xe dung, thoat bang Esc\n");
                trace("model_notice","road_worker_failed");
            }
            modelDone=true;
        });
        controllerDone=false;
        controllerThread=std::thread([&] {
            try {
                control::Planner planner;
                std::string lastParkNote;
                ControlInput input;
                uint64_t epoch=~uint64_t(0),id=0;
                bool goPending=false;
                auto previous=Clock::now();
                while (!quit) {
                    if (hooks.controllerStall) { ::poll(nullptr,0,2); continue; }
                    inputs.latest(input);
                    if (!input.params) { ::poll(nullptr,0,2); continue; }
                    const auto start=Clock::now();
                    const double interval=std::chrono::duration<double,std::milli>(start-previous).count();
                    previous=start;
                    try {
                        if (hooks.controllerThrow.exchange(false)) throw std::runtime_error("Tiêm lỗi controller");
                        control::PlannerInput pi;
                        pi.mask=input.model.mask; pi.mask_ms=input.model.frame.captured_ms;
                        pi.image_ms=input.image_ms;
                        pi.detections=input.yolo.detections; pi.yolo_enabled=!options.noYolo;
                        pi.yaw_deg=input.sensor.yaw_deg; pi.odo_cm=input.sensor.odo_cm;
                        pi.sensor_ok=input.sensor.sensor_ok; pi.sensor_ms=input.sensor.sensor_ms;
                        for (int i=0;i<4;++i) pi.tof[i]=input.sensor.tof[i];
                        if (epoch!=input.epoch) {
                            planner.Reset(!options.noYolo); epoch=input.epoch; goPending=input.enabled;
                        }
                        pi.go=goPending && input.enabled;
                        pi.stop=!input.enabled;
                        const auto now=nowMs();
                        pi.encoder_ok=std::isfinite(pi.odo_cm) && pi.sensor_ms>=0 &&
                            pi.sensor_ms<=now && now-pi.sensor_ms<=200;
                        if (hooks.plannerThrow.exchange(false)) throw std::runtime_error("Tiêm lỗi ngay trước Planner::Step");
                        auto plan=planner.Step(pi,now,*input.params);
                        if (planner.ParkNote()!=lastParkNote) {  // log quá trình đỗ: biển P, quét ToF, chọn ô, quyết định
                            lastParkNote=planner.ParkNote();
                            if (!lastParkNote.empty()) { std::cout<<"PARK: "<<lastParkNote<<std::endl; trace("park",lastParkNote); }
                        }
                        plan.debug.route_step=planner.RouteStep(); plan.debug.route_note=planner.RouteNote();
                        plan.debug.turn_dir=planner.TurnDirection();  // chỉ để ghi log chẩn đoán
                        {
                            const auto turn=planner.Turn();
                            plan.debug.turn_active=turn.active; plan.debug.turn_yaw0=turn.yaw0;
                            plan.debug.turn_yaw_tgt=turn.target; plan.debug.turn_yaw_err=turn.err; plan.debug.turn_end=turn.end;
                        }
                        if (plan.state==control::PlannerState::BAM_LAN || plan.state==control::PlannerState::CHO_DEN_XUAT_PHAT ||
                            plan.state==control::PlannerState::DUNG_DEN_DO ||
                            plan.state==control::PlannerState::DUNG_VAT_CAN || plan.state==control::PlannerState::GIU_HUONG ||
                            (input.params->turn_enable && (plan.state==control::PlannerState::RE_TRAI ||
                             plan.state==control::PlannerState::RE_PHAI ||
                             (plan.state==control::PlannerState::DUNG && plan.reason!="SPACE"))) || !input.enabled) goPending=false;
                        if (input.enabled && hooks.overridePending.load()) {
                            plan.cmd={hooks.overrideCms.load(),hooks.overrideDeg.load()};
                            hooks.overridePending=false;
                        }
                        const double loop=std::max(interval,std::chrono::duration<double,std::milli>(Clock::now()-start).count());
                        CommandStamped command{plan,input.model,input.params,++id,epoch,now,input.image_ms,loop,input.yolo,input.sensor};
                        if (commands.offer(command)) trace("command",std::to_string(id)+" "+std::to_string(now));
                    } catch (...) { controllerFault=true; }
                    const auto target=start+std::chrono::milliseconds(input.params->control_period_ms+hooks.controlDelay.load());
                    // Ngủ từng lát để dọn luồng có hạn, không ảnh hưởng vòng an toàn.
                    while (!quit && Clock::now()<target) ::poll(nullptr,0,2);
                }
            } catch (...) {
                controllerFault=true;
                warningNonBlocking("CANH BAO: luong dieu khien da dung, xe dung, thoat bang Esc\n");
            }
            controllerDone=true;
        });
#if !defined(UCR_AUTO_TEST_HOOKS) || !UCR_AUTO_TEST_HOOKS
        // Chỉ đọc khung mới nhất (chép header cv::Mat), gửi UDP ngoài khóa; lỗi mạng bỏ qua, không đụng điều khiển.
        telemetryDone=false;
        telemetryThread=std::thread([&] {
            try {
                common::TelemetryClient client(kTelemetryIp,kTelemetryPort,kTelemetryCarId);
                const bool ready=client.Init();
                CommandStamped latestCommand;
                YoloResult yolo;
                while (ready && !quit) {
                    const auto next=Clock::now()+std::chrono::milliseconds(kTelemetryPeriodMs);
                    Frame frame;
                    if (frames.latest(frame) && !frame.bgr.empty()) {
                        try {
                            // Chỉ đọc hộp thư có sẵn; nén/gửi ảnh nằm ngoài khóa và ngoài vòng an toàn.
                            detections.latest(yolo); commands.latest(latestCommand);
                            const auto sent=unpackTelemetryCommand(sentTelemetry.load(std::memory_order_relaxed));
                            const auto plan=telemetryPlanActive.load(std::memory_order_relaxed)
                                ? latestCommand.plan : control::Plan{};
                            const auto states=telemetryStates(yolo.detections,frame.captured_ms,plan,sent);
                            // Khung YOLO mới (≤ 500 ms so với khung camera mới nhất): gửi CHÍNH khung đó kèm hộp của nó; không thì khung mới nhất, không hộp.
                            // Hộp chỉ vẽ khi khung YOLO trùng id + thời điểm với lô hộp (telemetryYoloOverlay tự kiểm).
                            Frame yoloFrame;
                            const bool yoloFresh=yoloFrames.latest(yoloFrame) && !yoloFrame.bgr.empty() && yoloFrame.captured_ms>=0 &&
                                frame.captured_ms-yoloFrame.captured_ms<=500;
                            cv::Mat video=yoloFresh
                                ? telemetryYoloOverlay(yoloFrame.bgr,yoloFrame.id,yoloFrame.captured_ms,yolo.detections)
                                : frame.bgr;
                            client.SendFrameAndTelemetry(video,static_cast<float>(sent.cms),int(sent.deg),states);
                        } catch (...) {}
                    }
                    while (!quit && Clock::now()<next) ::poll(nullptr,0,2);
                }
            } catch (...) {}
            telemetryDone=true;
        });
#endif
        initMask.restore();
        if (stopping) throw SignalStop{};
        watchdog->heartbeat=nowMs(); watchdog->initialized=true;
        const auto boot=nowMs();
        bool enabled=false,ready=false,exit=false;
        control::TofSlot lastTof[4];
        double safetyPeakMs=0;
        auto safetyPrevious=Clock::now();
        std::string latch,state="CHO_SAN_SANG",reason="CHO_G",lastState,lastReason;
        uint64_t epoch=0,lastLog=0,lastImage=0,lastFrames=0;
        int every=params->record_every_n;
        int64_t nextStats=boot+2000,statsStart=boot,highSince=0,lastAdapt=boot,lastRowMs=boot;
        Statistics stats;
        std::deque<std::pair<int64_t,double>> timings;
        ModelResult model;
        YoloResult yolo;
        CommandStamped command;
        SensorSample sensor;
        SensorSample plannerSensor;
        uint64_t sensorFaultId=0;
        bool sensorFaultPending=false;
        // --debug-windows: bộ đệm cấp một lần; vòng này chỉ chép sang bộ nhớ chung, vẽ ở tiến trình preview.
        std::unique_ptr<drive::PreviewDebug> debugData;
        if (options.debugWindows) debugData.reset(new drive::PreviewDebug());
        uint64_t debugFrame=0,debugCommand=0;
        int64_t logT0=-1;  // t_ms dòng đầu control_log.csv: gốc "s=" trên overlay
        int64_t nextTui=0;
        {
            TuiState initial;
            initial.git=UCR_AUTO_GIT_HASH;
            const auto slash=options.modelDir.find_last_of('/');
            initial.model=slash==std::string::npos ? options.modelDir : options.modelDir.substr(slash+1);
            tui.start(initial);
        }
        while (!exit && !stopping) {
            const auto safetyStart=Clock::now();
            const double safetyInterval=std::chrono::duration<double,std::milli>(safetyStart-safetyPrevious).count();
            safetyPrevious=safetyStart;
            auto now=nowMs(); watchdog->heartbeat=now;
            try { pollHooks(now,trace); }
            catch (...) { trace("hook_error","CTRL invalide"); throw; }
            const auto child=preview.pollChild();
            if (child==drive::PreviewChildStatus::Closed || child==drive::PreviewChildStatus::Died)
                throw Failure(1,"Cửa sổ chính đã đóng/chết");
            models.latest(model); detections.latest(yolo); commands.latest(command);
            const auto image=imageMs.load();
            if (cameraError) throw Failure(1,"Camera đã lỗi");
            if (now-boot>kFrameTimeoutMs && (image<0 || model.mask.empty() || !preview.wasShown()))
                throw Failure(1,"Quá hạn ảnh/mask/preview đầu tiên");
            bool eventGo=false,eventStop=false,forceLog=false;
            bool imageFresh=image>=0 && now-image<=kStaleFrameMs;
            bool maskFresh=!model.mask.empty() && now-model.frame.captured_ms<=params->mask_max_age_ms;
            if (enabled && !imageFresh) { enabled=false; latch=control::kMissingImageReason; ++epoch; eventStop=true; forceLog=true; }
            if (controllerFault.exchange(false)) { enabled=false; latch="LOI_DIEU_KHIEN"; ++epoch; eventStop=true; forceLog=true; }
            const auto keys=preview.pollKeys();
            const bool batchStop=keys.find(' ')!=std::string::npos;
            for (unsigned char key:keys) {
                if (key==27) key='E';
                if (key==3) key='3';
                trace("key",std::string(1,key));
                if (key=='E' || key=='3') { exit=true; eventStop=true; break; }
                if (key==' ') {
                    enabled=false; latch.clear(); reason="SPACE"; ++epoch; eventStop=true; forceLog=true;
                    if (!stopVehicle(motor,servo)) throw Failure(1,"Lỗi dừng Space");
                    sentTelemetry.store(0,std::memory_order_relaxed); telemetryPlanActive=false;
                } else if (key=='g' || key=='G') {
                    eventGo=true; forceLog=true;
                    if (batchStop) { eventGo=false; trace("go","ignored SPACE"); }
                    else if (!imageFresh || !maskFresh) trace("go",imageFresh ? "ignored MAT_NAN_CU" : "ignored MAT_ANH");
                    else if (!enabled) { enabled=true; latch.clear(); reason="BAM_LAN"; ++epoch; }
                } else if (key=='r' || key=='R') {
                    if (enabled) trace("reload","ignored");
                    else {
                        try {
                            // Toàn bộ bộ tham số bất biến được thay một lần sau khi kiểm đủ.
                            std::ostringstream table;
                            auto replacement=std::make_shared<control::ControlParams>(control::ControlParams::Load(options.control,table));
                            if (replacement->turn_enable && !imu.initialized) {
                                replacement->turn_enable=0;
                                table.str(""); replacement->PrintTable(table);
                                std::cerr<<"Không bật rẽ bằng R: IMU chưa khởi tạo; cần khởi động lại chương trình\n";
                            }
                            if (yoloInfo && yoloInfo->conf_threshold>float(std::min({replacement->red_conf_min,replacement->green_conf_min,replacement->off_conf_min})))
                                throw std::runtime_error("conf_threshold YOLO cao hơn conf_min đèn");
                            params=replacement; every=params->record_every_n; yoloEvery=params->yolo_every_n; ++epoch;
                            std::cout<<table.str()<<std::flush; trace("reload","accepted");
                        } catch (const std::exception& e) { std::cerr<<"Lỗi nạp control.yaml: "<<e.what()<<'\n'; trace("reload","error"); }
                        catch (...) { trace("reload","error"); }
                    }
                }
            }
            if (exit || stopping) break;
            if (params->turn_enable) {
                if (sensorFaultPending && command.epoch==epoch && command.sensor.fault_id==plannerSensor.fault_id)
                    sensorFaultPending=false;
                if (eventStop) {
                    if (!stopVehicle(motor,servo)) throw Failure(1,"Lỗi dừng trước đọc cảm biến");
                    sentTelemetry.store(0,std::memory_order_relaxed); telemetryPlanActive=false;
                }
                sensor=SensorSample{};
                for (int i=0;i<4;++i) sensor.tof[i]=lastTof[i];
                const bool imuOk=imu.read(sensor);
                if (!imuOk) sensor.yaw_deg=std::numeric_limits<double>::quiet_NaN();
                float cm=0;
                const bool encoderOk=motor.GetPositionCm(&cm);
                if (encoderOk) sensor.odo_cm=cm;
                // IMU lỗi khởi tạo không ghi giờ mẫu: dùng giờ đọc encoder để Planner xét encoder_ok (sensor_ok vẫn sai).
                if (encoderOk && sensor.sensor_ms<0) sensor.sensor_ms=nowMs();
                sensor.sensor_ok=imuOk && encoderOk && std::isfinite(sensor.yaw_deg) && std::isfinite(sensor.odo_cm);
                // Giữ nguyên mẫu lỗi tới khi vòng an toàn đã nhận lệnh Planner xử lý đúng mẫu đó.
                if (!sensorFaultPending) {
                    if (!sensor.sensor_ok) { sensor.fault_id=++sensorFaultId; sensorFaultPending=true; }
                    plannerSensor=sensor;
                }
                // Đọc UART/I2C có thể chậm: kiểm tuổi lệnh/ảnh lại theo giờ sau đọc.
                now=nowMs();
                imageFresh=image>=0 && now-image<=kStaleFrameMs;
                maskFresh=!model.mask.empty() && now-model.frame.captured_ms<=params->mask_max_age_ms;
                if (enabled && !imageFresh) {
                    enabled=false; latch=control::kMissingImageReason; ++epoch; eventStop=true; forceLog=true;
                }
            } else { sensor=plannerSensor=SensorSample{}; sensorFaultPending=false; }
            inputs.offer(ControlInput{model,image,enabled,epoch,params,yolo,plannerSensor});
            double speed=0; int angle=0; bool parkSteer=false;
            const bool commandMaskFresh=!command.model.mask.empty() &&
                now-command.model.frame.captured_ms<=params->mask_max_age_ms;
            const bool commandFresh=command.created_ms>=0 && command.epoch==epoch &&
                now-command.created_ms<=params->cmd_max_age_ms;
            if (!latch.empty()) { state="DUNG"; reason=latch; }
            else if (!enabled) {
                state=reason=="SPACE" ? "DUNG" : "CHO_SAN_SANG";
                if (reason!="SPACE") reason="CHO_G";
            } else if (!maskFresh || !commandMaskFresh) { state="BAM_LAN"; reason="MAT_NAN_CU"; }
            else if (!commandFresh) { state="BAM_LAN"; reason="LENH_CU"; }
            else if (command.plan.state==control::PlannerState::DUNG && command.plan.reason==control::kMissingImageReason) {
                // Hai đồng hồ kiểm 500 ms có thể lệch nhau, nên phải chốt cả mất ảnh do Planner báo.
                enabled=false; latch=control::kMissingImageReason; ++epoch; eventStop=true; forceLog=true;
                state="DUNG"; reason=latch;
            }
            else {
                state=command.plan.debug.state; reason=command.plan.reason;
                speed=command.plan.cmd.cms;
                if (!std::isfinite(speed) || !std::isfinite(command.plan.cmd.deg)) {
                    enabled=false; latch="LOI_DIEU_KHIEN"; ++epoch;
                    speed=0; state="DUNG"; reason=latch;
                } else {
                    // Trần 45° chỉ khi lệnh hiện hành là pha đỗ (DO_XE); mọi trạng thái khác giữ 40°.
                    parkSteer=command.plan.state==control::PlannerState::DO_XE;
                    const double m=parkSteer ? kMaxParkSteerDeg : kMaxAutoSteerDeg;
                    angle=int(std::lround(std::max(-m,std::min(m,command.plan.cmd.deg))));
                }
            }
            if (stopping) break;
            // Kẹp độc lập YAML, ngay tại tầng gửi; motor luôn trước servo trên đường dừng.
            speed=clampAutoSpeed(static_cast<float>(speed)); angle=clampAutoSteer(angle,parkSteer);
            if (speed==0 && angle==0) {
                if (!stopVehicle(motor,servo)) throw Failure(1,"Không xác nhận được dừng");
            } else {
                if (!motor.SetSpeedCms(static_cast<float>(speed))) throw Failure(1,"Lỗi gửi motor");
                if (!servo.SetAngle(angle,parkSteer)) throw Failure(1,"Lỗi gửi servo");
            }
            sentTelemetry.store(packTelemetryCommand(static_cast<float>(speed),angle),std::memory_order_relaxed);
            telemetryPlanActive=enabled && commandFresh && commandMaskFresh && maskFresh && latch.empty();
            if (state!=lastState) { trace("state",state); lastState=state; }
            if (reason!=lastReason) { trace("reason",reason); lastReason=reason; }
            if (!ready && imageFresh && maskFresh && preview.wasShown() && writer->ready) {
                ready=true; trace("ready");
            }
            const bool newCommand=command.id && command.id!=lastLog;
            if (params->turn_enable) {
                safetyPeakMs=std::max({safetyPeakMs,safetyInterval,
                    std::chrono::duration<double,std::milli>(Clock::now()-safetyStart).count()});
                command.loop_ms=std::max(command.loop_ms,safetyPeakMs);
            }
            if (newCommand) {
                timings.emplace_back(now,command.loop_ms);
                safetyPeakMs=0;
                while (!timings.empty() && now-timings.front().first>5000) timings.pop_front();
                std::vector<double> values;
                for (const auto& t:timings) values.push_back(t.second);
                stats.loop_p95_ms=percentile95(values);
                if (stats.loop_p95_ms>2*params->control_period_ms) {
                    if (!highSince) highSince=now;
                    if (every>0 && now-highSince>=5000 && now-lastAdapt>=5000 && every<40) {
                        every=std::min(40,every*2); lastAdapt=now; highSince=now;
                        writer->offerStatistics(stats,every,true);
                    }
                } else highSince=0;
            }
            if (model.frame.id && model.frame.id!=lastImage) {
                lastImage=model.frame.id;
                // Chỉ ghi ảnh/mặt nạ khi đã bấm G (đang chạy); chờ G hoặc sau Space chỉ ghi CSV.
                if (every>0 && enabled && model.frame.id%every==0) writer->tryImage(model);
            }
            if (newCommand || forceLog) {
                const bool applied=commandFresh && enabled && maskFresh && commandMaskFresh && latch.empty() && !forceLog;
                const auto& decision=applied ? command.model : model;
                const auto& decisionParams=applied ? *command.params : *params;
                const auto decisionTime=applied ? std::max(lastRowMs,command.created_ms) : now;
                const auto decisionImage=applied ? command.image_ms : image;
                const auto maskFile=decision.frame.id ? "masks/"+std::to_string(decision.frame.id)+".png" : "";
                try {
                    writer->offerControl(csvRow(command,decision,decisionParams,decisionTime,decisionImage,speed,angle,state,reason,
                                                 eventGo,eventStop,maskFile,stats,*writer));
                } catch (...) { ++writer->csvDropped; }
                lastRowMs=decisionTime;
                if (logT0<0) logT0=decisionTime;
                lastLog=command.id;
            }
            if (now>=nextStats) {
                const auto count=frameCount.load();
                stats.camera_fps=double(count-lastFrames)*1000/std::max<int64_t>(1,now-statsStart);
                stats.road_ms=model.road_ms; stats.yolo_ms=yolo.yolo_ms; stats.free_mb=writer->freeMb; stats.rss_mb=writer->rssMb;
                lastFrames=count; statsStart=now; nextStats=now+2000;
                writer->offerStatistics(stats,every);
            }
            drive::PreviewState ps;
            std::snprintf(ps.state,sizeof(ps.state),"%s",state.c_str());
            ps.cmd_speed=speed; ps.cmd_steer=angle; ps.frame_age_ms=image<0 ? -1 : now-image;
            ps.images=frameCount; ps.recording=every>0 && enabled && writer->imagesEnabled;
            preview.publishState(ps);
            if (tui.active() && now>=nextTui) {  // <= 10 lần/giây, chỉ chép số dưới khóa; vẽ ở luồng TUI
                nextTui=now+100;
                const auto& dbg=command.plan.debug;
                tui.update([&](TuiState& t) {
                    t.mode=enabled ? "DANG CHAY" : state=="DUNG" ? "DUNG" : "CHO G";
                    t.state=state; t.reason=reason;
                    t.run_s=(now-boot)/1000.0; t.rec_s=logT0>=0 ? (now-logT0)/1000.0 : -1;
                    t.recording=ps.recording; t.cms=speed; t.deg=angle; t.e=dbg.e;
                    t.sensor_ok=command.sensor.sensor_ok && std::isfinite(command.sensor.yaw_deg);
                    t.yaw=std::isfinite(command.sensor.yaw_deg) ? command.sensor.yaw_deg : 0;
                    t.odo=std::isfinite(command.sensor.odo_cm) ? command.sensor.odo_cm : 0;
                    t.sensor_age_ms=command.sensor.sensor_ms>=0 ? int(now-command.sensor.sensor_ms) : -1;
                    t.turning=dbg.turn_active!=0; t.yaw_tgt=dbg.turn_yaw_tgt; t.yaw_err=dbg.turn_yaw_err;
                    const double span=dbg.turn_yaw_tgt-dbg.turn_yaw0;
                    t.progress_pct=std::abs(span)>1e-3 ? int(std::lround(100.0*(1.0-dbg.turn_yaw_err/span))) : 0;
                    t.route_step=dbg.route_step; t.turn_dir=dbg.turn_dir;
                    t.frame_age_ms=image<0 ? -1 : int(now-image);
                    t.camera_fps=stats.camera_fps; t.road_ms=model.road_ms; t.loop_p95_ms=stats.loop_p95_ms;
                    t.tof.clear();
                });
            }
            if (debugData) {
                const bool withRows=command.params && !command.plan.debug.rows.empty();
                const auto& shown=withRows ? command.model : model;
                if (!shown.mask.empty() && (shown.frame.id!=debugFrame || command.id!=debugCommand)) {
                    debugFrame=shown.frame.id; debugCommand=command.id;
                    auto fill=[&](const control::ControlDebug* plan,double offset) {
                        fillPreviewDebug(*debugData,shown.mask,shown.frame.captured_ms,shown.frame.id,plan,offset,info.source,info.crop);
                        fillPreviewBoxes(*debugData,yolo.detections,now,params->det_max_age_ms,!options.noYolo);
                        fillPreviewTrace(*debugData,plan,command.sensor.yaw_deg,command.sensor.odo_cm,command.sensor.sensor_ok,
                                         command.sensor.sensor_ms,command.created_ms,logT0,ps.recording);
                        preview.publishDebug(*debugData);
                    };
                    if (withRows) fill(&command.plan.debug,command.params->center_offset_frac);
                    else fill(nullptr,params->center_offset_frac);
                    trace("debug",std::to_string(shown.frame.id)+" "+std::to_string(debugData->nrows));
                }
            }
            ::poll(nullptr,0,kPollMs);
        }
    } catch (const SignalStop&) {}
    catch (const Failure& e) { code=e.code; error=e.what(); }
    catch (const std::exception& e) { code=1; error=e.what(); }
    catch (...) { code=1; error="Ngoại lệ vòng an toàn"; }

    if (stopping) code=0;
    if (watchdog) { watchdog->cleanupCode=code; watchdog->cleanupAt=nowMs(); }
    // Dừng trước mọi join, ghi đĩa, đóng preview.
    const bool stopped=stopVehicle(motor,servo);
    if (stopped) { sentTelemetry.store(0,std::memory_order_relaxed); telemetryPlanActive=false; }
    tui.stop();  // sau lệnh dừng: trả terminal; cảnh báo in SAU để khung cuối không xóa mất
    if (!stopped) { if (!code) code=1; warning(); }
    if (writer) {
        try {
            writer->offerControl(csvRow(CommandStamped{},ModelResult{},*params,nowMs(),imageMs.load(),0,0,
                "DUNG",stopping ? "TIN_HIEU" : code ? "LOI" : "THOAT",false,true,"",Statistics{},*writer));
        } catch (...) {}
    }
    quit=true;
    if (writer) writer->requestStop();
    const auto deadline=nowMs()+kCleanupTimeoutMs;
    while ((!cameraDone || !modelDone || !controllerDone || !telemetryDone || (writer && !writer->finished)) && nowMs()<deadline)
        ::poll(nullptr,0,kPollMs);
    if (!cameraDone || !modelDone || !controllerDone || !telemetryDone || (writer && !writer->finished)) {
        // Không hủy bộ nhớ mà luồng bị treo còn dùng; giữ khóa tới khi thoát.
        warning(); _exit(code ? code : stopping ? 0 : 4);
    }
    if (cameraThread.joinable()) cameraThread.join();
    if (modelThread.joinable()) modelThread.join();
    if (controllerThread.joinable()) controllerThread.join();
    if (telemetryThread.joinable()) telemetryThread.join();
    preview.shutdown(); camera.reset(); writer.reset(); watchdog.reset();
    terminateMotor=nullptr; terminateServo=nullptr;
    if (!error.empty()) std::cerr<<"Lỗi: "<<error<<'\n';
    return code;
}
}

int main(int argc,char** argv) {
    cv::setNumThreads(1);
    std::set_terminate(emergencyTerminate);
    Options options;
    try { options=parse(argc,argv); }
    catch (const std::exception& e) { std::cerr<<e.what()<<'\n'; return 2; }
    if (options.help) {
        std::cout<<"ucr_auto [--control control.yaml] [--model-dir $HOME/ucr_models] [--no-yolo] [--debug-windows] [--out-root runs_auto]\n"
                 <<"G chạy; Space dừng; R nạp khi dừng; Esc/Ctrl+C dừng rồi thoát.\n";
        return 0;
    }
    struct sigaction action{}; action.sa_handler=signalStop; sigemptyset(&action.sa_mask);
    for (int s:{SIGINT,SIGTERM,SIGHUP,SIGQUIT,SIGTSTP}) if (sigaction(s,&action,nullptr)!=0) return 1;
    signal(SIGPIPE,SIG_IGN);
    try {
        const auto paths=executablePaths();
        if (options.control.empty()) options.control=paths.root+"/control.yaml";
        if (options.outRoot.empty()) options.outRoot=paths.root+"/runs_auto";
        options.control=absolutePath(options.control); options.outRoot=absolutePath(options.outRoot);
        auto params=std::make_shared<const control::ControlParams>(control::ControlParams::Load(options.control,std::cout));
        std::cout<<"Trần cứng: "<<kMaxAutoSpeedCms<<" cm/s, "<<kMaxAutoSteerDeg<<" độ; + là trái\n"<<std::flush;
        const auto yaml=readBytes(options.control);
        if (options.modelDir.empty()) {
            const char* home=std::getenv("HOME");
            if (!home || !*home) throw Failure(2,"Thiếu HOME; cần --model-dir");
            options.modelDir=std::string(home)+"/ucr_models";
        }
        options.modelDir=absolutePath(options.modelDir);
        options.info=options.modelDir+"/unet_info.json";
        options.engine=options.modelDir+"/unet.engine";
        options.unetOnnxSha256=checkModel(options.modelDir,"unet");
#if defined(UCR_AUTO_TEST_HOOKS) && UCR_AUTO_TEST_HOOKS
        const perception::InfoJson info(options.info,true);
#else
        const perception::InfoJson info(options.info);
#endif
        warnRoadModel(info);
        std::unique_ptr<perception::IInferEngine> engine,yoloEngine;
        std::unique_ptr<perception::YoloInfo> yoloInfo;
        if (!options.noYolo) {
            options.yoloInfo=options.modelDir+"/yolo_info.json";
            options.yoloEngine=options.modelDir+"/yolo.engine";
            options.yoloOnnxSha256=checkModel(options.modelDir,"yolo");
            yoloInfo.reset(new perception::YoloInfo(options.yoloInfo));
            if (yoloInfo->conf_threshold>float(std::min({params->red_conf_min,params->green_conf_min,params->off_conf_min})))
                throw Failure(2,"conf_threshold YOLO cao hơn conf_min đèn");
#if defined(UCR_AUTO_TEST_HOOKS) && UCR_AUTO_TEST_HOOKS
            for (const char* key:{"UCR_AUTO_MODEL_LOAD_FAIL","UCR_AUTO_FAKE_MODEL_LOAD_FAIL"}) {
                const char* fail=std::getenv(key);
                if (fail && std::string(fail)=="yolo") throw Failure(1,"Không nạp được engine YOLO");
            }
            const char* tensors=std::getenv("UCR_AUTO_FAKE_YOLO_TENSORS");
            if (!tensors) throw Failure(1,"Thiếu UCR_AUTO_FAKE_YOLO_TENSORS");
            yoloEngine.reset(new FakeYoloEngine(absolutePath(tensors),*yoloInfo));
            std::cout<<"YOLO: nap OK "<<options.yoloEngine<<' '<<yoloInfo->class_names.size()<<" lop\n"<<std::flush;
#endif
        } else {
            std::cout<<"YOLO: TAT (--no-yolo)\n"<<std::flush;
        }
#if defined(UCR_AUTO_TEST_HOOKS) && UCR_AUTO_TEST_HOOKS
        if (const char* fail=std::getenv("UCR_AUTO_FAKE_MODEL_LOAD_FAIL")) {
            if (std::string(fail)=="road") throw Failure(1,"Không nạp được engine đường");
        }
        const char* list=std::getenv("UCR_AUTO_FAKE_ROAD_MASKS");
        if (!list) throw Failure(1,"Thiếu UCR_AUTO_FAKE_ROAD_MASKS");
        engine.reset(new FakeRoadEngine(absolutePath(list),info));
#endif
        VehicleLock lock;
#if !defined(UCR_AUTO_TEST_HOOKS) || !UCR_AUTO_TEST_HOOKS
        // Thay lời gọi dừng ngoài lúc khởi động (đã bỏ ucr_stop): lần chạy trước bị kill khi xe đang chạy thì
        // gửi motor 0 ngay (chỉ "N1 v0", không lệnh cấu hình); servo 0 gửi sau Init như cũ.
        { const Motor boot; if (!boot.EmergencyStop()) std::cerr<<"CANH BAO: khong gui duoc motor 0 luc khoi dong\n"; }
#endif
        std::cout<<"KHÔNG CÓ PHANH CHỐNG VẬT CẢN; chạy <=20 cm/s, người giám sát đứng sát xe\n"<<std::flush;
        int code=0;
        if (!stopping) code=driveAuto(options,paths,lock.fd(),params,yaml,info,engine,yoloInfo.get(),yoloEngine);
        return code;
    } catch (const Failure& e) { std::cerr<<e.what()<<'\n'; return e.code; }
    catch (const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
    catch (...) { return 1; }
}
