#include "pass_store.h"

#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <random>
#include <stdexcept>
#include <utility>

using namespace drive_collect;
namespace fs = drive_collect::pass_fs;

namespace {
int checks = 0;
#define CHECK(condition) do { ++checks; if (!(condition)) throw std::runtime_error( \
    std::string("Dòng ") + std::to_string(__LINE__) + ": " #condition); } while (false)

struct Temp {
    fs::path root;
    Temp() {
        std::random_device random;
        for (int attempt = 0; attempt < 100; ++attempt) {
            root = fs::temp_directory_path() / ("ucr_pass_store_" + std::to_string(random()) +
                                               "_" + std::to_string(attempt));
            if (fs::create_directory(root)) return;
        }
        throw std::runtime_error("Không tạo được thư mục kiểm thử riêng");
    }
    ~Temp() { std::error_code error; fs::remove_all(root, error); }
};

std::string read(const fs::path& path) {
    std::ifstream input(path.string(), std::ios::binary);
    CHECK(input.is_open());
    return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

void write(const fs::path& path, const std::string& value) {
    std::ofstream output(path.string(), std::ios::binary | std::ios::trunc);
    output << value;
    output.close();
    CHECK(output.good());
}

// Bộ đọc độc lập kiểm cả trường CSV có dấu phẩy, dấu nháy và xuống dòng.
std::vector<std::vector<std::string>> rows(const fs::path& path) {
    const auto text = read(path);
    std::vector<std::vector<std::string>> result;
    std::vector<std::string> row;
    std::string cell;
    bool quoted = false;
    for (size_t i = 0; i < text.size(); ++i) {
        const char ch = text[i];
        if (ch == '"') {
            if (quoted && i + 1 < text.size() && text[i + 1] == '"') { cell += '"'; ++i; }
            else quoted = !quoted;
        } else if (!quoted && (ch == ',' || ch == '\n')) {
            row.push_back(cell); cell.clear();
            if (ch == '\n') { result.push_back(row); row.clear(); }
        } else cell += ch;
    }
    CHECK(!quoted && row.empty() && cell.empty());
    return result;
}

CaptureColumns capture() {
    CaptureColumns result;
    result.frame = 45; result.monotonic_ns = 123456789;
    result.width = 1280; result.height = 720;
    result.mean_luma = 81.25; result.lap_var = 17.5;
    result.dropped_since_last = 3; result.cmd_speed = 5.25;
    result.cmd_steer = -15; result.state = "CHAY";
    return result;
}
const std::vector<unsigned char> jpeg{0xff, 0xd8, 0, 0x31, 0xff, 0xd9};
const std::string passHeader = "pass,event,ts_ms,frames,end_reason\n";

void formatAndFlush() {
    Temp temp;
    write(temp.root / ".pass_counter", "122\n");
    SessionInfo session;
    session.ts_ms = 1000; session.git = "badf47c";
    session.argv = {"drive", "--flag=a b", "x,\"y\"\nz"};
    session.label = "phiên, \"sáng\"\nxe 1";
    session.pipeline = "camera, \"BGR\"\nappsink";
    PassStore store(temp.root, session);
    CHECK(store.status()); CHECK(store.nextPass() == 123);
    CHECK(store.beginPass(1001));
    CHECK(rows(temp.root / "passes.csv")[1] == (std::vector<std::string>{
        "123", "start", "1001", "0", ""}));
    CHECK(read(temp.root / ".pass_counter") == "123\n");
    for (int i = 1; i <= 45; ++i) CHECK(store.saveFrame(jpeg, 2000 + i, capture()));
    const std::string name = "p0123_0045.jpg";
    CHECK(store.lastFile() == name);
    CHECK(read(temp.root / name) == std::string(reinterpret_cast<const char*>(jpeg.data()), jpeg.size()));
    CHECK(!fs::exists(temp.root / (name + ".tmp")));
    const auto frames = rows(temp.root / "frames.csv");
    CHECK(frames.size() == 46);
    CHECK(frames[0] == (std::vector<std::string>{"frame", "monotonic_ns", "wall_time_unix_ms",
        "filename", "width", "height", "mean_luma", "lap_var", "dropped_since_last",
        "cmd_speed", "cmd_steer", "state", "pass", "file"}));
    CHECK(frames.back() == (std::vector<std::string>{"45", "123456789", "2045", name,
        "1280", "720", "81.25", "17.5", "3", "5.25", "-15", "CHAY", "123", name}));
    CHECK(store.endPass("x,\"stop\"\nreason", 3000));
    const auto passes = rows(temp.root / "passes.csv");
    CHECK(passes.size() == 3); CHECK(passes[2][1] == "end");
    CHECK(passes[0] == (std::vector<std::string>{"pass", "event", "ts_ms", "frames", "end_reason"}));
    for (const auto& row : passes) CHECK(row.size() == 5);
    CHECK(passes[2][3] == "45"); CHECK(passes[2][4] == "x,\"stop\"\nreason");
    const auto sessions = rows(temp.root / "sessions.csv");
    CHECK(sessions.size() == 2);
    CHECK(sessions[0] == (std::vector<std::string>{"ts_ms", "git", "argv", "label", "pipeline"}));
    CHECK(sessions[1] == (std::vector<std::string>{"1000", "badf47c",
        "5:drive10:--flag=a b7:x,\"y\"\nz", session.label, session.pipeline}));
}

void counterRecovery() {
    // Từng nguồn phải thắng độc lập; gồm mất/cũ bộ đếm và giữ chỗ trước kill.
    struct Case { const char* counter; uint64_t csv, image, expected; };
    const Case cases[] = {
        {"40\n", 10, 20, 41}, {nullptr, 31, 20, 32}, {"2\n", 31, 20, 32},
        {nullptr, 12, 39, 40}, {"2\n", 12, 39, 40}, {"bad\n", 12, 39, 40},
        {"0\n", 0, 0, 1}, {"0\n", 81, 0, 82}};
    for (const auto& item : cases) {
        Temp temp;
        if (item.counter) write(temp.root / ".pass_counter", item.counter);
        if (item.csv) write(temp.root / "passes.csv", passHeader + std::to_string(item.csv) +
                           ",start,1,0,\n");
        fs::path oldImage;
        if (item.image) {
            oldImage = temp.root / ("p" + std::to_string(item.image) + "_0001.jpg");
            write(oldImage, "ảnh cũ");
        }
        {
            PassStore store(temp.root);
            CHECK(store.status()); CHECK(store.nextPass() == item.expected);
            CHECK(store.beginPass(5)); CHECK(store.pass() == item.expected);
            CHECK(store.saveFrame(jpeg, 6, capture()));
            // Hủy không endPass để mô phỏng khởi động lại sau kill.
        }
        PassStore restarted(temp.root);
        CHECK(restarted.status()); CHECK(restarted.nextPass() == item.expected + 1);
        CHECK(restarted.beginPass(7)); CHECK(restarted.saveFrame(jpeg, 8, capture()));
        if (item.image) CHECK(read(oldImage) == "ảnh cũ");
    }
}

void temporaryIgnored() {
    Temp temp;
    write(temp.root / ".pass_counter.tmp", "9000\n");
    write(temp.root / "p9999_0001.jpg.tmp", "dở dang");
    fs::create_directory(temp.root / "p8888_0001.jpg");
    write(temp.root / "unrelated.jpg", "x");
    PassStore store(temp.root);
    CHECK(store.status()); CHECK(store.nextPass() == 1);
    CHECK(store.beginPass(1)); CHECK(store.saveFrame(jpeg, 2, capture()));
    CHECK(store.lastFile() == "p0001_0001.jpg");
    CHECK(fs::exists(temp.root / "p9999_0001.jpg.tmp"));
}

void initializationProgress() {
    Temp temp;
    for (int i = 1; i <= 257; ++i)
        write(temp.root / ("p" + std::to_string(i) + "_0001.jpg"), "ảnh cũ");
    int beats = 0;
    PassStore store(temp.root, {}, [&] {
        ++beats;
        CHECK(!fs::exists(temp.root / "sessions.csv"));
    });
    CHECK(store.status()); CHECK(store.nextPass() == 258);
    CHECK(beats >= 259); // Đầu/cuối và từng mục, không chỉ báo sau cả lượt quét.
    CHECK(!store.active()); CHECK(!fs::exists(temp.root / ".pass_counter"));

    Temp csvTemp;
    std::string passes = passHeader;
    for (int i = 1; i <= 1024; ++i)
        passes += std::to_string(i) + ",start,1,0,\n";
    write(csvTemp.root / "passes.csv", passes);
    beats = 0;
    PassStore csvStore(csvTemp.root, {}, [&] { ++beats; });
    CHECK(csvStore.status()); CHECK(csvStore.nextPass() == 1025);
    CHECK(beats >= static_cast<int>(passes.size() / 1024) + 3);
}

void states() {
    Temp temp;
    PassStore store(temp.root);
    CHECK(store.status()); CHECK(!store.active()); CHECK(store.pass() == 0);
    CHECK(store.lastFile().empty());
    CHECK(store.saveFrame(jpeg, 0, capture()).code == StoreError::InvalidState);
    CHECK(store.endPass("x").code == StoreError::InvalidState);
    CHECK(store.beginPass(1));
    CHECK(store.beginPass(2).code == StoreError::InvalidState);
    CHECK(store.saveFrame({}, 3, capture()).code == StoreError::InvalidFrame);
    auto bad = capture(); bad.mean_luma = std::numeric_limits<double>::quiet_NaN();
    CHECK(store.saveFrame(jpeg, 3, bad).code == StoreError::InvalidFrame);
    CHECK(store.saveFrame(jpeg, 3, capture()));
    CHECK(store.lastFile() == "p0001_0001.jpg");
    CHECK(store.endPass("ok", 4));
    CHECK(store.beginPass(5)); CHECK(store.pass() == 2);
    CHECK(read(temp.root / ".pass_counter") == "2\n");
    CHECK(store.endPass("ok", 6));
    CHECK(rows(temp.root / "passes.csv").size() == 5);
}

void sessionsWithoutRememberedLabels() {
    Temp temp;
    {
        SessionInfo session;
        session.ts_ms = 1; session.label = "phiên đầu"; session.pipeline = "pipeline đầu";
        PassStore store(temp.root, session);
        CHECK(store.status()); CHECK(store.beginPass(2));
        CHECK(!fs::exists(temp.root / ".last_labels"));
    }
    // Dữ liệu nhãn cũ không được đọc, cập nhật hay xóa.
    write(temp.root / ".last_labels", "nhãn cũ không hợp lệ");
    {
        PassStore store(temp.root);
        CHECK(store.status()); CHECK(store.beginPass(3)); CHECK(store.endPass("ok", 4));
        CHECK(read(temp.root / ".last_labels") == "nhãn cũ không hợp lệ");
    }
    fs::remove(temp.root / ".last_labels"); fs::create_directory(temp.root / ".last_labels");
    { PassStore store(temp.root); CHECK(store.status()); CHECK(store.beginPass(5)); }
    const auto sessions = rows(temp.root / "sessions.csv");
    CHECK(sessions.size() == 4);
    CHECK(sessions[1][3] == "phiên đầu" && sessions[1][4] == "pipeline đầu");
    CHECK(sessions[2][3].empty() && sessions[2][4].empty());
    CHECK(sessions[3][3].empty() && sessions[3][4].empty());
}

void stoppedCadence() {
    CHECK(frameDue(false, 0)); CHECK(frameDue(false, 1));
    CHECK(frameDue(false, 1999)); CHECK(frameDue(false, 2000));
    CHECK(!frameDue(true, -1)); CHECK(!frameDue(true, 0));
    CHECK(!frameDue(true, 1999)); CHECK(frameDue(true, 2000));
    CHECK(frameDue(true, 2001)); CHECK(frameDue(true, std::numeric_limits<int64_t>::max()));
    // Caller giữ mốc của ảnh đã ghi thành công, kể cả ảnh lúc xe chạy.
    int saved = 1;
    int64_t lastSaved = 0; // Ảnh đầu tiên do caller cho phép.
    for (int64_t now = 1; now <= 6000; ++now) {
        if (frameDue(true, now - lastSaved)) { ++saved; lastSaved = now; }
    }
    CHECK(saved == 4 && lastSaved == 6000);
    CHECK(frameDue(false, 1)); lastSaved = 6001;
    CHECK(!frameDue(true, 8000 - lastSaved));
    CHECK(frameDue(true, 8001 - lastSaved));
}

void endPreservesImages() {
    Temp temp;
    PassStore store(temp.root);
    CHECK(store.beginPass(1)); CHECK(store.saveFrame(jpeg, 2, capture()));
    const auto path = temp.root / store.lastFile(); const auto original = read(path);
    const auto frameCsv = read(temp.root / "frames.csv");
    CHECK(store.endPass("operator", 3)); CHECK(!store.active());
    CHECK(store.saveFrame(jpeg, 4, capture()).code == StoreError::InvalidState);
    CHECK(read(path) == original && read(temp.root / "frames.csv") == frameCsv);
    CHECK(store.endPass("ok", 4).code == StoreError::InvalidState);
    CHECK(store.beginPass(5)); CHECK(store.saveFrame(jpeg, 6, capture()));
    CHECK(store.endPass("ok", 7)); CHECK(read(path) == original);
    const auto passes = rows(temp.root / "passes.csv");
    CHECK(passes.size() == 5);
    for (size_t i = 1; i < passes.size(); ++i)
        CHECK(passes[i][1] == (i % 2 ? "start" : "end"));
}

void writeErrors() {
    CHECK(static_cast<int>(StoreError::Io) == 4);
    Temp temp;
    write(temp.root / "file", "x");
    PassStore invalid(temp.root / "file" / "nested");
    CHECK(invalid.status().code == StoreError::Io && !invalid.status().message.empty());
    for (const auto& name : {"sessions.csv", ".pass_counter.tmp", ".pass_counter",
                             "passes.csv", "frames.csv",
                             "p0001_0001.jpg.tmp"}) {
        Temp failure;
        // Thư mục chặn đích là lỗi thật, không dựa vào quyền chmod trên Windows.
        if (std::string(name) == "sessions.csv") {
            fs::create_directory(failure.root / name);
            PassStore store(failure.root);
            CHECK(store.status().code == StoreError::Io);
            continue;
        }
        PassStore store(failure.root); CHECK(store.status());
        fs::create_directory(failure.root / name);
        auto result = store.beginPass(1);
        if (result) result = store.saveFrame(jpeg, 2, capture());
        CHECK(result.code == StoreError::Io && !result.message.empty());
        CHECK(store.status().code == StoreError::Io);
        CHECK(store.beginPass(3).code == StoreError::Io);
        CHECK(store.saveFrame(jpeg, 4, capture()).code == StoreError::Io);
        CHECK(store.endPass("error", 5).code == StoreError::Io);
    }
    Temp failure;
    PassStore store(failure.root); CHECK(store.beginPass(1));
    fs::remove(failure.root / "passes.csv"); fs::create_directory(failure.root / "passes.csv");
    CHECK(store.endPass("x", 2).code == StoreError::Io);
    CHECK(store.status().code == StoreError::Io);
    CHECK(store.saveFrame(jpeg, 3, capture()).code == StoreError::Io);
}

void noOverwriteAndBadHeader() {
    Temp temp;
    PassStore store(temp.root); CHECK(store.beginPass(1));
    const auto path = temp.root / "p0001_0001.jpg";
    write(path, "giữ ảnh cũ");
    CHECK(store.saveFrame(jpeg, 2, capture()).code == StoreError::Io);
    CHECK(read(path) == "giữ ảnh cũ");
    Temp malformed;
    write(malformed.root / "sessions.csv", "wrong,header\n");
    PassStore invalid(malformed.root); CHECK(invalid.status().code == StoreError::Io);
    Temp oldSession;
    const std::string oldSessionCsv = "ts_ms,git,argv,pipeline\n1,abc,0:,camera\n";
    write(oldSession.root / "sessions.csv", oldSessionCsv);
    PassStore incompatibleSession(oldSession.root);
    CHECK(incompatibleSession.status().code == StoreError::Io);
    CHECK(read(oldSession.root / "sessions.csv") == oldSessionCsv);
    Temp oldPass;
    const std::string oldPassCsv = "pass,event,ts_ms,c,t,s,e,k,v,frames,end_reason\n";
    write(oldPass.root / "passes.csv", oldPassCsv);
    PassStore incompatiblePass(oldPass.root); CHECK(incompatiblePass.status());
    CHECK(incompatiblePass.beginPass(1).code == StoreError::Io);
    CHECK(read(oldPass.root / "passes.csv") == oldPassCsv);
}

void tornAndLargeNumbers() {
    Temp multiline;
    {
        PassStore original(multiline.root);
        CHECK(original.beginPass(1));
        CHECK(original.endPass("lý do có xuống dòng\n900000,end,không phải sự kiện", 2));
    }
    PassStore recovered(multiline.root);
    CHECK(recovered.status()); CHECK(recovered.nextPass() == 2);
    Temp temp;
    const std::string torn = passHeader + "10000,start,1,0,";
    write(temp.root / "passes.csv", torn);
    PassStore store(temp.root); CHECK(store.status()); CHECK(store.nextPass() == 10001);
    CHECK(store.beginPass(2)); CHECK(store.saveFrame(jpeg, 3, capture()));
    CHECK(store.lastFile() == "p10001_0001.jpg");
    CHECK(read(temp.root / "passes.csv").find(torn + "\n10001,start,") == 0);
    Temp full;
    write(full.root / ".pass_counter", "18446744073709551615\n");
    PassStore exhausted(full.root); CHECK(exhausted.status().code == StoreError::Exhausted);
}
} // namespace

int main(int argc, char** argv) {
    const std::vector<std::pair<std::string, std::function<void()>>> tests = {
        {"format-flush", formatAndFlush}, {"counter-recovery", counterRecovery},
        {"tmp-ignored", temporaryIgnored}, {"init-progress", initializationProgress}, {"states", states},
        {"sessions", sessionsWithoutRememberedLabels}, {"stopped-cadence", stoppedCadence},
        {"end-preserves", endPreservesImages},
        {"write-errors", writeErrors}, {"no-overwrite-header", noOverwriteAndBadHeader},
        {"torn-large", tornAndLargeNumbers}};
    int ran = 0;
    for (const auto& test : tests) {
        if (argc > 1 && test.first != argv[1]) continue;
        ++ran;
        try { test.second(); std::cout << "PASS " << test.first << '\n'; }
        catch (const std::exception& error) {
            std::cerr << "FAIL " << test.first << ": " << error.what() << '\n'; return 1;
        }
    }
    if (ran == 0) { std::cerr << "Không có ca được chọn\n"; return 2; }
    std::cout << "PASS " << ran << " ca, " << checks << " phép kiểm\n";
    return 0;
}
