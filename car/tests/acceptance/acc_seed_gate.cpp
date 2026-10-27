// Dựng dữ liệu đầu vào smoke hợp lệ cho test CLI cũ; không chạy smoke hay chứng minh xe thật.
#include "acc_seed_gate.h"
#include <cstdio>
int main(int argc,char **argv) {
    if(argc!=3) return 2;
    // Cấu hình cố tình sai không cần gate và phải giữ runs rỗng để kiểm main từ chối trước thiết bị.
    common::Config c;
    try { c.Load(argv[2]); } catch(const std::exception &) { return 0; }
    try { c.log_dir=argv[1]; acc::SeedGate(c,argv[2]); }
    catch(const std::exception &e) { std::fprintf(stderr,"Không dựng được dữ liệu gate: %s\n",e.what()); return 1; }
    return 0;
}
