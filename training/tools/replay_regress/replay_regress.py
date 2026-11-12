"""Phát lại hồi quy trên bản nguồn cô lập; không checkout/stash hay sửa mã xe."""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import os
import subprocess
import sys
import tarfile
import uuid
from contextlib import contextmanager

ROOT = Path(__file__).resolve().parents[3]
sys.stdout.reconfigure(encoding='utf-8')
SCRATCH = ROOT / 'round3_final/test_logs/V1p_1213'
PREFIX = 'round3_final/ucr_car'
ENV = os.environ.copy()
ENV['PATH'] = 'C:/msys64/ucrt64/bin;' + ENV.get('PATH', '')
FIELDS = ('planner_state', 'state', 'reason', 'route_step', 'turn_dir', 'traj_mode', 'cmd_deg', 'cmd_cms')


@contextmanager
def workspace():
    folder = SCRATCH / ('build_' + uuid.uuid4().hex[:12])
    folder.mkdir(parents=True)
    yield str(folder)


def run(args, **kw):
    return subprocess.run([str(x) for x in args], check=True, timeout=kw.pop('timeout', 300), env=ENV, **kw)


def adapt(source):
    # Chỉ sửa bộ đọc/ghi của bản replay tạm, không sửa thuật toán Planner.
    text = source.read_text(encoding='utf-8')
    text = text.replace('#include <map>', '#include <map>\n#include <limits>')
    text = text.replace('control_hash\\n', 'control_hash,source_row,route_step,turn_dir,traj_mode,route_note\\n')
    needle = 'data.stop=stop; data.yolo_enabled=yoloEnabled;'
    extra = '''
            auto sensor=[&](const char *key) { const auto s=get(key); return s.empty() ? std::numeric_limits<double>::quiet_NaN() : Number(s,key); };
            data.yaw_deg=sensor("yaw_deg"); data.odo_cm=sensor("odo_cm");
            data.sensor_ok=std::isfinite(data.yaw_deg) && std::isfinite(data.odo_cm);
            data.encoder_ok=std::isfinite(data.odo_cm); data.sensor_ms=now;
'''
    if needle not in text:
        raise ValueError('Bản replay không tương thích vị trí nạp cảm biến')
    text = text.replace(needle, needle + extra)
    # Giữ G chờ tới lần Planner nhận được dữ liệu, rồi bỏ; không tự G sau lỗi dừng.
    text = text.replace('data.go=running;', 'data.go=goPending;')
    text = text.replace(needle, needle + '''
            if(stop) goPending=false;
            else if(indexes.count("go") && Flag(get("go"),"go")) { planner.Reset(true); goPending=true; }
''')
    # Dòng không có mask vẫn phải đi qua Planner với cảm biến/sự kiện hiện tại.
    start = text.index('            if(mask_file.empty()) {', text.index('control::PlannerInput data;'))
    end = text.index('            const auto frame=get("frame_id");', start)
    text = text[:start] + text[end:]
    text = text.replace('control::Planner planner;', '''std::map<int64_t,std::string> maskPaths;
        { std::ifstream scan(run+"/control_log.csv"); std::string row; std::getline(scan,row);
          while(std::getline(scan,row)) { if(!row.empty() && row.back()=='\\r') row.pop_back();
            const auto v=Csv(row); const auto file=v[indexes.at("mask_file")];
            if(!file.empty() && indexes.count("mask_ms") && !v[indexes.at("mask_ms")].empty())
                maskPaths[Milliseconds(v[indexes.at("mask_ms")],"mask_ms")]=file;
          }
        }
        control::Planner planner; bool goPending=false;''')
    a=text.index('            Require(mask_file=="masks/"')
    b=text.index('            const auto reference=get(', a)
    text=text[:a]+'''            const auto foundMask=maskPaths.find(mask_ms);
            const std::string usedMask=mask_file.empty() && foundMask!=maskPaths.end() ? foundMask->second : mask_file;
            if(!usedMask.empty()) {
                Require(usedMask.compare(0,6,"masks/")==0 && usedMask.find("..") == std::string::npos,"Đường dẫn mask sai");
                data.mask=cv::imread(run+"/"+usedMask,cv::IMREAD_UNCHANGED);
                Require(!data.mask.empty() && data.mask.type()==CV_8UC1,"Không đọc được mask: "+usedMask);
            } else ++missingMasks;
            data.mask_ms=mask_ms; data.image_ms=image_ms;
''' + text[b:]
    text = text.replace("out << ',' << hash << '\\n';", "out << ',' << hash << ',' << rows-1 << ',' << planner.RouteStep() << ',' << planner.TurnDirection() << ',' << control::TrajModeName(plan.mode) << ',' << Quote(planner.RouteNote()) << '\\n';")
    text = text.replace('const auto plan=planner.Step(data,now,params);', '''const auto plan=planner.Step(data,now,params);
            if(plan.state!=control::PlannerState::CHO_SAN_SANG && plan.reason.compare(0,10,"go ignored")!=0) goPending=false;''')
    # Tránh hàng chục nghìn ảnh phủ; dữ liệu so sánh vẫn giữ nguyên.
    text = text.replace('Require(cv::imwrite(output+', '// Require(cv::imwrite(output+')
    source.write_text(text, encoding='utf-8')


def build(args):
    SCRATCH.mkdir(parents=True, exist_ok=True)
    dest = Path(args.out).resolve()
    dest.parent.mkdir(parents=True, exist_ok=True)
    with workspace() as tmp:
        tree = Path(tmp)
        archive = tree / 'source.tar'
        run(['git', '-C', ROOT, 'archive', '--format=tar', '--output', archive, 'HEAD' if args.rev == 'WORKTREE' else args.rev, PREFIX])
        with tarfile.open(str(archive)) as tar:
            for member in tar:
                target = (tree / member.name).resolve()
                target.relative_to(tree.resolve())
                if member.isdir():
                    target.mkdir(parents=True, exist_ok=True)
                elif member.isfile():
                    target.parent.mkdir(parents=True, exist_ok=True)
                    target.write_bytes(tar.extractfile(member).read())
                else:
                    raise ValueError('Không nhận liên kết trong archive: ' + member.name)
        src = tree / PREFIX
        if args.rev == 'WORKTREE':
            # Chụp tệp được Git theo dõi và tệp mới không bị bỏ qua; không kéo thư mục build.
            files=subprocess.check_output(['git','-C',str(ROOT),'ls-files','-co','--exclude-standard','-z','--',PREFIX],env=ENV).decode('utf-8').split('\0')
            present=set()
            for file in files:
                if not file: continue
                original=ROOT/file; target=tree/file
                if original.is_file():
                    target.parent.mkdir(parents=True,exist_ok=True); target.write_bytes(original.read_bytes());present.add(file)
            for target in src.rglob('*'):
                if target.is_file() and target.relative_to(tree).as_posix() not in present: target.unlink()
        if args.patch:
            patch_text=Path(args.patch).read_text(encoding='utf-8')
            for line in patch_text.splitlines():
                if line.startswith(('--- ', '+++ ')):
                    file=line[4:].split('\t')[0]
                    if file=='/dev/null': continue
                    if not file.startswith(('a/'+PREFIX+'/', 'b/'+PREFIX+'/')) or '..' in Path(file).parts:
                        raise ValueError('Patch vượt cây mã xe: '+file)
            normalized=tree/'input.patch'
            normalized.write_bytes(patch_text.encode('utf-8'))
            run(['git', 'apply', '--unsafe-paths', '--directory=' + str(tree).replace('\\', '/'), normalized], cwd=str(ROOT))
        adapt(src / 'tools/replay_control/main.cpp')
        flags = subprocess.check_output(['pkg-config', '--cflags', 'opencv4'], env=ENV, text=True).split()
        cmd = ['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-I' + str(src / 'include'), src / 'tools/replay_control/main.cpp']
        cmd += sorted((src / 'src/control').glob('*.cpp')) + flags + ['-lopencv_core', '-lopencv_imgproc', '-lopencv_imgcodecs', '-o', dest]
        with dest.with_suffix('.build.log').open('w', encoding='utf-8') as log:
            run(cmd, stdout=log, stderr=subprocess.STDOUT)
        manifest = {'rev': args.rev, 'patch': args.patch, 'patch_sha256': hashlib.sha256(Path(args.patch).read_bytes()).hexdigest() if args.patch else None, 'exe_sha256': hashlib.sha256(dest.read_bytes()).hexdigest(),
                    'reader': 'yaw/odo hữu hạn coi là tươi tại t_ms; thiếu tuổi cảm biến/ToF/epoch; tìm mask cùng mask_ms, thiếu thì rỗng; G chờ dữ liệu rồi bỏ, Reset(true) tại G; xuất mọi dòng',
                    'source_snapshot': str(src), 'adapter_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
        dest.with_suffix('.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    print('BUILD OK', dest)


def read(path):
    with path.open(encoding='utf-8-sig', newline='') as f:
        return list(csv.DictReader(f))


def differences(a, b):
    result = []
    for field in FIELDS:
        x, y = a[field], b[field]
        same = abs(float(x)-float(y)) <= 1e-6 if field.startswith('cmd_') else x == y
        if not same:
            result.append(field)
    return result


def compare(args):
    SCRATCH.mkdir(parents=True, exist_ok=True)
    out = SCRATCH / ('compare_' + uuid.uuid4().hex[:12])
    out.mkdir()
    summary = []
    for name in args.runs:
        folder = Path(name).resolve()
        session = json.loads((folder / 'session.json').read_text(encoding='utf-8'))
        if not session.get('unet_onnx_sha256', '').startswith('1e1964'):
            raise ValueError('Hồi quy chỉ nhận t94: ' + str(folder))
        pair = []
        for label, exe in [('old', args.old), ('new', args.new)]:
            target = out / (folder.name + '_' + label)
            with (out / (folder.name + '_' + label + '.log')).open('w', encoding='utf-8') as log:
                run([Path(exe).resolve(), '--run', folder, '--control', folder / 'control.yaml', '--out', target], stdout=log, stderr=subprocess.STDOUT, timeout=900)
            pair.append(read(target / 'control_log.csv'))
        if len(pair[0]) != len(pair[1]):
            raise ValueError('Hai bản xuất khác số dòng')
        detail, segments, active = [], [], None
        for a, b in zip(*pair):
            if (a['source_row'], a['frame_id'], a['t_ms']) != (b['source_row'], b['frame_id'], b['t_ms']):
                raise ValueError('Hai bản lệch dòng/khung/thời gian')
            changed = differences(a, b)
            if changed:
                row = {'source_row': a['source_row'], 'frame_id': a['frame_id'], 't_ms': a['t_ms'], 'fields': ';'.join(changed)}
                for k in FIELDS:
                    row['old_' + k], row['new_' + k] = a[k], b[k]
                detail.append(row)
                signature = tuple((k, a[k], b[k]) for k in changed if not k.startswith('cmd_'))
                if active is None or active['signature'] != signature:
                    active = {'first': a['source_row'], 'last': a['source_row'], 't_first': a['t_ms'], 't_last': a['t_ms'], 'signature': signature, 'old': {k:a[k] for k in FIELDS}, 'new': {k:b[k] for k in FIELDS}, 'count':0}
                    segments.append(active)
                active['last'], active['t_last'] = a['source_row'], a['t_ms']
                active['count'] += 1
            else:
                active = None
        if detail:
            with (out / (folder.name + '_diff.csv')).open('w', encoding='utf-8-sig', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=list(detail[0])); writer.writeheader(); writer.writerows(detail)
        item = {'run': str(folder), 'samples': len(pair[0]), 'different': len(detail), 'unique_frames_different': len(set(r['frame_id'] for r in detail)), 'segments': segments}
        original=read(folder/'control_log.csv')
        saved={r['mask_ms'] for r in original if r['mask_file']}
        item['mask_rows_available']=sum(r['mask_ms'] in saved for r in original)
        item['mask_rows_missing']=len(original)-item['mask_rows_available']
        item['limitations']='Không phải tái hiện đầy đủ: mặt nạ không lưu được đưa vào rỗng; tuổi cảm biến/ToF/epoch Reset không có trong log. Không dùng 0 khác biệt để xác nhận hết hồi quy.'
        summary.append(item)
        print(folder.name, 'mẫu khác', len(detail), '/', len(pair[0]), 'đoạn', len(segments))
        print('  dòng đầu-cuối | t_ms đầu-cuối | trước -> sau (state/reason/route/mode/góc/tốc độ)')
        for s in segments:
            def label(v): return '/'.join(v[k] for k in ('planner_state','reason','route_step','traj_mode','cmd_deg','cmd_cms'))
            print(' ',s['first']+'-'+s['last'],'|',s['t_first']+'-'+s['t_last'],'|',label(s['old']),'->',label(s['new']))
    (out / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    with (out / 'segments.csv').open('w', encoding='utf-8-sig', newline='') as f:
        w=csv.writer(f); w.writerow(['run','first','last','t_first_ms','t_last_ms','count','old','new'])
        for item in summary:
            for s in item['segments']:
                w.writerow([Path(item['run']).name]+[s[k] for k in ('first','last','t_first','t_last','count')]+[json.dumps(s[k],ensure_ascii=False) for k in ('old','new')])
    print('Bảng đoạn và CSV chi tiết:', out)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command', required=True)
    p=sub.add_parser('build'); p.add_argument('rev'); p.add_argument('out'); p.add_argument('--patch'); p.set_defaults(func=build)
    p=sub.add_parser('compare'); p.add_argument('old'); p.add_argument('new'); p.add_argument('runs', nargs='+'); p.set_defaults(func=compare)
    args=parser.parse_args(); args.func(args)


if __name__ == '__main__':
    main()
