"""Test ucr_test_model: bộ T33 nguyên vẹn trên binary mới + ca phần model (Python 3.6, Linux).

Nạp collect_drive/test_collect_drive.py làm module, không chép, không sửa; chỉ thay cách build bản
mutation (thêm model_view, dnn) và thêm ca mới. ONNX giả KHÔNG chứng minh model đúng.
"""
import argparse
import importlib.util
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time

HERE = Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location('t33', str(HERE.parent / 'collect_drive' / 'test_collect_drive.py'))
td = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(td)
need, Failure = td.need, td.Failure

T33_CASES = [c for c in td.CASES if c != 'SC']
NEW_CASES = 'A01 A02 A03 A04 A05 A06 A08 A09 A10 A11 D01 M6 M7 M8 M9'.split()
VIEW = ('ModelInfo', 'DnnMaskModel', 'MaskRender', 'ModelWorker')
FAKE = HERE / 'make_fake_road_onnx.py'


def events(run, kind):
    return [e for e in run.events() if e[1] == kind]


def make_onnx(folder, fmt, name='road_fake'):
    folder.mkdir(parents=True, exist_ok=True)
    suffix = '_cvdnn.onnx' if fmt == 'cvdnn' else '.onnx'
    path = folder / (name + suffix)
    subprocess.check_call([sys.executable, str(FAKE), '--out', str(path), '--format', fmt], timeout=60)
    return path


def alive_for(run, seconds, tag):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        need(run.process.poll() is None, 'Chương trình thoát khi model lỗi: ' + run.text()[-1500:], tag)
        time.sleep(0.01)


def no_command(run, mark, tag='model_no_command'):
    moved = [e for e in run.events()[mark:] if e[1] in ('motor_speed', 'servo_angle') and e[2] != 0]
    need(not moved, 'Có lệnh chuyển động không do phím: ' + repr(moved[:3]), tag)


def notice_case(cfg, root, args, needle, env=None):
    with td.running(cfg, root, args=args, env=env) as run:
        run.boot()
        run.wait(lambda: any(needle.lower() in str(e[2]).lower() for e in events(run, 'model_notice')), 5,
                 'Thiếu thông báo "' + needle + '"', tag='model_notice')
        run.act('W', 5, 0)
        alive_for(run, 1.0, 'model_error_exit')
        run.act(' ', 0, 0)
        run.finish()


def windows(run):
    """Con cửa sổ (nice 10, xem E04 của T33) theo thứ tự fork: [chính, phụ mặt nạ, phụ phủ ảnh]."""
    return sorted(pid for pid in run.children() if td.nice(pid) == 10)


def test_case(case, cfg, root):
    if case == 'E04':
        # Như E04 của T33, nhưng có 3 cửa sổ: giết đúng cửa sổ CHÍNH (fork đầu, pid nhỏ nhất) → mã 1.
        for mode in ('kill', 'close_after_frame', 'die_after_frame', 'fail_start'):
            env = {} if mode == 'kill' else {'UCR_DRIVE_FAKE_PREVIEW': mode}
            with td.running(cfg, root / mode, env=env) as run:
                if mode == 'kill':
                    td.moving(run)
                    found = windows(run)
                    need(len(found) == 3, 'Cần 3 cửa sổ giả: ' + repr(found), 'interface')
                    mark = len(run.events())
                    os.kill(found[0], signal.SIGKILL)
                    run.finish(code=1, mark=mark)
                else:
                    run.wait(lambda: run.process.poll() is not None, 12, 'Không thoát khi cửa sổ hỏng', alive=False)
                    need(run.process.returncode == 1, 'Cửa sổ {} cần mã 1: {}'.format(mode, run.text()), 'exit_code')
                    td.no_forward(run.events(), 'window_dead')
        return
    if case in T33_CASES:
        return T33_TEST_CASE(case, cfg, root)
    if case == 'A01':
        return notice_case(cfg, root, [], '--model')
    if case == 'A02':
        good = make_onnx(root / 'models', 'cvdnn')
        info = root / 'models' / 'road_fake_info.json'
        broken = root / 'models' / 'broken.onnx'
        broken.write_bytes(b'khong phai onnx')
        wrong = root / 'models' / 'wrong_info.json'
        wrong.write_text(info.read_text(encoding='utf-8').replace('"line"', '"lane"', 1), encoding='utf-8')
        cases = [(['--model', str(root / 'models' / 'none.onnx')], 'none_info.json'),
                 (['--model', str(broken), '--info', str(info)], 'ONNX'),
                 (['--model', str(good), '--info', str(wrong)], 'lop la'),
                 (['--model', str(good), '--info', str(root / 'missing_info.json')], 'missing_info.json')]
        for index, (args, needle) in enumerate(cases):
            notice_case(cfg, root / str(index), args, needle)
        return
    if case == 'A03':
        with td.running(cfg, root, env={'UCR_DRIVE_FAKE_MODEL': 'throw'}) as run:
            try:
                run.boot()
            except Failure:
                need(run.process.poll() is None, 'Thoát khi model ném ngoại lệ: ' + run.text()[-1500:],
                     'model_error_exit')
                raise
            alive_for(run, 1.5, 'model_error_exit')
            run.wait(lambda: any('loi model gia' in str(e[2]) for e in events(run, 'model_notice')), 5,
                     'Lỗi model không hiện ở cửa sổ phụ', tag='model_notice')
            run.act('W', 5, 0)
            alive_for(run, 1.0, 'model_error_exit')
            need(td.last_commands(run.events()) == (5, 0), 'Lỗi model làm xe dừng', 'model_error_exit')
            run.finish()
        return
    if case == 'A04':
        with td.running(cfg, root, env={'UCR_DRIVE_FAKE_MODEL': 'ok'}, args=['--infer-hz', '5']) as run:
            run.boot()
            mark = len(run.events())
            run.wait(lambda: len(events(run, 'model_result')) >= 2, 5, 'Không có kết quả model', tag='model_run')
            start = len(events(run, 'model_result'))
            run.observe(2.0, lambda: no_command(run, mark))
            produced = len(events(run, 'model_result')) - start
            need(6 <= produced <= 12, '--infer-hz 5 cho {} kết quả/2 s'.format(produced), 'infer_rate')
            td.recording(run, 1)
            run.wait(lambda: run.rows(), message='Không ghi ảnh khi model chạy', tag='record_format')
            td.recording_off(run, 1)
            run.finish()
            # .pass_counter là tệp sẵn có của PassStore; ngoài ra chỉ được có ảnh/CSV/JSON của lượt.
            extra = [p.name for p in run.out.rglob('*') if p.is_file() and p.name != '.pass_counter' and
                     (p.suffix not in ('.jpg', '.csv', '.json') or 'mask' in p.name.lower())]
            need(not extra, 'Ghi thêm tệp ngoài định dạng lượt (mặt nạ?): ' + repr(extra), 'record_format')
        return
    if case == 'A05':
        with td.running(cfg, root, env={'UCR_DRIVE_FAKE_MODEL': 'slow'}) as run:
            run.boot()
            run.wait(lambda: events(run, 'model_result'), 5, 'Model chậm không có kết quả', tag='model_run')
            limit = 150 + cfg.slack
            try:
                for index in range(8):
                    key, speed = ('W', 5) if index % 2 == 0 else ('S', 0)
                    latency = run.act(key, speed, 0, latency=True)
                    need(latency <= limit, 'Phím → motor {:.0f} ms > {} ms khi model chậm'.format(latency, limit),
                         'model_blocks_control')
                run.act('W', 5, 0)
            except Failure as error:
                # Phím không thành lệnh đúng hạn trong lúc model chậm = vòng điều khiển bị chặn.
                if error.tag == 'model_blocks_control':
                    raise
                raise Failure('model_blocks_control', str(error))
            mark = len(run.events())
            run.observe(2.0)
            stops = [e for e in run.events()[mark:] if e[1] in ('motor_speed', 'motor_emergency') and e[2] != 5]
            need(not stops, 'Model chậm gây dừng "mất ảnh": ' + repr(stops[:2]), 'model_blocks_control')
            run.finish()
        return
    if case == 'A06':
        for mode in ('die_after_frame', 'close_after_frame', 'kill'):
            env = {} if mode == 'kill' else {'UCR_DRIVE_FAKE_SIDE_PREVIEW': mode}
            with td.running(cfg, root / mode, env=env) as run:
                run.boot()
                if mode == 'kill':
                    found = windows(run)
                    need(len(found) == 3, 'Cần 3 cửa sổ giả: ' + repr(found), 'interface')
                    for pid in found[1:]:
                        os.kill(pid, signal.SIGKILL)
                run.wait(lambda: len(events(run, 'side_closed')) == 2, 5, 'Cửa sổ phụ chưa đóng', tag='side_window')
                run.act('W', 5, 0)
                alive_for(run, 1.0, 'side_window')
                need(td.last_commands(run.events()) == (5, 0), 'Đóng cửa sổ phụ làm xe dừng', 'side_window')
                run.finish()
        return
    if case == 'A08':
        with td.running(cfg, root, env={'UCR_DRIVE_FAKE_MODEL': 'ok'}) as run:
            run.boot()
            need(len(run.children()) >= 4, 'Cần con camera + 3 cửa sổ: ' + repr(run.children()), 'side_window')
            run.wait(lambda: events(run, 'model_result'), 5, 'Không có kết quả model', tag='model_run')
            need(not events(run, 'side_closed'), 'Cửa sổ phụ chết khi bình thường', 'side_window')
            run.finish()
        return
    if case == 'A09':
        model = make_onnx(root / 'models', 'cvdnn')
        with td.running(cfg, root / 'run', args=['--model', str(model)]) as run:
            run.boot()
            run.wait(lambda: len(events(run, 'model_result')) >= 2, 20,
                     'ONNX giả không chạy: ' + repr(events(run, 'model_notice')[-1:]), tag='model_run')
            need('_cvdnn' in run.text(), 'Không in rõ đang dùng bản _cvdnn', 'cvdnn_banner')
            run.finish()
        return
    if case == 'A10':
        bad = [['--infer-hz', '0'], ['--infer-hz', '16'], ['--infer-hz', 'abc'], ['--infer-hz', '-5'],
               ['--infer-hz'], ['--model'], ['--model', ''], ['--info', '']]
        for index, args in enumerate([['--help']] + bad):
            with td.running(cfg, root / str(index), args=args) as run:
                run.wait(lambda: run.process.poll() is not None, alive=False)
                need(run.process.returncode == (0 if index == 0 else 2), 'Sai mã cờ: ' + repr(args))
                need(not any(e[1].startswith('motor_') for e in run.events()), 'Cờ chạm motor')
                if index == 0:
                    for flag in ('--model', '--info', '--infer-hz'):
                        need(flag in run.text(), 'Help thiếu ' + flag)
        return
    if case == 'A11':
        root.mkdir(parents=True)
        code, output = td.tool([cfg.nm, '-C', cfg.viewlib], root)
        need(code == 0 and 'model_view::ModelWorker' in output, 'nm thiếu bằng chứng: ' + output[-500:],
             'symbols_setup')
        hits = sorted(set(re.findall(r'\b(?:Motor|Servo)\b|SetSpeedCms|SetAngle|ttyTHS|i2c', output)))
        need(not hits, 'model_view chứa ký hiệu điều khiển: ' + repr(hits), 'isolation')
        cmake = (Path(cfg.source) / 'CMakeLists.txt').read_text(encoding='utf-8')
        library = re.search(r'add_library\(test_model_view[^)]*\)', cmake).group(0)
        links = ' '.join(re.findall(r'target_link_libraries\(test_model_view\b[^)]*\)', cmake))
        need(not re.search(r'Motor|Servo|Camera|hardware', library + links), 'model_view liên kết phần cứng', 'isolation')
        for path in (Path(cfg.source) / 'model_view').iterdir():
            text = path.read_text(encoding='utf-8')
            need('hardware/' not in text and 'Control' not in text and 'SetSpeedCms' not in text,
                 path.name + ' chạm mã điều khiển', 'isolation')
        return
    if case == 'D01':
        folder = root / 'models'
        cvdnn = make_onnx(folder, 'cvdnn')
        original = make_onnx(folder, 'resize12')
        code, output = td.tool([cfg.dnn, str(cvdnn), str(folder / 'road_fake_info.json'), str(original)], root, 60)
        print(output.strip())
        need(code == 0 and 'OK model_view dnn' in output, 'DNN trên ONNX giả hỏng: ' + output[-1500:], 'model_run')
        return
    raise Failure('setup', 'Chưa đăng ký ca ' + case)


def build_targets(cfg, root, targets, budget):
    """Như T33 nhưng bản sao có model_view + dnn; kiểu 'worker' dựng test luồng model từ nguồn sao."""
    project = root / 'mutation-project'
    project.mkdir()
    lines = ['cmake_minimum_required(VERSION 3.10)', 'project(T42Mutation LANGUAGES CXX)',
             'set(CMAKE_CXX_STANDARD 17)', 'set(CMAKE_CXX_STANDARD_REQUIRED ON)',
             'find_package(OpenCV 4.1 REQUIRED COMPONENTS core imgcodecs videoio imgproc dnn OPTIONAL_COMPONENTS highgui)',
             'find_package(Threads REQUIRED)']
    for name, src, kind in targets:
        src = Path(src).as_posix()
        view = ' '.join('"{}/model_view/{}.cpp"'.format(src, v) for v in VIEW)
        if kind == 'limits':
            lines.append('add_executable({} "{}/test_limits.cpp")'.format(name, src))
            continue
        if kind == 'worker':
            lines.append('add_executable({} "{}/test_model_view_worker.cpp" {})'.format(
                name, Path(cfg.tests).as_posix(), view))
        else:
            lines += ['add_executable({0} "{1}/main.cpp" "{1}/../collect_common/preview.cpp" '
                      '"{1}/../collect_common/pass_store.cpp" {2})'.format(name, src, view),
                      'target_compile_definitions({} PRIVATE UCR_DRIVE_TEST_HOOKS=1 '
                      'UCR_DRIVE_GIT_REVISION="${{REVISION}}")'.format(name)]
        lines += ['target_include_directories({} PRIVATE "{}/model_view" ${{OpenCV_INCLUDE_DIRS}})'.format(name, src),
                  'target_link_libraries({} PRIVATE ${{OpenCV_LIBS}} Threads::Threads)'.format(name),
                  'if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9)',
                  ' target_link_libraries({} PRIVATE stdc++fs)'.format(name), 'endif()']
    (project / 'CMakeLists.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    build = root / 'build'
    build.mkdir()
    deadline = time.monotonic() + budget
    commands = [[cfg.cmake, str(project), '-DREVISION=' + cfg.revision, '-DCMAKE_CXX_COMPILER=' + cfg.cxx,
                 '-DCMAKE_BUILD_TYPE=Release', '-DCMAKE_CXX_FLAGS=-O0 -g0']]
    commands += [[cfg.cmake, '--build', '.', '--target', name, '--', '-j1'] for name, _, _ in targets]
    for command in commands:
        remaining = deadline - time.monotonic()
        need(remaining > 0, 'Hết hạn build mutation', 'mutation_setup')
        code, output = td.tool(command, build, remaining)
        need(code == 0, 'Build mutation hỏng: ' + output[-4000:], 'mutation_setup')
    return build


def replace_once(path, old, new):
    text = path.read_text(encoding='utf-8')
    need(text.count(old) == 1, 'Neo mutation phải xuất hiện đúng 1 lần trong {}: {!r}'.format(path.name, old[:60]),
         'mutation_setup')
    path.write_text(text.replace(old, new), encoding='utf-8')


def patch_mutant(case, folder):
    main, worker = folder / 'main.cpp', folder / 'model_view' / 'ModelWorker.cpp'
    if case == 'M6':
        # Lỗi: kết quả model được đưa vào lệnh tốc độ.
        replace_once(main, 'namespace {\nstatic_assert', 'namespace {\nstd::atomic<bool> t42Result{false};\nstatic_assert')
        replace_once(main, '                trace(panels.notice.empty() ? "model_result"',
                     '                if (panels.notice.empty()) t42Result = true;\n'
                     '                trace(panels.notice.empty() ? "model_result"')
        # Mỗi kết quả thứ 6 (~1,2 s ở 5 Hz): lệnh đầu rơi sau kiểm khởi động, vào cửa sổ quan sát của A04.
        replace_once(main, '            // Cửa sổ phụ: bỏ qua phím',
                     '            static int t42Count = 0;\n'
                     '            if (t42Result.exchange(false) && !stale && ready && ++t42Count % 6 == 0) control.key(\'w\');\n'
                     '            // Cửa sổ phụ: bỏ qua phím')
    elif case == 'M7':
        # Lỗi: vòng điều khiển chờ model (gửi khung phải lấy khóa mà model giữ suốt lượt suy luận).
        replace_once(worker, 'std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);\n    if (!lock.owns_lock()) return;',
                     'std::unique_lock<std::mutex> lock(mutex_);')
        replace_once(worker, '    const auto start = Clock::now();\n    const cv::Mat mask = model.infer(bgr);',
                     '    const auto start = Clock::now();\n    cv::Mat mask;\n'
                     '    { std::lock_guard<std::mutex> hold(mutex_); mask = model.infer(bgr); }')
    elif case == 'M8':
        # Lỗi: ngoại lệ suy luận thoát khỏi luồng model (noexcept → terminate → chương trình thoát).
        replace_once(worker, '            } catch (const std::exception& e) { reason = e.what(); }\n'
                             '            catch (...) { reason = "loi khong ro khi suy luan"; }',
                     '            } catch (...) { throw; }')
        replace_once(worker, '    } catch (...) {}\n    finished_ = true;', '    } catch (...) { throw; }\n    finished_ = true;')
    elif case == 'M9':
        # Lỗi: thu nhỏ khung MỚI NHẤT sau suy luận thay vì chính khung đã suy luận.
        replace_once(worker, '    cv::Mat thumb;\n    cv::resize(bgr, thumb, cv::Size(kThumbWidth, kThumbHeight), 0, 0, cv::INTER_AREA);\n', '')
        replace_once(worker, '    ModelPanels panels;\n    panels.mask = renderMaskPanel(mask, info);',
                     '    cv::Mat latest, thumb;\n    { std::lock_guard<std::mutex> l(mutex_); latest = mailbox_.empty() ? bgr : mailbox_; }\n'
                     '    cv::resize(latest, thumb, cv::Size(kThumbWidth, kThumbHeight), 0, 0, cv::INTER_AREA);\n'
                     '    ModelPanels panels;\n    panels.mask = renderMaskPanel(mask, info);')
    else:
        T33_PATCH(case, folder)


def mutation(case, cfg, root):
    if case in T33_CASES:
        return T33_MUTATION(case, cfg, root)
    root.mkdir(parents=True)
    if case == 'M9':
        build = build_targets(cfg, root, [('baseline', cfg.source, 'worker')], 200)
        code, output = td.tool([str(build / 'baseline')], root, 60)
        need(code == 0, 'Baseline test luồng model hỏng: ' + output, 'mutation_setup')
        folder = td.copy_source(cfg, root / 'source')
        patch_mutant(case, folder)
        (root / 'm').mkdir()
        build = build_targets(cfg, root / 'm', [('mutant', folder, 'worker')], 200)
        code, output = td.tool([str(build / 'mutant')], root, 60)
        need(code != 0 and 'frame_mismatch' in output, 'M9 (lệch khung) không bị phát hiện: ' + output,
             'mutation_survived')
        print('M9: baseline đạt, bản lỗi HỎNG đúng frame_mismatch')
        return
    probe, expected = {'M6': ('A04', 'model_no_command'), 'M7': ('A05', 'model_blocks_control'),
                       'M8': ('A03', 'model_error_exit')}[case]
    test_case(probe, cfg, root / 'baseline')
    folder = td.copy_source(cfg, root / 'source')
    patch_mutant(case, folder)
    build = build_targets(cfg, root, [('mutant', folder, 'hooks')], 200)
    try:
        test_case(probe, td.mutant_cfg(cfg, build / 'mutant'), root / 'probe')
    except Failure as error:
        need(error.tag == expected, 'Hỏng sai phép kiểm: ' + str(error), 'mutation_setup')
    else:
        raise Failure('mutation_survived', case + ' không bị phát hiện')
    print(case + ': baseline đạt, bản lỗi HỎNG đúng phép kiểm ' + expected)


# Ca T33 gọi lại test_case/build_targets/patch_mutant theo tên toàn cục của module: thay bằng bản ở đây.
T33_TEST_CASE, T33_PATCH, T33_MUTATION = td.test_case, td.patch_mutant, td.mutation
td.test_case, td.build_targets, td.patch_mutant = test_case, build_targets, patch_mutant


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', required=True, choices=T33_CASES + NEW_CASES)
    for name in ('hooks', 'real', 'limits', 'viewlib', 'dnn', 'tests', 'source', 'cmake', 'cxx', 'nm', 'revision'):
        parser.add_argument('--' + name, required=True)
    parser.add_argument('--slack', type=int, choices=(0, 300), default=0)
    cfg = parser.parse_args()
    need(os.name == 'posix', 'Chỉ chạy trên Linux', 'environment')
    with tempfile.TemporaryDirectory(prefix='ucr-t42-') as tmp:
        if re.fullmatch(r'M\d+[bs]?', cfg.case):
            mutation(cfg.case, cfg, Path(tmp) / cfg.case)
        else:
            test_case(cfg.case, cfg, Path(tmp) / cfg.case)
    print('PASS ' + cfg.case)


if __name__ == '__main__':
    main()
