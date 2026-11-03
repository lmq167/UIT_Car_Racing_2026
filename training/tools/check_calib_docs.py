#!/usr/bin/env python3
"""D01–D06/QĐ11: kiểm chuỗi tài liệu hiệu chuẩn trên laptop, thiếu mục thì thoát 1.

Chạy: python round3_final/tools/check_calib_docs.py [gốc round3_final]
Không suy ra số trang A4 từ chuỗi; phần đó vẫn cần duyệt tay.
"""
import argparse
import io
from pathlib import Path
import re
import sys
import tempfile


REQUIRED = (
    ("calibration_sheet.md", "phiếu", (
        "--mode calib", "--step 1", "--step 2", "--step 3", "--step 4", "--step 5",
        "--step 6", "--step 7", "--step 8", "--step 9", "--step 10",
        "env_check.sh", "thước", "3 m", "băng dính", "điện thoại", "bút")),
    ("calibration.md", "bảng", ("khoá config", "tof", "mất ảnh", "10/20", "500 ms", "model_fps.md")),
    ("ucr_car.md", "tài liệu xe", (
        "max_speed_cms", "(0, 100]", "camera_resume_max_s", "soft_start_s", "resume_stable_s", "[0.5, 30]",
        "phanh_tof_sau", "mat_anh", "dung_cho_anh_on_dinh", "--check-config --config", "cau_hinh_ok:",
        "số nguyên 0/1", "tof_mapping_confirmed",
        "xoá lệnh tốc độ đang giữ về 0", "ảnh có lại vẫn đứng yên tới khi người bấm lại",
        "smoke_gate.json", "cả bản thật lẫn", "stopcontroller", "config_text")),
)


def check_docs(root):
    missing = []
    for name, label, words in REQUIRED:
        text = (Path(root) / "docs" / name).read_text(encoding="utf-8").lower()
        for word in words:
            if word not in text:
                missing.append(label + ": " + word)
        if name == "calibration_sheet.md" and not re.search(
                r"(?m)^(?:#{1,6}\s+|(?:□|[-*])\s+)(?:\*\*)?không lùi", text):
            missing.append("đầu mục lùi bắt đầu bằng 'không lùi'")
        if name == "ucr_car.md" and not re.search(r"(?m)^.*\bmax_speed_cms\b[^\n]*\b25\b", text):
            missing.append("tài liệu xe: max_speed_cms kèm mặc định 25 cùng dòng")
        examples = re.findall(r"```[^\n]*\n(.*?)```|`([^`\n]*--[^`\n]*)`", text, re.S)
        command_text = "\n".join(block or inline for block, inline in examples)
        scoped = text if name == "calibration_sheet.md" else command_text
        for old in ("resume_key", "dung_chot_can_xac_nhan"):
            if any(old in line and "đã bỏ" not in line for line in scoped.splitlines()):
                missing.append(label + ": còn " + old + " trong phiếu/ví dụ lệnh")
        if re.search(r"--check-config\s+(?!--config\b)\S", command_text):
            missing.append(label + ": cú pháp check-config cũ")
    config = (Path(root) / "ucr_car/config.yaml").read_text(encoding="utf-8").lower()
    for old in ("resume_key", "dung_chot_can_xac_nhan"):
        if any(old in line and "đã bỏ" not in line for line in config.splitlines()):
            missing.append("config: còn " + old)
    return missing


def self_test():
    # Đối chứng đủ/thiếu từng chuỗi, đầu mục sai và thiếu file; không dùng tài liệu thật làm chuẩn.
    with tempfile.TemporaryDirectory(prefix="check_calib_docs_") as name:
        root = Path(name)
        (root / "docs").mkdir()
        (root / "ucr_car").mkdir()
        config = root / "ucr_car/config.yaml"
        config.write_text("max_speed_cms: 25\n", encoding="utf-8")
        for filename, _, words in REQUIRED:
            text = "\n".join(words) + "\n# Không lùi khi xe trên sa bàn\n"
            if filename == "ucr_car.md":
                text = text.replace("max_speed_cms\n", "max_speed_cms | 25\n")
            (root / "docs" / filename).write_text(text, encoding="utf-8")
        assert not check_docs(root), "Bản đủ chuỗi phải đạt"
        for filename, label, words in REQUIRED:
            path = root / "docs" / filename
            original = path.read_text(encoding="utf-8")
            for word in words:
                path.write_text(original.replace(word, ""), encoding="utf-8")
                assert label + ": " + word in check_docs(root), "Phải bắt chuỗi thiếu: " + word
            path.write_text(original, encoding="utf-8")
        sheet = root / "docs/calibration_sheet.md"
        original = sheet.read_text(encoding="utf-8")
        sheet.write_text(original.replace("# Không lùi", "# Lưu ý không lùi"), encoding="utf-8")
        assert check_docs(root) == ["đầu mục lùi bắt đầu bằng 'không lùi'"], "Phải kiểm đầu mục"
        sheet.write_text(original, encoding="utf-8")
        car = root / "docs/ucr_car.md"
        original_car = car.read_text(encoding="utf-8")
        car.write_text(original_car.replace("max_speed_cms | 25", "max_speed_cms\n25"), encoding="utf-8")
        assert "tài liệu xe: max_speed_cms kèm mặc định 25 cùng dòng" in check_docs(root), "25 rời rạc không đủ"
        car.write_text(original_car + "\nGhi chú lịch sử: đã bỏ resume_key/DUNG_CHOT_CAN_XAC_NHAN.\n", encoding="utf-8")
        assert not check_docs(root), "Cho phép ghi chú lịch sử ngoài ví dụ lệnh"
        for old in ("resume_key", "DUNG_CHOT_CAN_XAC_NHAN"):
            car.write_text(original_car + "\n```bash\nmain --" + old + "\n```\n", encoding="utf-8")
            assert check_docs(root), "Bắt tên đã bỏ trong ví dụ lệnh"
            car.write_text(original_car, encoding="utf-8")
            config.write_text(old + ": 1\n", encoding="utf-8")
            assert check_docs(root), "Bắt tên đã bỏ trong config"
            config.write_text("max_speed_cms: 25\n", encoding="utf-8")
        car.write_text(original_car + "\n`main --check-config file.yaml`\n", encoding="utf-8")
        assert "tài liệu xe: cú pháp check-config cũ" in check_docs(root), "Bắt cú pháp F cũ"
        car.write_text(original_car, encoding="utf-8")
        sheet.unlink()
        try:
            check_docs(root)
        except OSError:
            pass
        else:
            raise AssertionError("Thiếu file không được đạt")
    print("self-test docs: ĐẠT")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", nargs="?", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--self-test", action="store_true", help="Tự kiểm bằng tài liệu đối chứng tạm")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return 0
    missing = check_docs(args.root)
    if missing:
        print("[HỎNG] docs thiếu: " + "; ".join(missing), file=sys.stderr)
        return 1
    print("docs hiệu chuẩn: ĐẠT")
    return 0


if __name__ == "__main__":
    # Terminal Windows có thể mặc định bảng mã không in được thông báo tiếng Việt.
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8")
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8")
    try:
        sys.exit(main())
    except (OSError, ValueError) as error:
        print("[HỎNG] " + str(error), file=sys.stderr)
        sys.exit(1)
