"""Kiểm tra ảnh thu, xuất đề xuất loại và ảnh lưới; giữ nguyên dữ liệu nguồn."""
from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sys
from collections import Counter
from pathlib import Path


NAME = re.compile(
    r"(?P<session>[a-z]\d{8})_p(?P<pass>\d{3,})_l(?P<luot>[1-9])_"
    r"t(?P<target>\d{2,})_(?P<index>\d{4,})\.jpg$"
)


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__, add_help=False)
    p.add_argument("-h", "--help", action="help", help="Hiện trợ giúp rồi thoát")
    p._positionals.title = "Tham số vị trí"
    p._optionals.title = "Tùy chọn"
    p.add_argument("source", type=Path, help="Thư mục phiên chứa ảnh gốc và capture_log.csv")
    p.add_argument("--csv", type=Path, help="Nhật ký khác; mặc định source/capture_log.csv")
    p.add_argument("--output", type=Path, help="Thư mục kết quả mới; mặc định cạnh source, thêm _review")
    p.add_argument("--blur-threshold", type=float, default=50.0, help="Độ nét Laplacian tối thiểu (mặc định 50)")
    p.add_argument("--dark-threshold", type=float, default=35.0, help="Độ sáng trung bình tối thiểu, thang 0–255")
    p.add_argument("--bright-threshold", type=float, default=220.0, help="Độ sáng trung bình tối đa, thang 0–255")
    p.add_argument("--duplicate-threshold", type=float, default=0.8, help="Sai khác xám trung bình dưới ngưỡng: gần trùng")
    p.add_argument("--compare-scale", type=float, default=0.25, help="Tỉ lệ thu nhỏ ảnh để so trùng, trong (0, 1]")
    p.add_argument("--sheet-size", type=int, default=48, help="Số ảnh mỗi trang lưới (1–100)")
    return p


def analyze(args: argparse.Namespace) -> dict:
    import numpy as np
    from PIL import Image, ImageDraw

    source = args.source.resolve()
    csv_path = (args.csv or source / "capture_log.csv").resolve()
    output = (args.output or source.with_name(source.name + "_review")).resolve()
    if not source.is_dir():
        raise ValueError(f"Không có thư mục nguồn: {source}")
    if output == source or source in output.parents:
        raise ValueError("Thư mục kết quả phải nằm ngoài thư mục ảnh nguồn")
    if (not all(math.isfinite(getattr(args, key)) for key in
                ("blur_threshold", "duplicate_threshold", "dark_threshold", "bright_threshold", "compare_scale"))
            or not 0 <= args.blur_threshold or not 0 <= args.duplicate_threshold <= 255
            or not 0 <= args.dark_threshold < args.bright_threshold <= 255
            or not 0 < args.compare_scale <= 1 or not 1 <= args.sheet_size <= 100):
        raise ValueError("Ngưỡng hoặc số ảnh mỗi trang không hợp lệ")
    with csv_path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"frame", "session", "pass", "luot", "target"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f"CSV thiếu cột: {sorted(required - set(reader.fieldnames or []))}")
        logged = list(reader)
    log_by_frame = {}
    repeated_rows = []
    for line_number, row in enumerate(logged, 2):
        if any(row.get(key) is None or row[key] == "" for key in required):
            raise ValueError(f"CSV dòng {line_number} thiếu giá trị bắt buộc")
        if row["frame"] in log_by_frame:
            repeated_rows.append(row["frame"])
        log_by_frame[row["frame"]] = row
    def image_order(path):
        match = NAME.fullmatch(path.name)
        if match:
            return (match["session"], int(match["pass"]), int(match["index"]), path.name)
        return ("", 0, 0, path.name)

    files = sorted((p for p in source.iterdir() if p.suffix.lower() in {".jpg", ".jpeg", ".png"}), key=image_order)
    if not files:
        raise ValueError("Không tìm thấy ảnh trong thư mục phiên")
    # Không ghi đè kết quả đã có, kể cả khi người dùng chọn nhầm thư mục.
    output.mkdir(parents=True, exist_ok=False)
    sheets = output / "contact_sheets"
    sheets.mkdir()

    def read_image(path):
        try:
            with Image.open(path) as image:
                return image.convert("RGB")
        except OSError:
            return None

    def write_image(path, image):
        Image.fromarray(image).save(path, quality=90)

    records, counts, anchors = [], Counter(), {}
    thumbnails = []
    page = 0

    def flush_sheet():
        nonlocal page
        if not thumbnails:
            return
        page += 1
        columns = min(4, len(thumbnails))
        canvas = np.full((((len(thumbnails) + columns - 1) // columns) * 160, columns * 280, 3), 35, np.uint8)
        for index, tile in enumerate(thumbnails):
            y, x = (index // columns) * 160, (index % columns) * 280
            canvas[y:y + 160, x:x + 280] = tile
        write_image(sheets / f"review_{page:04d}.jpg", canvas)
        thumbnails.clear()

    for path in files:
        reasons = []
        match = NAME.fullmatch(path.name)
        info = match.groupdict() if match else dict(session="?", **{"pass": "?"}, luot="?", target="?", index="0")
        if not match:
            reasons.append("sai_ten")
        row = log_by_frame.get(path.name)
        if row is None:
            reasons.append("thieu_csv")
        elif match:
            try:
                consistent = row["session"] == info["session"] and all(
                    int(row[key]) == int(info[key]) for key in ("pass", "luot", "target"))
            except (TypeError, ValueError):
                consistent = False
            if not consistent:
                reasons.append("csv_khong_khop_ten")
        group = tuple(info[key] for key in ("session", "pass", "luot", "target"))
        counts[group] += 1
        image = read_image(path)
        sharpness = brightness = difference = ""
        duplicate_of = ""
        tile = np.zeros((160, 280, 3), np.uint8)
        if image is None:
            reasons.append("anh_loi")
        else:
            gray_image = image.convert("L")
            gray = np.asarray(gray_image, dtype=np.float64)
            # Laplacian bốn láng giềng; đệm phản chiếu để biên ảnh không tạo độ nét giả.
            padded = np.pad(gray, 1, mode="reflect")
            laplacian = (padded[:-2, 1:-1] + padded[2:, 1:-1]
                         + padded[1:-1, :-2] + padded[1:-1, 2:] - 4 * gray)
            sharpness = float(laplacian.var())
            brightness = float(gray.mean())
            if sharpness < args.blur_threshold:
                reasons.append("mo")
            if brightness < args.dark_threshold:
                reasons.append("toi")
            if brightness > args.bright_threshold:
                reasons.append("sang")
            small = np.asarray(gray_image.resize((max(1, round(gray.shape[1] * args.compare_scale)),
                                                  max(1, round(gray.shape[0] * args.compare_scale))),
                                                 Image.Resampling.BOX), dtype=np.float32)
            previous = anchors.get(group) if match else None
            if previous is not None and previous[1].shape == small.shape:
                difference = float(np.abs(small - previous[1]).mean())
                if difference < args.duplicate_threshold:
                    reasons.append("gan_trung")
                    duplicate_of = previous[0]
            if not duplicate_of and match:
                anchors[group] = (path.name, small)
            ratio = min(280 / image.width, 115 / image.height)
            preview = np.asarray(image.resize((max(1, round(image.width * ratio)), max(1, round(image.height * ratio)))))
            tile[:preview.shape[0], :preview.shape[1]] = preview
        tile_image = Image.fromarray(tile)
        draw = ImageDraw.Draw(tile_image)
        draw.text((3, 120), path.name, fill=(255, 255, 255))
        draw.text((3, 140), ";".join(reasons) or "dat", fill=(255, 60, 40) if reasons else (60, 255, 60))
        tile = np.asarray(tile_image)
        thumbnails.append(tile)
        if len(thumbnails) == args.sheet_size:
            flush_sheet()
        records.append(dict(frame=path.name, session=info["session"], **{"pass": info["pass"]},
                            luot=info["luot"], target=info["target"], sharpness=sharpness,
                            brightness=brightness, difference=difference, duplicate_of=duplicate_of,
                            reasons=";".join(reasons)))
    flush_sheet()
    with (output / "filter_report.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    proposals = [r["frame"] for r in records if r["reasons"]]
    (output / "suggested_drop.txt").write_text("".join(name + "\n" for name in proposals), encoding="utf-8")
    chains = {}
    for r in records:
        if r["duplicate_of"]:
            chains.setdefault(r["duplicate_of"], []).append(r["frame"])
    present = {p.name for p in files}
    summary = dict(
        source=str(source), images=len(files), suggested_drop=len(proposals),
        counts=[dict(session=g[0], **{"pass": g[1]}, luot=g[2], target=g[3], images=n) for g, n in sorted(counts.items())],
        reasons=dict(Counter(reason for r in records for reason in r["reasons"].split(";") if reason)),
        missing_images=sorted(set(log_by_frame) - present), duplicate_csv_rows=repeated_rows,
        duplicate_chains=chains,
        thresholds={key: getattr(args, key) for key in ("blur_threshold", "dark_threshold", "bright_threshold",
                                                       "duplicate_threshold", "compare_scale")},
    )
    (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"Đã kiểm tra {len(files)} ảnh; đề xuất xem lại {len(proposals)} ảnh. Kết quả: {output}")
    for group, count in sorted(counts.items()):
        print(f"Phiên {group[0]}, đợt {group[1]}, lượt {group[2]}, mục tiêu {group[3]}: {count} ảnh")
    print(f"Ảnh thiếu so với CSV: {len(summary['missing_images'])}; dòng CSV lặp: {len(repeated_rows)}")
    return summary


def main() -> int:
    # Console Windows có thể mặc định cp1258 và không in được đủ dấu tiếng Việt.
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8")
    args = parser().parse_args()
    try:
        analyze(args)
    except (OSError, ValueError, ImportError) as error:
        print(f"Lỗi: {error}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
