"""Kiểm bản _cvdnn khớp ONNX gốc: tỉ lệ trùng argmax >= 99,9% trên >= 20 ảnh thật.

Hai bước vì cần hai bản OpenCV khác nhau (Python 3.6+, numpy, cv2; onnxruntime nếu có):
  1) python check_cvdnn_argmax.py ref  --onnx road.onnx --images THƯ_MỤC_ẢNH --out ref.npz
     (OpenCV mới hoặc onnxruntime chạy file GỐC; lưu cả đầu vào đã tiền xử lý)
  2) <python có opencv-python 4.1.1> check_cvdnn_argmax.py test --onnx road_cvdnn.onnx --ref ref.npz
     (OpenCV 4.1.1 DNN chạy bản _cvdnn trên CÙNG đầu vào rồi so)
Tiền xử lý theo _info.json: cắt crop_y/crop_x, INTER_AREA về input_hw, BGR->RGB, /255, NCHW.
_info.json có fake_data=true, hoặc dùng --untrained: chỉ kiểm đường chạy, in "CHƯA CHỨNG MINH".
"""
import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np

THRESHOLD = 0.999
MIN_IMAGES = 20


def info_for(onnx_path):
    stem = onnx_path.name[:-len('.onnx')]
    if stem.endswith('_cvdnn'):
        stem = stem[:-len('_cvdnn')]
    return json.loads(onnx_path.with_name(stem + '_info.json').read_text(encoding='utf-8'))


def preprocess(path, info):
    image = cv2.imread(str(path))
    if image is None:
        raise SystemExit('Không đọc được ảnh ' + str(path))
    if list(image.shape[:2]) != list(info['source_image_hw']):
        raise SystemExit('{} cỡ {} khác source_image_hw {}'.format(path.name, image.shape[:2], info['source_image_hw']))
    (y0, y1), (x0, x1), (h, w) = info['crop_y'], info['crop_x'], info['input_hw']
    small = cv2.resize(image[y0:y1, x0:x1], (w, h), interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
    return np.ascontiguousarray(rgb.transpose(2, 0, 1)[None], dtype=np.float32) / 255.0


def run_cv(onnx_path, inputs):
    net = cv2.dnn.readNetFromONNX(str(onnx_path))
    outputs = []
    for x in inputs:
        net.setInput(x)
        outputs.append(net.forward().copy())
    return np.concatenate(outputs)


def run_reference(onnx_path, inputs):
    try:
        import onnxruntime
    except ImportError:
        return run_cv(onnx_path, inputs), 'cv2 ' + cv2.__version__
    session = onnxruntime.InferenceSession(str(onnx_path), providers=['CPUExecutionProvider'])
    name = session.get_inputs()[0].name
    return np.concatenate([session.run(None, {name: x})[0] for x in inputs]), 'onnxruntime ' + onnxruntime.__version__


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('step', choices=('ref', 'test'))
    parser.add_argument('--onnx', type=Path, required=True)
    parser.add_argument('--images', type=Path, help='Bước ref: thư mục ảnh thật 1280x720')
    parser.add_argument('--out', type=Path, help='Bước ref: tệp .npz ghi ra')
    parser.add_argument('--ref', type=Path, help='Bước test: .npz của bước ref')
    parser.add_argument('--limit', type=int, default=60)
    parser.add_argument('--untrained', action='store_true', help='Model chưa train/giả: chỉ kiểm đường chạy')
    args = parser.parse_args()
    info = info_for(args.onnx)
    if args.step == 'ref':
        if args.onnx.name.endswith('_cvdnn.onnx') or not args.images or not args.out:
            raise SystemExit('Bước ref cần --onnx GỐC, --images, --out')
        paths = sorted(p for p in args.images.iterdir() if p.suffix.lower() in ('.jpg', '.jpeg', '.png'))[:args.limit]
        inputs = [preprocess(p, info) for p in paths]
        logits, engine = run_reference(args.onnx, inputs)
        np.savez_compressed(str(args.out), inputs=np.concatenate(inputs), logits=logits,
                            names=np.array([p.name for p in paths]), engine=np.array(engine))
        print('Đã lưu {} ảnh, tham chiếu bằng {}'.format(len(paths), engine))
        return
    if not args.onnx.name.endswith('_cvdnn.onnx') or not args.ref:
        raise SystemExit('Bước test cần --onnx <tên>_cvdnn.onnx và --ref')
    ref = np.load(str(args.ref))
    inputs = list(ref['inputs'][:, None])
    logits = run_cv(args.onnx, [np.ascontiguousarray(x) for x in inputs])
    agree = (logits.argmax(1) == ref['logits'].argmax(1)).mean(axis=(1, 2))
    worst = int(agree.argmin())
    print('Tham chiếu: {} | thử: cv2 {} | {} ảnh'.format(ref['engine'], cv2.__version__, len(agree)))
    print('Trùng argmax: trung bình {:.4%}, thấp nhất {:.4%} ({})'.format(agree.mean(), agree.min(), ref['names'][worst]))
    print('Sai lệch logits lớn nhất: {:.5f}'.format(float(np.abs(logits - ref['logits']).max())))
    ok = len(agree) >= MIN_IMAGES and agree.min() >= THRESHOLD
    unproven = args.untrained or info.get('fake_data', False)
    if unproven:
        print('KẾT LUẬN: đường chạy {} - CHƯA CHỨNG MINH (model giả/chưa train)'.format('đạt' if ok else 'KHÔNG đạt'))
    else:
        print('KẾT LUẬN: ' + ('ĐẠT' if ok else 'KHÔNG ĐẠT') +
              ' (cần >= {} ảnh, mọi ảnh trùng >= {:.1%})'.format(MIN_IMAGES, THRESHOLD))
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
