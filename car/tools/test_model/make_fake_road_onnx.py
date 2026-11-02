#!/usr/bin/env python3
"""Tạo ONNX giả 3 lớp cho test (chỉ thư viện chuẩn, Python 3.6; không chạy trên xe).

--format resize12: đúng dạng notebook xuất (opset 12, Resize có hằng rỗng).
--format cvdnn:    dạng OpenCV 4.1.1 đọc được (opset 7, Upsample ×2 bilinear).
Trọng số ngẫu nhiên có hạt giống: KHÔNG chứng minh chất lượng model.
"""
import argparse
import json
from pathlib import Path
import random
import struct

FLOAT, INT64 = 1, 7
ATTR_FLOAT, ATTR_INT, ATTR_STRING, ATTR_TENSOR, ATTR_INTS, ATTR_FLOATS = 1, 2, 3, 4, 7, 6


def varint(value):
    out = bytearray()
    while True:
        byte = value & 0x7f
        value >>= 7
        out.append(byte | (0x80 if value else 0))
        if not value:
            return bytes(out)


def number(field, value):
    return varint(field << 3) + varint(value)


def blob(field, data):
    if isinstance(data, str):
        data = data.encode('utf-8')
    return varint(field << 3 | 2) + varint(len(data)) + data


def tensor(name, dims, kind, values):
    code = '<%d%s' % (len(values), 'f' if kind == FLOAT else 'q')
    body = b''.join(number(1, d) for d in dims) + number(2, kind) + blob(8, name)
    return body + blob(9, struct.pack(code, *values))


def attr_int(name, value):
    return blob(1, name) + number(3, value) + number(20, ATTR_INT)


def attr_ints(name, values):
    return blob(1, name) + b''.join(number(8, v) for v in values) + number(20, ATTR_INTS)


def attr_str(name, value):
    return blob(1, name) + blob(4, value) + number(20, ATTR_STRING)


def attr_floats(name, values):
    return blob(1, name) + blob(7, struct.pack('<%df' % len(values), *values)) + number(20, ATTR_FLOATS)


def attr_tensor(name, body):
    return blob(1, name) + blob(5, body) + number(20, ATTR_TENSOR)


def node(op, inputs, outputs, attrs=()):
    return (b''.join(blob(1, i) for i in inputs) + b''.join(blob(2, o) for o in outputs) +
            blob(3, outputs[0] + '_node') + blob(4, op) + b''.join(blob(5, a) for a in attrs))


def value_info(name, shape):
    dims = b''.join(blob(1, number(1, d)) for d in shape)
    return blob(1, name) + blob(2, blob(1, number(1, FLOAT) + blob(2, dims)))


class Graph:
    def __init__(self, fmt, seed):
        self.fmt, self.rng = fmt, random.Random(seed)
        self.nodes, self.inits, self.count = [], [], 0

    def fresh(self, prefix):
        self.count += 1
        return '%s_%d' % (prefix, self.count)

    def conv(self, source, a, b, kernel, name=None):
        name = name or self.fresh('conv')
        weights, bias = name + '_w', name + '_b'
        limit = 0.6 / (a * kernel * kernel) ** 0.5
        data = [self.rng.uniform(-limit, limit) for _ in range(a * b * kernel * kernel)]
        self.inits.append(tensor(weights, [b, a, kernel, kernel], FLOAT, data))
        self.inits.append(tensor(bias, [b], FLOAT, [self.rng.uniform(-0.1, 0.1) for _ in range(b)]))
        pad = kernel // 2
        self.nodes.append(node('Conv', [source, weights, bias], [name], [
            attr_ints('dilations', [1, 1]), attr_int('group', 1), attr_ints('kernel_shape', [kernel, kernel]),
            attr_ints('pads', [pad] * 4), attr_ints('strides', [1, 1])]))
        return name

    def relu_conv(self, source, a, b):
        name = self.fresh('relu')
        self.nodes.append(node('Relu', [self.conv(source, a, b, 3)], [name]))
        return name

    def pool(self, source):
        name = self.fresh('pool')
        self.nodes.append(node('MaxPool', [source], [name], [attr_ints('kernel_shape', [2, 2]),
                                                             attr_ints('strides', [2, 2])]))
        return name

    def up(self, source, channels, height, width):
        name = self.fresh('up')
        if self.fmt == 'cvdnn':
            self.nodes.append(node('Upsample', [source], [name], [attr_str('mode', 'bilinear'),
                                                                  attr_floats('scales', [1.0, 1.0, 2.0, 2.0])]))
            return name
        empty, sizes = name + '_empty', name + '_sizes'
        self.nodes.append(node('Constant', [], [empty], [attr_tensor('value', tensor(empty, [0], FLOAT, []))]))
        self.nodes.append(node('Constant', [], [sizes], [
            attr_tensor('value', tensor(sizes, [4], INT64, [1, channels, height, width]))]))
        self.nodes.append(node('Resize', [source, empty, empty, sizes], [name], [
            attr_str('coordinate_transformation_mode', 'half_pixel'), attr_str('mode', 'linear'),
            attr_str('nearest_mode', 'floor')]))
        return name

    def concat(self, first, second):
        name = self.fresh('cat')
        self.nodes.append(node('Concat', [first, second], [name], [attr_int('axis', 1)]))
        return name


def build(fmt, height, width, seed):
    """U-Net (4,8), đủ chính xác loại toán tử của S1-S3 với kích thước tệp rất nhỏ."""
    g = Graph(fmt, seed)
    top = g.relu_conv('images', 3, 4)
    low = g.relu_conv(g.pool(top), 4, 8)
    up = g.up(low, 8, height, width)
    mix = g.relu_conv(g.concat(up, top), 12, 4)
    g.conv(mix, 4, 3, 1, 'logits')
    graph = (b''.join(blob(1, n) for n in g.nodes) + blob(2, 'fake_road') + b''.join(blob(5, i) for i in g.inits) +
             blob(11, value_info('images', [1, 3, height, width])) +
             blob(12, value_info('logits', [1, 3, height, width])))
    opset = number(2, 7 if fmt == 'cvdnn' else 12)
    return number(1, 6) + blob(2, 'make_fake_road_onnx') + blob(7, graph) + blob(8, opset)


def info(height, width):
    return {'task': 'segmentation', 'architecture': 'unet_b_minus', 'class_names': ['background', 'track_surface', 'line'],
            'num_classes': 3, 'mask_values': {'background': 0, 'track_surface': 1, 'line': 2},
            'source_image_hw': [720, 1280], 'crop_y': [320, 720], 'crop_x': [0, 1280],
            'resize': 'cv2.resize INTER_AREA (crop -> input)', 'input_name': 'images',
            'input_shape': [1, 3, height, width], 'input_hw': [height, width], 'input_dtype': 'float32',
            'color_order': 'RGB', 'layout': 'NCHW', 'normalization': {'scale': 1 / 255, 'mean': [0, 0, 0],
                                                                      'std': [1, 1, 1]},
            'imagenet_normalization_in_model': False, 'preprocessing_in_model': False, 'output_name': 'logits',
            'output_shape': [1, 3, height, width], 'output_type': 'logits',
            'activation': 'softmax_over_channels_argmax', 'opset': 12, 'dynamic': False, 'fake_data': True,
            'variant': 'S1'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True, help='Đường dẫn .onnx; _info.json ghi cạnh tệp')
    parser.add_argument('--format', choices=('resize12', 'cvdnn'), default='resize12')
    parser.add_argument('--hw', type=int, nargs=2, default=(96, 160))
    parser.add_argument('--seed', type=int, default=0)
    args = parser.parse_args()
    height, width = args.hw
    out = Path(args.out)
    out.write_bytes(build(args.format, height, width, args.seed))
    stem = out.name[:-len('.onnx')]
    if stem.endswith('_cvdnn'):
        stem = stem[:-len('_cvdnn')]
    out.with_name(stem + '_info.json').write_text(json.dumps(info(height, width), indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
