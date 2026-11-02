"""Đổi ONNX đường S1-S3 (opset 12, Resize) sang bản OpenCV 4.1.1 DNN đọc được: <tên>_cvdnn.onnx.

Chỉ dùng cho chương trình xem model ucr_test_model. KHÔNG thay file gốc: file gốc vẫn dùng cho TensorRT.
Cách đổi: mọi Resize ×2 bilinear → Upsample opset 7 (scales ở thuộc tính), bỏ hằng rỗng của Resize.
Từ chối S4 (mbv3_unet) và mọi đồ thị có toán tử/Resize khác dạng S1-S3.

Chạy trên laptop (cần gói `onnx`):  python onnx_to_cvdnn.py đường/road_S1_s0.onnx
Kiểm sau khi đổi: check_cvdnn_argmax.py.
"""
import argparse
import json
from pathlib import Path
import sys

ALLOWED_OPS = {'Constant', 'Conv', 'Relu', 'MaxPool', 'Resize', 'Concat'}
ALLOWED_ARCH = {'unet_b_minus', 'nanounet'}
ALLOWED_VARIANTS = {'S1', 'S2', 'S3'}


class Refused(Exception):
    pass


def check_info(info_path):
    if not info_path.exists():
        raise Refused('Thiếu ' + info_path.name + ' cạnh ONNX (cần để biết biến thể S1-S3)')
    info = json.loads(info_path.read_text(encoding='utf-8'))
    if info.get('architecture') not in ALLOWED_ARCH or info.get('imagenet_normalization_in_model'):
        raise Refused('Chỉ hỗ trợ S1-S3 (unet_b_minus/nanounet); gặp ' + str(info.get('architecture')))
    if 'variant' in info and info['variant'] not in ALLOWED_VARIANTS:
        raise Refused('Chỉ hỗ trợ S1-S3; gặp biến thể ' + str(info['variant']))


def convert(model):
    import onnx
    from onnx import helper, numpy_helper, shape_inference
    graph = model.graph
    if [o.version for o in model.opset_import if o.domain in ('', 'ai.onnx')] != [12]:
        raise Refused('Cần ONNX opset 12 như notebook xuất')
    ops = {n.op_type for n in graph.node}
    if not ops <= ALLOWED_OPS:
        raise Refused('Toán tử ngoài dạng S1-S3: ' + ', '.join(sorted(ops - ALLOWED_OPS)))
    inferred = shape_inference.infer_shapes(model)
    shapes = {v.name: [d.dim_value for d in v.type.tensor_type.shape.dim]
              for v in list(inferred.graph.value_info) + list(inferred.graph.input) + list(inferred.graph.output)}
    nodes, count = [], 0
    for n in graph.node:
        if n.op_type != 'Resize':
            nodes.append(n)
            continue
        attrs = {a.name: helper.get_attribute_value(a) for a in n.attribute}
        mode = attrs.get('mode', b'nearest')
        if mode != b'linear' or attrs.get('coordinate_transformation_mode') != b'half_pixel':
            raise Refused('Resize không phải bilinear half_pixel')
        src, dst = shapes.get(n.input[0]), shapes.get(n.output[0])
        if not src or not dst or len(src) != 4 or dst != [src[0], src[1], src[2] * 2, src[3] * 2]:
            raise Refused('Resize không đúng ×2: {} -> {}'.format(src, dst))
        nodes.append(helper.make_node('Upsample', [n.input[0]], list(n.output), name=n.name,
                                      mode='bilinear', scales=[1.0, 1.0, 2.0, 2.0]))
        count += 1
    if not count:
        raise Refused('Không có Resize nào; không phải đồ thị U-Net S1-S3')
    used = {i for n in nodes for i in n.input}
    nodes = [n for n in nodes if n.op_type != 'Constant' or n.output[0] in used]
    for n in nodes:
        if n.op_type == 'Constant' and numpy_helper.to_array(n.attribute[0].t).size == 0:
            raise Refused('Còn hằng rỗng sau khi đổi')
    del graph.node[:]
    graph.node.extend(nodes)
    for opset in model.opset_import:
        if opset.domain in ('', 'ai.onnx'):
            opset.version = 7
    model.producer_name = 'onnx_to_cvdnn (tu ' + (model.producer_name or 'khong ro') + ')'
    onnx.checker.check_model(model)
    return count


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('onnx', type=Path, help='ONNX gốc <tên>.onnx (không bị sửa)')
    parser.add_argument('--info', type=Path, help='Mặc định <tên>_info.json cạnh ONNX')
    args = parser.parse_args()
    src = args.onnx
    if src.name.endswith('_cvdnn.onnx') or src.suffix != '.onnx':
        sys.exit('Cần file .onnx gốc, không phải bản _cvdnn')
    out = src.with_name(src.stem + '_cvdnn.onnx')
    try:
        import onnx
        check_info(args.info or src.with_name(src.stem + '_info.json'))
        model = onnx.load(str(src))
        count = convert(model)
    except Refused as error:
        sys.exit('TỪ CHỐI: ' + str(error))
    onnx.save(model, str(out))
    print('Đã ghi {} ({} Resize -> Upsample opset 7). File gốc giữ nguyên cho TensorRT.'.format(out, count))
    print('Bắt buộc kiểm: check_cvdnn_argmax.py trên >= 20 ảnh thật trước khi tin hình hiển thị.')


if __name__ == '__main__':
    main()
