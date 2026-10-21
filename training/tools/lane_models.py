"""Hai mạng phân vùng giữ nguyên định nghĩa và tên trọng số của notebook xe thật.

Lưu ý: các kiến trúc trong module này chỉ là tạm thời, dùng để chạy thử quy trình
xuất ONNX và so sánh kết quả. Kiến trúc chính thức sẽ được chọn sau khi có sa bàn,
thể lệ và số đo FPS trên Jetson Nano.
"""

try:
    import torch
    from torch import nn
    import torch.nn.functional as F
except ImportError as exc:
    raise ImportError("Thiếu PyTorch; cài trên Ubuntu/Colab: python -m pip install torch") from exc


class DoubleConv(nn.Sequential):
    """Hai tích chập 3×3, mỗi tích chập theo sau bởi ReLU."""

    def __init__(self, a, b):
        super().__init__(nn.Conv2d(a, b, 3, padding=1), nn.ReLU(inplace=False),
                         nn.Conv2d(b, b, 3, padding=1), nn.ReLU(inplace=False))


class SmallUNet(nn.Module):
    """Mã hoá, giải mã với nối tắt; đầu ra là điểm trước sigmoid."""

    def __init__(self, widths, nc):
        super().__init__()
        self.enc = nn.ModuleList([DoubleConv(a, b) for a, b in
                                  zip([3] + list(widths[:-1]), widths)])
        self.dec = nn.ModuleList([DoubleConv(widths[i + 1] + widths[i], widths[i])
                                  for i in reversed(range(len(widths) - 1))])
        self.head = nn.Conv2d(widths[0], nc, 1)

    def forward(self, x):
        skips = []
        for i, block in enumerate(self.enc):
            if i:
                x = F.max_pool2d(x, 2)
            x = block(x)
            skips.append(x)
        for block, skip in zip(self.dec, reversed(skips[:-1])):
            x = F.interpolate(x, size=skip.shape[-2:], mode="bilinear", align_corners=False)
            x = block(torch.cat([x, skip], dim=1))
        return self.head(x)


class NanoUNet(SmallUNet):
    def __init__(self, nc=1):
        super().__init__((16, 32, 64, 128), nc)


class UNetBMinus(SmallUNet):
    def __init__(self, nc=1):
        super().__init__((8, 16, 32), nc)


def build_model(arch, nc=1):
    """Không đoán kiến trúc hay chuyển đổi trọng số cũ theo tên gần giống."""
    choices = {"nanounet": NanoUNet, "unet_b_minus": UNetBMinus}
    if arch not in choices:
        raise ValueError("Kiến trúc không hỗ trợ: {}".format(arch))
    return choices[arch](nc)
