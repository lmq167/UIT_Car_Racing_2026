# Môi trường kiểm tra biên dịch kiểu xe: Ubuntu 18.04 (GCC 7, CMake 3.10) + OpenCV 4.1.1.
# OpenCV chỉ dựng các module mã xe dùng, tắt CUDA, Python, test, ví dụ, cài vào /opt/opencv-4.1.1.
# Dựng image:  bash build_image.sh   (cần Internet để tải mã nguồn OpenCV, khoảng 20-40 phút)
FROM ubuntu:18.04

ARG OPENCV_VERSION=4.1.1
ARG JOBS=8
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential cmake pkg-config ca-certificates wget \
        libgtk2.0-dev libjpeg-dev libpng-dev libi2c-dev \
    && rm -rf /var/lib/apt/lists/*

RUN wget -q -O /tmp/opencv.tar.gz \
        "https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.tar.gz" \
    && tar xzf /tmp/opencv.tar.gz -C /tmp \
    && mkdir /tmp/opencv-build \
    && cd /tmp/opencv-build \
    && cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/opt/opencv-${OPENCV_VERSION} \
        -DBUILD_LIST=core,imgproc,imgcodecs,videoio,highgui,dnn \
        -DWITH_CUDA=OFF \
        -DBUILD_opencv_python2=OFF \
        -DBUILD_opencv_python3=OFF \
        -DBUILD_JAVA=OFF \
        -DBUILD_TESTS=OFF \
        -DBUILD_PERF_TESTS=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_DOCS=OFF \
        /tmp/opencv-${OPENCV_VERSION} \
    && make -j"${JOBS}" \
    && make install \
    && cd / \
    && rm -r /tmp/opencv-build /tmp/opencv-${OPENCV_VERSION} /tmp/opencv.tar.gz

ENV OpenCV_DIR=/opt/opencv-${OPENCV_VERSION}/lib/cmake/opencv4
