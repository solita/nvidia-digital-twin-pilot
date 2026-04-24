FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip libgl1 libglib2.0-0 && \
    rm -rf /var/lib/apt/lists/*

# PyTorch with CUDA 12.1 support (includes bundled CUDA runtime)
RUN pip3 install --no-cache-dir \
    torch torchvision --index-url https://download.pytorch.org/whl/cu121

# YOLOv8 + headless OpenCV for JPEG decoding
RUN pip3 install --no-cache-dir ultralytics opencv-python-headless

COPY yolo_detector_node.py /app/

CMD ["python3", "-u", "/app/yolo_detector_node.py"]
