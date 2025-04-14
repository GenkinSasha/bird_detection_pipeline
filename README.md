Goal: Detect a Bird in a Video Stream and Send a ROS Message

Pipeline Logic
[MP4 File] → [DeepStream GStreamer Pipeline] → [Bird Detection Event] → [ROS Publisher]

- Based on nvidia image contains CUDA 12.2, DeepStream 7.0. OS - Ubuntu 22.04
- Added YOLOv5s tuned for bird detection, converted to ONNX format to fit DeepStream requirements.
- Added ROS2 Humble to publish ROS messages

Birds detection should start to run on container running (use sudo docker run --gpus all --rm bird_detection_container)

- test mp4 video is in media/
