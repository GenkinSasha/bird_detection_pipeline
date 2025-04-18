Goal: Detect a Bird in a Video Stream and Send a ROS Message

Pipeline Logic
[MP4 File] → [DeepStream GStreamer Pipeline] → [Bird Detection Event] → [ROS Publisher]

- Based on nvidia image (nvcr.io/nvidia/deepstream:7.0-gc-triton-devel) contains CUDA 12.2, DeepStream 7.0. OS - Ubuntu 22.04
- Added YOLOv5s tuned for bird detection, converted to ONNX format to fit DeepStream requirements.
- Added ROS2 Humble to publish ROS messages

Birds detection should start to run on container running (use sudo docker run --gpus all --rm bird_detection_container)

- test mp4 video is in media/
- Everything is running on Google VM with Nvidia T4 GPU enabled 50GB storage.
- to build the container: sudo docker build -t bird_detection_container .

Debugging:

sudo docker run -it --gpus all --entrypoint /bin/bash bird_detection_container

cd /opt/bird_detection_app
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run bird_detection_app bird_publisher /opt/media/birds.mp4 /opt/models/config_infer_primary.txt
or
python3 -m bird_detection_app.bird_publisher /opt/media/birds.mp4 /opt/models/config_infer_primary.txt /opt/models/labels.txt

