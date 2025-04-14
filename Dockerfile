# Base DeepStream 7.0 with Triton
FROM nvcr.io/nvidia/deepstream:7.0-gc-triton-devel

ENV DEBIAN_FRONTEND=noninteractive

# Install base dependencies
RUN apt-get update && apt-get install -y \
    locales curl gnupg2 lsb-release python3-pip \
    build-essential cmake git && \
    rm -rf /var/lib/apt/lists/*

# Locale setup for ROS 2
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Add ROS 2 Humble repo and key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    echo "deb [arch=amd64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update

# Install ROS 2 core and required tools
RUN apt-get install -y \
    ros-humble-ros-core \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# YOLOv5 setup
WORKDIR /opt
RUN git clone https://github.com/ultralytics/yolov5 && \
    pip3 install --no-cache-dir -r yolov5/requirements.txt && \
    python3 yolov5/export.py --weights yolov5s.pt --include onnx

# pin setuptools version to 68
RUN pip install --no-cache-dir setuptools==68.2.2

# Copy bird detection app
COPY bird_detection_app /opt/bird_detection_app
WORKDIR /opt/bird_detection_app
RUN . /opt/ros/humble/setup.sh && colcon build

# Set environment
ENV LD_LIBRARY_PATH=/opt/nvidia/deepstream/deepstream-7.0/lib:$LD_LIBRARY_PATH
ENV ROS_DISTRO=humble
SHELL ["/bin/bash", "-c"]

# Copy media and models
COPY media /opt/media
COPY models /opt/models

# Launch app at container start
#CMD source /opt/ros/$ROS_DISTRO/setup.bash && \
#    source install/setup.bash && \
#    ros2 run bird_detection_app bird_publisher /opt/media/birds.mp4 /opt/models/config_infer_primary.txt

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run bird_detection_app bird_publisher /opt/media/birds.mp4 /opt/models/config_infer_primary.txt"]
