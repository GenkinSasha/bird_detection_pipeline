# Base DeepStream 7.0 with Triton
FROM nvcr.io/nvidia/deepstream:7.0-gc-triton-devel

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=en_US.UTF-8

# Install base dependencies
RUN apt-get update && apt-get install -y \
    locales curl gnupg2 lsb-release python3-pip \
    build-essential cmake git \
    python3-argcomplete \
    software-properties-common && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

# Install Python deps
RUN pip3 install --no-cache-dir --upgrade pip setuptools==68.2.2 pybind11

# Install pybind11 (system and Python versions) + dependencies for building bindings
RUN apt-get update && apt-get install -y \
    python3-dev libglib2.0-dev pybind11-dev 
    #pip3 install --no-cache-dir pybind11

# Clone and build only DeepStream Python bindings
WORKDIR /opt/deepstream_pyds
RUN git clone --depth=1 --filter=blob:none --sparse https://github.com/NVIDIA-AI-IOT/deepstream_python_apps.git && \
    cd deepstream_python_apps && \
    git sparse-checkout init --cone && \
    git sparse-checkout set bindings

# Build and install pyds
WORKDIR /opt/deepstream_pyds/deepstream_python_apps/bindings
RUN mkdir build && cd build && \
    export CPLUS_INCLUDE_PATH=/usr/include/pybind11 && \
    cmake .. && \
    make && ldconfig && \
    pip3 install ..

# Clean up bindings to reduce image size
RUN rm -rf /opt/deepstream_pyds

# Add ROS 2 Humble repo
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    echo "deb [arch=amd64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update

# Install ROS 2 core and required tools
RUN apt-get install -y \
    ros-humble-ros-core \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Setup YOLOv5 and convert to ONNX
WORKDIR /opt
RUN git clone https://github.com/ultralytics/yolov5 && \
    pip3 install --no-cache-dir -r yolov5/requirements.txt && \
    python3 yolov5/export.py --weights yolov5s.pt --include onnx

# Pin setuptools to avoid incompatibilities with colcon
RUN pip install --no-cache-dir setuptools==68.2.2

# Copy bird detection app and build with colcon
COPY bird_detection_app /opt/bird_detection_app
WORKDIR /opt/bird_detection_app
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build

# Set runtime environment
ENV LD_LIBRARY_PATH=/opt/nvidia/deepstream/deepstream-7.0/lib:$LD_LIBRARY_PATH
ENV ROS_PACKAGE_PATH=/opt/bird_detection_app:$ROS_PACKAGE_PATH
SHELL ["/bin/bash", "-c"]

# Copy media, models, and entrypoint
COPY media /opt/media
COPY models /opt/models
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Entry point sources ROS + overlay and runs ROS node
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "run", "bird_detection_app", "bird_publisher"]
