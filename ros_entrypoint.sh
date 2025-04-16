#!/bin/bash
set -e

# Source ROS setup
source /opt/ros/humble/setup.bash
source /opt/bird_detection_app/install/setup.bash

exec "$@"
