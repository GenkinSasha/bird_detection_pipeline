import sys
import gi
import os
import time
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pyds import *

# ROS Node Setup
class BirdDetectionNode(Node):
    def __init__(self):
        super().__init__('bird_detection_node')
        self.publisher_ = self.create_publisher(String, 'bird_detection_topic', 10)

    def publish_bird_detected(self, detected: bool):
        msg = String()
        msg.data = "Bird detected" if detected else "No bird detected"
        self.publisher_.publish(msg)

# Global instance
bird_labels = set()

def load_labels(labels_path):
    global bird_labels
    with open(labels_path, 'r') as f:
        bird_labels = {line.strip().lower() for line in f if 'bird' in line.lower()}

def bus_call(bus, message, loop):
    msg_type = message.type
    if msg_type == Gst.MessageType.EOS or msg_type == Gst.MessageType.ERROR:
        loop.quit()
    return True

def osd_sink_pad_buffer_probe(pad, info, node):
    frame_meta_list = pyds.glist_to_list(pyds.NvDsBatchMeta.cast(info.get_buffer().p_buffer_meta).frame_meta_list)
    bird_detected = False

    for frame_meta in frame_meta_list:
        obj_meta_list = pyds.glist_to_list(frame_meta.obj_meta_list)
        for obj_meta in obj_meta_list:
            class_name = obj_meta.obj_label.strip().lower()
            if class_name in bird_labels:
                bird_detected = True
                break
        if bird_detected:
            break

    node.publish_bird_detected(bird_detected)
    return Gst.PadProbeReturn.OK

def create_pipeline(input_file, model_path, labels_path, ros_node):
    load_labels(labels_path)

    pipeline = Gst.Pipeline.new("bird-detection-pipeline")
    source = Gst.ElementFactory.make("filesrc", "file-source")
    source.set_property("location", input_file)

    decodebin = Gst.ElementFactory.make("decodebin", "decodebin")
    pgie = Gst.ElementFactory.make("nvinfer", "primary-inference")
    pgie.set_property("config-file-path", model_path)

    nvvidconv = Gst.ElementFactory.make("nvvideoconvert", "nvvideo-converter")
    nvosd = Gst.ElementFactory.make("nvdsosd", "onscreendisplay")
    sink = Gst.ElementFactory.make("fakesink", "fake-sink")

    if not all([pipeline, source, decodebin, pgie, nvvidconv, nvosd, sink]):
        raise RuntimeError("Failed to create one or more GStreamer elements.")

    pipeline.add(source)
    pipeline.add(decodebin)
    pipeline.add(pgie)
    pipeline.add(nvvidconv)
    pipeline.add(nvosd)
    pipeline.add(sink)

    source.link(decodebin)

    def on_pad_added(src, pad):
        sink_pad = pgie.get_static_pad("sink")
        if not sink_pad.is_linked():
            pad.link(sink_pad)

    decodebin.connect("pad-added", on_pad_added)

    pgie.link(nvvidconv)
    nvvidconv.link(nvosd)
    nvosd.link(sink)

    # Attach probe to analyze metadata
    osd_sink_pad = nvosd.get_static_pad("sink")
    osd_sink_pad.add_probe(Gst.PadProbeType.BUFFER, lambda pad, info: osd_sink_pad_buffer_probe(pad, info, ros_node))

    return pipeline

def main(input_file, model_path, labels_path):
    rclpy.init()
    node = BirdDetectionNode()

    Gst.init(None)
    pipeline = create_pipeline(input_file, model_path, labels_path, node)

    loop = GLib.MainLoop()
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect("message", bus_call, loop)

    print("Starting pipeline...")
    pipeline.set_state(Gst.State.PLAYING)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            loop.get_context().iteration(False)
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.set_state(Gst.State.NULL)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python3 bird_detection.py <input_video> <model_config.txt> <labels.txt>")
        sys.exit(1)

    video_path = sys.argv[1]
    model_config = sys.argv[2]
    labels_path = sys.argv[3]
    main(video_path, model_config, labels_path)
