# gst_bridge

This ROS package builds gstreamer elements that can be loaded into a gstreamer pipeline.
In this package, ROS is treated as a network-transport protocol for video, audio, and text.
This package does not build ROS nodes.

## Usage:
GStreamer needs to be able to find the plugin, so we need to tell it where to look.
In the example below, we use `--gst-plugin-path=install/lib/gst_bridge/` as an argument to `gst-launch-1.0`. Everything else, is the media pipeline.

This example sends dummy video data from a videotestsrc into ros from the command line.
`gst-launch-1.0 --gst-plugin-path=install/lib/gst_bridge/ videotestsrc ! videoconvert ! rosimagesink ros-topic="image"`

GStreamer makes it easy to change where the video comes from, like a webcam:
`gst-launch-1.0 --gst-plugin-path=install/lib/gst_bridge/ v4l2src ! videoconvert ! rosimagesink ros-topic="image"`

These elements can be loaded into any application that allows you to define a gstreamer pipeline, as long as you can add this ros package to the application's gst-plugin-path, perhaps using the `GST_PLUGIN_PATH` environment variable:
`export GST_PLUGIN_PATH=$COLCON_PREFIX_PATH/gst_bridge/lib/gst_bridge`

## Camera info:
`rosimagesink` and `roscompressedimagesink` can publish a `sensor_msgs/CameraInfo` with the same
header stamp as each image.
`roscompressedimagesink` publishes jpeg/png as `sensor_msgs/CompressedImage` on `<ros-topic>/<ros-transport>`
like image_transport; `ros-transport` defaults to `compressed`, if empty it publishes on `ros-topic` directly.
Set `camera-info-url` to a calibration yaml/ini (`file://`, `package://` or a plain path); without it no
camera info publisher is created, and nothing is published if the file fails to load.
* `camera-info-topic` defaults to `camera_info` next to the image topic
  (`ros-topic=/cam/image_raw` gives `/cam/camera_info` for both sinks)
* the camera info frame_id is, in order: `camera-info-frame-id` as is, a frame_id provided by the
  calibration as is (the calibration format currently has none), `<camera_name>_optical` from the
  calibration's `camera_name` (REP 103, no suffix added if it already ends in `_optical`), or the image's
  `ros-frame-id` if the calibration names no camera

`gst-launch-1.0 --gst-plugin-path=install/lib/gst_bridge/ v4l2src ! image/jpeg ! roscompressedimagesink ros-topic="/cam/image_raw" camera-info-url="file:///path/to/calibration.yaml"`

## Compressed images back into a pipeline:
`roscompressedimagesrc` subscribes to `<ros-topic>/<ros-transport>` (jpeg/png `sensor_msgs/CompressedImage`,
`ros-transport` defaults to `compressed`) and outputs `image/jpeg` or `image/png` following each message's `format`.
Buffers are timestamped from the message stamps; stamps from before the stream start (e.g. `ros2 bag play`
without sim time) fall back to the arrival time with a warning, `use-msg-stamp=false` always uses the arrival time.

`gst-launch-1.0 --gst-plugin-path=install/lib/gst_bridge/ roscompressedimagesrc ros-topic="/cam/image_raw" use-msg-stamp=false ! jpegdec ! videoconvert ! autovideosink`

## More complex usage:

This package has some ROS libraries for interacting with the elements in the pipeline, See the related gst_pipeline package for how that works

## Future work:

The efficiency and utility of the ros-gst-bridge elements could be improved with a few additional features:
* gst_bridge.cpp needs an allocator class that allows a derived vector
  type to wrap a gst-buffer. This allows true zero-copy transport of all outbound
  gst buffer data
* gst_bridge.cpp needs a gst buffer unref callback that allows downstream
  elements to defer dereference of a ros shared message pointer
* the rosimagesink class needs a call to borrow memory from the shared-memory
  daemon. the memory should be borrowed as ros messages, and cast to a gst
  buffer pool which is passed to upstream gst elements.


