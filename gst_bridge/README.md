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

`gst-launch-1.0 --gst-plugin-path=install/lib/gst_bridge/ roscompressedimagesrc ros-topic="/cam/image_raw" timestamp-mode=arrival ! jpegdec ! videoconvert ! autovideosink`

## Sink timestamps:
All sinks set `header.stamp` according to `timestamp-mode`:
* `ros-offset` (default): buffer PTS mapped to the ROS clock with the clock offset sampled at start (capture time).
* `reference`: the `GstReferenceTimestampMeta` matching `reference-caps` (default `timestamp/x-unix`), else
  `timestamp/x-ntp` converted to the Unix epoch, else `ros-offset` (warns once).
* `pts`: the buffer PTS (pipeline running time, relative to the stream start).
* `arrival-pipeline`: the absolute pipeline clock time when the sink renders the buffer.
* `arrival-ros`: the ROS node clock when the sink renders the buffer, like a ROS driver stamping on receipt
  (follows `use_sim_time`).
* `arrival-realtime`: the system wall clock when the sink renders the buffer (ignores sim time).

The arrival modes include the pipeline latency; with `sync=true` (default) the sink waits until the buffer is due.

## Source timestamps:
All sources (`rosimagesrc`, `roscompressedimagesrc`, `rosrawsrc`, `rosaudiosrc`, `rostextsrc`) set the buffer
PTS according to `timestamp-mode`:
* `ros-offset` (default): the message header stamp, mapped to pipeline time with the ROS/pipeline clock offset
  sampled at start. Stamps from before the stream start (e.g. `ros2 bag play` without sim time) fall back to the
  arrival time with a warning.
* `arrival`: the arrival time, best compatibility (e.g. bag playback).
* `pts`: the header stamp as is, without any offset (the counterpart of the sinks' `timestamp-mode=pts`).
  Only meaningful if the stamps are in the pipeline's time base, use `sync=false` sinks otherwise.

Independent of the PTS, `attach-reference-timestamp=true` keeps each header stamp as a `GstReferenceTimestampMeta`
with the caps given in `time-caps` (default `timestamp/x-rostime`, use `timestamp/x-unix` if the recording clock
was UTC wall clock). A sink with `timestamp-mode=reference reference-caps=timestamp/x-unix` then republishes the
original stamps, e.g. after processing:

`gst-launch-1.0 roscompressedimagesrc ros-topic="/cam/image_raw" timestamp-mode=arrival attach-reference-timestamp=true time-caps=timestamp/x-unix ! jpegdec ! videoflip method=horizontal-flip ! jpegenc ! roscompressedimagesink ros-topic="/relay/image_raw" timestamp-mode=reference reference-caps=timestamp/x-unix`

## Testing:
`colcon test --packages-select gst_bridge` runs end-to-end gtests of the elements (`test/`): pipelines in the
test process talk to a test node, one executable per area, each in an isolated ROS domain.

Optionally the same tests also run under valgrind (memory errors and definite leaks fail, `test/valgrind.supp`
holds the one-time leaks of GLib and Cyclone DDS); this takes a few minutes:
```
colcon build --packages-select gst_bridge --cmake-args -DGST_BRIDGE_VALGRIND_TESTS=ON
colcon test --packages-select gst_bridge --ctest-args -R valgrind
```

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


