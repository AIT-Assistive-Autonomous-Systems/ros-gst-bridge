/* gst_bridge
 * Copyright (C) 2026 Austrian Institute of Technology (AIT)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */
#ifndef _GST_BRIDGE_SINK_DIAGNOSTICS_H_
#define _GST_BRIDGE_SINK_DIAGNOSTICS_H_

#include <gst_bridge/gst_bridge.h>

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

namespace gst_bridge
{
/*
  Health of a bridge sink on /diagnostics (diagnostic_msgs/DiagnosticArray), once a second from a
  thread of its own (the sink's node has no executor, and a stalled stream must still report):

  * rate of the published messages (over the last period), against expected-rate if set
  * missing frames: gaps in the message stamps above 1.5x the expected (else the median) interval
  * stamp age: node clock at publish - message stamp (catches a TAI or unsynchronized camera:
    seconds off, and stamps in the future)
  * publish time: how long the publish call took (DDS / subscriber backpressure)
  * stall: no message for 2 s (or 2 frame intervals, if longer)

  Expected rate: the element's expected-rate if set, else the framerate of the caps (video), else
  unknown (gaps against the median interval, the rate itself not checked).

  Level: ERROR stalled; WARN frames missing in the period, rate off the expected by more than
  10 %, stamp age above max-stamp-age or more than 50 ms in the future, publish time above half a
  frame interval, waiting for the first message; else OK.
*/
class SinkDiagnostics
{
public:
  struct Config
  {
    std::string name;            // status name: the node's fully qualified name
    double expected_rate = 0.0;  // Hz, the element's expected-rate: overrides the caps' framerate
    double max_stamp_age = 1.0;  // s, 0: not checked
  };

  SinkDiagnostics(std::shared_ptr<node_interface_collection> node_if, Config config);
  ~SinkDiagnostics();

  // after each publish: the message stamp and how long the publish call took
  void on_message(const rclcpp::Time & stamp, int64_t publish_ns);
  void set_topic(const std::string & topic);
  // the stream's nominal rate from the caps (framerate), 0: none / variable
  void set_caps_rate(double rate);

private:
  // expected rate (Hz) and where it comes from: expected-rate, else the caps; 0: unknown
  double expected_rate_locked(const char ** source) const;
  void run();
  void publish_status();

  std::shared_ptr<node_interface_collection> node_if_;
  Config config_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_;

  std::mutex mutex_;
  std::condition_variable cv_;
  bool stop_ = false;
  std::thread thread_;

  // statistics, under mutex_
  std::string topic_;
  double caps_rate_ = 0.0;
  uint64_t frames_ = 0;
  uint64_t missing_ = 0;
  uint64_t gaps_ = 0;
  int64_t last_stamp_ns_ = 0;
  std::deque<int64_t> intervals_;
  std::chrono::steady_clock::time_point last_message_;
  std::chrono::steady_clock::time_point period_start_;
  uint64_t period_frames_ = 0;
  uint64_t period_missing_ = 0;
  int64_t period_age_min_ns_ = 0;
  int64_t period_age_max_ns_ = 0;
  int64_t period_publish_max_ns_ = 0;
  int64_t period_publish_sum_ns_ = 0;
};

}  // namespace gst_bridge

#endif  // _GST_BRIDGE_SINK_DIAGNOSTICS_H_
