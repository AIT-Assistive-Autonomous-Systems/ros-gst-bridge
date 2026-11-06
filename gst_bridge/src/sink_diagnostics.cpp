/* gst_bridge
 * Copyright (C) 2026 Austrian Institute of Technology (AIT)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */
#include <gst_bridge/sink_diagnostics.h>

#include <algorithm>
#include <cstdio>
#include <vector>

namespace gst_bridge
{
namespace
{
constexpr size_t MEDIAN_WINDOW = 15;
constexpr size_t MIN_INTERVALS = 5;
constexpr int64_t MIN_STALL_NS = 2'000'000'000;
constexpr int64_t FUTURE_TOLERANCE_NS = 50'000'000;

std::string fmt(const char * format, double value)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), format, value);
  return buf;
}

diagnostic_msgs::msg::KeyValue kv(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue v;
  v.key = key;
  v.value = value;
  return v;
}
}  // namespace

SinkDiagnostics::SinkDiagnostics(std::shared_ptr<node_interface_collection> node_if, Config config)
: node_if_(std::move(node_if)), config_(std::move(config))
{
  pub_ = rclcpp::create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    node_if_->parameters, node_if_->topics, "/diagnostics", rclcpp::QoS(10));
  period_ = std::chrono::nanoseconds(
    std::max<int64_t>(1'000'000, std::llround(std::max(config_.period, 0.0) * 1e9)));
  period_start_ = std::chrono::steady_clock::now();
  thread_ = std::thread([this] { run(); });
}

SinkDiagnostics::~SinkDiagnostics()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  cv_.notify_all();
  if (thread_.joinable()) thread_.join();
}

void SinkDiagnostics::set_topic(const std::string & topic)
{
  std::lock_guard<std::mutex> lock(mutex_);
  topic_ = topic;
}

void SinkDiagnostics::set_caps_rate(double rate)
{
  std::lock_guard<std::mutex> lock(mutex_);
  caps_rate_ = rate;
}

double SinkDiagnostics::expected_rate_locked(const char ** source) const
{
  if (config_.expected_rate > 0) {
    if (source) *source = "expected-rate";
    return config_.expected_rate;
  }
  if (caps_rate_ > 0) {
    if (source) *source = "caps";
    return caps_rate_;
  }
  if (source) *source = "unknown";
  return 0.0;
}

void SinkDiagnostics::on_message(const rclcpp::Time & stamp, int64_t publish_ns)
{
  const int64_t now_ns = node_if_->clock->get_clock()->now().nanoseconds();
  const int64_t stamp_ns = stamp.nanoseconds();
  std::lock_guard<std::mutex> lock(mutex_);
  const int64_t age = now_ns - stamp_ns;
  if (period_frames_ == 0) {
    period_age_min_ns_ = period_age_max_ns_ = age;
  } else {
    period_age_min_ns_ = std::min(period_age_min_ns_, age);
    period_age_max_ns_ = std::max(period_age_max_ns_, age);
  }
  period_publish_max_ns_ = std::max(period_publish_max_ns_, publish_ns);
  period_publish_sum_ns_ += publish_ns;
  ++period_frames_;
  ++frames_;
  last_message_ = std::chrono::steady_clock::now();

  if (last_stamp_ns_ != 0 && stamp_ns > last_stamp_ns_) {
    const int64_t dt = stamp_ns - last_stamp_ns_;
    int64_t expected = 0;
    const double expected_rate = expected_rate_locked(nullptr);
    if (expected_rate > 0) {
      expected = static_cast<int64_t>(1e9 / expected_rate);
    } else if (intervals_.size() >= MIN_INTERVALS) {
      std::vector<int64_t> v(intervals_.begin(), intervals_.end());
      std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
      expected = v[v.size() / 2];
    }
    if (expected > 0 && dt * 2 > expected * 3) {
      const uint64_t missing = std::max<int64_t>(1, (dt + expected / 2) / expected - 1);
      missing_ += missing;
      period_missing_ += missing;
      ++gaps_;
    } else {
      if (intervals_.size() == MEDIAN_WINDOW) intervals_.pop_front();
      intervals_.push_back(dt);
    }
  }
  last_stamp_ns_ = stamp_ns;
}

void SinkDiagnostics::run()
{
  std::unique_lock<std::mutex> lock(mutex_);
  while (!stop_) {
    if (cv_.wait_for(lock, period_, [this] { return stop_; })) break;
    lock.unlock();
    try {
      publish_status();
    } catch (const std::exception & e) {
      // the context may go down before the element is closed
      RCLCPP_WARN_ONCE(node_if_->logging->get_logger(), "diagnostics: %s", e.what());
    }
    lock.lock();
  }
}

void SinkDiagnostics::publish_status()
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  std::string node = config_.node_name;
  if (!node.empty() && node[0] == '/') node.erase(0, 1);
  status.hardware_id = config_.hardware_id.empty() ? node : config_.hardware_id;
  std::vector<std::string> problems;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  auto raise = [&](uint8_t l, const std::string & what) {
    level = std::max(level, l);
    problems.push_back(what);
  };
  using diagnostic_msgs::msg::DiagnosticStatus;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = std::chrono::steady_clock::now();
    const double period_s = std::chrono::duration<double>(now - period_start_).count();
    period_start_ = now;
    const double rate = period_s > 0 ? period_frames_ / period_s : 0.0;

    const char * rate_source = "unknown";
    const double expected_rate = expected_rate_locked(&rate_source);
    int64_t interval = 0;
    if (expected_rate > 0) {
      interval = static_cast<int64_t>(1e9 / expected_rate);
    } else if (!intervals_.empty()) {
      std::vector<int64_t> v(intervals_.begin(), intervals_.end());
      std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
      interval = v[v.size() / 2];
    }

    status.name = topic_.empty() ? node : node + ": " + topic_ + " topic status";
    status.values.push_back(kv("messages", std::to_string(frames_)));
    status.values.push_back(kv("rate (Hz)", fmt("%.2f", rate)));
    status.values.push_back(kv(
      std::string("expected rate (Hz, ") + rate_source + ")",
      expected_rate > 0 ? fmt("%.2f", expected_rate) : "-"));
    if (interval > 0)
      status.values.push_back(kv("frame interval (ms)", fmt("%.3f", interval / 1e6)));
    status.values.push_back(kv("missing frames", std::to_string(missing_)));
    status.values.push_back(kv("missing frames (period)", std::to_string(period_missing_)));
    status.values.push_back(kv("gaps", std::to_string(gaps_)));

    if (frames_ == 0) {
      raise(DiagnosticStatus::WARN, "waiting for the first message");
    } else {
      const int64_t since_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now - last_message_).count();
      status.values.push_back(kv("since last message (s)", fmt("%.3f", since_ns / 1e9)));
      if (since_ns > std::max(MIN_STALL_NS, 2 * interval)) {
        raise(DiagnosticStatus::ERROR, fmt("stalled: no message for %.1f s", since_ns / 1e9));
      }
    }
    if (period_frames_ > 0) {
      status.values.push_back(kv("stamp age min (ms)", fmt("%.1f", period_age_min_ns_ / 1e6)));
      status.values.push_back(kv("stamp age max (ms)", fmt("%.1f", period_age_max_ns_ / 1e6)));
      status.values.push_back(
        kv("publish time mean (ms)", fmt("%.2f", period_publish_sum_ns_ / 1e6 / period_frames_)));
      status.values.push_back(
        kv("publish time max (ms)", fmt("%.2f", period_publish_max_ns_ / 1e6)));
      if (period_missing_ > 0) {
        raise(DiagnosticStatus::WARN, std::to_string(period_missing_) + " frame(s) missing");
      }
      if (
        config_.max_stamp_age > 0 &&
        period_age_max_ns_ > static_cast<int64_t>(config_.max_stamp_age * 1e9)) {
        raise(DiagnosticStatus::WARN, fmt("stamps %.3f s old", period_age_max_ns_ / 1e9));
      }
      if (period_age_min_ns_ < -FUTURE_TOLERANCE_NS) {
        raise(
          DiagnosticStatus::WARN, fmt("stamps %.3f s in the future", -period_age_min_ns_ / 1e9));
      }
      if (interval > 0 && period_publish_max_ns_ * 2 > interval) {
        raise(
          DiagnosticStatus::WARN,
          fmt("publish slow: %.1f ms (backpressure)", period_publish_max_ns_ / 1e6));
      }
    }
    if (expected_rate > 0 && frames_ > 0 && period_s > 0.5 * config_.period) {
      if (rate < 0.9 * expected_rate || rate > 1.1 * expected_rate) {
        raise(DiagnosticStatus::WARN, fmt("rate %.2f Hz", rate));
      }
    }
    period_frames_ = 0;
    period_missing_ = 0;
    period_publish_max_ns_ = 0;
    period_publish_sum_ns_ = 0;
  }
  status.level = level;
  if (problems.empty()) {
    status.message = "OK";
  } else {
    for (size_t i = 0; i < problems.size(); ++i) {
      status.message += (i ? ", " : "") + problems[i];
    }
  }

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = node_if_->clock->get_clock()->now();
  array.status.push_back(std::move(status));
  pub_->publish(array);
}

}  // namespace gst_bridge
