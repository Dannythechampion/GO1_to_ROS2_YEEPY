#pragma once

#include <chrono>
#include <cstddef>
#include <deque>

namespace go1_mapping {

struct RateMeasurement {
  double rate_hz{0.0};
  double gap_sec{0.0};
};

class RateWindow {
 public:
  using Clock = std::chrono::steady_clock;
  explicit RateWindow(double window_seconds = 10.0)
      : window_(std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(window_seconds))),
        started_(Clock::now()) {}

  void observe(Clock::time_point stamp = Clock::now()) {
    receipts_.push_back(stamp);
  }

  RateMeasurement measure(Clock::time_point now = Clock::now()) {
    const auto cutoff = now - window_;
    while (!receipts_.empty() && receipts_.front() < cutoff) {
      receipts_.pop_front();
    }
    RateMeasurement result;
    if (receipts_.size() > 1) {
      const double span = std::chrono::duration<double>(
          receipts_.back() - receipts_.front()).count();
      if (span > 0.0) {
        result.rate_hz = static_cast<double>(receipts_.size() - 1) / span;
      }
    }
    result.gap_sec = std::chrono::duration<double>(
        now - (receipts_.empty() ? started_ : receipts_.back())).count();
    if (result.gap_sec < 0.0) {
      result.gap_sec = 0.0;
    }
    return result;
  }

 private:
  Clock::duration window_;
  Clock::time_point started_;
  std::deque<Clock::time_point> receipts_;
};

}  // namespace go1_mapping