#include "go1_mapping/rate_window.hpp"

#include <chrono>

#include <gtest/gtest.h>

namespace {
using go1_mapping::RateWindow;

TEST(RateWindow, MeasuresRateAndGapAcrossWindow) {
  RateWindow window(10.0);
  const auto base = RateWindow::Clock::now();
  for (int index = 0; index <= 100; ++index) {
    window.observe(base + std::chrono::milliseconds(index * 100));
  }
  const auto result = window.measure(base + std::chrono::milliseconds(10050));
  EXPECT_NEAR(result.rate_hz, 10.0, 1e-9);
  EXPECT_NEAR(result.gap_sec, 0.05, 1e-9);
}

TEST(RateWindow, DropsSamplesOutsideWindow) {
  RateWindow window(1.0);
  const auto base = RateWindow::Clock::now();
  window.observe(base);
  window.observe(base + std::chrono::milliseconds(500));
  window.observe(base + std::chrono::milliseconds(1500));
  const auto result = window.measure(base + std::chrono::milliseconds(1500));
  EXPECT_DOUBLE_EQ(result.rate_hz, 1.0);
  EXPECT_DOUBLE_EQ(result.gap_sec, 0.0);
}
}  // namespace