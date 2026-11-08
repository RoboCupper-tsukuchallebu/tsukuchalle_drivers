// Prints `[BENCH] median=... p99=...` for one revolution (181 packets). The time limit is only
// asserted without AddressSanitizer, which slows the loop several times over.
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "synthetic_packets.hpp"
#include "tsukuchalle_velodyne/hdl32e_converter.hpp"

namespace tsukuchalle_velodyne
{
namespace
{

constexpr int WARMUP_ITERATIONS = 20;
constexpr int MEASURED_ITERATIONS = 200;
constexpr double MEDIAN_LIMIT_MILLISECONDS = 30.0;
constexpr std::uint32_t RANDOM_SEED = 42;

TEST(BenchmarkConverter, convertsOneRevolutionQuickly)
{
  const Hdl32eConverter converter(
    Hdl32eCalibration::loadFromYamlFile(
      std::string(TSUKUCHALLE_VELODYNE_CONFIG_DIRECTORY) + "/32db.yaml"),
    0.9, 130.0);
  const auto scan = test::makeRandomScan(test::PACKETS_PER_REVOLUTION, RANDOM_SEED);
  for (int iteration = 0; iteration < WARMUP_ITERATIONS; ++iteration) {
    ASSERT_EQ(converter.convert(scan).cloud->height, 2172U);
  }

  std::vector<double> milliseconds;
  for (int iteration = 0; iteration < MEASURED_ITERATIONS; ++iteration) {
    const auto start = std::chrono::steady_clock::now();
    const auto result = converter.convert(scan);
    const auto end = std::chrono::steady_clock::now();
    ASSERT_EQ(result.cloud->height, 2172U);
    milliseconds.push_back(std::chrono::duration<double, std::milli>(end - start).count());
  }
  std::sort(milliseconds.begin(), milliseconds.end());
  const double median = milliseconds[milliseconds.size() / 2];
  const double p99 = milliseconds[milliseconds.size() * 99 / 100];
  std::printf("[BENCH] median=%.3f ms p99=%.3f ms\n", median, p99);

#ifndef __SANITIZE_ADDRESS__
  EXPECT_LE(median, MEDIAN_LIMIT_MILLISECONDS);
#endif
}

}  // namespace
}  // namespace tsukuchalle_velodyne
