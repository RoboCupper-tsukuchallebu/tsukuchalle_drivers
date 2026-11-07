// Reads one cell (x y z intensity ring time, 22 B) of an organized PointCloud2 with memcpy.
#ifndef TSUKUCHALLE_VELODYNE__TEST__CLOUD_CELL_READER_HPP_
#define TSUKUCHALLE_VELODYNE__TEST__CLOUD_CELL_READER_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace tsukuchalle_velodyne::test
{

struct Cell final
{
  float x;
  float y;
  float z;
  float intensity;
  std::uint16_t ring;
  float time;

  [[nodiscard]] bool isFinite() const
  {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && std::isfinite(intensity);
  }
  [[nodiscard]] bool isAllNan() const
  {
    return std::isnan(x) && std::isnan(y) && std::isnan(z) && std::isnan(intensity);
  }
  [[nodiscard]] float distanceFromOrigin() const {return std::sqrt(x * x + y * y + z * z);}
};

inline Cell readCell(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::size_t row, const std::size_t column)
{
  Cell cell{};
  const std::uint8_t * source = cloud.data.data() + row * cloud.row_step + column * cloud.point_step;
  std::memcpy(&cell.x, source + 0, sizeof(float));
  std::memcpy(&cell.y, source + 4, sizeof(float));
  std::memcpy(&cell.z, source + 8, sizeof(float));
  std::memcpy(&cell.intensity, source + 12, sizeof(float));
  std::memcpy(&cell.ring, source + 16, sizeof(std::uint16_t));
  std::memcpy(&cell.time, source + 18, sizeof(float));
  return cell;
}

}  // namespace tsukuchalle_velodyne::test

#endif  // TSUKUCHALLE_VELODYNE__TEST__CLOUD_CELL_READER_HPP_
