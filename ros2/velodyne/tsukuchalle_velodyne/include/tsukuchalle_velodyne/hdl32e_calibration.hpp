// Calibration of one HDL-32E: the ros-drivers yaml format (num_lasers, distance_resolution, lasers).
#ifndef TSUKUCHALLE_VELODYNE__HDL32E_CALIBRATION_HPP_
#define TSUKUCHALLE_VELODYNE__HDL32E_CALIBRATION_HPP_

#include <array>
#include <string>

#include "tsukuchalle_velodyne/hdl32e_packet.hpp"

namespace tsukuchalle_velodyne
{

// Same members and types as velodyne_pointcloud::LaserCorrection (ros-drivers): float, not double.
struct LaserCorrection final
{
  float rot_correction;
  float vert_correction;
  float dist_correction;
  bool two_pt_correction_available;
  float dist_correction_x;
  float dist_correction_y;
  float vert_offset_correction;
  float horiz_offset_correction;
  int max_intensity;
  int min_intensity;
  float focal_distance;
  float focal_slope;

  float cos_rot_correction;
  float sin_rot_correction;
  float cos_vert_correction;
  float sin_vert_correction;

  int laser_ring;  // 0 for the lowest vert_correction
};

class Hdl32eCalibration final
{
public:
  // Throws std::runtime_error when the file cannot be read or the content is invalid.
  [[nodiscard]] static Hdl32eCalibration loadFromYamlFile(const std::string & file_path);
  [[nodiscard]] static Hdl32eCalibration loadFromYamlText(const std::string & yaml_text);

  [[nodiscard]] float distance_resolution_m() const {return distance_resolution_m_;}
  // Indexed by laser_id.
  [[nodiscard]] const std::array<LaserCorrection, LASER_COUNT> & laser_corrections() const
  {
    return laser_corrections_;
  }

private:
  Hdl32eCalibration() = default;

  float distance_resolution_m_{0.002f};
  std::array<LaserCorrection, LASER_COUNT> laser_corrections_{};
};

}  // namespace tsukuchalle_velodyne

#endif  // TSUKUCHALLE_VELODYNE__HDL32E_CALIBRATION_HPP_
