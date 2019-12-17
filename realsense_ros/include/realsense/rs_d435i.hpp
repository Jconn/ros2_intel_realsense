// Copyright (c) 2019 Intel Corporation. All Rights Reserved
//
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef REALSENSE__RS_D435I_HPP_
#define REALSENSE__RS_D435I_HPP_

#include <rclcpp/time.hpp>
#include "realsense/rs_d435.hpp"
#include <atomic>
#include <eigen3/Eigen/Geometry>

using IMUInfo = realsense_msgs::msg::IMUInfo;

namespace realsense
{
class RealSenseD435I : public RealSenseD435
{
public:
  RealSenseD435I(rs2::context ctx, rs2::device dev, rclcpp::Node & node);
  virtual ~RealSenseD435I();
  virtual void publishTopicsCallback(const rs2::frame & frame) override;
  virtual Result paramChangeCallback(const std::vector<rclcpp::Parameter> & params) override;
  void publishIMUTopic(const rs2::frame & frame, const rclcpp::Time & time);
  IMUInfo getIMUInfo(const rs2::frame & frame, const stream_index_pair & stream_index);

  enum imu_sync_method{NONE, COPY, LINEAR_INTERPOLATION};

private:
  class CIMUHistory
  {
    public:
      enum sensor_name {mGYRO, mACCEL};
      class imuData
      {
        public:
          imuData() {};

          imuData(const imuData& other):
            imuData(other.m_reading, other.m_time)
        {};
          imuData(const float3 reading, double time):
            m_reading(reading),
            m_time(time)
        {};
          imuData operator*(const double factor);
          imuData operator+(const imuData& other);
        public:
          float3 m_reading;
          double                    m_time;
      };

    private:
      size_t m_max_size;
      std::map<sensor_name, std::list<imuData> > m_map;

    public:
      CIMUHistory(size_t size);
      void add_data(sensor_name module, imuData data);
      bool is_all_data(sensor_name);
      bool is_data(sensor_name);
      const std::list<imuData>& get_data(sensor_name module);
      imuData last_data(sensor_name module);
  };
private:
  void processImuData();
  void imu_callback(rs2::frame frame);
  void imu_callback_sync(rs2::frame frame, imu_sync_method sync_method=imu_sync_method::COPY);
  void onInit();
  void getParameters();
  void publishSyncedIMUTopic(RealSenseD435I::CIMUHistory::imuData & accel_data,
    const RealSenseD435I::CIMUHistory::imuData & gyro_data,
    const rclcpp::Time & time);
  void setBaseTime(double frame_time);
  double FillImuData_LinearInterpolation(const RealSenseD435I::CIMUHistory::imuData & accel_data, const RealSenseD435I::CIMUHistory::imuData & gyro_data, sensor_msgs::msg::Imu& imu_msg);
  double FillImuData_Copy(const RealSenseD435I::CIMUHistory::imuData & accel_data, const RealSenseD435I::CIMUHistory::imuData & gyro_data, sensor_msgs::msg::Imu& imu_msg);

private:
  std::mutex imu_lock_;
  bool imu_active_ = true;
  std::vector<rs2::frame> imu_processor_;
  std::unique_ptr<std::thread> imu_thread_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  const std::vector<stream_index_pair> IMAGE_STREAMS = {COLOR, DEPTH, INFRA1, INFRA2};
  const std::vector<stream_index_pair> MOTION_STREAMS = {ACCEL, GYRO};
  double linear_accel_cov_;
  double angular_velocity_cov_;
  bool initialized_ = false;
  imu_sync_method imu_sync_method_;
  rclcpp::Time ros_time_base_;  
  double camera_time_base_;
  double accel_factor;
  std::atomic_bool _is_initialized_time_base;
};
}  // namespace perception
#endif // REALSENSE__RS_D435I_HPP_
