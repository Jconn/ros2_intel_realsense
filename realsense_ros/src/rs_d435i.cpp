// Copyright (c) 2019 Intel Corporation. All Rights Reserved
//
// Licensed under the Apache License, Version 2.0 (the "License");
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

#include "realsense/rs_d435i.hpp"

namespace realsense
{
  RealSenseD435I::~RealSenseD435I()
  {
    imu_active_ = false;
    imu_thread_->join();
  }

RealSenseD435I::RealSenseD435I(rs2::context ctx, rs2::device dev, rclcpp::Node & node)
  : RealSenseD435(ctx, dev, node),
    _is_initialized_time_base(false)
{ 
  for (auto & stream : MOTION_STREAMS) {
    setupStream(stream);
  }
  // if (enable_[ACCEL] == true) {
  //   linear_accel_cov_ = node_.declare_parameter("accel0.linear_acceleration_covariance", DEFAULT_LINEAR_ACCEL_COV);
  // }
  // if (enable_[GYRO] == true) {
  //   angular_velocity_cov_ = node_.declare_parameter("gyro0.angular_velocity_covariance", DEFAULT_ANGULAR_VELOCITY_COV);
  // }
  linear_accel_cov_ = DEFAULT_LINEAR_ACCEL_COV;
  angular_velocity_cov_ = DEFAULT_ANGULAR_VELOCITY_COV;
  initialized_ = true;
  onInit();
}

void RealSenseD435I::onInit()
{
  imu_publisher_ = node_.create_publisher<sensor_msgs::msg::Imu>(
      "imu_raw",
      rclcpp::SensorDataQoS()
      ); 
  getParameters();
  imu_active_ = true;
  imu_thread_ = std::make_unique<std::thread>(std::bind(&RealSenseD435I::processImuData, this) );
}

void RealSenseD435I::getParameters()
{
  std::string unite_imu_method_str;
  node_.declare_parameter("unite_imu_method",  rclcpp::ParameterValue(std::string("linear_interpolation")));
  node_.get_parameter("unite_imu_method", unite_imu_method_str);

  if (unite_imu_method_str == "linear_interpolation")
    imu_sync_method_ = imu_sync_method::LINEAR_INTERPOLATION;
  else if (unite_imu_method_str == "copy")
    imu_sync_method_ = imu_sync_method::COPY;
  else
    imu_sync_method_ = imu_sync_method::NONE;

  RCLCPP_WARN(node_.get_logger(), "using sync method: %s(%d)", 
      unite_imu_method_str.c_str(),
      imu_sync_method_);

}

void RealSenseD435I::processImuData()
{
  while(imu_active_)
  {
    rs2::frame frame;
    {
      const std::lock_guard<std::mutex> lock(imu_lock_);
      if(imu_processor_.empty()) {
        continue;
      }
      frame = std::move(imu_processor_.back()); 
      imu_processor_.clear();
    }
    publishSyncedIMUTopic(frame, node_.now());
  }
}

void RealSenseD435I::publishTopicsCallback(const rs2::frame & frame)
{
  rclcpp::Time t = node_.now();
  if (frame.is<rs2::frameset>()) {
    RealSenseD435::publishTopicsCallback(frame);
  } else if (frame.is<rs2::motion_frame>()) {
    //if ((enable_[ACCEL] && (imu_pub_[ACCEL]->get_subscription_count() > 0 || imu_info_pub_[ACCEL]->get_subscription_count() > 0))
    //    || (enable_[GYRO] && (imu_pub_[GYRO]->get_subscription_count() > 0 || imu_info_pub_[GYRO]->get_subscription_count() > 0))) {
    //  publishSyncedIMUTopic(frame, t);
    //}
    const std::lock_guard<std::mutex> lock(imu_lock_);
    imu_processor_.push_back(frame); //implicit copy here
    //publishSyncedIMUTopic(frame, t);
  }
}

Result RealSenseD435I::paramChangeCallback(const std::vector<rclcpp::Parameter> & params)
{
  auto result = Result();
  result.successful = true;
  if (this->initialized_ == true) {
    result = RealSenseD435::paramChangeCallback(params);
    for (auto & param : params) {
      auto param_name = param.get_name();
      if (param_name == "accel0.enabled") {
        result = toggleStream(ACCEL, param);
      } else if (param_name == "gyro0.enabled") {
        result = toggleStream(GYRO, param);
      }

      RCLCPP_WARN(node_.get_logger(), "param change notify on: %s", param_name.c_str());
    }
  }
  return result;
}

void RealSenseD435I::setBaseTime(double frame_time, bool warn_no_metadata)
{
    //ROS_WARN_COND(warn_no_metadata, "Frame metadata isn't available! (frame_timestamp_domain = RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME)");

    ros_time_base_ = node_.now();
    camera_time_base_ = frame_time;
}

//this relies on the global state imu sync method
void RealSenseD435I::publishSyncedIMUTopic(const rs2::frame & frame, const rclcpp::Time & time)
{

  static bool init_gyro(false), init_accel(false);

  auto type = frame.get_profile().stream_type();
  auto index = frame.get_profile().stream_index();
  auto type_index = std::pair<rs2_stream, int>(type, index);
  auto m_frame = frame.as<rs2::motion_frame>();
  sensor_msgs::msg::Imu imu_msg;
  realsense_msgs::msg::IMUInfo info_msg;

  imu_msg.header.frame_id = OPTICAL_FRAME_ID.at(type_index);
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 0.0;
  imu_msg.orientation_covariance = {-1.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0};
  imu_msg.linear_acceleration_covariance = {linear_accel_cov_, 0.0, 0.0,
                                            0.0, linear_accel_cov_, 0.0,
                                            0.0, 0.0, linear_accel_cov_};
  imu_msg.angular_velocity_covariance = {angular_velocity_cov_, 0.0, 0.0,
                                         0.0, angular_velocity_cov_, 0.0,
                                         0.0, 0.0, angular_velocity_cov_};

  while (true)
  {
    auto stream = frame.get_profile().stream_type();
    auto stream_index = (stream == GYRO.first)?GYRO:ACCEL;
    double frame_time = frame.get_timestamp();
    //RCLCPP_WARN(node_.get_logger(), "frame time is: %f", frame_time);

    bool placeholder_false(false);
    if (_is_initialized_time_base.compare_exchange_strong(placeholder_false, true) )
    {
      RCLCPP_WARN(node_.get_logger(), "setting base time");
      setBaseTime(frame_time, RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME == frame.get_frame_timestamp_domain());
    }

    double elapsed_camera_s = (/*ms*/ frame_time - /*ms*/ camera_time_base_) / 1000.0;

    RCLCPP_WARN(node_.get_logger(), "elapsed_camera_s is: %f, stream index is %d", elapsed_camera_s, stream_index);
    std::thread::id this_id = std::this_thread::get_id();
    //RCLCPP_WARN(node_.get_logger(), "current thread id is %d, sensor id is %d", this_id, stream_index);
    if (0 != imu_publisher_->get_subscription_count())
    {
      auto crnt_reading = *(reinterpret_cast<const float3*>(frame.get_data()));
      if (GYRO == stream_index)
      {
        init_gyro = true;
      }
      if (ACCEL == stream_index)
      {
        if (!init_accel)
        {
          // Init accel_factor:
          Eigen::Vector3d v(crnt_reading.x, crnt_reading.y, crnt_reading.z);
          accel_factor = 9.81 / v.norm();
          RCLCPP_INFO(node_.get_logger(), "accel_factor set to: %f", accel_factor);
        }
        init_accel = true;
        if (true)
        {
          Eigen::Vector3d v(crnt_reading.x, crnt_reading.y, crnt_reading.z);
          v*=accel_factor;
          crnt_reading.x = v.x();
          crnt_reading.y = v.y();
          crnt_reading.z = v.z();
        }
      }
      CIMUHistory::imuData imu_data(crnt_reading, elapsed_camera_s);
      switch (imu_sync_method_)
      {
        case NONE: //Cannot really be NONE. Just to avoid compilation warning.
        case COPY:
          elapsed_camera_s = FillImuData_Copy(stream_index, imu_data, imu_msg);
          break;
        case LINEAR_INTERPOLATION:
          elapsed_camera_s = FillImuData_LinearInterpolation(stream_index, imu_data, imu_msg);
          break;
      }
      if (elapsed_camera_s < 0)
        break;
      rclcpp::Time t(ros_time_base_.seconds() + elapsed_camera_s);
      imu_msg.header.stamp = t;
      //imu_msg.header.stamp = node_.now();
      if (!(init_gyro && init_accel))
        break;
      imu_publisher_->publish(imu_msg);
      //ROS_DEBUG("Publish united %s stream", rs2_stream_to_string(frame.get_profile().stream_type()));
    }
    break;
  }

  imu_msg.header.stamp = time;
  imu_pub_[type_index]->publish(imu_msg);
  imu_info_pub_[type_index]->publish(info_msg);
}

void RealSenseD435I::publishIMUTopic(const rs2::frame & frame, const rclcpp::Time & time)
{
  auto type = frame.get_profile().stream_type();
  auto index = frame.get_profile().stream_index();
  auto type_index = std::pair<rs2_stream, int>(type, index);
  auto m_frame = frame.as<rs2::motion_frame>();
  sensor_msgs::msg::Imu imu_msg;
  realsense_msgs::msg::IMUInfo info_msg;

  imu_msg.header.frame_id = OPTICAL_FRAME_ID.at(type_index);
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 0.0;
  imu_msg.orientation_covariance = {-1.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0};
  imu_msg.linear_acceleration_covariance = {linear_accel_cov_, 0.0, 0.0,
                                            0.0, linear_accel_cov_, 0.0,
                                            0.0, 0.0, linear_accel_cov_};
  imu_msg.angular_velocity_covariance = {angular_velocity_cov_, 0.0, 0.0,
                                         0.0, angular_velocity_cov_, 0.0,
                                         0.0, 0.0, angular_velocity_cov_};

  auto imu_data = m_frame.get_motion_data();
  if (type_index == GYRO) {
    imu_msg.angular_velocity.x = imu_data.x;
    imu_msg.angular_velocity.y = imu_data.y;
    imu_msg.angular_velocity.z = imu_data.z;
    info_msg = getIMUInfo(frame, GYRO);
  } else if (type_index == ACCEL) {
    imu_msg.linear_acceleration.x = imu_data.x;
    imu_msg.linear_acceleration.y = imu_data.y;
    imu_msg.linear_acceleration.z = imu_data.z;
    info_msg = getIMUInfo(frame, ACCEL);
  }
  imu_msg.header.stamp = time;
  imu_pub_[type_index]->publish(imu_msg);
  imu_info_pub_[type_index]->publish(info_msg);
}

IMUInfo RealSenseD435I::getIMUInfo(const rs2::frame & frame, const stream_index_pair & stream_index)
{
  auto m_profile = frame.get_profile().as<rs2::motion_stream_profile>();
  realsense_msgs::msg::IMUInfo info;
  rs2_motion_device_intrinsic imu_intrinsics;
  try {
    imu_intrinsics = m_profile.get_motion_intrinsics();
  } catch (const std::runtime_error &ex) {
    RCLCPP_INFO(node_.get_logger(), "No Motion Intrinsics available. Please calibrate it by rs-imu-calibration tool first.");
    imu_intrinsics = {{{1,0,0,0},
                       {0,1,0,0},
                       {0,0,1,0}}, {0,0,0}, {0,0,0}};
  }

  auto index = 0;
  info.header.frame_id = OPTICAL_FRAME_ID.at(stream_index);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 4; ++j) {
      info.data[index] = imu_intrinsics.data[i][j];
      ++index;
    }
    info.noise_variances[i] =  imu_intrinsics.noise_variances[i];
    info.bias_variances[i] = imu_intrinsics.bias_variances[i];
  }
  return info;
}


RealSenseD435I::CIMUHistory::CIMUHistory(size_t size)
{
    m_max_size = size;
}
void RealSenseD435I::CIMUHistory::add_data(sensor_name module, RealSenseD435I::CIMUHistory::imuData data)
{
    m_map[module].push_front(data);
    if (m_map[module].size() > m_max_size)
        m_map[module].pop_back();
}
bool RealSenseD435I::CIMUHistory::is_all_data(sensor_name module)
{
    return m_map[module].size() == m_max_size;
}
bool RealSenseD435I::CIMUHistory::is_data(sensor_name module)
{
    return m_map[module].size() > 0;
}
const std::list<RealSenseD435I::CIMUHistory::imuData>& RealSenseD435I::CIMUHistory::get_data(sensor_name module)
{
    return m_map[module];
}
RealSenseD435I::CIMUHistory::imuData RealSenseD435I::CIMUHistory::last_data(sensor_name module)
{
    return m_map[module].front();
}
RealSenseD435I::CIMUHistory::imuData RealSenseD435I::CIMUHistory::imuData::operator*(const double factor)
{
    RealSenseD435I::CIMUHistory::imuData new_data(*this);
    new_data.m_reading *= factor;
    new_data.m_time *= factor;
    return new_data;
}

RealSenseD435I::CIMUHistory::imuData RealSenseD435I::CIMUHistory::imuData::operator+(const RealSenseD435I::CIMUHistory::imuData& other)
{
    RealSenseD435I::CIMUHistory::imuData new_data(*this);
    new_data.m_reading += other.m_reading;
    new_data.m_time += other.m_time;
    return new_data;
}


double RealSenseD435I::FillImuData_LinearInterpolation(const stream_index_pair stream_index, const RealSenseD435I::CIMUHistory::imuData imu_data, sensor_msgs::msg::Imu& imu_msg)
{
    static bool expecting_accel_data = false; 
    static CIMUHistory _imu_history(2);

    CIMUHistory::sensor_name this_sensor(static_cast<CIMUHistory::sensor_name>(ACCEL == stream_index));
    CIMUHistory::sensor_name that_sensor(static_cast<CIMUHistory::sensor_name>(!this_sensor));

    if(ACCEL == stream_index){
      if(!expecting_accel_data){
        RCLCPP_WARN(node_.get_logger(), "was expecting accel data");
      }
      else
      {
        _imu_history.add_data(this_sensor, imu_data);
      }
      expecting_accel_data = false;

    }
    else{
      if(expecting_accel_data){
        RCLCPP_WARN(node_.get_logger(), "was expecting gyro data");
      }
      else
      {
        _imu_history.add_data(this_sensor, imu_data);
      }
      expecting_accel_data = true;
    }

    if (!_imu_history.is_all_data(this_sensor) || !_imu_history.is_data(that_sensor) )
        return -1;
    const std::list<CIMUHistory::imuData> this_data = _imu_history.get_data(this_sensor);
    CIMUHistory::imuData that_last_data = _imu_history.last_data(that_sensor);
    std::list<CIMUHistory::imuData>::const_iterator this_data_iter = this_data.begin();
    CIMUHistory::imuData this_last_data(*this_data_iter);
    this_data_iter++;
    CIMUHistory::imuData this_prev_data(*this_data_iter);
    if (this_prev_data.m_time > that_last_data.m_time)
        return -1;  // "that" data was already sent.
    double factor( (that_last_data.m_time - this_prev_data.m_time) / (this_last_data.m_time - this_prev_data.m_time) );
    CIMUHistory::imuData interp_data = this_prev_data*(1-factor) + this_last_data*factor;

    CIMUHistory::imuData accel_data = that_last_data;
    CIMUHistory::imuData gyro_data = interp_data;
    if (this_sensor == CIMUHistory::sensor_name::mACCEL)
    {
        std::swap(accel_data, gyro_data);
    }
    imu_msg.angular_velocity.x = gyro_data.m_reading.x;
    imu_msg.angular_velocity.y = gyro_data.m_reading.y;
    imu_msg.angular_velocity.z = gyro_data.m_reading.z;

    imu_msg.linear_acceleration.x = accel_data.m_reading.x;
    imu_msg.linear_acceleration.y = accel_data.m_reading.y;
    imu_msg.linear_acceleration.z = accel_data.m_reading.z;
    return that_last_data.m_time;
}


double RealSenseD435I::FillImuData_Copy(const stream_index_pair stream_index, const RealSenseD435I::CIMUHistory::imuData imu_data, sensor_msgs::msg::Imu& imu_msg)
{
    if (GYRO == stream_index)
    {
        imu_msg.angular_velocity.x = imu_data.m_reading.x;
        imu_msg.angular_velocity.y = imu_data.m_reading.y;
        imu_msg.angular_velocity.z = imu_data.m_reading.z;
    }
    else if (ACCEL == stream_index)
    {
        imu_msg.linear_acceleration.x = imu_data.m_reading.x;
        imu_msg.linear_acceleration.y = imu_data.m_reading.y;
        imu_msg.linear_acceleration.z = imu_data.m_reading.z;
    }
    return imu_data.m_time;
}


}  // namespace realsense
