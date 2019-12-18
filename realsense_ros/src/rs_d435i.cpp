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
  //JC: do not set up motion streams - give them an independent thread for polling and syncing
  //for (auto & stream : MOTION_STREAMS) {
  //  setupStream(stream);
  //}
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
      "imu/data_raw",
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
  using imuData =  RealSenseD435I::CIMUHistory::imuData;
  //it looks like the imu data is the same sensor, and the accel and gyro data are two different streams on that sensor
  //I think this means that there is no way to sync the two, because they are going to come in on the same frame
  //different profile of the same device means that you just have to screen from the callback
  /*
  rs2::syncer sync(1);
  auto dev_sensors = _dev_.query_sensors();
  accel_sensor.open(accel_stream);
  gyro_sensor.open(gyro_stream);
  accel_sensor.start(sync);
  gyro_sensor.start(sync);
  */
  rs2::pipeline motion_pipe;
  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_GYRO);
  cfg.enable_stream(RS2_STREAM_ACCEL);
  motion_pipe.start(cfg);
  //std::pair<rs2::rs2_vector, double> accel_data = nullptr;
  //std::pair<rs2::rs2_vector, double> gyro_data = nullptr;
  imuData accel_data;
  imuData gyro_data;
  while(imu_active_)
  {
    rs2::frameset fset = motion_pipe.wait_for_frames();
    auto time = node_.now();

    // Find and retrieve IMU and/or tracking data
    if (rs2::motion_frame accel_frame = fset.first_or_default(RS2_STREAM_ACCEL))
    {
      double ts = accel_frame.get_timestamp();
      //accel_data = std::make_pair<rs2::rs2_vector, double>(accel_frame.get_motion_data(), ts);
      accel_data.m_time = ts;
      auto accel_vec_data = accel_frame.get_motion_data();
      accel_data.m_reading.x = accel_vec_data.x; 
      accel_data.m_reading.y = accel_vec_data.y; 
      accel_data.m_reading.z = accel_vec_data.z; 

      bool placeholder_false(false);
      if (_is_initialized_time_base.compare_exchange_strong(placeholder_false, true) )
      {
        RCLCPP_WARN(node_.get_logger(), "setting base time");
        setBaseTime(ts);
      }
    }
 
    if (rs2::motion_frame gyro_frame = fset.first_or_default(RS2_STREAM_GYRO))
    {
      double ts = gyro_frame.get_timestamp();
      //gyro_data = std::make_pair<rs2::rs2_vector, double>(gyro_frame.get_motion_data(), ts);
      gyro_data.m_time = ts;
      auto gyro_vec_data = gyro_frame.get_motion_data();
      gyro_data.m_reading.x = gyro_vec_data.x; 
      gyro_data.m_reading.y = gyro_vec_data.y; 
      gyro_data.m_reading.z = gyro_vec_data.z; 
      bool placeholder_false(false);
      if (_is_initialized_time_base.compare_exchange_strong(placeholder_false, true) )
      {
        RCLCPP_WARN(node_.get_logger(), "setting base time");
        setBaseTime(ts);
      }
    }
    
    publishSyncedIMUTopic(accel_data, gyro_data, time);
  }
  motion_pipe.stop();
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

void RealSenseD435I::setBaseTime(double frame_time)
{
    //ROS_WARN_COND(warn_no_metadata, "Frame metadata isn't available! (frame_timestamp_domain = RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME)");

    ros_time_base_ = node_.now();
    camera_time_base_ = frame_time;
}

//this relies on the global state imu sync method
void RealSenseD435I::publishSyncedIMUTopic(RealSenseD435I::CIMUHistory::imuData & accel_data,
    const RealSenseD435I::CIMUHistory::imuData & gyro_data,
    const rclcpp::Time & time)
{
  using imuData =  RealSenseD435I::CIMUHistory::imuData;

  sensor_msgs::msg::Imu imu_msg;
  realsense_msgs::msg::IMUInfo info_msg;

  //imu_msg.header.frame_id = OPTICAL_FRAME_ID.at(type_index);
  imu_msg.header.frame_id = DEFAULT_BASE_FRAME_ID;
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



  double accel_time_s = (/*ms*/ accel_data.m_time - /*ms*/ camera_time_base_) / 1000.0;
  double gyro_time_s = (/*ms*/ gyro_data.m_time - /*ms*/ camera_time_base_) / 1000.0;
  double elapsed_camera_s = 0.0;

  if(gyro_time_s > accel_time_s)
    elapsed_camera_s = gyro_time_s;
  else
    elapsed_camera_s = accel_time_s;

  //RCLCPP_WARN(node_.get_logger(), "accel time is: %f", accel_time_s);
  //RCLCPP_WARN(node_.get_logger(), "gyro time is: %f", gyro_time_s);

  if (0 != imu_publisher_->get_subscription_count())
  {

    /*
    Eigen::Vector3d v(accel_data.m_reading.x, accel_data.m_reading.y, accel_data.m_reading.z);
    //TODO: debug something is wrong here
    accel_factor = 9.81 / v.norm();
    RCLCPP_INFO(node_.get_logger(), "accel_factor set to: %f", accel_factor);
    v*=accel_factor;
    // Init accel_factor:

    accel_data.m_reading.x = v.x();
    accel_data.m_reading.y = v.y();
    accel_data.m_reading.z = v.z();
    */

    switch (imu_sync_method_)
    {
      case NONE: //Cannot really be NONE. Just to avoid compilation warning.
      case COPY:
        FillImuData_Copy(accel_data, gyro_data, imu_msg);
        break;
      case LINEAR_INTERPOLATION:
        FillImuData_LinearInterpolation(accel_data, gyro_data, imu_msg);
        break;
    }
    if (elapsed_camera_s < 0)
      return;
    //this thing only takes int nanoseconds, or int seconds nanoseconds
    uint64_t nano_seconds = ros_time_base_.nanoseconds() + static_cast<uint64_t>(elapsed_camera_s * 1e9);
    rclcpp::Time t(nano_seconds);
    imu_msg.header.stamp = t;
    //imu_msg.header.stamp = node_.now();
    imu_publisher_->publish(imu_msg);
  }

 // imu_msg.header.stamp = time;
 // imu_pub_[type_index]->publish(imu_msg);
 // imu_info_pub_[type_index]->publish(info_msg);
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


double RealSenseD435I::FillImuData_LinearInterpolation(const RealSenseD435I::CIMUHistory::imuData & accel_data, const RealSenseD435I::CIMUHistory::imuData & gyro_data, sensor_msgs::msg::Imu& imu_msg)
{
  /*

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
  TODO: linear interpolate this fn
  */
  return 0.0;
}


double RealSenseD435I::FillImuData_Copy(const RealSenseD435I::CIMUHistory::imuData & accel_data, const RealSenseD435I::CIMUHistory::imuData & gyro_data, sensor_msgs::msg::Imu& imu_msg)
{
  imu_msg.angular_velocity.x = gyro_data.m_reading.x;
  imu_msg.angular_velocity.y = gyro_data.m_reading.y;
  imu_msg.angular_velocity.z = gyro_data.m_reading.z;

  imu_msg.linear_acceleration.x = accel_data.m_reading.x;
  imu_msg.linear_acceleration.y = accel_data.m_reading.y;
  imu_msg.linear_acceleration.z = accel_data.m_reading.z;

  return accel_data.m_time;
}


}  // namespace realsense
