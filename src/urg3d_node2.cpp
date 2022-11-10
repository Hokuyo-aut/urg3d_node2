// Copyright 2022 HOKUYO AUTOMATIC CO., LTD.
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

#include "urg3d_node2/urg3d_node2.hpp"

namespace urg3d_node2
{

Urg3dNode2::UrgNode2(const rclcpp::NodeOptions & node_options)
: rclcpp_lifecycle::LifecycleNode("urg3d_node2", node_options),
  error_count_(0),
  is_connected_(false),
  is_measurement_started_(false),
  is_stable_(false),
  use_intensity_(false),
  system_latency_(0ns),
  user_latency_(0ns)
{
  // urg_open後にLiDARの電源がOFFになった状態でLiDARと通信しようとするとSIGPIPEシグナルが発生する
  // ROS1ではROSのライブラリで設定されていたがROS2では未対応のため、ここで設定する
  std::signal(SIGPIPE, SIG_IGN);
  
  // パラメータの登録
  ip_address_ = declare_parameter<std::string>("ip_address", "");
  ip_port_ = declare_parameter<int>("ip_port", 10940);
  frame_id_ = declare_parameter<std::string>("frame_id", "hokuyo3d");
  range_min_ = declare_parameter<double>("range_min", 0.0);
  interlace_h_ = declare_parameter<int>("interlace_h", 1);
  interlace_v_ = declare_parameter<int>("interlace_v", 1);
  output_cycle_ = declare_parameter<std::string>("output_cycle", "field");
  calibrate_time_ = declare_parameter<bool>("calibrate_time", false);
  synchronize_time_ = declare_parameter<bool>("synchronize_time", false);
  publish_intensity_ = declare_parameter<bool>("publish_intensity", false);
  publish_auxiliary_ = declare_parameter<bool>("publish_auxiliary", false);
  error_limit_ = declare_parameter<int>("error_limit", 4);
  error_reset_period_ = declare_parameter<double>("error_reset_period", 5.0),
  diagnostics_tolerance_ = declare_parameter<double>("diagnostics_tolerance", 0.05);
  diagnostics_window_time_ = declare_parameter<double>("diagnostics_window_time", 5.0);
  time_offset_ = declare_parameter<double>("time_offset", 0.0);
}

// デストラクタ
Urg3dNode2::~Urg3dNode2()
{
  // スレッドの停止
  stop_thread();
}

// onConfigure
UrgNode2::CallbackReturn UrgNode2::on_configure(const rclcpp_lifecycle::State & state)
{
    return CallbackReturn::SUCCESS;
}

// onActivate
UrgNode2::CallbackReturn UrgNode2::on_activate(const rclcpp_lifecycle::State & state)
{
    return CallbackReturn::SUCCESS;
}

// onDeactivate
UrgNode2::CallbackReturn UrgNode2::on_deactivate(const rclcpp_lifecycle::State & state)
{
    return CallbackReturn::SUCCESS;
}

// onCleanup
UrgNode2::CallbackReturn UrgNode2::on_cleanup(const rclcpp_lifecycle::State & state)
{
    return CallbackReturn::SUCCESS;
}

// onShutdown
UrgNode2::CallbackReturn UrgNode2::on_shutdown(const rclcpp_lifecycle::State & state)
{
    return CallbackReturn::SUCCESS;
}

// onError
UrgNode2::CallbackReturn UrgNode2::on_error(const rclcpp_lifecycle::State & state)
{
    return CallbackReturn::SUCCESS;
}

// 初期化
void UrgNode2::initialize()
{
    return true;
}

// Lidarとの接続処理
bool UrgNode2::connect()
{
    return true;
}

// スキャン設定
void UrgNode2::set_scan_parameter()
{
    
}

// Lidarとの切断処理
void UrgNode2::disconnect()
{
    
}

// Lidarとの再接続処理
void UrgNode2::reconnect()
{
    
}

// scanスレッド
void UrgNode2::scan_thread()
{
    
}

// スキャントピック作成(PointCloud型)
bool UrgNode2::create_scan_message(sensor_msgs::PointCloud & msg)
{
    return true;
}

// スキャントピック作成(PointCloud2型)
bool create_scan_message2(sensor_msgs::PointCloud2 & msg)
{
    return true;
}

// システムレイテンシの計測
void UrgNode2::calibrate_system_latency(size_t num_measurements)
{
    
}

// ROS時刻とLiDAR時刻の差の計算
rclcpp::Duration UrgNode2::get_native_clock_offset(size_t num_measurements)
{
    return rclcpp::Duration time_offsets;
}

// システム時刻とLiDAR時刻の差の計算
rclcpp::Duration UrgNode2::get_time_stamp_offset(size_t num_measurements)
{
    return rclcpp::Duration time_offsets;
}

// 指数移動平均による動的補正
rclcpp::Time UrgNode2::get_synchronized_time(long time_stamp, rclcpp::Time system_time_stamp)
{
    return rclcpp::Time stamp;
}

// 接続先LiDARが強度出力に対応しているかどうか
bool UrgNode2::is_intensity_supported(void)
{
    return true;
}

// 診断情報入力
void UrgNode2::populate_diagnostics_status(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    
}

// スキャンスレッドの開始
void UrgNode2::start_thread(void)
{
    
}

// スキャンスレッドの停止
void UrgNode2::stop_thread(void)
{
    
}

// Diagnosticsの開始
void UrgNode2::start_diagnostics(void)
{
    
}

// Diagnosticsの停止
void UrgNode2::stop_diagnostics(void)
{
    
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(urg3d_node2::Urg3dNode2)
