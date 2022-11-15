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

Urg3dNode2::Urg3dNode2(const rclcpp::NodeOptions & node_options)
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
Urg3dNode2::CallbackReturn Urg3dNode2::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "transition Configuring from %s", state.label().c_str());

    initialize();

    if(!connect()) {
        return CallbackReturn::FAILURE;
    }

    // Publisher設定
    scan_pub_1 = create_publisher<sensor_msgs::msg::PointCloud>("hokuyo_cloud", rclcpp::QoS(20));
    scan_pub_2 = create_publisher<sensor_msgs::msg::PointCloud2>("hokuyo_cloud2", rclcpp::QoS(20));

    // スレッド起動
    start_thread();

    return CallbackReturn::SUCCESS;
}

// onActivate
Urg3dNode2::CallbackReturn Urg3dNode2::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "transition Activating from %s", state.label().c_str());

    if(!is_connected_) {
        return CallbackReturn::ERROR;
    }
    else{
        // publisherの有効化
        if(scan_pub_1){
            scan_pub_1->on_activate();
        }
        if(scan_pub_2){
            scan_pub_2->on_activate();
        }

        // Diagnostics開始
        start_diagnostics();

        // 累計エラーカウントの初期化
        total_error_count_ = 0;
    }

    return CallbackReturn::SUCCESS;
}

// onDeactivate
Urg3dNode2::CallbackReturn Urg3dNode2::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "transition Deactivating from %s", state.label().c_str());

    // Diagnostics停止
    stop_diagnostics();

    if(!is_connected_){
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

// onCleanup
Urg3dNode2::CallbackReturn Urg3dNode2::on_cleanup(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "transition CleaningUp from %s", state.label().c_str());

    // スレッド停止
    stop_thread();

    // publisherの解放
    scan_pub_1.reset();
    scan_pub_2.reset();

    // 切断
    disconnect();

    return CallbackReturn::SUCCESS;
}

// onShutdown
Urg3dNode2::CallbackReturn Urg3dNode2::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "transition Shutdown from %s", state.label().c_str());

    // スレッド停止
    stop_thread();

    // Diagnostics停止
    stop_diagnostics();

    // publisherの解放
    scan_pub_1.reset();
    scan_pub_2.reset();

    // 切断
    disconnect();

    return CallbackReturn::SUCCESS;
}

// onError
Urg3dNode2::CallbackReturn Urg3dNode2::on_error(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "transition Error from %s", state.label().c_str());

    // スレッドの停止
    stop_thread();

    // Diagnostics停止
    stop_diagnostics();

    // publisherの解放
    scan_pub_1.reset();
    scan_pub_2.reset();

    // 切断
    disconnect();

    return CallbackReturn::SUCCESS;
}

// 初期化
void Urg3dNode2::initialize()
{
    // パラメータ取得
    ip_address_ = get_parameter("ip_address").as_string();
    ip_port_ = get_parameter("ip_port").as_int();
    frame_id_ = get_parameter("frame_id").as_string();
    range_min_ = get_parameter("range_min").as_double();
    interlace_h_= get_parameter("interlace_h").as_int();
    interlace_v_ = get_parameter("interlace_v").as_int();
    output_cycle_ = get_parameter("output_cycle").as_string();
    publish_intensity_ = get_parameter("publish_intensity").as_bool();
    publish_auxiliary_ = get_parameter("publish_auxiliary").as_bool();
    calibrate_time_ = get_parameter("calibrate_time").as_bool();
    synchronize_time_ = get_parameter("synchronize_time").as_bool();
    error_limit_ = get_parameter("error_limit").as_int();
    error_reset_period_ = get_parameter("error_reset_period").as_double();
    diagnostics_tolerance_ = get_parameter("diagnostics_tolerance").as_double();
    diagnostics_window_time_ = get_parameter("diagnostics_window_time").as_double();
    time_offset_ = get_parameter("time_offset").as_double();

    // 内部変数初期化
    is_connected_ = false;
    is_measurement_started_ = false;
    is_stable_ = false;
    user_latency_ = rclcpp::Duration::from_seconds(time_offset_);

    // メッセージヘッダのframe_id設定
    header_frame_id_ =
      (frame_id_.find_first_not_of('/') == std::string::npos) ? "" : frame_id_.substr(frame_id_.find_first_not_of('/'));
      
    hardware_clock_ = 0.0;
    last_hardware_time_stamp_ = 0;
    hardware_clock_adj_ = 0;
    adj_count_ = 0;
}

// Lidarとの接続処理
bool Urg3dNode2::connect()
{
    return true;
}

// スキャン設定
void Urg3dNode2::set_scan_parameter()
{
    
}

// Lidarとの切断処理
void Urg3dNode2::disconnect()
{
    
}

// Lidarとの再接続処理
void Urg3dNode2::reconnect()
{
    
}

// scanスレッド
void Urg3dNode2::scan_thread()
{
    
}

// スキャントピック作成(PointCloud型)
bool Urg3dNode2::create_scan_message(sensor_msgs::msg::PointCloud & msg)
{
    return true;
}

// スキャントピック作成(PointCloud2型)
bool create_scan_message2(sensor_msgs::msg::PointCloud2 & msg)
{
    return true;
}

// システムレイテンシの計測
void Urg3dNode2::calibrate_system_latency(size_t num_measurements)
{
    
}

// ROS時刻とLiDAR時刻の差の計算
rclcpp::Duration Urg3dNode2::get_native_clock_offset(size_t num_measurements)
{
    return rclcpp::Duration::from_seconds(0);
}

// システム時刻とLiDAR時刻の差の計算
rclcpp::Duration Urg3dNode2::get_time_stamp_offset(size_t num_measurements)
{
    return rclcpp::Duration::from_seconds(0);
}

// 指数移動平均による動的補正
rclcpp::Time Urg3dNode2::get_synchronized_time(long time_stamp, rclcpp::Time system_time_stamp)
{
    rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
    rclcpp::Time now = system_clock.now();
    return now;
}

// 接続先LiDARが強度出力に対応しているかどうか
bool Urg3dNode2::is_intensity_supported(void)
{
    return true;
}

// 診断情報入力
void Urg3dNode2::populate_diagnostics_status(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    
}

// スキャンスレッドの開始
void Urg3dNode2::start_thread(void)
{
    
}

// スキャンスレッドの停止
void Urg3dNode2::stop_thread(void)
{
    
}

// Diagnosticsの開始
void Urg3dNode2::start_diagnostics(void)
{
    
}

// Diagnosticsの停止
void Urg3dNode2::stop_diagnostics(void)
{
    
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(urg3d_node2::Urg3dNode2)
