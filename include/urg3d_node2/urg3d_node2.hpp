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

/**
 * @file urg3d_node2.hpp
 * @brief ROS2対応3DLiDARドライバ
 */
 
 #ifndef URG3D_NODE2_HPP
 #define URG3D_NODE2_HPP

#include <chrono>
#include <string>
#include <sstream>
#include <utility>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <memory>
#include <thread>
#include <functional>
#include <limits>
#include <csignal>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
//#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include "urg3d_sensor.h"

#define MODE_LIO (0) // Modified version for Lidar odometory.

using namespace std::chrono_literals;

/** @def
 * 受信可能なスキャンデータの最大サイズ
 */
#define URG_NODE2_MAX_DATA_SIZE 5000  // FIXME

/** @def
 * 調整モード時のシステムレイテンシ計測における通信実施回数
 */
#define URG_NODE2_CALIBRATION_MEASUREMENT_TIME 10

namespace urg3d_node2
{

class Urg3dNode2 : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  
  /**
   * @brief コンストラクタ
   * @details クラス内メンバ変数の初期化、パラメータの宣言及び"SIGPIPE"の無効化設定を行う
   */
  explicit Urg3dNode2(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
   
  /**
   * @brief デストラクタ
   * @details スキャンスレッドの終了処理を行う
   */
  ~Urg3dNode2();
  
  /**
   * @brief Lifecycle制御におけるUnconfiguredからInactiveへの遷移時の処理
   * @details パラメータの取得や内部の初期化およびLiDAR接続処理、スレッドの作成を実施する
   * @param[in] state 遷移前の状態
   * @retval CallbackReturn::SUCCESS 遷移処理成功
   * @retval CallbackReturn::FAILURE LiDAR未接続
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  /**
   * @brief Lifecycle制御におけるInactiveからActiveへの遷移時の処理
   * @details LiDARが接続されている場合はDiagnosticsを有効化する
   * @param[in] state 遷移前の状態
   * @retval CallbackReturn::SUCCESS 遷移処理成功
   * @retval CallbackReturn::ERROR LiDAR未接続
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  
  /**
   * @brief Lifecycle制御におけるActiveからInactiveへの遷移時の処理
   * @details Diagnosticsの資源を解放し、LiDARが接続されているかの判定を行う
   * @param[in] state 遷移前の状態
   * @retval CallbackReturn::SUCCESS 遷移処理成功
   * @retval CallbackReturn::ERROR LiDAR未接続
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  
  /**
   * @brief Lifecycle制御におけるInactiveからUnconfiguredへの遷移時の処理
   * @details スレッドの停止、publisherなどの資源の解放およびLiDAR切断処理を実施する
   * @param[in] state 遷移前の状態
   * @retval CallbackReturn::SUCCESS 遷移処理成功
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  
  /**
   * @brief Lifecycle制御における各状態からFinalizedへの遷移時の処理
   * @details スレッドを停止させ、publisherとDiagnosticsの資源の解放およびLiDARとの接続を解除する
   * @param[in] state 遷移前の状態
   * @retval CallbackReturn::SUCCESS 遷移処理成功
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  
  /**
   * @brief Lifecycle制御における各状態からErrorへの遷移時の処理
   * @details スレッドの停止、publisherとDiagnosticsの資源の解放およびLiDAR切断処理を実施する
   * @param[in] state 遷移前の状態
   * @retval CallbackReturn::SUCCESS 遷移処理成功
   */
  CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
  
private:
  /**
   * @brief 初期化
   * @details 各種パラメータの設定、内部変数の初期化およびスキャンデータのpublisherの設定を行う
   */
  void initialize(void);
  
  /**
   * @brief LiDAR接続
   * @details LiDARとの接続処理とLiDAR情報の取得を行う
   * @retval true 接続成功
   * @retval false 接続失敗
   */
  bool connect(void);
  
  /**
   * @brief センサ接続時の初期化
   * @details urg3dライブラリの各種初期化処理を行う
   * @retval true 成功
   * @retval false 失敗
   */
  bool sensor_init(void);
  
  /**
   * @brief LiDAR切断
   * @details LiDARからの切断処理を行う
   */
  void disconnect(void);
  
  /**
   * @brief LiDAR再接続
   * @details LiDARからの切断処理および接続処理を行う
   */
  void reconnect(void);
  
  /**
   * @brief スキャンスレッド
   * @details LiDARからスキャンデータを受信しトピックとして配信を行う
   */
  void scan_thread(void);
  
  /**
   * @brief スキャントピック作成(PointCloud2型)
   * @details LiDARから取得したスキャン情報のトピックへの変換を行う
   * @param[out] msg スキャンデータメッセージ
   * @retval true 正常終了
   * @retval false 取得失敗
   */
  bool create_scan_message2(sensor_msgs::msg::PointCloud2 & msg);

  /**
   * @brief スキャントピック作成(auxiliary)
   * @details LiDARから取得した補助データ情報のトピックへの変換を行う
   * @retval true 正常終了
   * @retval false 取得失敗
  */
  bool create_auxiliary_message();
  
  /**
   * @brief システムレイテンシの計算
   * @details 調整モード（calibrate_time_==true）時に実行、内部変数system_latency_を設定する
   * @param[in] num_measurements 計測回数
   */
  void calibrate_system_latency(size_t num_measurements);
  
  /**
   * @brief ROS時刻とLiDAR時刻の差の計算
   * @details ROS時刻とLiDAR時刻の差を指定回数取得し中央値を返す
   * @param[in] num_measurements 計測回数
   * @return 計測結果
   */
  rclcpp::Duration get_native_clock_offset(size_t num_measurements);
  
  /**
   * @brief システム時刻とLiDAR時刻の差の計算
   * @details システム時刻とLiDAR時刻の差を指定回数取得し中央値を返す
   * @param[in] num_measurements 計測回数
   * @return 計測結果
   */
  rclcpp::Duration get_time_stamp_offset(size_t num_measurements);
  
  /**
   * @brief タイムスタンプの動的補正
   * @details システム時刻とLiDAR時刻の指数移動平均で補正する
   * @param[in] time_stamp LiDAR時刻
   * @param[in] system_time_stamp システム時刻
   * @return 補正後タイムスタンプ
   */
  rclcpp::Time get_synchronized_time(long time_stamp, rclcpp::Time system_time_stamp);
  
  /**
   * @brief 強度モード対応確認
   * @details 接続先のLiDARが強度モードに対応しているかを確認する
   * @retval true 強度モード対応
   * @retval false 強度モード非対応
   */
  bool is_intensity_supported(void);
  
  /**
   * @brief 診断情報の作成
   * @details Diagnosticsで出力するHardware status情報を作成を行う
   */
  void populate_diagnostics_status(diagnostic_updater::DiagnosticStatusWrapper & status);
  
  /**
   * @brief スキャンスレッドの開始
   * @details LiDARとの通信を行うスキャンスレッドの開始を行う
   */
  void start_thread(void);
  
  /**
   * @brief スキャンスレッドの停止
   * @details LiDARとの通信を行うスキャンスレッドの停止および停止待機を行う
   */
  void stop_thread(void);
  
  /**
   * @brief Diagnosticsの開始
   * @details Diagnosticsの設定を行う
   */
  void start_diagnostics(void);
  
  /**
   * @brief Diagnosticsの停止
   * @details Diagnosticsの資源解放と登録されたパラメータの削除を行う
   */
  void stop_diagnostics(void);

  /** PointCloud2データ */
  sensor_msgs::msg::PointCloud2 cloud2_;

  // YVT sensor sends 10 imu data at once.
  sensor_msgs::msg::Imu imu_array_[10];           // IMU (gyro and accerelometer.
  sensor_msgs::msg::MagneticField mag_array_[10]; // Magnetic field ( MPU-6200 does not have this )
  sensor_msgs::msg::Temperature temp_array_[10];  // tempreerature
  int imu_array_cnt_;
  
  /** スキャンデータのpublisher */
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>> scan_pub_2;

  /** 補助データのpublisher */
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>> imu_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::MagneticField>> mag_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Temperature>> temp_pub_;
  
  /** Diagnositcs Updater */
  std::unique_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  /** スキャンデータトピック診断用設定 */
  std::unique_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> scan_freq_;
  
  /** スレッドの終了フラグ */
  bool close_thread_;
  /** スキャンスレッドのスレッド変数 */
  std::thread scan_thread_;
  
  /** LiDAR管理構造体 */
  urg3d_t urg_;
  /** LiDARヘッダ構造体 */
  urg3d_vssp_header_t header_;
  /** データ格納 */
  char data_[URG3D_MAX_RX_LENGTH];
  /** 受信データ長 */
  int length_data_;
  /** LiDARバージョン構造体 */
  urg3d_sensor_version_t version_;
  
  /** パラメータ"ip_address" : 接続先IPアドレス */
  std::string ip_address_;
  /** パラメータ"ip_port" : 接続ポート */
  int ip_port_;
  /** パラメータ"frame_id" : スキャンデータのframe_id */
  std::string frame_id_;
  /** パラメータ"range_min" : 点を削除する距離 */
  double range_min_;
  /** パラメータ"interlace_h" : 水平インターレース設定 */
  int interlace_h_;
  /** パラメータ"interlace_v" : 垂直インターレース設定 */
  int interlace_v_;
  /** パラメータ "output_cycle" : 点群の出力タイミング */
  std::string output_cycle_;
  /** パラメータ"calibrate_time" : 調整モード */
  bool calibrate_time_;
  /** パラメータ"synchronize_time" : 同期モード */
  bool synchronize_time_;
  /** パラメータ"publish_intensity" : 強度出力モード */
  bool publish_intensity_;
  /** パラメータ"publish_auxiliary" : 補助データ出力モード */
  bool publish_auxiliary_;
  /** パラメータ"error_limit" : 再接続を行うエラー回数 */
  int error_limit_;
  /** パラメータ"error_reset_period" : エラーをリセットする期間 */
  double error_reset_period_;
  /** パラメータ"diagnostics_tollerance" : Diagnositcs許容範囲 */
  double diagnostics_tolerance_;
  /** パラメータ"diagnostics_window_time" : Diagnositcsウィンドウタイム */
  double diagnostics_window_time_;
  /** パラメータ"time_offset" : ユーザレイテンシ[sec] */
  double time_offset_;
  
  /** ベンダ名 */
  std::string vendor_name_;
  /** 製品名 */
  std::string product_name_;
  /** デバイスID */
  std::string device_id_;
  /** ファームウェアバージョン */
  std::string firmware_version_;
  /** プロトコル名 */
  std::string protocol_name_;

  /** スキャン時間(sec) */
  double scan_period_ = 0.05;
  /** 1フレームのライン数 */
  double frame_per_lines_ = 35;

  /** 計測データ */
  urg3d_measurement_data_t measurement_data_;
  /** 補助データ */
  urg3d_auxiliary_data_t auxiliary_data_;
  /** 前フレーム */
  int prev_frame_;
  /** 前フィールド */
  int prev_field_;
  /** サイクル設定 */
  enum PublishCycle
  {
    CYCLE_FIELD,
    CYCLE_FRAME,
    CYCLE_LINE
  };
  PublishCycle cycle_;
  
  /** 通信エラーカウンタ（再接続時にリセット） */
  int error_count_;
  /** 通信エラー合計カウンタ（Active遷移時にリセット） */
  int total_error_count_;
  /** 再接続カウンタ（Active&Inactive時） */
  int reconnect_count_;
  
  /** LiDAR接続状態 */
  bool is_connected_;
  /** LiDAR計測状態 */
  bool is_measurement_started_;
  /** LiDAR安定状態 */
  bool is_stable_;
  /** 強度モードが使用できるかどうか */
  bool use_intensity_;
  
  /** Diagnosticsのtarget frequency */
  double diagnostics_freq_;
  
  /** トピックのframe_id設定 */
  std::string header_frame_id_;
  
  /** システムレイテンシ */
  rclcpp::Duration system_latency_;
  
  /** ユーザレイテンシ[ns] */
  rclcpp::Duration user_latency_;
  
  /** 同期モード用変数 */
  double hardware_clock_;
  /** 同期モード用変数 */
  long int last_hardware_time_stamp_;
  /** 同期モード用変数 */
  double hardware_clock_adj_;
  /** 同期モード用変数 */
  const double adj_alpha_ = 0.01;
  /** 同期モード用変数 */
  int adj_count_;

};

}
 #endif // URG3D_NODE2_HPP
