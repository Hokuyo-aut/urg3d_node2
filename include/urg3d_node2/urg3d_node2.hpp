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

/*!!! include を増やす */

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

class Urg3dNode2 // : public rclcpp_lifecycle::LifecycleNode
{
public:

private:

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

}

}

 #endif // URG3D_NODE2_HPP
