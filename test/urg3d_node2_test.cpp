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

#include "gtest/gtest.h"
#include "urg3d_node2/urg3d_node2.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

using namespace std::chrono_literals;

double test_scan_period = 10.0;

bool scan_flag = false;
int receive_count = 0;

rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
rclcpp::Time scan_time;

// single scan callback

// multiecho scan callback

// first(multiecho) scan callback

// last(multiecho) scan callback

// most_intense(multiecho) scan callback

/*
void scan_wait(
  rclcpp::executors::SingleThreadedExecutor & exe1, double wait_period)
{
  // queue flush
  exe1.spin_some();

  // timer count
  rclcpp::Time prev_time = system_clock.now();

  // start receive scan count
  scan_flag = true;

  // receive scan for wait_period seconds
  while (rclcpp::ok()) {
    exe1.spin_some();

    rclcpp::Time current_time = system_clock.now();
    rclcpp::Duration period = current_time - prev_time;
    if (period.seconds() >= wait_period) {
      break;
    }
  }
  scan_flag = false;
}
*/

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
