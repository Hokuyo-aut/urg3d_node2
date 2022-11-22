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
sensor_msgs::msg::PointCloud hokuyo_cloud;
sensor_msgs::msg::PointCloud2 hokuyo_cloud2;
void scan_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (scan_flag) {
    // store last receive scan topic
    hokuyo_cloud2 = *msg;
    scan_time = system_clock.now();
    receive_count++;
  }
}

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

TEST(YVT_30LX, normal_scan) {

    // initialize
    hokuyo_cloud = sensor_msgs::msg::PointCloud();
    hokuyo_cloud2 = sensor_msgs::msg::PointCloud2();
    scan_flag = false;
    receive_count = 0;
    scan_time = rclcpp::Time(0);
    
    // ros init
    rclcpp::init(0, nullptr);
    
    rclcpp::executors::SingleThreadedExecutor exe1;
    
    // urg3d_node2 setup
    std::vector<std::string> args;
    std::vector<rclcpp::Parameter> params = {rclcpp::Parameter("ip_address", "192.168.0.10")};
    rclcpp::NodeOptions node_options;
    node_options.arguments(args);
    node_options.parameter_overrides(params);
    
    std::shared_ptr<urg3d_node2::Urg3dNode2> node = std::make_shared<urg3d_node2::Urg3dNode2>(node_options);
    exe1.add_node(node->get_node_base_interface());
    
    // subscriber test node setup
    auto sub_node1 = rclcpp::Node::make_shared("test_subscription");
    auto subscriber1 = sub_node1->create_subscription<sensor_msgs::msg::PointCloud>(
        "hokuyo_cloud", 10, 
        scan_callback);
    auto sub_node2 = rclcpp::Node::make_shared("test_subscription");
    auto subscriber2 = sub_node2->create_subscription<sensor_msgs::msg::PointCloud2>(
        "hokuyo_cloud2", 10, 
        scan_callback);
    
    // publisher test
    std::vector<rclcpp::TopicEndpointInfo> ep_scan = node->get_publishers_info_by_topic("hokuyo_cloud2");
    std::vector<rclcpp::TopicEndpointInfo> ep_diag = node->get_publishers_info_by_topic("diagnostics");
    EXPECT_EQ((int)ep_scan.size(), 0);
    EXPECT_EQ((int)ep_diag.size(), 0);
  
    // urg3d_node2 transition (Uncondigured -> Inactive)
    node->configure();
    
    EXPECT_EQ(node->get_current_state().label(), "inactive");
    
    // publisher test
    ep_scan = node->get_publishers_info_by_topic("hokuyo_cloud2");
    ep_diag = node->get_publishers_info_by_topic("diagnostics");
    EXPECT_EQ((int)ep_scan.size(), 1);
    EXPECT_EQ((int)ep_diag.size(), 0);
    
    // scan_wait for 1sec
    scan_wait(exe1, 1.0);
    
    EXPECT_EQ(receive_count, 0);   // no receive
    receive_count = 0;
    
    // urg3d_node2 transmition (Inactive -> Active)
    node->activate();
    
    EXPECT_EQ(node->get_current_state().label(), "active");
    
    // publisher test
    ep_scan = node->get_publishers_info_by_topic("scan");
    ep_diag = node->get_publishers_info_by_topic("diagnostics");
    EXPECT_EQ((int)ep_scan.size(), 1);
    EXPECT_EQ((int)ep_diag.size(), 1);
    
    // scan wait for 10sec
    scan_wait(exe1, 10.0);
    
    // compare
    // ���C�u�����̎d�l�𒲂ׂă`�F�b�N���ڂ����߂�
    
    bool flag_nan = false;
    // !!! need fix
    //for(size_t i = 0; i < hokuyo_cloud2.ranges.size(); ++i) {
    //    if(hokuyo_cloud2.ranges[i] == std::numeric_limits<float>::quiet_Nan()){
    //        flag_nan = true;
    //   }
    //}
    EXPECT_EQ(flag_nan, false);
    // !!! need fix
    //EXPECT_EQ(hokuyo_cloud2.intensities.empty(), true);
    
    // urg3d_node2 transition (Active -> Finalize)
    node->shutdown();
    
    EXPECT_EQ(node->get_current_state().label(), "finalized");
    
    // publisher test
    ep_scan = node->get_publishers_info_by_topic("scan");
    ep_diag = node->get_publishers_info_by_topic("diagnostics");
    EXPECT_EQ((int)ep_scan.size(), 0);
    EXPECT_EQ((int)ep_diag.size(), 0);
    
    rclcpp::shutdown();
}

TEST(YVT_30LX, intensity_scan) {
   
}

TEST(YVT_30LX, auxiliary_scan){

}

TEST(YVT_30LX, intensity_auxiliary_scan){
  
}

TEST(YVT_30LX, change_interlace_h_scan) {

}

TEST(YVT_30LX, change_interlace_v_scan) {

}

TEST(YVT_30LX, change_output_cycle_scan) {

}

TEST(YVT_30LX, normal_scan_param) {

}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
