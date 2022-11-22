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
  // urg_open���LiDAR�̓d����OFF�ɂȂ�����Ԃ�LiDAR�ƒʐM���悤�Ƃ����SIGPIPE�V�O�i������������
  // ROS1�ł�ROS�̃��C�u�����Őݒ肳��Ă�����ROS2�ł͖��Ή��̂��߁A�����Őݒ肷��
  std::signal(SIGPIPE, SIG_IGN);
  
  // �p�����[�^�̓o�^
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

// �f�X�g���N�^
Urg3dNode2::~Urg3dNode2()
{
  // �X���b�h�̒�~
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

    // Publisher�ݒ�
    scan_pub_1 = create_publisher<sensor_msgs::msg::PointCloud>("hokuyo_cloud", rclcpp::QoS(20));
    scan_pub_2 = create_publisher<sensor_msgs::msg::PointCloud2>("hokuyo_cloud2", rclcpp::QoS(20));

    // �X���b�h�N��
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
        // publisher�̗L����
        if(scan_pub_1){
            scan_pub_1->on_activate();
        }
        if(scan_pub_2){
            scan_pub_2->on_activate();
        }

        // Diagnostics�J�n
        start_diagnostics();

        // �݌v�G���[�J�E���g�̏�����
        total_error_count_ = 0;
    }

    return CallbackReturn::SUCCESS;
}

// onDeactivate
Urg3dNode2::CallbackReturn Urg3dNode2::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "transition Deactivating from %s", state.label().c_str());

    // Diagnostics��~
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

    // �X���b�h��~
    stop_thread();

    // publisher�̉��
    scan_pub_1.reset();
    scan_pub_2.reset();

    // �ؒf
    disconnect();

    return CallbackReturn::SUCCESS;
}

// onShutdown
Urg3dNode2::CallbackReturn Urg3dNode2::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "transition Shutdown from %s", state.label().c_str());

    // �X���b�h��~
    stop_thread();

    // Diagnostics��~
    stop_diagnostics();

    // publisher�̉��
    scan_pub_1.reset();
    scan_pub_2.reset();

    // �ؒf
    disconnect();

    return CallbackReturn::SUCCESS;
}

// onError
Urg3dNode2::CallbackReturn Urg3dNode2::on_error(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "transition Error from %s", state.label().c_str());

    // �X���b�h�̒�~
    stop_thread();

    // Diagnostics��~
    stop_diagnostics();

    // publisher�̉��
    scan_pub_1.reset();
    scan_pub_2.reset();

    // �ؒf
    disconnect();

    return CallbackReturn::SUCCESS;
}

// ������
void Urg3dNode2::initialize()
{
    // �p�����[�^�擾
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

    // �����ϐ�������
    is_connected_ = false;
    is_measurement_started_ = false;
    is_stable_ = false;
    user_latency_ = rclcpp::Duration::from_seconds(time_offset_);

    // ���b�Z�[�W�w�b�_��frame_id�ݒ�
    header_frame_id_ =
      (frame_id_.find_first_not_of('/') == std::string::npos) ? "" : frame_id_.substr(frame_id_.find_first_not_of('/'));
      
    hardware_clock_ = 0.0;
    last_hardware_time_stamp_ = 0;
    hardware_clock_adj_ = 0;
    adj_count_ = 0;

    prev_frame_ = -1;
    prev_field_ = -1;
}

// Lidar�Ƃ̐ڑ�����
bool Urg3dNode2::connect()
{
    int result = urg3d_open(&urg_, ip_address_.c_str(), ip_port_);
    if(result < 0){
        RCLCPP_ERROR(get_logger(), "Could not open network Hokuyo 3D LiDAR\n%s:%d\n%s",
        ip_address_.c_str(), ip_port_, &urg_.last_errno);

      return false;
    }

    is_connected_ = true;

    // �^�C���A�E�g�w��(2000ms)
    urg3d_high_set_blocking_timeout_ms(&urg_, 2000);

    // 3D�Z�b�V����������
    result = urg3d_high_blocking_init(&urg_);
    if(result < 0){
        RCLCPP_ERROR(get_logger(), "Could not init library");
        disconnect();

        return false;
    }

    // �o�[�W�������擾
    result = urg3d_high_blocking_get_sensor_version(&urg_, &version_);
    if(result < 0){
        RCLCPP_ERROR(get_logger(), "Could not get version");
        disconnect();

        return false;
    }

    // LiDAR���i�[
    vendor_name_ = version_.vendor;
    product_name_ = version_.product;
    device_id_ = version_.serial;
    firmware_version_ = version_.firmware;
    protocol_name_ = version_.protocol;

    std::stringstream ss;
    ss << "Connected to a newtork";
    ss << "scan. Hardware ID: " << device_id_;
    //ss << ", version:" << vendor_name_ << "," << product_name_ << "," << device_id_ << "," << firmware_version_ << "," << protocol_name_ << ",";
    RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());

    return true;
}

// �X�L�����ݒ�
void Urg3dNode2::set_scan_parameter()
{
    
}

// Lidar�Ƃ̐ؒf����
void Urg3dNode2::disconnect()
{
    if(is_connected_){
        urg3d_close(&urg_);
        is_connected_ = false;
    }
}

// Lidar�Ƃ̍Đڑ�����
void Urg3dNode2::reconnect()
{
    disconnect();

    connect();
}

// scan�X���b�h
void Urg3dNode2::scan_thread()
{
    RCLCPP_ERROR(get_logger(), "start thread.");

    int result = 0;
    reconnect_count_ = 0;

    while(!close_thread_){
        
        if(!is_connected_){
            if(!connect()){
                rclcpp::sleep_for(500ms);
                continue;
            }
        }
        
        // Inactive��Ԕ���
        rclcpp_lifecycle::State state = get_current_state();
        if(state.label() == "inactive"){
            // �Đڑ�����
            reconnect();
            reconnect_count_++;

            rclcpp::sleep_for(100ms);
            continue;
        }

        // �X�L�����ݒ�
        set_scan_parameter();

        // �������[�h
        if (calibrate_time_) {
            calibrate_system_latency(URG_NODE2_CALIBRATION_MEASUREMENT_TIME);
        }

        // LiDAR��ԍX�V
        // !!!�ǉ������� 

        // �v���J�n
        int ret = urg3d_high_start_data(&urg_, URG3D_DISTANCE_INTENSITY);
        if(ret < 0){
            RCLCPP_WARN(get_logger(), "Could not start Hokuyo measurement\n");
    
            // �Đڑ�����
            reconnect();
            reconnect_count_++;
    
            continue;
        }

        is_measurement_started_ = true;
        error_count_ = 0;
        
        rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
        rclcpp::Time prev_time = system_clock.now();

        while(!close_thread_){
            // Inactive��Ԕ���
            rclcpp_lifecycle::State state = get_current_state();
            if (state.label() == "inactive") {
                urg3d_high_stop_data(&urg_, URG3D_DISTANCE_INTENSITY);
                is_measurement_started_ = false;
                break;
            }

            // �v���f�[�^����
            if(urg3d_next_receive_ready(&urg_)){
                if(urg3d_high_get_measurement_data(&urg_, &measurement_data_)){
                    if(prev_frame_ == -1 && prev_field_ == -1){
                        if(measurement_data_.line_number == 0){
                            prev_frame_ = measurement_data_.frame_number;
                            prev_field_ = measurement_data_.h_field_number;
                        }
                    }

                    if(prev_frame_ != -1 && prev_field_ != -1){
                        if(prev_frame_ != measurement_data_.frame_number || prev_field_ != measurement_data_.h_field_number){
                            break;
                        }

                        // publish
                        //if(create_scan_message2(msg)){
                        //scan_pub_2->publish(msg);
                        RCLCPP_DEBUG(get_logger(), "publish data.");
                        if(scan_freq_){
                            scan_freq_->tick();
                        }
                    }
                }
            }
            /*
            sensor_msgs::msg::PointCloud2 msg;
            if(create_scan_message2(msg)){
                scan_pub_2->publish(msg);
                if(scan_freq_){
                    scan_freq_->tick();
                }
            }
            else{
                RCLCPP_WARN(get_logger(), "Could not get scan.");
                error_count_++;
                total_error_count_++;
                //device_status_ = urg_sensor_status(&urg_);
                //sensor_status_ = urg_sensor_state(&urg_);
                //is_stable_ = urg_is_stable(&urg_);
            }
            */

            // �G���[�J�E���g����
            if(error_count_ > error_limit_){
                RCLCPP_ERROR(get_logger(), "Error count exceeded limit, reconnecting.");
                // �Đڑ�����
                reconnect();
                reconnect_count_++;
                break;
            }
            else{
                // �G���[�J�E���g�̃��Z�b�g
                rclcpp::Time current_time = system_clock.now();
                rclcpp::Duration period = current_time - prev_time;
                if (period.seconds() >= error_reset_period_) {
                    prev_time = current_time;
                    error_count_ = 0;
                }
            }
        }
    }

    // �ؒf����
    disconnect();
}

// �X�L�����g�s�b�N�쐬(PointCloud�^)
bool Urg3dNode2::create_scan_message(sensor_msgs::msg::PointCloud & msg)
{
    return true;
}

// �X�L�����g�s�b�N�쐬(PointCloud2�^)
bool Urg3dNode2::create_scan_message2(sensor_msgs::msg::PointCloud2 & msg)
{
    return true;
}

// �V�X�e�����C�e���V�̌v��
void Urg3dNode2::calibrate_system_latency(size_t num_measurements)
{
    
}

// ROS������LiDAR�����̍��̌v�Z
rclcpp::Duration Urg3dNode2::get_native_clock_offset(size_t num_measurements)
{
    return rclcpp::Duration::from_seconds(0);
}

// �V�X�e��������LiDAR�����̍��̌v�Z
rclcpp::Duration Urg3dNode2::get_time_stamp_offset(size_t num_measurements)
{
    return rclcpp::Duration::from_seconds(0);
}

// �w���ړ����ςɂ�铮�I�␳
rclcpp::Time Urg3dNode2::get_synchronized_time(long time_stamp, rclcpp::Time system_time_stamp)
{
    rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
    rclcpp::Time now = system_clock.now();
    return now;
}

// �ڑ���LiDAR�����x�o�͂ɑΉ����Ă��邩�ǂ���
bool Urg3dNode2::is_intensity_supported(void)
{
    return true;
}

// �f�f������
void Urg3dNode2::populate_diagnostics_status(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    if(!is_connected_){
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Not Connected");
    }

    status.add("IP Address", ip_address_);
    status.add("IP Port", ip_port_);

    if (!is_measurement_started_) {
        diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Not started: ";
    }
    //else if(){
    //
    //}
    else{
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Streaming");
    }

    status.add("Vendor Name", vendor_name_);
    status.add("Product Name", product_name_);
    status.add("Firmware Version", firmware_version_);
    status.add("Device ID", device_id_);
    status.add("Protocol name", protocol_name_);
    status.add("Computed Latency", system_latency_.seconds());
    status.add("User Time Offset", user_latency_.seconds());
    //status.add("Device Status", device_status_);
    status.add("Scan Retrieve Error Count", error_count_);
    status.add("Scan Retrieve Total Error Count", total_error_count_);
    status.add("Reconnection Count", reconnect_count_);
}

// �X�L�����X���b�h�̊J�n
void Urg3dNode2::start_thread(void)
{
    close_thread_ = false;
    scan_thread_ = std::thread(std::bind(&Urg3dNode2::scan_thread, this));
}

// �X�L�����X���b�h�̒�~
void Urg3dNode2::stop_thread(void)
{
    close_thread_ = true;
    if(scan_thread_.joinable()){
        scan_thread_.join();
    }
}

// Diagnostics�̊J�n
void Urg3dNode2::start_diagnostics(void)
{
    // Diagnostics�ݒ�
    diagnostic_updater_.reset(new diagnostic_updater::Updater(this));
    diagnostic_updater_->setHardwareID(device_id_);
    diagnostic_updater_->add("Hardware Status", this, &Urg3dNode2::populate_diagnostics_status);

    // Diagnostics�g�s�b�N�ݒ�
    diagnostics_freq_ = 1.0 / scan_period_;
    scan_freq_.reset(
      new diagnostic_updater::HeaderlessTopicDiagnostic(
        "Point Cloud2",
        *diagnostic_updater_,
        diagnostic_updater::FrequencyStatusParam(
          &diagnostics_freq_, &diagnostics_freq_, diagnostics_tolerance_,
          diagnostics_window_time_)));
}

// Diagnostics�̒�~
void Urg3dNode2::stop_diagnostics(void)
{
    // Diagnostics���
    scan_freq_.reset();

    // �b��Ή�
    // Diagnostics���ǉ������p�����[�^�̉��
    // diagnostic_updater���Ăѐ��������declare_parameter���Ă΂�G���[�ɂȂ�
    //   https://github.com/ros/diagnostics/pull/227
    if (has_parameter("diagnostic_updater.period")) {
        try {
            undeclare_parameter("diagnostic_updater.period");
        } 
        catch (const std::runtime_error & e) {
            RCLCPP_WARN(get_logger(), "undeclare_parameter failed: %s", e.what());
        }
    }
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(urg3d_node2::Urg3dNode2)
