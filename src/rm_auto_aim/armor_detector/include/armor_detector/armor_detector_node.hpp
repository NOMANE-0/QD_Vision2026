// Copyright Chen Jun 2023. Licensed under the MIT License.
//
// Additional modifications and features by Chengfu Zou, Labor. Licensed under
// Apache License 2.0.
//
// Copyright (C) FYT Vision Group. All rights reserved.
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

#ifndef ARMOR_DETECTOR_DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR_DETECTOR_NODE_HPP_

// ros2
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/create_timer_ros.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
// std
#include <array>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>
// project
#include "armor_detector/armor_pose_estimator.hpp"
#include "armor_detector/detector.hpp"
#include "armor_detector/number_classifier.hpp"
#include "armor_detector/traditional_detector.hpp"
#include "armor_detector/yolov5_detector.hpp"
#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "rm_interfaces/srv/set_mode.hpp"
#include "rm_utils/HighPerfDataRecorder.hpp"
#include "rm_utils/heartbeat.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/vision_state_machine.hpp"

namespace qd::auto_aim {

// Armor Detector Node
// Subscribe to the image topic, run the armor detection alogorithm and publish
// the detected armors
class ArmorDetectorNode: public rclcpp::Node {
public:
    ArmorDetectorNode(const rclcpp::NodeOptions& options);
    ~ArmorDetectorNode() override;

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
    void image_callback();
    /**
     * @brief 检测图像并发布装甲板结果，跳过启动后的前几帧有效装甲板结果。
     * @param img 当前相机图像。
     * @param header 当前帧时间戳和坐标系。
     * @param camera_info 当前帧的相机内参。
     */
    void processImageAndPublish(
        cv::Mat img,
        const std_msgs::msg::Header& header,
        sensor_msgs::msg::CameraInfo camera_info
    );

    std::string detector_type_;
    std::unique_ptr<Detector> detector_;
    std::unique_ptr<Detector> initDetector();
    std::unique_ptr<TraditionalDetector> initTraditionalDetector();
    std::unique_ptr<Yolov5Detector> initYoloDetector();

    std::vector<Armor> detectArmors(cv::Mat& img, const std_msgs::msg::Header& header);

    void createDebugPublishers() noexcept;
    void destroyDebugPublishers() noexcept;

    void publishMarkers(
        const std_msgs::msg::Header& header,
        const std::vector<rm_interfaces::msg::Armor>& armors
    ) noexcept;

    void setModeCallback(
        const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
        std::shared_ptr<rm_interfaces::srv::SetMode::Response> response
    );

    // Dynamic Parameter
    // 动态参数更新
    rcl_interfaces::msg::SetParametersResult
    onSetParameters(std::vector<rclcpp::Parameter> parameters);
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

    // Heartbeat
    HeartBeatPublisher::SharedPtr heartbeat_;

    // Pose Solver
    // 坐标估计器
    bool use_ba_;
    std::unique_ptr<ArmorPoseEstimator> armor_pose_estimator_;

    rclcpp::Publisher<rm_interfaces::msg::Armors>::SharedPtr armors_pub_;

    // Visualization marker publisher
    // marker 可视化
    visualization_msgs::msg::Marker armor_marker_;
    visualization_msgs::msg::Marker text_marker_;
    visualization_msgs::msg::MarkerArray marker_array_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // Camera info part
    // 相机内参订阅端
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    cv::Point2f cam_center_;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;

    // Image subscription
    // 图像订阅端
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

    // ReceiveData subscripiton
    std::string odom_frame_;
    Eigen::Matrix3d imu_to_camera_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    // 视觉识别模式切换服务
    rclcpp::Service<rm_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;

    // 图像处理模式选择
    // true: 使用 image_callback (状态机线程模式)
    // false: 使用 imageCallback (话题订阅模式)
    bool use_state_machine_camera_;
    // 启动后跳过指定数量的有效装甲板结果；空帧不消耗次数。
    std::atomic<int> startup_detections_remaining_ { 0 };

    // Debug information
    bool debug_;
    std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
    rclcpp::Publisher<rm_interfaces::msg::DebugLights>::SharedPtr lights_data_pub_;
    rclcpp::Publisher<rm_interfaces::msg::DebugArmors>::SharedPtr armors_data_pub_;
    image_transport::Publisher binary_img_pub_;
    image_transport::Publisher number_img_pub_;

    // 相机内参发布
    sensor_msgs::msg::CameraInfo camera_info_msg;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

    // 状态机
    qd::utils::VisionStateMachine& state_machine_;
    std::thread camera_thread_; // 相机线程
    bool update_state_machine_msg(
        cv::Mat& img,
        std_msgs::msg::Header& header,
        sensor_msgs::msg::CameraInfo& camera_info
    );

    // 异步图像发布（避免 publish 阻塞检测循环）
    std::thread image_publish_thread_;
    std::queue<std::pair<std_msgs::msg::Header, cv::Mat>> image_queue_;

    std::mutex image_queue_mutex_;
    std::condition_variable image_queue_cv_;
    std::atomic<bool> should_stop_image_publish_ { false };
    void imagePublishLoop(); // 异步发布线程函数

    std::unique_ptr<HighPerfDataRecorder> data_recorder_;
    rclcpp::Subscription<rm_interfaces::msg::SerialReceiveData>::SharedPtr serial_sub_;

    double max_yaw_angle_;

    // 缓存的相机内参，仅在内参实际变化时刷新 ArmorPoseEstimator，
    // 避免每帧都进行 cv::Mat::clone() 与 setCameraParame 的开销。
    std::array<double, 9> last_camera_k_ {};
    std::vector<double> last_camera_d_;
    bool camera_params_initialized_ { false };

    // 比较 CameraInfo 的 k/d 是否发生变化；若变化则刷新缓存并返回 true。
    bool refreshCameraParamsCache(const sensor_msgs::msg::CameraInfo& camera_info);
};

} // namespace qd::auto_aim

#endif // ARMOR_DETECTOR_DETECTOR_NODE_HPP_
