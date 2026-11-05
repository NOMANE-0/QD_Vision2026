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

#include "armor_detector/armor_detector_node.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

namespace qd::auto_aim {
ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions& options):
    Node("armor_detector", options),
    state_machine_(qd::utils::VisionStateMachine::getInstance()) {
    FYT_REGISTER_LOGGER("armor_detector", "qd2026-log", INFO);
    FYT_INFO("armor_detector", "Starting ArmorDetectorNode!");

    // Choose detector
    detector_type_ = this->declare_parameter("detector", "tradition");
    detector_ = initDetector();

    // Set dynamic parameter callback
    on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&ArmorDetectorNode::onSetParameters, this, std::placeholders::_1)
    );

    // Tricks to make pose more accurate
    // use_ba_ = this->declare_parameter("use_ba", true);

    max_yaw_angle_ = this->declare_parameter("max_yaw_angle", 60.0);
    const int startup_skip_detections = this->declare_parameter("startup_skip_detections", 5);
    startup_detections_remaining_.store(
        startup_skip_detections > 0 ? startup_skip_detections : 0
    );

    // Armors Publisher
    armors_pub_ = this->create_publisher<rm_interfaces::msg::Armors>(
        "armor_detector/armors",
        rclcpp::SensorDataQoS()
    );

    // Transform initialize
    odom_frame_ = this->declare_parameter("target_frame", "odom");
    imu_to_camera_ = Eigen::Matrix3d::Identity();

    // Visualization Marker Publisher
    // See http://wiki.ros.org/rviz/DisplayTypes/Marker
    armor_marker_.ns = "armors";
    armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
    armor_marker_.scale.x = 0.03;
    armor_marker_.scale.y = 0.15;
    armor_marker_.scale.z = 0.12;
    armor_marker_.color.a = 0.5;
    armor_marker_.color.r = 1.0;
    armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

    text_marker_.ns = "classification";
    text_marker_.action = visualization_msgs::msg::Marker::ADD;
    text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker_.scale.z = 0.1;
    text_marker_.color.a = 0.5;
    text_marker_.color.r = 1.0;
    text_marker_.color.g = 1.0;
    text_marker_.color.b = 1.0;
    text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

    marker_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("armor_detector/marker", 10);

    // Debug Publishers
    debug_ = this->declare_parameter("debug", true);
    if (debug_) {
        createDebugPublishers();
    }
    // Debug param change moniter
    debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    debug_cb_handle_ =
        debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter& p) {
            debug_ = p.as_bool();
            debug_ ? createDebugPublishers() : destroyDebugPublishers();
        });

    // 图像获取模式选择
    // true: 状态机线程模式 (image_callback)
    // false: 话题订阅模式 (imageCallback)
    use_state_machine_camera_ = this->declare_parameter("use_state_machine_camera", false);
    FYT_INFO(
        "armor_detector",
        "Image processing mode: {}",
        use_state_machine_camera_ ? "State Machine Thread (image_callback)"
                                  : "Topic Subscription (imageCallback)"
    );

    if (!use_state_machine_camera_) {
        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera_info",
            rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
                cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
                cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
                camera_info_msg = *camera_info;
                if (armor_pose_estimator_ == nullptr) {
                    armor_pose_estimator_ = std::make_unique<ArmorPoseEstimator>(cam_info_);
                    refreshCameraParamsCache(*camera_info);
                } else if (refreshCameraParamsCache(*camera_info)) {
                    cv::Mat camera_matrix =
                        cv::Mat(3, 3, CV_64F, const_cast<double*>(camera_info->k.data())).clone();
                    cv::Mat distortion_coefficients = cv::Mat(
                                                          1,
                                                          static_cast<int>(camera_info->d.size()),
                                                          CV_64F,
                                                          const_cast<double*>(camera_info->d.data())
                    )
                                                          .clone();
                    armor_pose_estimator_->setCameraParame(camera_matrix, distortion_coefficients);
                }
                armor_pose_estimator_->setMaxYawAngle(max_yaw_angle_);
            }
        );
    }

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface()
    );
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    set_mode_srv_ = this->create_service<rm_interfaces::srv::SetMode>(
        "armor_detector/set_mode",
        std::bind(
            &ArmorDetectorNode::setModeCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    heartbeat_ = HeartBeatPublisher::create(this);

    // 图像和相机信息发布器（用于状态机模式）
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "armor_detector/camera_info",
        rclcpp::SensorDataQoS()
    );

    // 初始化 HighPerfDataRecorder
    bool enable_record = this->declare_parameter("enable_record", false);
    if (enable_record) {
        std::string record_dir = this->declare_parameter("record_dir", "/ros_ws/log");
        double record_fps = this->declare_parameter("record_fps", 200.0);
        int record_width = this->declare_parameter("record_width", 1440);
        int record_height = this->declare_parameter("record_height", 1080);

        data_recorder_ = std::make_unique<HighPerfDataRecorder>(
            record_dir,
            record_fps,
            record_width,
            record_height
        );

        serial_sub_ = this->create_subscription<rm_interfaces::msg::SerialReceiveData>(
            "serial/receive",
            rclcpp::SensorDataQoS(),
            [this](const rm_interfaces::msg::SerialReceiveData::ConstSharedPtr msg) {
                if (data_recorder_) {
                    std::string data = std::to_string(msg->mode) + ","
                        + std::to_string(msg->bullet_speed) + "," + std::to_string(msg->roll) + ","
                        + std::to_string(msg->yaw) + "," + std::to_string(msg->pitch);
                    double timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
                    data_recorder_->pushSerialData(data, timestamp);
                }
            }
        );
    }

    if (use_state_machine_camera_) {
        // 状态机线程模式：启动状态机和图像处理线程
        state_machine_.initialize(this);
        state_machine_.initializeCamera(qd::utils::CameraType::CAMERA_NEAR_RANGE);
        // 当前未安装 8 mm 相机，暂时只初始化 6 mm 相机。
        // state_machine_.initializeCamera(qd::utils::CameraType::CAMERA_FAR_RANGE);
    } else {
        // 话题订阅模式：订阅图像话题，使用 imageCallback
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw",
            rclcpp::SensorDataQoS(),
            std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1)
        );
    }
    // 创建多线程
    this->image_publish_thread_ = std::thread(&ArmorDetectorNode::imagePublishLoop, this);
    if (use_state_machine_camera_) {
        this->camera_thread_ = std::thread(&ArmorDetectorNode::image_callback, this);
    }
}

ArmorDetectorNode::~ArmorDetectorNode() {
    should_stop_image_publish_ = true;
    image_queue_cv_.notify_one();

    if (camera_thread_.joinable()) {
        camera_thread_.join();
    }
    if (image_publish_thread_.joinable()) {
        image_publish_thread_.join();
    }
}

/**
 * @brief 初始化装甲板识别器
 * 
 * @return std::unique_ptr<Detector> 
 */
std::unique_ptr<Detector> ArmorDetectorNode::initDetector() {
    if (detector_type_ == "tradition") {
        FYT_INFO("armor_detector", "Using Traditional Vision Detector");
        return initTraditionalDetector();
    } else if (detector_type_ == "yolo") {
        FYT_INFO("armor_detector", "Using YOLOv5 Neural Network Detector");
        return initYoloDetector();
    }
    FYT_ERROR("armor_detector", "Unknown detector type: {}", detector_type_);
    return nullptr;
}

/**
 * @brief 单进程模式下的图像识别线程，包含图像获取和图像识别
 * 
 */
void ArmorDetectorNode::image_callback() {
    if (!use_state_machine_camera_) {
        return;
    }

    cv::Mat img;
    std_msgs::msg::Header header;
    sensor_msgs::msg::CameraInfo camera_info;

    while (rclcpp::ok()) {
        utils::AutoTimer timer("armor_detector", "ArmorDetectorNode::image_callback");

        if (!update_state_machine_msg(img, header, camera_info)) {
            continue;
        }

        processImageAndPublish(std::move(img), header, std::move(camera_info));
    }

    // 通知发布线程停止
    should_stop_image_publish_ = true;
    image_queue_cv_.notify_one();
}

/**
 * @brief 图像发布线程，异步发布
 * 
 */
void ArmorDetectorNode::imagePublishLoop() {
    while (!should_stop_image_publish_) {
        {
            std::unique_lock<std::mutex> lock(image_queue_mutex_);
            image_queue_cv_.wait(lock, [this] {
                return !image_queue_.empty() || should_stop_image_publish_;
            });
            if (should_stop_image_publish_ && image_queue_.empty()) {
                break;
            }
            if (!image_queue_.empty()) {
                auto img_data = std::move(image_queue_.front());
                image_queue_.pop();

                if (data_recorder_) {
                    double timestamp =
                        img_data.first.stamp.sec * 1e9 + img_data.first.stamp.nanosec;
                    data_recorder_->pushImage(img_data.second, timestamp);
                }
            }
        }
    }
}

/**
 * @brief 图像话题订阅回调
 * 
 * @param img_msg 
 */
void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
    auto final_time = this->now();
    auto latency = (final_time - img_msg->header.stamp).seconds() * 1000;
    FYT_DEBUG("armor_detector", "Latency(img to imageCallback): {:.2f}ms", latency);

    if (!state_machine_.shouldDetect()) {
        return;
    }

    switch (state_machine_.getVisionMode()) {
        case VisionMode::AUTO_AIM_RED:
            detector_->detect_color = EnemyColor::RED;
            break;
        case VisionMode::AUTO_AIM_BLUE:
            detector_->detect_color = EnemyColor::BLUE;
            break;
        default:
            break;
    }

    if (cam_info_ == nullptr || armor_pose_estimator_ == nullptr) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "camera_info is not ready, skip current image"
        );
        return;
    }

    auto cv_image = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);
    sensor_msgs::msg::CameraInfo camera_info = camera_info_msg;
    camera_info.header = img_msg->header;
    camera_info.width = static_cast<uint32_t>(cv_image->image.cols);
    camera_info.height = static_cast<uint32_t>(cv_image->image.rows);
    // cam_center_ 已经在 cam_info_sub_ 回调中设置，这里不再重复赋值

    processImageAndPublish(std::move(cv_image->image), img_msg->header, std::move(camera_info));
}

void ArmorDetectorNode::processImageAndPublish(
    cv::Mat img,
    const std_msgs::msg::Header& header,
    sensor_msgs::msg::CameraInfo camera_info
) {
    // 检测装甲板
    utils::AutoTimer detect_timer("armor_detector", "detectArmors");
    auto armors = detectArmors(img, header);
    detect_timer.stop();

    std::vector<rm_interfaces::msg::Armor> armors_msg;

    // 提取装甲板位姿，这里还会写入一些 armors_msg 信息
    if (armor_pose_estimator_ != nullptr) {
        utils::AutoTimer pnp_timer("armor_detector", "ArmorPoseEstimator::extractArmorPoses");
        armors_msg = armor_pose_estimator_->extractArmorPoses(armors, imu_to_camera_);
        pnp_timer.stop();
    } else {
        FYT_WARN("armor_detector", "PnP Failed!");
    }

    if (!armors_msg.empty()) {
        int remaining = startup_detections_remaining_.load();
        while (remaining > 0) {
            if (startup_detections_remaining_.compare_exchange_weak(remaining, remaining - 1)) {
                FYT_INFO(
                    "armor_detector",
                    "Skipping startup armor detection result ({} remaining)",
                    remaining - 1
                );
                return;
            }
        }
    }

    // 发布marker（用于可视化）
    publishMarkers(header, armors_msg);

    if (debug_) {
        auto debug_armors_msg = std::make_unique<rm_interfaces::msg::Armors>();
        debug_armors_msg->header = header;
        debug_armors_msg->armors = armors_msg;
        armors_pub_->publish(std::move(debug_armors_msg));
    }

    // 串行发布 camera_info
    camera_info_pub_->publish(
        std::make_unique<sensor_msgs::msg::CameraInfo>(camera_info)
    );

    // 提前在锁外完成图像克隆，缩短临界区
    cv::Mat queued_img;
    if (data_recorder_) {
        queued_img = img.clone();
    }
    {
        std::lock_guard<std::mutex> lock(image_queue_mutex_);
        if (data_recorder_ && image_queue_.size() < 2) {
            image_queue_.push({ header, std::move(queued_img) });
        }
        image_queue_cv_.notify_one();
    }

    state_machine_
        .publishArmorFrame(header, std::move(armors_msg), std::move(img), std::move(camera_info));

    // 逻辑执行成功，发布心跳
    if (heartbeat_ != nullptr) {
        heartbeat_->publish();
    }
}

/**
 * @brief 初始化传统视觉类
 * 
 * @return std::unique_ptr<TraditionalDetector> 初始化好的识别器
 */
std::unique_ptr<TraditionalDetector> ArmorDetectorNode::initTraditionalDetector() {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;
    param_desc.integer_range[0].from_value = 0;
    param_desc.integer_range[0].to_value = 255;
    int binary_thres = declare_parameter("binary_thres", 160, param_desc);

    TraditionalDetector::LightParams l_params = {
        .min_ratio = declare_parameter("light.min_ratio", 0.08),
        .max_ratio = declare_parameter("light.max_ratio", 0.4),
        .max_angle = declare_parameter("light.max_angle", 40.0),
        .color_diff_thresh = static_cast<int>(declare_parameter("light.color_diff_thresh", 25))
    };

    TraditionalDetector::ArmorParams a_params = {
        .min_light_ratio = declare_parameter("armor.min_light_ratio", 0.6),
        .min_small_center_distance = declare_parameter("armor.min_small_center_distance", 0.8),
        .max_small_center_distance = declare_parameter("armor.max_small_center_distance", 3.2),
        .min_large_center_distance = declare_parameter("armor.min_large_center_distance", 3.2),
        .max_large_center_distance = declare_parameter("armor.max_large_center_distance", 5.0),
        .max_angle = declare_parameter("armor.max_angle", 35.0)
    };

    auto detector =
        std::make_unique<TraditionalDetector>(binary_thres, EnemyColor::RED, l_params, a_params);

    // Init classifier
    namespace fs = std::filesystem;
    fs::path model_path =
        utils::URLResolver::getResolvedPath("package://armor_detector/model/lenet.onnx");
    fs::path label_path =
        utils::URLResolver::getResolvedPath("package://armor_detector/model/label.txt");
    FYT_ASSERT_MSG(
        fs::exists(model_path) && fs::exists(label_path),
        model_path.string() + " Not Found!"
    );

    double threshold = this->declare_parameter("classifier_threshold", 0.7);
    std::vector<std::string> ignore_classes =
        this->declare_parameter("ignore_classes", std::vector<std::string> { "negative" });
    detector->classifier =
        std::make_unique<NumberClassifier>(model_path, label_path, threshold, ignore_classes);

    // Init Corrector
    bool use_pca = this->declare_parameter("use_pca", true);
    if (use_pca) {
        detector->corner_corrector = std::make_unique<LightCornerCorrector>();
    }

    return detector;
}

/**
 * @brief 构造 yolo 识别器
 * 
 * @return std::unique_ptr<Yolov5Detector> 初始化好的识别器
 */
std::unique_ptr<Yolov5Detector> ArmorDetectorNode::initYoloDetector() {
    // model path
    namespace fs = std::filesystem;
    fs::path xml_path =
        utils::URLResolver::getResolvedPath("package://armor_detector/model/IR/0526.xml");
    fs::path bin_path =
        utils::URLResolver::getResolvedPath("package://armor_detector/model/IR/0526.bin");

    FYT_ASSERT_MSG(
        fs::exists(xml_path) && fs::exists(bin_path),
        xml_path.string() + ", " + bin_path.string() + " Not Found!"
    );

    // inferece device
    std::string device = this->declare_parameter("device", "CPU");

    Yolov5Detector::Yolov5Params y_params = {
        .conf_thresh = this->declare_parameter("yolo.conf_threshold", 0.65),
        .nms_thresh = this->declare_parameter("yolo.nms_threshold", 0.45),
        .min_large_center_distance = this->declare_parameter("yolo.min_large_center_distance", 3.2),
        .ignore_yoloclasses =
            this->declare_parameter("ignore_classes", std::vector<std::string> { "negative" })
    };

    auto detector =
        std::make_unique<Yolov5Detector>(xml_path, bin_path, device, EnemyColor::RED, y_params);

    return detector;
}

/**
 * @brief 装甲板识别并绘制可视化
 * 
 * @return std::vector<Armor> 
 */
std::vector<Armor>
ArmorDetectorNode::detectArmors(cv::Mat& img, const std_msgs::msg::Header& header) {
    auto armors = detector_->detect(img);

    auto final_time = this->now();
    auto latency = (final_time - header.stamp).seconds() * 1000;

    // Publish debug info
    if (debug_) {
        detector_->drawResults(img);
        if (detector_type_ == "tradition") {
            auto* trad_detector = dynamic_cast<TraditionalDetector*>(detector_.get());
            binary_img_pub_.publish(
                cv_bridge::CvImage(header, "mono8", trad_detector->binary_img).toImageMsg()
            );

            // Sort lights and armors data by x coordinate
            std::sort(
                trad_detector->debug_lights.data.begin(),
                trad_detector->debug_lights.data.end(),
                [](const auto& l1, const auto& l2) { return l1.center_x < l2.center_x; }
            );
            std::sort(
                trad_detector->debug_armors.data.begin(),
                trad_detector->debug_armors.data.end(),
                [](const auto& a1, const auto& a2) { return a1.center_x < a2.center_x; }
            );

            lights_data_pub_->publish(trad_detector->debug_lights);
            armors_data_pub_->publish(trad_detector->debug_armors);

            if (!armors.empty()) {
                auto all_num_img = trad_detector->getAllNumbersImage();
                number_img_pub_.publish(
                    *cv_bridge::CvImage(header, "mono8", all_num_img).toImageMsg()
                );
            }
        }

        // Draw camera center
        cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);
        // Draw latency
        std::stringstream latency_ss;
        latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
        auto latency_s = latency_ss.str();
        cv::putText(
            img,
            latency_s,
            cv::Point(10, 30),
            cv::FONT_HERSHEY_SIMPLEX,
            1.0,
            cv::Scalar(0, 255, 0),
            2
        );
        FYT_DEBUG("armor_detector", "Latency(img to detected): {:.2f}ms", latency);
    }
    return armors;
}

/**
 * @brief 检测到参数更新后的回调函数
 * 
 * @param parameters 
 * @return rcl_interfaces::msg::SetParametersResult 
 */
rcl_interfaces::msg::SetParametersResult
ArmorDetectorNode::onSetParameters(std::vector<rclcpp::Parameter> parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto& param: parameters) {
        if (detector_type_ == "tradition") {
            auto* trad_detector = dynamic_cast<TraditionalDetector*>(detector_.get());
            if (param.get_name() == "binary_thres") {
                trad_detector->binary_thres = param.as_int();
            } else if (param.get_name() == "classifier_threshold") {
                trad_detector->classifier->threshold = param.as_double();
            } else if (param.get_name() == "light.min_ratio") {
                trad_detector->light_params.min_ratio = param.as_double();
            } else if (param.get_name() == "light.max_ratio") {
                trad_detector->light_params.max_ratio = param.as_double();
            } else if (param.get_name() == "light.max_angle") {
                trad_detector->light_params.max_angle = param.as_double();
            } else if (param.get_name() == "light.color_diff_thresh") {
                trad_detector->light_params.color_diff_thresh = param.as_int();
            } else if (param.get_name() == "armor.min_light_ratio") {
                trad_detector->armor_params.min_light_ratio = param.as_double();
            } else if (param.get_name() == "armor.min_small_center_distance") {
                trad_detector->armor_params.min_small_center_distance = param.as_double();
            } else if (param.get_name() == "armor.max_small_center_distance") {
                trad_detector->armor_params.max_small_center_distance = param.as_double();
            } else if (param.get_name() == "armor.min_large_center_distance") {
                trad_detector->armor_params.min_large_center_distance = param.as_double();
            } else if (param.get_name() == "armor.max_large_center_distance") {
                trad_detector->armor_params.max_large_center_distance = param.as_double();
            } else if (param.get_name() == "armor.max_angle") {
                trad_detector->armor_params.max_angle = param.as_double();
            } else if (param.get_name() == "binary_thres") {
                trad_detector->binary_thres = static_cast<int>(param.as_int());
            }
        } else if (detector_type_ == "yolo") {
            auto* yolo_detector = dynamic_cast<Yolov5Detector*>(detector_.get());
            if (param.get_name() == "yolo.conf_threshold") {
                yolo_detector->yolov5_params.conf_thresh = param.as_double();
            } else if (param.get_name() == "yolo.nms_threshold") {
                yolo_detector->yolov5_params.nms_thresh = param.as_double();
            } else if (param.get_name() == "yolo.min_large_center_distance") {
                yolo_detector->yolov5_params.min_large_center_distance = param.as_double();
            }
        }
        // 相机参数更改 - 曝光时间
        if (param.get_name() == "camera_6mm.exposure_time")
        {
            state_machine_.updateExposureTime(param.as_double());
            FYT_INFO("armor_detector", "Updated exposure_time to: {}", param.as_double());
        }
        // 相机参数更改 - 增益
        else if (param.get_name() == "camera_6mm.gain")
        {
            state_machine_.updateGain(param.as_double());
            FYT_INFO("armor_detector", "Updated gain to: {}", param.as_double());
        } else if (param.get_name() == "max_yaw_angle") {
            max_yaw_angle_ = param.as_double();
            if (armor_pose_estimator_ != nullptr) {
                armor_pose_estimator_->setMaxYawAngle(max_yaw_angle_);
            }
        }
    }
    return result;
}

void ArmorDetectorNode::createDebugPublishers() noexcept {
    lights_data_pub_ =
        this->create_publisher<rm_interfaces::msg::DebugLights>("armor_detector/debug_lights", 10);
    armors_data_pub_ =
        this->create_publisher<rm_interfaces::msg::DebugArmors>("armor_detector/debug_armors", 10);
    this->declare_parameter("armor_detector.result_img.jpeg_quality", 50);
    this->declare_parameter("armor_detector.binary_img.jpeg_quality", 50);
    binary_img_pub_ = image_transport::create_publisher(this, "armor_detector/binary_img");
    number_img_pub_ = image_transport::create_publisher(this, "armor_detector/number_img");
}

void ArmorDetectorNode::destroyDebugPublishers() noexcept {
    lights_data_pub_.reset();
    armors_data_pub_.reset();

    binary_img_pub_.shutdown();
    number_img_pub_.shutdown();
}

void ArmorDetectorNode::publishMarkers(
    const std_msgs::msg::Header& header,
    const std::vector<rm_interfaces::msg::Armor>& armors
) noexcept {
    using Marker = visualization_msgs::msg::Marker;
    marker_array_.markers.clear();
    armor_marker_.id = 0;
    text_marker_.id = 0;
    armor_marker_.header = text_marker_.header = header;
    for (const auto& armor: armors) {
        armor_marker_.pose = armor.pose;
        armor_marker_.id++;
        text_marker_.pose.position = armor.pose.position;
        text_marker_.id++;
        text_marker_.pose.position.y -= 0.1;
        text_marker_.text = armor.number;
        marker_array_.markers.emplace_back(armor_marker_);
        marker_array_.markers.emplace_back(text_marker_);
    }

    armor_marker_.action = armors.empty() ? Marker::DELETEALL : Marker::ADD;
    marker_array_.markers.emplace_back(armor_marker_);
    marker_pub_->publish(
        std::make_unique<visualization_msgs::msg::MarkerArray>(std::move(marker_array_))
    );
}

/**
 * @brief 视觉识别模式设置的服务通信回调
 * 
 * @param request 
 * @param response 
 */
void ArmorDetectorNode::setModeCallback(
    const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
    std::shared_ptr<rm_interfaces::srv::SetMode::Response> response
) {
    response->success = true;
    response->message = "0";

    VisionMode mode = static_cast<VisionMode>(request->mode);
    std::string mode_name = visionModeToString(mode);
    if (mode_name == "UNKNOWN") {
        FYT_ERROR("armor_detector", "Invalid mode: {}", request->mode);
        return;
    }

    switch (mode) {
        case VisionMode::AUTO_AIM_RED: {
            detector_->detect_color = EnemyColor::RED;
            state_machine_.setVisionMode(VisionMode::AUTO_AIM_RED);
            break;
        }
        case VisionMode::AUTO_AIM_BLUE: {
            detector_->detect_color = EnemyColor::BLUE;
            state_machine_.setVisionMode(VisionMode::AUTO_AIM_BLUE);
            break;
        }
        default: {
            break;
        }
    }

    FYT_WARN("armor_detector", "Set mode to {}", mode_name);
}

bool ArmorDetectorNode::update_state_machine_msg(
    cv::Mat& img,
    std_msgs::msg::Header& header,
    sensor_msgs::msg::CameraInfo& camera_info
) {
    // 视觉识别模式设置
    if (!state_machine_.shouldDetect()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        return false;
    }

    utils::AutoTimer timer_mode("armor_detector", "getVisionMode");
    auto mode = state_machine_.getVisionMode();
    switch (mode) {
        case VisionMode::AUTO_AIM_RED: {
            detector_->detect_color = EnemyColor::RED;
            break;
        }
        case VisionMode::AUTO_AIM_BLUE: {
            detector_->detect_color = EnemyColor::BLUE;
            break;
        }
        default: {
            break;
        }
    }
    timer_mode.stop();

    // 获得图像与当前帧相机内参
    utils::AutoTimer timer("armor_detector", "getCurrentImage");
    std::chrono::system_clock::time_point timestamp;
    if (!state_machine_.getCurrentImage(img, camera_info, timestamp)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        return false;
    }
    timer.stop();

    // 设置发送头 header
    // 时间戳格式转换
    auto nanos =
        std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp.time_since_epoch()).count();
    rclcpp::Time ros_time(nanos);
    header.stamp = ros_time;
    header.frame_id = state_machine_.getCurrentTargetFrame();
    camera_info.header = header;
    cam_center_ = cv::Point2f(camera_info.k[2], camera_info.k[5]);

    // 初始化或更新 armor_pose_estimator
    // 仅在相机内参实际发生变化时刷新，避免每帧都做 clone + setCameraParame
    const bool params_changed = refreshCameraParamsCache(camera_info);
    if (armor_pose_estimator_ == nullptr) {
        armor_pose_estimator_ = std::make_unique<ArmorPoseEstimator>();
    }
    if (params_changed) {
        utils::AutoTimer timer_cam("armor_detector", "setCameraParame");
        cv::Mat camera_matrix =
            cv::Mat(3, 3, CV_64F, const_cast<double*>(camera_info.k.data())).clone();
        cv::Mat distortion_coefficients = cv::Mat(
                                              1,
                                              static_cast<int>(camera_info.d.size()),
                                              CV_64F,
                                              const_cast<double*>(camera_info.d.data())
        )
                                              .clone();
        armor_pose_estimator_->setCameraParame(camera_matrix, distortion_coefficients);
        armor_pose_estimator_->setMaxYawAngle(max_yaw_angle_);
        timer_cam.stop();
    }

    return true;
}

bool ArmorDetectorNode::refreshCameraParamsCache(const sensor_msgs::msg::CameraInfo& camera_info) {
    const bool k_changed = !camera_params_initialized_
        || !std::equal(camera_info.k.begin(), camera_info.k.end(), last_camera_k_.begin());
    const bool d_changed = !camera_params_initialized_ || camera_info.d.size() != last_camera_d_.size()
        || !std::equal(camera_info.d.begin(), camera_info.d.end(), last_camera_d_.begin());

    if (!k_changed && !d_changed) {
        return false;
    }

    std::copy(camera_info.k.begin(), camera_info.k.end(), last_camera_k_.begin());
    last_camera_d_ = camera_info.d;
    camera_params_initialized_ = true;
    return true;
}

} // namespace qd::auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(qd::auto_aim::ArmorDetectorNode)
