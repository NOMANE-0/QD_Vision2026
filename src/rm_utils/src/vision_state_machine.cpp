#include "rm_utils/vision_state_machine.hpp"
#include <opencv2/highgui.hpp>

namespace qd::utils {

// 静态成员初始化
std::unique_ptr<VisionStateMachine> VisionStateMachine::instance_ = nullptr;
std::mutex VisionStateMachine::instance_mutex_;

VisionStateMachine& VisionStateMachine::getInstance() {
    std::lock_guard<std::mutex> lock(instance_mutex_);
    if (!instance_) {
        instance_ = std::unique_ptr<VisionStateMachine>(new VisionStateMachine());
        FYT_REGISTER_LOGGER("vision_state_machine", "qd2026-log", INFO);
    }
    return *instance_;
}

bool VisionStateMachine::initialize(rclcpp::Node* node, const std::string& config_file) {
    if (node == nullptr) {
        FYT_ERROR("vision_state_machine", "Node pointer is null!");
        return false;
    }

    if (node_ != nullptr && !camera_configs_.empty()) {
        FYT_INFO("vision_state_machine", "VisionStateMachine already initialized, skip reinit");
        return true;
    }

    node_ = node;
    image_available_ = false;

    // 加载配置
    if (!config_file.empty()) {
        if (!loadConfig(config_file)) {
            FYT_WARN("vision_state_machine", "Failed to load config file, using default values");
        }
    } else {
        // 使用默认配置
        CameraConfig config_near_range; // 近距离相机配置
        config_near_range.hik_params.device_serial_number =
            node_->declare_parameter("camera_6mm.device_serial_number", "DA1564615");
        config_near_range.hik_params.enable_DeviceSerialNumber =
            node_->declare_parameter("camera_6mm.enable_DeviceSerialNumber", true);
        config_near_range.hik_params.exposure_time =
            node_->declare_parameter("camera_6mm.exposure_time", 1800.0);
        config_near_range.hik_params.gain = node_->declare_parameter("camera_6mm.gain", 1.0);
        config_near_range.hik_params.frame_rate =
            node_->declare_parameter("camera_6mm.acquisition_frame_rate", 200.0);
        config_near_range.hik_params.enable_frame_rate =
            node_->declare_parameter("camera_6mm.enable_frame_rate", true);
        config_near_range.hik_params.pixel_format =
            node_->declare_parameter("camera_6mm.pixel_format", "BayerRG8");
        config_near_range.hik_params.adc_bit_depth =
            node_->declare_parameter("camera_6mm.adc_bit_depth", "ADCBitDepth_8");
        config_near_range.hik_params.cvt_quality =
            node_->declare_parameter("camera_6mm.cvt_quality", 2);
        config_near_range.hik_params.reverseXY =
            node_->declare_parameter("camera_6mm.ReverseXY", false);
        config_near_range.hik_params.enable_opencv =
            node_->declare_parameter("camera_6mm.enable_opencv", false);
        config_near_range.camera_info_url = node_->declare_parameter(
            "camera_6mm.camera_info_url",
            "package://rm_bringup/config/camera_info.yaml"
        );
        config_near_range.camera_name =
            node_->declare_parameter("camera_6mm.camera_name", "camera_6mm");
        config_near_range.target_frame =
            node_->declare_parameter("camera_6mm.target_frame", "camera_optical_frame");
        config_near_range.switch_to_8mm_distance =
            node_->declare_parameter("camera_6mm.switch_to_8mm_distance", 5.0);
        config_near_range.switch_to_6mm_distance =
            node_->declare_parameter("camera_6mm.switch_to_6mm_distance", 3.0);
        camera_configs_[CameraType::CAMERA_NEAR_RANGE] = config_near_range;

        CameraConfig config_far_range; // 远距离相机配置
        config_far_range.hik_params.device_serial_number =
            node_->declare_parameter("camera_8mm.device_serial_number", "DA1041822");
        config_far_range.hik_params.enable_DeviceSerialNumber =
            node_->declare_parameter("camera_8mm.enable_DeviceSerialNumber", true);
        config_far_range.hik_params.exposure_time =
            node_->declare_parameter("camera_8mm.exposure_time", 1800.0);
        config_far_range.hik_params.gain = node_->declare_parameter("camera_8mm.gain", 1.0);
        config_far_range.hik_params.frame_rate =
            node_->declare_parameter("camera_8mm.acquisition_frame_rate", 200.0);
        config_far_range.hik_params.enable_frame_rate =
            node_->declare_parameter("camera_8mm.enable_frame_rate", true);
        config_far_range.hik_params.pixel_format =
            node_->declare_parameter("camera_8mm.pixel_format", "BayerRG8");
        config_far_range.hik_params.adc_bit_depth =
            node_->declare_parameter("camera_8mm.adc_bit_depth", "ADCBitDepth_8");
        config_far_range.hik_params.cvt_quality =
            node_->declare_parameter("camera_8mm.cvt_quality", 2);
        config_far_range.hik_params.reverseXY =
            node_->declare_parameter("camera_8mm.ReverseXY", false);
        config_far_range.hik_params.enable_opencv =
            node_->declare_parameter("camera_8mm.enable_opencv", false);
        config_far_range.camera_info_url = node_->declare_parameter(
            "camera_8mm.camera_info_url",
            "package://rm_bringup/config/camera_info_copy.yaml"
        );
        config_far_range.camera_name =
            node_->declare_parameter("camera_8mm.camera_name", "camera_8mm");
        config_far_range.target_frame =
            node_->declare_parameter("camera_8mm.target_frame", "camera_optical_frame2");
        config_far_range.switch_to_8mm_distance =
            node_->declare_parameter("camera_8mm.switch_to_8mm_distance", 5.0);
        config_far_range.switch_to_6mm_distance =
            node_->declare_parameter("camera_8mm.switch_to_6mm_distance", 3.0);
        camera_configs_[CameraType::CAMERA_FAR_RANGE] = config_far_range;
    }

    // 初始化近距离相机（默认相机）
    /*     if (!initializeCamera(CameraType::CAMERA_NEAR_RANGE)) {
        FYT_ERROR("vision_state_machine", "Failed to initialize near-range camera!");
        return false;
    } */

    FYT_INFO("vision_state_machine", "VisionStateMachine initialized successfully");
    return true;
}

bool VisionStateMachine::initializeCamera(CameraType camera_type) {
    const auto& config = camera_configs_[camera_type];

    try {
        // 使用 emplace 直接构造 HikCamera 对象（避免赋值操作）
        // 因为 HikCamera 包含 std::thread，不能被赋值

        hik_camera_.emplace(camera_type, config.hik_params);

        // 检查相机句柄是否有效（get_camera_handle 找不到相机会返回 nullptr）
        if (!hik_camera_.at(camera_type).isValid()) {
            FYT_ERROR(
                "vision_state_machine",
                "HikCamera {} handle is null, camera not found or failed to open",
                (camera_type == CameraType::CAMERA_NEAR_RANGE ? "near-range" : "far-range")
            );
            hik_camera_.erase(camera_type);
            return false;
        }

        FYT_INFO(
            "vision_state_machine",
            "HikCamera {} created successfully",
            (camera_type == CameraType::CAMERA_NEAR_RANGE ? "near-range" : "far-range")
        );

    } catch (const std::exception& e) {
        FYT_ERROR(
            "vision_state_machine",
            "Failed to create HikCamera {}: {}",
            (camera_type == CameraType::CAMERA_NEAR_RANGE ? "near-range" : "far-range"),
            e.what()
        );
        return false;
    }

    // 加载相机内参
    load_camera_info_managers(camera_type, config);

    // 启动采集线程
    // camera_running_[camera_type] = true;
    // capture_threads_[camera_type] = std::thread(&VisionStateMachine::cameraCaptureThread, this, camera_type);

    FYT_INFO(
        "vision_state_machine",
        "Camera {} initialized successfully",
        (camera_type == CameraType::CAMERA_NEAR_RANGE ? "near-range" : "far-range")
    );
    return true;
}

void VisionStateMachine::load_camera_info_managers(
    CameraType camera_type,
    const CameraConfig& config
) {
    // 加载相机内参
    auto camera_info_manager =
        std::make_unique<camera_info_manager::CameraInfoManager>(node_, config.camera_name);
    std::string resolved_url = config.camera_info_url;
    if (camera_info_manager->validateURL(resolved_url)) {
        camera_info_manager->loadCameraInfo(resolved_url);
    } else {
        FYT_WARN("vision_state_machine", "Invalid camera info URL: {}", config.camera_info_url);
    }
    camera_info_managers_[camera_type] = std::move(camera_info_manager);
}

// 不打算使用
void VisionStateMachine::cameraCaptureThread(CameraType camera_type) {
    const auto& config = camera_configs_[camera_type];
    auto& hik_camera = hik_camera_[camera_type];

    cv::Mat cv_img;
    std::chrono::system_clock::time_point timestamp;
    sensor_msgs::msg::Image image_msg;
    image_msg.encoding = "bgr8"; // HikCamera 返回的是 BGR8 格式
    image_msg.header.frame_id = config.target_frame;

    int fail_count = 0;

    while (rclcpp::ok() && camera_running_[camera_type].load()) {
        // 只处理当前激活的相机
        if (current_camera_type_.get() != camera_type) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        try {
            // 从 HikCamera 获取图像
            hik_camera.read_img(cv_img, timestamp);

            if (cv_img.empty()) {
                FYT_WARN(
                    "vision_state_machine",
                    "Empty image received from camera {}",
                    (camera_type == CameraType::CAMERA_NEAR_RANGE ? "near-range" : "far-range")
                );
                fail_count++;
                if (fail_count > 5) {
                    FYT_ERROR(
                        "vision_state_machine",
                        "Camera {} failed!",
                        (camera_type == CameraType::CAMERA_NEAR_RANGE ? "near-range" : "far-range")
                    );
                    break;
                }
                continue;
            }

            auto now = node_->now();

            // 将 cv::Mat 转换为 sensor_msgs::msg::Image
            // 使用 cv_bridge 进行转换
            std_msgs::msg::Header header;
            header.stamp = now;
            header.frame_id = config.target_frame;
            cv_bridge::CvImage cv_bridge_img(header, "bgr8", cv_img);
            cv_bridge_img.toImageMsg(image_msg);

            // 获取相机内参
            auto camera_info = camera_info_managers_[camera_type]->getCameraInfo();
            camera_info.header = image_msg.header;

            // 更新当前图像（线程安全）
            {
                std::lock_guard<std::mutex> lock(image_mutex_);
                current_image_ = image_msg;
                current_camera_info_ = camera_info;
                image_available_ = true;
            }

            fail_count = 0;
        } catch (const std::exception& e) {
            FYT_WARN(
                "vision_state_machine",
                "Failed to read image from camera {}: {}",
                (camera_type == CameraType::CAMERA_NEAR_RANGE ? "near-range" : "far-range"),
                e.what()
            );
            fail_count++;
            if (fail_count > 5) {
                FYT_ERROR(
                    "vision_state_machine",
                    "Camera {} failed!",
                    (camera_type == CameraType::CAMERA_NEAR_RANGE ? "near-range" : "far-range")
                );
                break;
            }
        }
    }
}

void VisionStateMachine::setVisionMode(VisionMode mode) {
    if (current_vision_mode_.set(mode)) {
        FYT_INFO("vision_state_machine", "Vision mode set to: {}", visionModeToString(mode));
    }
}

VisionMode VisionStateMachine::getVisionMode() const {
    return current_vision_mode_.get();
}

void VisionStateMachine::setTrackerState(TrackerState state) {
    if (current_tracker_state_.set(state)) {
        FYT_INFO("vision_state_machine", "Tracker state set to: {}", static_cast<int>(state));
    }
    // 更新相机切换逻辑
    updateCameraSwitchLogic();
}

TrackerState VisionStateMachine::getTrackerState() const {
    return current_tracker_state_.get();
}

void VisionStateMachine::updateTargetDistance(double distance) {
    last_target_distance_.set(distance);
    updateCameraSwitchLogic();
}

void VisionStateMachine::updateCameraSwitchLogic() {
    // 当前只使用 6 mm 相机；目标距离和跟踪状态变化时保持使用它。
    if (current_camera_type_.get() != CameraType::CAMERA_NEAR_RANGE) {
        switchCamera(CameraType::CAMERA_NEAR_RANGE);
    }
}

void VisionStateMachine::switchCamera(CameraType target_type) {
    CameraType current_type = current_camera_type_.get();
    if (current_type == target_type) {
        return;
    }

    // 检查目标相机是否存在（已初始化）
    if (hik_camera_.find(target_type) == hik_camera_.end()) {
        FYT_WARN(
            "vision_state_machine",
            "Target camera {} does not exist, keeping current camera {}",
            (target_type == CameraType::CAMERA_NEAR_RANGE ? "near-range" : "far-range"),
            (current_type == CameraType::CAMERA_NEAR_RANGE ? "near-range" : "far-range")
        );
        return;
    }

    FYT_INFO(
        "vision_state_machine",
        "Switching camera from {} to {}",
        (current_type == CameraType::CAMERA_NEAR_RANGE ? "near-range" : "far-range"),
        (target_type == CameraType::CAMERA_NEAR_RANGE ? "near-range" : "far-range")
    );

    // 标记当前相机停止，新相机启动
    // 注意：实际的相机切换由采集线程根据current_camera_type_自动处理
    current_camera_type_.set(target_type);

    // 清空当前图像，等待新相机图像
    {
        std::lock_guard<std::mutex> lock(image_mutex_);
        image_available_ = false;
    }
}

bool VisionStateMachine::getCurrentImage(
    sensor_msgs::msg::Image& image,
    sensor_msgs::msg::CameraInfo& camera_info
) {
    std::lock_guard<std::mutex> lock(image_mutex_);
    if (!image_available_) {
        return false;
    }
    image = current_image_;
    camera_info = current_camera_info_;
    return true;
}

bool VisionStateMachine::getCurrentImage(
    cv::Mat& img,
    sensor_msgs::msg::CameraInfo& camera_info,
    std::chrono::system_clock::time_point& timestamp
) {
    const CameraType camera_type = getCurrentCameraType();

    if (hik_camera_.find(camera_type) == hik_camera_.end()) {
        FYT_WARN(
            "vision_state_machine",
            "Target camera {} does not exist",
            (camera_type == CameraType::CAMERA_NEAR_RANGE ? "near-range" : "far-range")
        );
        img = cv::Mat();
        return false;
    }

    if (camera_info_managers_.find(camera_type) == camera_info_managers_.end()) {
        FYT_WARN(
            "vision_state_machine",
            "Camera info manager for {} does not exist",
            (camera_type == CameraType::CAMERA_NEAR_RANGE ? "near-range" : "far-range")
        );
        img = cv::Mat();
        return false;
    }

    auto& hik_camera = hik_camera_.at(camera_type);
    hik_camera.get_img(img, timestamp);
    if (img.empty()) {
        FYT_DEBUG(
            "vision_state_machine",
            "Failed to get image from camera {}",
            (camera_type == CameraType::CAMERA_NEAR_RANGE ? "near-range" : "far-range")
        );
        return false;
    }

    camera_info = camera_info_managers_.at(camera_type)->getCameraInfo();
    camera_info.header.frame_id = camera_configs_.at(camera_type).target_frame;
    camera_info.header.stamp = rclcpp::Time(
        std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp.time_since_epoch()).count()
    );
    camera_info.width = static_cast<uint32_t>(img.cols);
    camera_info.height = static_cast<uint32_t>(img.rows);
    return true;
}

bool VisionStateMachine::getCurrentCameraInfo(sensor_msgs::msg::CameraInfo& camera_info) {
    CameraType current_type = current_camera_type_.get();

    // 检查相机信息管理器是否存在
    if (camera_info_managers_.find(current_type) == camera_info_managers_.end()) {
        FYT_WARN(
            "vision_state_machine",
            "Camera info manager for {} does not exist",
            (current_type == CameraType::CAMERA_NEAR_RANGE ? "near-range" : "far-range")
        );
        return false;
    }

    // 直接从相机信息管理器获取相机内参
    camera_info = camera_info_managers_[current_type]->getCameraInfo();

    return true;
}

bool VisionStateMachine::getCurrentCameraParams(
    cv::Mat& camera_matrix,
    cv::Mat& distortion_coefficients
) {
    CameraType current_type = current_camera_type_.get();

    // 检查相机信息管理器是否存在
    if (camera_info_managers_.find(current_type) == camera_info_managers_.end()) {
        FYT_WARN(
            "vision_state_machine",
            "Camera info manager for {} does not exist",
            (current_type == CameraType::CAMERA_NEAR_RANGE ? "near-range" : "far-range")
        );
        return false;
    }

    // 获取相机内参
    auto camera_info = camera_info_managers_[current_type]->getCameraInfo();

    // 转换为OpenCV格式
    // camera_matrix: 3x3矩阵，从CameraInfo的k数组转换
    camera_matrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(camera_info.k.data())).clone();

    // distortion_coefficients: 1x5矩阵，从CameraInfo的d数组转换
    distortion_coefficients =
        cv::Mat(1, 5, CV_64F, const_cast<double*>(camera_info.d.data())).clone();

    return true;
}

void VisionStateMachine::publishArmorFrame(
    const std_msgs::msg::Header& header,
    std::vector<rm_interfaces::msg::Armor>&& armors,
    cv::Mat&& image,
    sensor_msgs::msg::CameraInfo&& camera_info
) {
    {
        std::lock_guard<std::mutex> lock(armor_frame_mutex_);
        // 直接在 optional 中构造 ArmorFrame，省去一次 ArmorFrame 移动赋值。
        latest_armor_frame_.emplace();
        auto& slot = *latest_armor_frame_;
        slot.header = header;
        slot.armors = std::move(armors);
        slot.image = std::move(image);
        slot.camera_info = std::move(camera_info);
        slot.sequence = ++armor_frame_sequence_;
    }
    armor_frame_cv_.notify_one();
}

bool VisionStateMachine::waitAndPopLatestArmorFrame(
    ArmorFrame& armor_frame,
    std::chrono::milliseconds timeout
) {
    std::unique_lock<std::mutex> lock(armor_frame_mutex_);
    if (!armor_frame_cv_.wait_for(lock, timeout, [this] {
            return latest_armor_frame_.has_value();
        })) {
        return false;
    }

    armor_frame = std::move(*latest_armor_frame_);
    latest_armor_frame_.reset();
    return true;
}

void VisionStateMachine::notifyArmorFrameWaiters() {
    armor_frame_cv_.notify_all();
}

CameraType VisionStateMachine::getCurrentCameraType() const {
    return current_camera_type_.get();
}

std::string VisionStateMachine::getCurrentTargetFrame() const {
    CameraType current_type = current_camera_type_.get();

    // 检查相机配置是否存在
    if (camera_configs_.find(current_type) == camera_configs_.end()) {
        FYT_WARN(
            "vision_state_machine",
            "Camera config for {} does not exist",
            (current_type == CameraType::CAMERA_NEAR_RANGE ? "near-range" : "far-range")
        );
        return "";
    }

    // 直接从相机配置中获取target_frame
    return camera_configs_.at(current_type).target_frame;
}

bool VisionStateMachine::shouldDetect() const {
    VisionMode mode = current_vision_mode_.get();
    // 只有自瞄模式才进行识别
    return (mode == VisionMode::AUTO_AIM_RED || mode == VisionMode::AUTO_AIM_BLUE);
}

void VisionStateMachine::updateExposureTime(double exposure_time) {
    CameraType current_type = current_camera_type_.get();

    // 检查相机是否存在
    if (hik_camera_.find(current_type) == hik_camera_.end()) {
        FYT_WARN(
            "vision_state_machine",
            "Camera {} does not exist, cannot update exposure time",
            (current_type == CameraType::CAMERA_NEAR_RANGE ? "near-range" : "far-range")
        );
        return;
    }

    // 更新相机参数
    auto& hik_camera = hik_camera_.at(current_type);
    hik_camera.params_.exposure_time = exposure_time;
    hik_camera.update_camera_setting();

    // 同步更新配置
    camera_configs_[current_type].hik_params.exposure_time = exposure_time;

    FYT_INFO(
        "vision_state_machine",
        "Updated camera {} exposure time to: {}",
        (current_type == CameraType::CAMERA_NEAR_RANGE ? "near-range" : "far-range"),
        exposure_time
    );
}

void VisionStateMachine::updateGain(double gain) {
    CameraType current_type = current_camera_type_.get();

    // 检查相机是否存在
    if (hik_camera_.find(current_type) == hik_camera_.end()) {
        FYT_WARN(
            "vision_state_machine",
            "Camera {} does not exist, cannot update gain",
            (current_type == CameraType::CAMERA_NEAR_RANGE ? "near-range" : "far-range")
        );
        return;
    }

    // 更新相机参数
    auto& hik_camera = hik_camera_.at(current_type);
    hik_camera.params_.gain = gain;
    hik_camera.update_camera_setting();

    // 同步更新配置
    camera_configs_[current_type].hik_params.gain = gain;

    FYT_INFO(
        "vision_state_machine",
        "Updated camera {} gain to: {}",
        (current_type == CameraType::CAMERA_NEAR_RANGE ? "near-range" : "far-range"),
        gain
    );
}

bool VisionStateMachine::loadConfig(const std::string& config_file) {
    try {
        YAML::Node config = YAML::LoadFile(config_file);

        // 加载近距离相机配置
        if (config["camera_6mm"]) {
            auto& cfg = camera_configs_[CameraType::CAMERA_NEAR_RANGE];
            cfg.hik_params.device_serial_number =
                config["camera_6mm"]["device_serial_number"].as<std::string>();
            cfg.hik_params.enable_DeviceSerialNumber = true;
            cfg.hik_params.exposure_time = config["camera_6mm"]["exposure_time"].as<double>();
            cfg.hik_params.gain = config["camera_6mm"]["gain"].as<double>();
            cfg.hik_params.frame_rate = config["camera_6mm"]["acquisition_frame_rate"].as<double>();
            cfg.hik_params.enable_frame_rate = true;
            cfg.hik_params.pixel_format = config["camera_6mm"]["pixel_format"].as<std::string>();
            cfg.hik_params.adc_bit_depth = config["camera_6mm"]["adc_bit_depth"].as<std::string>();
            cfg.hik_params.cvt_quality = config["camera_6mm"]["cvt_quality"].as<int>();
            cfg.hik_params.enable_opencv = config["camera_6mm"]["enable_opencv"].as<bool>(false);
            cfg.camera_info_url = config["camera_6mm"]["camera_info_url"].as<std::string>();
            cfg.camera_name = config["camera_6mm"]["camera_name"].as<std::string>();
            cfg.target_frame = config["camera_6mm"]["target_frame"].as<std::string>();
            cfg.switch_to_8mm_distance =
                config["camera_6mm"]["switch_to_8mm_distance"].as<double>();
            cfg.switch_to_6mm_distance =
                config["camera_6mm"]["switch_to_6mm_distance"].as<double>();
        }

        // 加载远距离相机配置
        if (config["camera_8mm"]) {
            auto& cfg = camera_configs_[CameraType::CAMERA_FAR_RANGE];
            cfg.hik_params.device_serial_number =
                config["camera_8mm"]["device_serial_number"].as<std::string>();
            cfg.hik_params.enable_DeviceSerialNumber = true;
            cfg.hik_params.exposure_time = config["camera_8mm"]["exposure_time"].as<double>();
            cfg.hik_params.gain = config["camera_8mm"]["gain"].as<double>();
            cfg.hik_params.frame_rate = config["camera_8mm"]["acquisition_frame_rate"].as<double>();
            cfg.hik_params.enable_frame_rate = true;
            cfg.hik_params.pixel_format = config["camera_8mm"]["pixel_format"].as<std::string>();
            cfg.hik_params.adc_bit_depth = config["camera_8mm"]["adc_bit_depth"].as<std::string>();
            cfg.hik_params.cvt_quality = config["camera_8mm"]["cvt_quality"].as<int>();
            cfg.hik_params.enable_opencv = config["camera_8mm"]["enable_opencv"].as<bool>(false);
            cfg.camera_info_url = config["camera_8mm"]["camera_info_url"].as<std::string>();
            cfg.camera_name = config["camera_8mm"]["camera_name"].as<std::string>();
            cfg.target_frame = config["camera_8mm"]["target_frame"].as<std::string>();
            cfg.switch_to_8mm_distance =
                config["camera_8mm"]["switch_to_8mm_distance"].as<double>();
            cfg.switch_to_6mm_distance =
                config["camera_8mm"]["switch_to_6mm_distance"].as<double>();
        }

        return true;
    } catch (const std::exception& e) {
        FYT_ERROR("vision_state_machine", "Failed to load config: {}", e.what());
        return false;
    }
}

void VisionStateMachine::closeCamera(CameraType camera_type) {
    // 停止采集线程
    camera_running_[camera_type] = false;

    if (capture_threads_[camera_type].joinable()) {
        capture_threads_[camera_type].join();
    }

    // 删除 HikCamera 对象（析构函数会自动关闭相机）
    if (hik_camera_.find(camera_type) != hik_camera_.end()) {
        hik_camera_.erase(camera_type);
    }
}

void VisionStateMachine::getCurrentImage(
    cv::Mat& img,
    std::chrono::system_clock::time_point& timestamp
) {
    auto camera_type = getCurrentCameraType();

    // 检查目标相机是否存在（已初始化）
    if (hik_camera_.find(camera_type) == hik_camera_.end()) {
        FYT_WARN(
            "vision_state_machine",
            "Target camera {} does not exist",
            (camera_type == CameraType::CAMERA_NEAR_RANGE ? "near-range" : "far-range")
        );
        img = cv::Mat();
        return;
    }

    auto& hik_camera = hik_camera_[camera_type];
    hik_camera.get_img(img, timestamp);

    // 如果获取失败，清空图像
    if (img.empty()) {
        FYT_DEBUG(
            "vision_state_machine",
            "Failed to get image from camera {}",
            (camera_type == CameraType::CAMERA_NEAR_RANGE ? "near-range" : "far-range")
        );
    }
}

VisionStateMachine::~VisionStateMachine() {
    armor_frame_cv_.notify_all();
    closeCamera(CameraType::CAMERA_NEAR_RANGE);
    closeCamera(CameraType::CAMERA_FAR_RANGE);
}

} // namespace qd::utils
