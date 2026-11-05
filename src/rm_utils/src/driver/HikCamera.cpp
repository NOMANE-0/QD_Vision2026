#include "rm_utils/driver/HikCamera.hpp"

namespace qd::Device {

HikCamera::HikCamera(const Parameters& params): queue_(1), camera_handle_(nullptr), running_(true) {
    FYT_REGISTER_LOGGER("hik_camera", "qd2026-log", INFO);
    this->params_ = params;

    // 创建句柄
    this->camera_handle_ = get_camera_handle(params_);
}

HikCamera::~HikCamera() {
    running_ = false;
    if (camera_handle_ != nullptr) {
        MV_CC_StopGrabbing(camera_handle_);
        MV_CC_CloseDevice(camera_handle_);
        MV_CC_DestroyHandle(camera_handle_);
    }
    if (daemon_thread_.joinable()) {
        daemon_thread_.join();
    }
}

void HikCamera::starThread() {
    this->daemon_thread_ = std::thread(&HikCamera::image_callback, this);
}

void HikCamera::image_callback() {
    while (running_) {
        if (this->camera_handle_ == nullptr) {
            std::cout << " camera_handle_ is null!!!" << std::endl;
            return;
        }
        
        cv::Mat img;
        std::chrono::system_clock::time_point timestamp;
        get_img(img, timestamp);
        if (!img.empty()) {
            queue_.push({ img, timestamp });
        }
    }
}

void HikCamera::get_img(cv::Mat& img, std::chrono::system_clock::time_point& timestamp) {
    if (this->camera_handle_ == nullptr) {
        std::cout << " camera_handle_ is null!!!" << std::endl;
        return;
    }

    // 获得图像缓存
    nRet = MV_CC_GetImageBuffer(camera_handle_, &out_frame_buffer_, 1000);
    if (nRet != MV_OK) {
        img = cv::Mat();
        timestamp = std::chrono::system_clock::time_point();
        printf("No image data! nRet [0x%x]\n", nRet);
        return;
    }

    auto img_time = get_time(out_frame_buffer_.stFrameInfo.nHostTimeStamp);

    // 存储数据进队列
    cv::Mat dst_img(
        cv::Size(out_frame_buffer_.stFrameInfo.nWidth, out_frame_buffer_.stFrameInfo.nHeight),
        CV_8UC3
    );
    // 像素格式转换
    convert_param.nWidth = out_frame_buffer_.stFrameInfo.nWidth;
    convert_param.nHeight = out_frame_buffer_.stFrameInfo.nHeight;
    // 输入src
    convert_param.enSrcPixelType = out_frame_buffer_.stFrameInfo.enPixelType;
    convert_param.pSrcData = out_frame_buffer_.pBufAddr;
    convert_param.nSrcDataLen = out_frame_buffer_.stFrameInfo.nFrameLen;
    // 输出dst
    convert_param.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
    convert_param.pDstBuffer = dst_img.data; // 输出数据缓冲区
    convert_param.nDstBufferSize =
        out_frame_buffer_.stFrameInfo.nExtendWidth * out_frame_buffer_.stFrameInfo.nExtendHeight * 4
        + 2048;
    // 执行转换
    auto start = std::chrono::high_resolution_clock::now();
    if (this->params_.enable_opencv) {
        // 使用 opencv 进行插值
        auto pixel_type = out_frame_buffer_.stFrameInfo.enPixelType;
        const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
            { PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2BGR },
            { PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2BGR },
            { PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2BGR },
            { PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2BGR }
        };
        dst_img = cv::Mat(
            cv::Size(out_frame_buffer_.stFrameInfo.nWidth, out_frame_buffer_.stFrameInfo.nHeight),
            CV_8U,
            out_frame_buffer_.pBufAddr
        );
        cv::cvtColor(dst_img, dst_img, type_map.at(pixel_type));
    } else {
        nRet = MV_CC_ConvertPixelType(camera_handle_, &convert_param);
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    FYT_DEBUG("hik_camera", "Latecy(ConvertPixel) : {:.2f} ms", elapsed.count());
    
    img = dst_img;
    timestamp = img_time;

    // 释放图像缓存
    MV_CC_FreeImageBuffer(camera_handle_, &out_frame_buffer_);
}

cv::Mat HikCamera::get_cv_img() {
    return this->queue_.pop().img;
}

void HikCamera::read_img(cv::Mat& img, std::chrono::system_clock::time_point& timestamp) {
    CameraData data = this->queue_.pop();
    img = data.img;
    timestamp = data.timestamp;
}

void* HikCamera::get_camera_handle(const Parameters& params) {
    void* handle = nullptr;
    MVCC_STRINGVALUE string_value {};

    // 枚举设备
    MV_CC_DEVICE_INFO_LIST device_list {};
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &device_list);

    // 等待设备连接
    int count = 4;
    while (device_list.nDeviceNum == 0) {
        printf("No camera found! nRet [0x%x]\n", nRet);
        printf("waiting for camera to be connected...\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (count-- < 0) {
            return nullptr;
        }

        nRet = MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &device_list);
    }

    // 查找序列号
    MV_CC_DEVICE_INFO* device_info = device_list.pDeviceInfo[0];
    bool matched = false;
    for (unsigned int i = 0; i < device_list.nDeviceNum; i++) {
        if (!params.enable_DeviceSerialNumber) {
            break;
        }
        auto *device = device_list.pDeviceInfo[i];
        
        // 打开设备
        nRet = MV_CC_CreateHandle(&handle, device);
        if (nRet != MV_OK) {
            printf("CreateHandle failed! nRet [0x%x]\n", nRet);
            continue;
        }
        nRet = MV_CC_OpenDevice(handle);
        if (nRet != MV_OK) {
            printf("OpenDevice failed! nRet [0x%x]\n", nRet);
            MV_CC_DestroyHandle(handle);
            continue;
        }

        // 获取序列号
        nRet = MV_CC_GetStringValue(handle, "DeviceSerialNumber", &string_value);
        std::cout << "Found DeviceSerialNumber: " << string_value.chCurValue << std::endl;

        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);

        // 比较序列号
        if (nRet == MV_OK && std::string(string_value.chCurValue) == params.device_serial_number) {
            device_info = device;
            std::cout << "Found Target DeviceSerialNumber: " << string_value.chCurValue
                      << std::endl;
            matched = true;
            break;
        }
    }

    if (params.enable_DeviceSerialNumber && !matched) {
        std::cout << "enable_DeviceSerialNumber but matched is false" << std::endl;
        return nullptr;
    }

    // 打开设备
    nRet = MV_CC_CreateHandle(&handle, device_info);
    printf("MV_CC_CreateHandle! nRet [0x%x]\n", nRet);
    if (nRet != MV_OK) {
        return nullptr;
    }
    nRet = MV_CC_OpenDevice(handle);
    printf("MV_CC_OpenDevice! nRet [0x%x]\n", nRet);
    if (nRet != MV_OK) {
        MV_CC_DestroyHandle(handle);
        return nullptr;
    }
    // 网口相机根据链路 MTU 配置包长，避免默认包长造成取流失败。
    if (device_info->nTLayerType == MV_GIGE_DEVICE) {
        const int packet_size = MV_CC_GetOptimalPacketSize(handle);
        if (packet_size > 0) {
            nRet = MV_CC_SetIntValue(handle, "GevSCPSPacketSize", packet_size);
            printf("Set GevSCPSPacketSize! nRet [0x%x]\n", nRet);
        }
    }
    MV_CC_GetStringValue(handle, "DeviceSerialNumber", &string_value);
    std::cout << "Select DeviceSerialNumber: " << string_value.chCurValue << std::endl;
    // 设置相机参数
    set_hik_param(handle, params);
    // 开始取流
    nRet = MV_CC_StartGrabbing(handle);
    printf("MV_CC_StartGrabbing! nRet [0x%x]\n", nRet);
    if (nRet != MV_OK) {
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        return nullptr;
    }

    return handle;
}

void HikCamera::set_hik_param(void* handle, const Parameters& params) {
    if (handle == nullptr) {
        std::cout << " handle is null!!!" << std::endl;
        return;
    }

    // 锁帧
    nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", params.enable_frame_rate);
    printf("Set AcquisitionFrameRateEnable! nRet [0x%x]\n", nRet);
    // 帧率限制
    nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", params.frame_rate);
    printf("Set AcquisitionFrameRate! nRet [0x%x]\n", nRet);
    // 关闭设备内的自动曝光、自动增益与外部触发，使用配置文件中的固定值。
    nRet = MV_CC_SetEnumValueByString(handle, "TriggerMode", "Off");
    printf("Set TriggerMode! nRet [0x%x]\n", nRet);
    nRet = MV_CC_SetEnumValueByString(handle, "ExposureAuto", "Off");
    printf("Set ExposureAuto! nRet [0x%x]\n", nRet);
    nRet = MV_CC_SetEnumValueByString(handle, "GainAuto", "Off");
    printf("Set GainAuto! nRet [0x%x]\n", nRet);
    // 曝光
    nRet = MV_CC_SetFloatValue(handle, "ExposureTime", params.exposure_time);
    printf("Set ExposureTime! nRet [0x%x]\n", nRet);
    // 增益
    nRet = MV_CC_SetFloatValue(handle, "Gain", params.gain);
    printf("Set Gain! nRet [0x%x]\n", nRet);
    // 像素格式
    nRet = MV_CC_SetEnumValueByString(handle, "PixelFormat", params.pixel_format.c_str());
    printf("Set PixelFormat! nRet [0x%x]\n", nRet);
    // ADC比特深度
    nRet = MV_CC_SetEnumValueByString(handle, "ADCBitDepth", params.adc_bit_depth.c_str());
    printf("Set ADCBitDepth! nRet [0x%x]\n", nRet);
    //设置插值方法
    nRet = MV_CC_SetBayerCvtQuality(handle, params.cvt_quality);
    printf("Set BayerCvtQuality! nRet [0x%x]\n", nRet);
    // 画面水平、垂直翻转
    nRet = MV_CC_SetBoolValue(handle, "ReverseX", params.reverseXY);
    printf("Set ReverseX! nRet [0x%x]\n", nRet);
    nRet = MV_CC_SetBoolValue(handle, "ReverseY", params.reverseXY);
    printf("Set ReverseY! nRet [0x%x]\n", nRet);

    this->half_exposure_duration = std::chrono::duration_cast<std::chrono::system_clock::duration>(
        std::chrono::duration<double, std::micro>(params.exposure_time / 2.0)
    );
}

void HikCamera::update_camera_setting() {
    set_hik_param(this->camera_handle_, this->params_);
}

std::chrono::system_clock::time_point HikCamera::get_time() {
    return std::chrono::system_clock::now() - this->half_exposure_duration;
}

std::chrono::system_clock::time_point HikCamera::get_time(const int64_t& nHostTimeStamp) {
    // 1. 将 int64_t 的 ms 转换为毫秒 duration
    auto host_ts_duration = std::chrono::milliseconds(nHostTimeStamp);

    // 2. 处理 double 类型的曝光时间（单位 us），使用 duration<double, micro> 保持浮点精度
    std::chrono::duration<double, std::micro> half_exposure_duration(
        this->params_.exposure_time / 2.0
    );

    // 3. 计算曝光中点的总持续时间（系统会自动转换单位为 us，因为 us 精度更高）
    // 此时 result_duration 是一个以微秒为单位的浮点型 duration
    auto result_duration = host_ts_duration - half_exposure_duration;

    // 4. 将结果转换回系统时钟的 time_point
    // 注意：system_clock::time_point 通常是整数，需要进行 duration_cast
    std::chrono::system_clock::time_point img_time_center(
        std::chrono::duration_cast<std::chrono::system_clock::duration>(result_duration)
    );

    return img_time_center;
}

} // namespace qd::Device
