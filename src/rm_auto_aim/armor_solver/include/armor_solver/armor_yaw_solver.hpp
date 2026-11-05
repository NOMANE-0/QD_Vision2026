#pragma once
//std
#include <cmath>
#include <memory>
#include <vector>
// ros2
#include <Eigen/Geometry>
#include <angles/angles.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
// 3rd party
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
// project
#include "rm_interfaces/msg/armors.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/math/math.hpp"
#include "rm_utils/math/utils.hpp"

namespace qd::auto_aim {

#define DETECTOR_ERROR_PIXEL_BY_SLOPE 2

constexpr double FIND_ANGLE_ITERATIONS = 12; //三分法迭代次数
constexpr double ANGLE_UP_15 = M_PI / 12;
constexpr double ANGLE_DOWN_15 = -M_PI / 12;
// Armor size, Unit: m
constexpr double SMALL_ARMOR_WIDTH = 135. / 1000.0 / 2;
constexpr double SMALL_ARMOR_HEIGHT = 55. / 1000.0 / 2;
constexpr double LARGE_ARMOR_WIDTH = 225.0 / 1000.0 / 2;
constexpr double LARGE_ARMOR_HEIGHT = 55.0 / 1000.0 / 2;

/**
 * @brief 设置装甲板坐标系点
 * 
 * @param type 装甲板尺寸 small 或 large
 * @return std::vector<Eigen::Vector3d> 装甲板坐标系下的四个点，单位 m，坐标系FLU
 */
const std::vector<cv::Point3f> get_world_points(std::string type);

/**
 * @brief 计算重投影误差
 * 
 * @param cv_refs 重投影回像素坐标系的点
 * @param cv_pts  真实识别到的像素坐标系点
 * @param inclined 猜测的 yaw
 * @return double 
 */
double get_pts_cost(
    const std::vector<cv::Point2f>& cv_refs,
    const std::vector<cv::Point2f>& cv_pts,
    const double& inclined
);
/**
 * @brief 装甲板数据
 * 
 */
struct ArmorData {
    /**
     * @brief 从 armor消息中提取装甲板数据
     * 
     * @param armor_msg ros消息中的单块装甲板数据
     */
    explicit ArmorData(const rm_interfaces::msg::Armor& armor_msg) {
        type = armor_msg.type;
        pitch_inline = (armor_msg.number == "outpost" ? ANGLE_DOWN_15 : ANGLE_UP_15);

        projected_points.resize(4);
        pix_points.reserve(4);
        for (auto p: armor_msg.image_points) {
            pix_points.emplace_back(p.x, p.y);
        }

        world_points = get_world_points(type);

        pose_in_odom = Eigen::Vector3d(
            armor_msg.pose.position.x,
            armor_msg.pose.position.y,
            armor_msg.pose.position.z
        );
    }
    ~ArmorData() = default;

    std::string type; // 装甲板类型 small 或 large
    std::vector<cv::Point2f> pix_points; // 像素坐标系点
    std::vector<cv::Point2f> projected_points; // 重投影点,后续定义
    std::vector<cv::Point3f> world_points; // 世界坐标系点(装甲板坐标系)
    Eigen::Vector3d pose_in_odom; // 装甲板在odom坐标系下的位置
    double pitch_inline; // 世界坐标系到装甲板坐标系要旋转的pitch
};

class CoordConverter {
public:
    CoordConverter() = default;
    ~CoordConverter() = default;

    void set_camera_params(const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs);
    void set_gimbal_rpy(const Eigen::Vector3d& rpy) {
        gimbal_rpy = rpy;
    }
    void set_world2camera(const Eigen::Matrix3d& R, const Eigen::Vector3d& T) {
        R_odom2camera_ = R;
        T_odom2camera_ = T;
    }

    bool check_camera_params() const {
        return !camera_matrix.empty() && !distort_coeffs.empty();
    }

    /**
     * @brief 进行重投影到像素坐标系
     * 
     * @param armor_data 
     * @param z_to_v_exp 
     */
    void update_projected_points(ArmorData& armor_data, double z_to_v_exp);

    /**
     * @brief 在更新完 yaw 修复后利用残留的相机内参和旋转矩阵进行重投影使用
     * 
     * @param pose 装甲板在 odom 下的位置
     * @param yaw 装甲板相对于 odom 的yaw
     * @param number 装甲板类型
     * @param type 装甲板尺寸
     * @return std::vector<cv::Point2f> 重投影回像素坐标系的四点角点
     */
    std::vector<cv::Point2f> projected_points(
        const Eigen::Vector3d& pose,
        double yaw,
        const std::string& number,
        const std::string& type
    );

public:
    Eigen::Vector3d gimbal_rpy; // 云台当前姿态

private:
    cv::Mat camera_matrix; // 相机内参矩阵
    cv::Mat distort_coeffs; // 相机畸变参数
    Eigen::Vector3d T_odom2camera_; // 世界坐标系到相机坐标系平移向量
    Eigen::Matrix3d R_odom2camera_; // 世界坐标系到相机坐标系旋转矩阵

    Eigen::Matrix3d R_armor2camera; // 装甲板坐标系到相机坐标系旋转矩阵
};

class SingleArmor {
public:
    explicit SingleArmor(
        const ArmorData& armor_msg,
        double z_to_v_exp,
        std::shared_ptr<CoordConverter> converter
    ):
        armor_data(armor_msg),
        z_to_v_exp(z_to_v_exp),
        converter(converter) {};
    ~SingleArmor() = default;
    double operator()(double x);

public:
    ArmorData armor_data;
    const double z_to_v_exp; //EKF预测的装甲板yaw
    std::shared_ptr<CoordConverter> converter;
};

class DoubleArmor {
public:
    explicit DoubleArmor(
        const std::vector<ArmorData>& armor_msg,
        double z_to_v_exp,
        std::shared_ptr<CoordConverter> converter
    ):
        armor_datas(armor_msg),
        z_to_v_exp(z_to_v_exp),
        converter(converter) {};
    ~DoubleArmor() = default;
    double operator()(double x);

public:
    std::vector<ArmorData> armor_datas; // 两块装甲板数据
    const double z_to_v_exp; //EKF预测的装甲板yaw
    std::shared_ptr<CoordConverter> converter;
};

class SolveYawPnP {
public:
    explicit SolveYawPnP(std::weak_ptr<rclcpp::Node> node);
    ~SolveYawPnP() = default;

    /**
     * @brief 使用图像采样时刻的云台与相机变换拟合装甲板朝向。
     * @param armors_msg 已转换到目标坐标系的装甲板，本函数更新其朝向。
     * @param source_header 原始图像的坐标系和采样时间，用于查询全部历史变换。
     * @param predict_yaw EKF 预测的装甲板朝向，单位 rad，目标坐标系。
     * @param tracked_id 本次拟合的装甲板编号。
     * @param tf2_buffer 保存图像时刻变换的共享 TF 缓冲区。
     */
    void solve(
        const rm_interfaces::msg::Armors::SharedPtr armors_msg,
        const std_msgs::msg::Header& source_header,
        double predict_yaw,
        std::string tracked_id,
        std::shared_ptr<tf2_ros::Buffer> tf2_buffer
    );

    /**
     * @brief 修改四元数中的yaw值
     * 
     * @param q armor_msg中的装甲板朝向四元数
     * @param append_yaw 拟合出来的yaw值 in odom
     */
    void revise_orientation(geometry_msgs::msg::Quaternion& q, double append_yaw);

    std::vector<cv::Point2f> projected_points(
        const Eigen::Vector3d& pose,
        double yaw,
        const std::string& number,
        const std::string& type
    ) {
        return this->converter->projected_points(pose, yaw, number, type);
    }

    bool check_camera_params() const {
        return this->converter->check_camera_params();
    }

    void set_camera_params(const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs) {
        this->converter->set_camera_params(cam_matrix, dist_coeffs);
    }

    /**
     * @brief 使用黄金分割法寻找极小值
     * 直接移植的方瞄
     * @tparam ValueT 
     * @tparam Func 
     * @param left 最做值
     * @param right 最右值
     * @param cost_function 重投影类
     * @param iterations_num 迭代次数 
     * @return std::pair<ValueT, ValueT> 
     */
    template<typename ValueT, class Func>
    std::pair<ValueT, ValueT>
    find(ValueT left, ValueT right, Func&& cost_function, const int& iterations_num) {
        ValueT phi = (std::sqrt(5.) - 1.) / 2.;
        ValueT ml_cost = 0., mr_cost = 0.;
        int reserved = -1;
        for (int i = 0; i < iterations_num; ++i) {
            ValueT ml = left + (right - left) * (1. - phi);
            ValueT mr = left + (right - left) * phi;
            if (reserved != 0) {
                ml_cost = cost_function(ml);
            }
            if (reserved != 1) {
                mr_cost = cost_function(mr);
            }
            if (ml_cost < mr_cost) {
                right = mr;
                mr_cost = ml_cost;
                reserved = 1;
            } else {
                left = ml;
                ml_cost = mr_cost;
                reserved = 0;
            }
        }
        return std::make_pair((left + right) / ValueT(2), right - left);
    }

private:
    std::weak_ptr<rclcpp::Node> node_; // 节点指针
    std::shared_ptr<CoordConverter> converter;
};
} // namespace qd::auto_aim
