#include "armor_solver/armor_yaw_solver.hpp"

namespace qd::auto_aim {
SolveYawPnP::SolveYawPnP(std::weak_ptr<rclcpp::Node> n): node_(n) {
    converter = std::make_shared<CoordConverter>();
}

void SolveYawPnP::solve(
    const rm_interfaces::msg::Armors::SharedPtr armors_msg,
    const std_msgs::msg::Header& source_header,
    double predict_yaw,
    std::string tracked_id,
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer
) {
    if (armors_msg->armors.empty()) {
        return;
    }
    utils::AutoTimer time1("armor_solver", "solve armor yaw");

    // 云台姿态与相机变换必须对应同一图像时刻，避免转动时混入新姿态。
    try {
        auto node = node_.lock();
        auto odom_frame_ = node->get_parameter("target_frame").as_string();

        auto gimble_frame_ = node->get_parameter("gimble_frame").as_string();
        {
            //获得电控的yaw和设置odom_frame名
            Eigen::Vector3d gimbal_rpy;
            auto gimbal_tf = tf2_buffer->lookupTransform(
                odom_frame_,
                gimble_frame_,
                source_header.stamp,
                rclcpp::Duration::from_seconds(0.01)
            );
            auto msg_q = gimbal_tf.transform.rotation;

            tf2::Quaternion tf_q;
            tf2::fromMsg(msg_q, tf_q);
            tf2::Matrix3x3(tf_q).getRPY(gimbal_rpy[0], gimbal_rpy[1], gimbal_rpy[2]);
            converter->set_gimbal_rpy(gimbal_rpy);
        }

        {
            // 获得旋转矩阵
            geometry_msgs::msg::TransformStamped camera2odom = tf2_buffer->lookupTransform(
                odom_frame_, // odom
                source_header.frame_id, // camera_optical_frame
                source_header.stamp,
                rclcpp::Duration::from_seconds(0.01)
            );
            // 1. 提取 Camera -> World 的平移向量 (t_c2w)
            Eigen::Vector3d t_c2w(
                camera2odom.transform.translation.x,
                camera2odom.transform.translation.y,
                camera2odom.transform.translation.z
            );

            // 2. 提取 Camera -> World 的旋转矩阵 (R_c2w)
            // 注意：Eigen Quaternion 构造函数参数顺序是 (w, x, y, z)
            Eigen::Quaterniond q_c2w(
                camera2odom.transform.rotation.w,
                camera2odom.transform.rotation.x,
                camera2odom.transform.rotation.y,
                camera2odom.transform.rotation.z
            );
            Eigen::Matrix3d R_c2w = q_c2w.toRotationMatrix();

            // 3. 计算 World -> Camera (求逆变换)
            // 公式: P_camera = R_w2c * P_world + T_w2c
            // 其中 R_w2c = R_c2w的转置
            //      T_w2c = - (R_c2w的转置 * t_c2w)

            Eigen::Matrix3d R_odom2camera = R_c2w.transpose();
            Eigen::Vector3d T_odom2camera = -R_odom2camera * t_c2w;
            converter->set_world2camera(R_odom2camera, T_odom2camera);
        }
        node.reset();
    } catch (tf2::TransformException& ex) {
        FYT_ERROR("armor_solver", "{}", ex.what());
        FYT_ERROR("armor_solver", "Something Wrong when lookUpTransform");
        throw ex;
    }

    if (!check_camera_params())
        return;
    std::vector<rm_interfaces::msg::Armor*> armor_msg_ptrs;
    std::vector<ArmorData> armor_datas;
    int count = 0;
    for (auto& armor: armors_msg->armors) {
        if (armor.number == tracked_id) {
            // 取地址传入，存入指针
            armor_msg_ptrs.push_back(&armor);
            ArmorData armor_data(armor);
            armor_datas.emplace_back(armor_data);
            count++;
        }
    }

    if (count == 0) {
        FYT_WARN("armor_solver", "No tracked armor found in armors_msg");
        return;
    }

    double z_to_v_exp = reduced_angle(predict_yaw - converter->gimbal_rpy[2]);
    // 这里计算可能看见的装甲板最大偏转角度
    int armor_num = armor_msg_ptrs[0]->number == "outpost" ? 3 : 4;
    const double angle_between_armors = 2.0 * M_PI / armor_num;
    // 朝向角与相机 z 轴反方向的夹角在该角度之内的装甲板几乎必定被观察
    const double must_see_angle = M_PI / 4.0;
    const double must_not_see_angle = M_PI / 2.0;
    if (count == 1) {
        const double z_to_armor_min =
            std::max(-must_not_see_angle, +must_see_angle - angle_between_armors);
        const double z_to_armor_max =
            std::min(+must_not_see_angle, -must_see_angle + angle_between_armors);

        SingleArmor single_armor(armor_datas[0], z_to_v_exp, converter);
        std::pair<double, double> res =
            this->find(z_to_armor_min, z_to_armor_max, single_armor, FIND_ANGLE_ITERATIONS);
        revise_orientation(armor_msg_ptrs[0]->pose.orientation, res.first);
    } else if (count == 2) {
        DoubleArmor double_armor(armor_datas, z_to_v_exp, converter);
        std::pair<double, double> res = this->find(
            -must_not_see_angle,
            +must_not_see_angle - angle_between_armors,
            double_armor,
            FIND_ANGLE_ITERATIONS
        );
        revise_orientation(armor_msg_ptrs[0]->pose.orientation, res.first);
        revise_orientation(armor_msg_ptrs[1]->pose.orientation, res.first + (M_PI / 2));
    }
}

void SolveYawPnP::revise_orientation(geometry_msgs::msg::Quaternion& q, double append_yaw) {
    // 1. 将 geometry_msgs::Quaternion 转换为 tf2::Quaternion
    tf2::Quaternion tf_q;
    tf2::fromMsg(q, tf_q);

    // 2. 提取当前的 Roll 和 Pitch (我们需要保留这两个值)
    double roll, pitch, cur_yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, cur_yaw);

    // 3. 计算新的 Yaw (使用你的逻辑)
    double new_yaw = reduced_angle(append_yaw + converter->gimbal_rpy[2]);

    // 4. 使用旧的 Roll/Pitch 和新的 Yaw 重设四元数
    tf_q.setRPY(roll, pitch, new_yaw);

    // 5. 将修改后的 tf2 对象转回 msg 并赋值给引用 q
    q = tf2::toMsg(tf_q);
}

const std::vector<cv::Point3f> get_world_points(std::string type) {
    // 使用 static 只在第一次调用时初始化，后续直接从内存读取，零开销
    static const std::vector<cv::Point3f> small_armor_points = {
        cv::Point3f(0, SMALL_ARMOR_WIDTH, SMALL_ARMOR_HEIGHT), // 左上
        cv::Point3f(0, SMALL_ARMOR_WIDTH, -SMALL_ARMOR_HEIGHT), // 左下
        cv::Point3f(0, -SMALL_ARMOR_WIDTH, -SMALL_ARMOR_HEIGHT), // 右下
        cv::Point3f(0, -SMALL_ARMOR_WIDTH, SMALL_ARMOR_HEIGHT) // 右上
    };

    static const std::vector<cv::Point3f> large_armor_points = {
        cv::Point3f(0, LARGE_ARMOR_WIDTH, LARGE_ARMOR_HEIGHT), // 左上
        cv::Point3f(0, LARGE_ARMOR_WIDTH, -LARGE_ARMOR_HEIGHT), // 左下
        cv::Point3f(0, -LARGE_ARMOR_WIDTH, -LARGE_ARMOR_HEIGHT), // 右下
        cv::Point3f(0, -LARGE_ARMOR_WIDTH, LARGE_ARMOR_HEIGHT) // 右上
    };

    // 用于错误返回的空容器
    static const std::vector<cv::Point3f> empty_points;
    if (type == "small") {
        return small_armor_points;
    } else if (type == "large") {
        return large_armor_points;
    } else {
        FYT_WARN("armor_solver", "get_world_points: type error");
    }
    return empty_points;
}

double get_pts_cost(
    const std::vector<cv::Point2f>& cv_refs,
    const std::vector<cv::Point2f>& cv_pts,
    const double& inclined
) {
    std::size_t size = cv_refs.size();
    std::vector<Eigen::Vector2d> refs;
    std::vector<Eigen::Vector2d> pts;
    for (std::size_t i = 0u; i < size; ++i) {
        refs.emplace_back(cv_refs[i].x, cv_refs[i].y);
        pts.emplace_back(cv_pts[i].x, cv_pts[i].y);
    }
    double cost = 0.;
    for (std::size_t i = 0u; i < size; ++i) {
        std::size_t p = (i + 1u) % size;
        // i - p 构成线段。过程：先移动起点，再补长度，再旋转
        Eigen::Vector2d ref_d = refs[p] - refs[i]; // 标准
        Eigen::Vector2d pt_d = pts[p] - pts[i];
        // 长度差代价 + 起点差代价(1 / 2)（0 度左右应该抛弃)
        double pixel_dis = // dis 是指方差平面内到原点的距离

            (0.5 * ((refs[i] - pts[i]).norm() + (refs[p] - pts[p]).norm())
             + std::fabs(ref_d.norm() - pt_d.norm()))
            / ref_d.norm();
        double angular_dis = ref_d.norm() * get_abs_angle(ref_d, pt_d) / ref_d.norm();
        // 平方可能是为了配合 sin 和 cos
        // 弧度差代价（0 度左右占比应该大）
        double cost_i = sq(pixel_dis * std::sin(inclined))
            + sq(angular_dis * std::cos(inclined)) * DETECTOR_ERROR_PIXEL_BY_SLOPE;
        // 重投影像素误差越大，越相信斜率
        cost += std::sqrt(cost_i);
    }
    return cost;
}

void CoordConverter::set_camera_params(const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs) {
    camera_matrix = cam_matrix;
    distort_coeffs = dist_coeffs;
}

void CoordConverter::update_projected_points(ArmorData& armor_data, double z_to_v_exp) {
    // 定义装甲板姿态
    double roll = 0.;
    double pitch = armor_data.pitch_inline;
    double yaw = z_to_v_exp + gimbal_rpy[2]; //  预测 yaw + 云台yaw = 装甲板 yaw in odom
    // 构建 armor 到 odom 的旋转矩阵
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q_armor2odom = yawAngle * pitchAngle * rollAngle;
    Eigen::Matrix3d R_armor2odom = q_armor2odom.toRotationMatrix();

    // 计算 armor 到 camera 的旋转矩阵和平移向量
    Eigen::Matrix3d R_armor2camera = R_odom2camera_ * R_armor2odom;
    Eigen::Vector3d T_armor2odom = armor_data.pose_in_odom;
    Eigen::Vector3d T_armor2camera = R_odom2camera_ * T_armor2odom + T_odom2camera_;

    // 转为 OpenCV 格式
    cv::Mat R_mat, tvec, rvec;
    cv::eigen2cv(R_armor2camera, R_mat);
    cv::eigen2cv(T_armor2camera, tvec);

    // 旋转矩阵 -> 旋转向量
    cv::Rodrigues(R_mat, rvec);

    cv::projectPoints(
        armor_data.world_points,
        rvec,
        tvec,
        camera_matrix,
        distort_coeffs,
        armor_data.projected_points // OUT 这里输出重投影点
    );
}

std::vector<cv::Point2f> CoordConverter::projected_points(
    const Eigen::Vector3d& pose,
    double yaw,
    const std::string& number,
    const std::string& type
) {
    // 定义装甲板姿态
    double roll = 0.;
    double pitch = number == "outpost" ? ANGLE_DOWN_15 : ANGLE_UP_15;

    // 构建 armor 到 odom 的旋转矩阵
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q_armor2odom = yawAngle * pitchAngle * rollAngle;
    Eigen::Matrix3d R_armor2odom = q_armor2odom.toRotationMatrix();

    // 计算 armor 到 camera 的旋转矩阵和平移向量
    Eigen::Matrix3d R_armor2camera = this->R_odom2camera_ * R_armor2odom;
    Eigen::Vector3d T_armor2odom = pose;
    Eigen::Vector3d T_armor2camera = this->R_odom2camera_ * T_armor2odom + this->T_odom2camera_;

    // 转为 OpenCV 格式
    cv::Mat R_mat, tvec, rvec;
    cv::eigen2cv(R_armor2camera, R_mat);
    cv::eigen2cv(T_armor2camera, tvec);

    // 旋转矩阵 -> 旋转向量
    cv::Rodrigues(R_mat, rvec);

    std::vector<cv::Point3f> world_points = get_world_points(type);
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(
        world_points,
        rvec,
        tvec,
        this->camera_matrix,
        this->distort_coeffs,
        projected_points // OUT 这里输出重投影点
    );

    return projected_points;
}

double SingleArmor::operator()(double x) {
    converter->update_projected_points(this->armor_data, x);
    return get_pts_cost(
        this->armor_data.projected_points,
        this->armor_data.pix_points,
        this->z_to_v_exp
    );
}

double DoubleArmor::operator()(double x) {
    double cost = 0.;
    for (auto& armor: this->armor_datas) {
        converter->update_projected_points(armor, x);
        cost += get_pts_cost(armor.projected_points, armor.pix_points, this->z_to_v_exp);
    }
    return cost;
}
} // namespace qd::auto_aim
