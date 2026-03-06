#include <detector/uav_detector.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>

namespace nn_detector
{

UAVDetector::UAVDetector(const rclcpp::Logger& logger) : logger_(logger) {}

void UAVDetector::detect(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                         std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                         radar_interface::msg::RadarWarn& lidar_detect,
                         pcl::PointCloud<pcl::PointXYZ>& other_accumulated_cloud_out,
                         geometry_msgs::msg::Point& uav_position)
{
    // 初始化告警状态，0表示未检测到
    lidar_detect.dart_state = 0;
    lidar_detect.fly_state = 0;
    uav_position.x = 0;
    uav_position.y = 0;
    uav_position.z = 0;

    // 区域过滤器：检测飞镖靶区
    // auto dart_cloud_filter = [](pcl::PointXYZ& point) {
    //     return (point.x > 28 - 0.5889 - 0.1885 && point.x < 28 - 0.5889) &&
    //            (point.y > 3.925 && point.y < 4.525) &&
    //            (point.z > 2.4722 - 0.859 + 0.1 && point.z < 2.4722);
    // };

    // 区域过滤器：检测无人机飞行区
    auto fly_filter = [](pcl::PointXYZ& point) {
        return (point.x > 13 && point.x < 27.5) &&
               (point.y > 0.2 && point.y < 1.356 + 2.4 + 0.8) &&
               (point.z > 1.7 && point.z < 3);
    };

    // 将ROS点云消息转换为PCL点云格式
    pcl::PointCloud<pcl::PointXYZ> receive_cloud;
    pcl::fromROSMsg(*msg, receive_cloud);

    // 坐标变换：将点云从原始frame变换到lidar_mid70_frame
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        if (!tf_buffer->canTransform("lidar_mid70_frame", msg->header.frame_id, tf2::TimePointZero)) {
            RCLCPP_WARN(logger_, "Waiting for transform lidar_mid70_frame -> %s", msg->header.frame_id.c_str());
            return;
        }
        transform_stamped = tf_buffer->lookupTransform("lidar_mid70_frame", msg->header.frame_id, tf2::TimePointZero);
    } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(logger_, "Transform error: %s", ex.what());
        return;
    }

    // 使用Eigen进行点云坐标变换
    Eigen::Affine3d transform_eigen = tf2::transformToEigen(transform_stamped);
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::transformPointCloud(receive_cloud, transformed_cloud, transform_eigen);

    // 过滤出关注区域的点，用于后续累积和检测
    pcl::PointCloud<pcl::PointXYZ> other_filtered_cloud;
    for (const auto& point : transformed_cloud.points) {
        if (// ((point.x > 28 - 0.5889 - 0.1885 && point.x < 28 - 0.5889) &&  // 飞镖靶区（已禁用）
            //  (point.y > 3.925 && point.y < 4.525) &&
            //  (point.z > 2.4722 - 0.859 + 0.1 && point.z < 2.4722)) ||
            ((point.x > 13 && point.x < 27.5) &&
             (point.y > 0.2 && point.y < 1.356 + 2.4 + 0.8) &&
             (point.z > 1.7 && point.z < 3))) {
            other_filtered_cloud.push_back(point);
        }
    }

    // 点云累积：多帧融合，提升检测鲁棒性
    if (accumulate_count < accumulate_time) {
        other_accumulated_clouds_.push_back(other_filtered_cloud.makeShared());
        accumulate_count++;
    } else {
        if (!other_accumulated_clouds_.empty()) {
            other_accumulated_clouds_.erase(other_accumulated_clouds_.begin());
            other_accumulated_clouds_.push_back(other_filtered_cloud.makeShared());
        }
    }

    // 合并所有累积帧点云
    for (auto& cloud : other_accumulated_clouds_) {
        other_accumulated_cloud_out += *cloud;
    }

    // 检测逻辑：对累积点云应用各区域过滤器，统计点数
    // pcl::PointCloud<pcl::PointXYZ> dart_cloud;  // 飞镖检测已禁用
    pcl::PointCloud<pcl::PointXYZ> fly_cloud;

    for (auto& point : other_accumulated_cloud_out.points) {
        // if (dart_cloud_filter(point)) dart_cloud.push_back(point);  // 飞镖检测已禁用
        if (fly_filter(point))        fly_cloud.push_back(point);
    }

    // 根据点数阈值设置告警状态
    // if (dart_cloud.size() > 5) {  // 飞镖检测已禁用
    //     lidar_detect.dart_state = 1;
    // }
    if (fly_cloud.size() > 40) {
        lidar_detect.fly_state = 1;
    }

    // 计算目标位置（质心）
    if (lidar_detect.fly_state > 0 && !fly_cloud.empty()) {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(fly_cloud, centroid);
        uav_position.x = centroid[0];
        uav_position.y = centroid[1];
        uav_position.z = centroid[2];
    }
    // else if (lidar_detect.dart_state == 1 && !dart_cloud.empty()) {  // 飞镖检测已禁用
    //     Eigen::Vector4f centroid;
    //     pcl::compute3DCentroid(dart_cloud, centroid);
    //     uav_position.x = centroid[0];
    //     uav_position.y = centroid[1];
    //     uav_position.z = centroid[2];
    // }
}

} // namespace nn_detector
