#include "pc_aligner/aligner_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <memory>
#include <open3d/geometry/PointCloud.h>
#include <open3d/visualization/utility/ColorMap.h>
#include <radar_interface/livox_struct.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <vector>

// 构造函数：初始化节点并声明参数
AlignerNode::AlignerNode() : Node("pc_aligner")
{
    // 声明参数，设置默认值
    declare_parameter("sample_lidar", "lidar");
    declare_parameter("init_sample", 100000);
    declare_parameter("startup_manual_align", false);

    declare_parameter("mesh_sample", 10000);
    // 默认场地大小为 28 * 15(m)
    declare_parameter("crop_box.min", std::vector<double> { 0.150, 0.150, 0. });
    declare_parameter("crop_box.max", std::vector<double> { 27.850, 14.850, 1.500 });
    declare_parameter("max_corr_dist", 5.);
    declare_parameter("max_iteration", 30);
    declare_parameter("quality_gate.enable", true);
    declare_parameter("quality_gate.max_rmse", 0.24);
    declare_parameter("quality_gate.min_fitness", 0.80);

    declare_parameter("use_preselect", false); // 根据预选点进行对齐
    declare_parameter("preselect_pcd", "preselect.pcd");

    declare_parameter("vis_auto", true);
    declare_parameter("align_model", "mesh");
    declare_parameter("mesh", "bg2align.stl");
    declare_parameter("mesh_scale", 0.001);
    declare_parameter("pointcloud", "bg2align.pcd");

    declare_parameter("manual_crop.min", std::vector<double> { 0, -15., -15. });
    declare_parameter("manual_crop.max", std::vector<double> { 25., 15., 15. });
    declare_parameter("manual_align.auto_retry_on_fail", false);
    declare_parameter("manual_align.max_retries", 0);

    declare_parameter("tf_pub_interval_ms", 1000);
    // x, y, z, qw, qx, qy, qz
    auto init_trans = declare_parameter("init_trans", std::vector<double> { 0., 0., 0., 1., 0., 0., 0. });

    middle_tf = std::make_shared<geometry_msgs::msg::TransformStamped>();
    middle_tf->header.frame_id = get_parameter("sample_lidar").as_string() + "_frame";
    middle_tf->child_frame_id = "middle";
    middle_tf->transform.translation.x = init_trans[0];
    middle_tf->transform.translation.y = init_trans[1];
    middle_tf->transform.translation.z = init_trans[2];
    middle_tf->transform.rotation.w = init_trans[3];
    middle_tf->transform.rotation.x = init_trans[4];
    middle_tf->transform.rotation.y = init_trans[5];
    middle_tf->transform.rotation.z = init_trans[6];

    world_tf = std::make_shared<geometry_msgs::msg::TransformStamped>();
    world_tf->header.frame_id = "middle";
    world_tf->child_frame_id = "world";
    world_tf->transform.translation.x = 0.0;
    world_tf->transform.translation.y = 0.0;
    world_tf->transform.translation.z = 0.0;
    world_tf->transform.rotation.w = 1.0;
    world_tf->transform.rotation.x = 0.0;
    world_tf->transform.rotation.y = 0.0;
    world_tf->transform.rotation.z = 0.0;

    tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    marker_pub = create_publisher<visualization_msgs::msg::MarkerArray>("selected_point", tf2_ros::StaticBroadcasterQoS());
    prepare_meshes();

    tf_pub_timer = create_wall_timer(std::chrono::milliseconds(get_parameter("tf_pub_interval_ms").as_int()), std::bind(&AlignerNode::timer_callback, this));

    start_sample(std::bind(&AlignerNode::startup_callback, this, std::placeholders::_1));
    manual_align_service = create_service<std_srvs::srv::Empty>("manual_align", std::bind(&AlignerNode::manual_align_service_callback, this, std::placeholders::_1, std::placeholders::_2));
    auto_align_service = create_service<radar_interface::srv::AutoAlign>("auto_align", std::bind(&AlignerNode::auto_align_service_callback, this, std::placeholders::_1, std::placeholders::_2));
}

// 启动点云采样
// @param callback: 采样完成后的回调函数
void AlignerNode::start_sample(std::function<void(std::shared_ptr<open3d::geometry::PointCloud>)> callback)
{
    if (pc_sample_context) {
        RCLCPP_ERROR(get_logger(), "pc_sample_context already created");
        return;
    }
    pc_sample_context = std::make_shared<PcSampleContext>(
        PcSampleContext {
            std::make_shared<open3d::geometry::PointCloud>(),
            static_cast<size_t>(get_parameter("init_sample").as_int()),
            create_subscription<sensor_msgs::msg::PointCloud2>(
                get_parameter("sample_lidar").as_string() + "/pc_raw", rclcpp::SystemDefaultsQoS(),
                std::bind(&AlignerNode::sample_sub_callback, this, std::placeholders::_1)),
            callback });
    pc_sample_context->recv_pc->points_.reserve(pc_sample_context->sample_size);
}

// 定时器回调函数：发布 TF 变换
void AlignerNode::timer_callback()
{
    if (world_tf && !world_tf->header.frame_id.empty() && !world_tf->child_frame_id.empty()) {
        world_tf->header.stamp = now();
        tf_broadcaster->sendTransform(*world_tf);
    }
    if (middle_tf && !middle_tf->header.frame_id.empty() && !middle_tf->child_frame_id.empty()) {
        middle_tf->header.stamp = now();
        tf_broadcaster->sendTransform(*middle_tf);
    }
}

// 自动配准服务回调函数
// @param request: 服务请求，包含最大对应距离和最大迭代次数
// @param response: 服务响应（未使用）
void AlignerNode::auto_align_service_callback(const std::shared_ptr<radar_interface::srv::AutoAlign_Request> request,
    std::shared_ptr<radar_interface::srv::AutoAlign_Response>)
{
    RCLCPP_INFO(get_logger(), "auto align service called");
    if (request) {
        set_parameter(rclcpp::Parameter("max_corr_dist", rclcpp::ParameterValue(request->max_corr_dist)));
        set_parameter(rclcpp::Parameter("max_iteration", rclcpp::ParameterValue(request->max_iteration)));
    }
    start_sample(std::bind(&AlignerNode::auto_align, this, std::placeholders::_1, get_middle_trans(), request->inherit_last));
}

// 手动配准服务回调函数
// @param request: 空请求
// @param response: 空响应
void AlignerNode::manual_align_service_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
{
    RCLCPP_INFO(get_logger(), "manual align service called");
    start_sample(std::bind(&AlignerNode::manual_align, this, std::placeholders::_1));
}

// 启动回调函数：根据参数决定是否手动配准或自动配准
// @param sample_pc: 采样点云
void AlignerNode::startup_callback(std::shared_ptr<open3d::geometry::PointCloud> sample_pc)
{
    if (get_parameter("startup_manual_align").as_bool())
        manual_align(sample_pc);
    auto_align(sample_pc, get_middle_trans());
}

// 获取中间变换矩阵
// @return 返回中间变换矩阵的仿射变换
Eigen::Isometry3d AlignerNode::get_middle_trans()
{
    if (middle_tf)
        return tf2::transformToEigen(*middle_tf).inverse();
    while (rclcpp::ok())
    {
        try {
            return tf2::transformToEigen(tf_buffer->lookupTransform(
                "middle",
                get_parameter("sample_lidar").as_string() + "_frame",
                tf2::TimePointZero, tf2::durationFromSec(1.0)));
        } catch (const tf2::TransformException& e) {
            RCLCPP_ERROR(get_logger(), "Waiting for Middle Trans. Exception: %s", e.what());
        }
    }
    throw std::runtime_error("Failed to get Middle trans.");
}

void AlignerNode::prepare_meshes()
{
    std::string align_model = get_parameter("align_model").as_string();
    std::filesystem::path meshes_path = std::filesystem::path(ament_index_cpp::get_package_share_directory("radar_bringup")) / "resource";

    RCLCPP_INFO(get_logger(), "align model: %s", align_model.c_str());
    RCLCPP_INFO(get_logger(), "meshes path: %s", meshes_path.c_str());
    if (align_model == "mesh") {
        align_using_mesh = true;
        const double mesh_scale = get_parameter("mesh_scale").as_double();
        mesh_ori = open3d::io::CreateMeshFromFile(meshes_path / get_parameter("mesh").as_string());
        mesh_ori->ComputeVertexNormals();
        mesh_ori->ComputeTriangleNormals();
        mesh_ori->Scale(mesh_scale, Eigen::Vector3d::Zero());
    } else if (align_model == "pointcloud") {
        align_using_mesh = false;
        pc_align = open3d::io::CreatePointCloudFromFile(meshes_path / get_parameter("pointcloud").as_string());
        // pc_align->EstimateNormals();
    } else {
        RCLCPP_WARN(get_logger(), "Unknown align_model: %s", align_model.c_str());
        align_using_mesh = true;
    }

    RCLCPP_INFO(get_logger(), "meshes prepared");
}

void AlignerNode::sample_sub_callback(const sensor_msgs::msg::PointCloud2 &msg)
{
    if (!pc_sample_context)
    {
        RCLCPP_ERROR(get_logger(), "pc_sample_context not created");
        return;
    }
    if (!radar_interface::check_lidar_msg(msg))
        return;

    const auto* field_x = radar_interface::find_lidar_field(msg, "x");
    const auto* field_y = radar_interface::find_lidar_field(msg, "y");
    const auto* field_z = radar_interface::find_lidar_field(msg, "z");
    const auto* field_intensity = radar_interface::find_lidar_field(msg, "intensity");
    if (!field_x || !field_y || !field_z)
        return;

    const uint8_t* raw = msg.data.data();
    for (size_t i = 0; i < msg.height * msg.width; ++i)
    {
        const uint8_t* point = raw + i * msg.point_step;
        float x, y, z, intensity = 0.0f;
        std::memcpy(&x, point + field_x->offset, sizeof(float));
        std::memcpy(&y, point + field_y->offset, sizeof(float));
        std::memcpy(&z, point + field_z->offset, sizeof(float));
        if (field_intensity && field_intensity->datatype == sensor_msgs::msg::PointField::FLOAT32 && field_intensity->offset + sizeof(float) <= msg.point_step) {
            std::memcpy(&intensity, point + field_intensity->offset, sizeof(float));
        }
        pc_sample_context->recv_pc->points_.emplace_back(x, y, z);
        pc_sample_context->recv_pc->colors_.emplace_back(open3d::visualization::GetGlobalColorMap()->GetColor(intensity / 150.0));
    }
    if (pc_sample_context->recv_pc->points_.size() >= pc_sample_context->sample_size)
    {
        pc_sample_context->recv_pc->points_.resize(pc_sample_context->sample_size);
        pc_sample_context->recv_pc->colors_.resize(pc_sample_context->sample_size);
        pc_sample_context->pc_sub.reset();
        pc_sample_context->next_step(pc_sample_context->recv_pc);
        pc_sample_context.reset();
        RCLCPP_INFO(get_logger(), "point sample complete");
    }
}
