#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "common_types.hpp"
#include "controller_base.hpp"
#include "reference_generator.hpp"
#include "pure_pursuit_controller.hpp"

#ifdef HAS_NMPC
#include "nmpc_controller.hpp"
#endif

#include <cmath>
#include <memory>
#include <string>

class DiffDriveNmpcNode : public rclcpp::Node {
public:
    explicit DiffDriveNmpcNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("diff_drive_nmpc_node", options)
    {
        // 声明参数
        declare_parameter("controller_type", "nmpc");
        declare_parameter("trajectory_type", "circle");
        declare_parameter("dt", 0.05);
        declare_parameter("horizon_steps", 20);
        declare_parameter("frame_id", "odom");
        declare_parameter("child_frame_id", "base_link");
        declare_parameter("circle_radius", 2.0);
        declare_parameter("circle_period", 20.0);
        declare_parameter("eight_amplitude", 2.0);
        declare_parameter("eight_period", 20.0);
        declare_parameter("sine_forward_velocity", 0.3);
        declare_parameter("sine_amplitude", 1.0);
        declare_parameter("sine_omega", 0.5);

        // 读取参数
        controller_type_ = get_parameter("controller_type").as_string();
        dt_ = get_parameter("dt").as_double();
        horizon_steps_ = static_cast<int>(get_parameter("horizon_steps").as_int());
        frame_id_ = get_parameter("frame_id").as_string();
        child_frame_id_ = get_parameter("child_frame_id").as_string();

        // 解析轨迹类型
        const std::string traj_str = get_parameter("trajectory_type").as_string();
        if (traj_str == "circle") trajectory_type_ = TrajectoryType::Circle;
        else if (traj_str == "eight") trajectory_type_ = TrajectoryType::Eight;
        else if (traj_str == "sine") trajectory_type_ = TrajectoryType::Sine;
        else {
            RCLCPP_ERROR(get_logger(), "未知轨迹类型: %s，使用 circle", traj_str.c_str());
            trajectory_type_ = TrajectoryType::Circle;
        }

        // 构造参考轨迹生成器
        ReferenceGeneratorConfig ref_config;
        ref_config.circle_radius = get_parameter("circle_radius").as_double();
        ref_config.circle_period = get_parameter("circle_period").as_double();
        ref_config.eight_amplitude = get_parameter("eight_amplitude").as_double();
        ref_config.eight_period = get_parameter("eight_period").as_double();
        ref_config.sine_forward_velocity = get_parameter("sine_forward_velocity").as_double();
        ref_config.sine_amplitude = get_parameter("sine_amplitude").as_double();
        ref_config.sine_omega = get_parameter("sine_omega").as_double();
        reference_generator_ = ReferenceGenerator(ref_config);

        // 创建控制器
        create_controller();

        // 订阅 /odom
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(),
            std::bind(&DiffDriveNmpcNode::odom_callback, this, std::placeholders::_1));

        // 发布 /cmd_vel、/reference_path、/robot_path
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", rclcpp::SystemDefaultsQoS());
        ref_path_pub_ = create_publisher<nav_msgs::msg::Path>(
            "/reference_path", rclcpp::SystemDefaultsQoS());
        robot_path_pub_ = create_publisher<nav_msgs::msg::Path>(
            "/robot_path", rclcpp::SystemDefaultsQoS());

        // 初始化 robot path 消息
        robot_path_msg_.header.frame_id = frame_id_;

        // 控制定时器
        control_timer_ = create_wall_timer(
            std::chrono::duration<double>(dt_),
            std::bind(&DiffDriveNmpcNode::control_timer_callback, this));

        RCLCPP_INFO(get_logger(), "控制节点已启动 (controller=%s, trajectory=%s, dt=%.3f)",
                     controller_type_.c_str(), traj_str.c_str(), dt_);
    }

private:
    void create_controller()
    {
        if (controller_type_ == "nmpc") {
#ifdef HAS_NMPC
            NmpcControllerConfig nmpc_config;
            nmpc_config.dt = dt_;
            nmpc_config.horizon_steps = horizon_steps_;
            controller_ = std::make_unique<NmpcController>(nmpc_config);
#else
            RCLCPP_FATAL(get_logger(),
                "请求 NMPC 控制器但未编译 HAS_NMPC，请使用 ENABLE_NMPC=ON 重新构建");
            rclcpp::shutdown();
            return;
#endif
        } else if (controller_type_ == "pure_pursuit") {
            PurePursuitConfig pp_config;
            controller_ = std::make_unique<PurePursuitController>(pp_config);
        } else {
            RCLCPP_FATAL(get_logger(), "未知控制器类型: %s（支持 nmpc / pure_pursuit）",
                          controller_type_.c_str());
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(get_logger(), "控制器已创建: %s", controller_type_.c_str());
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_state_.x = msg->pose.pose.position.x;
        current_state_.y = msg->pose.pose.position.y;
        const auto& q = msg->pose.pose.orientation;
        current_state_.theta = std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        has_odom_ = true;
    }

    void control_timer_callback()
    {
        if (!has_odom_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "等待 /odom 里程计数据...");
            return;
        }

        const auto ref = reference_generator_.getReference(sim_time_, trajectory_type_);
        const auto ref_horizon = reference_generator_.getHorizonReference(
            sim_time_, dt_, horizon_steps_, trajectory_type_);

        const auto control = controller_->computeControl(current_state_, ref_horizon);

        publish_cmd_vel(control);
        publish_reference_path(sim_time_);
        publish_robot_path(current_state_);
        publish_diagnostics(control, sim_time_, ref, current_state_);

        sim_time_ += dt_;
    }

    void publish_cmd_vel(const Control& control)
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = control.v;
        msg.angular.z = control.omega;
        cmd_vel_pub_->publish(msg);
    }

    void publish_reference_path(double current_time)
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = now();
        path_msg.header.frame_id = frame_id_;

        const int num_points = 200;
        const double preview_duration = 20.0;
        for (int i = 0; i < num_points; ++i) {
            double t = current_time + (preview_duration / num_points) * i;
            auto ref = reference_generator_.getReference(t, trajectory_type_);

            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = ref.x;
            pose.pose.position.y = ref.y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.z = std::sin(ref.theta / 2.0);
            pose.pose.orientation.w = std::cos(ref.theta / 2.0);
            path_msg.poses.push_back(pose);
        }
        ref_path_pub_->publish(path_msg);
    }

    void publish_robot_path(const State& state)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now();
        pose.header.frame_id = frame_id_;
        pose.pose.position.x = state.x;
        pose.pose.position.y = state.y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.z = std::sin(state.theta / 2.0);
        pose.pose.orientation.w = std::cos(state.theta / 2.0);
        robot_path_msg_.poses.push_back(pose);
        robot_path_msg_.header.stamp = now();
        robot_path_pub_->publish(robot_path_msg_);
    }

    void publish_diagnostics(const Control& control, double time,
                             const Reference& ref, const State& state)
    {
        const double ex = state.x - ref.x;
        const double ey = state.y - ref.y;
        const double pos_err = std::sqrt(ex * ex + ey * ey);

#ifdef HAS_NMPC
        if (auto* nmpc = dynamic_cast<NmpcController*>(controller_.get())) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "t=%.2f pos_err=%.4f v=%.3f w=%.3f solve=%.2fms ok=%d",
                time, pos_err, control.v, control.omega,
                nmpc->lastSolveTimeMs(), nmpc->lastSolveOk());
            return;
        }
#endif
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "t=%.2f pos_err=%.4f v=%.3f w=%.3f",
            time, pos_err, control.v, control.omega);
    }

    // ROS2 基础设施
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ref_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr robot_path_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // 核心组件
    std::unique_ptr<ControllerBase> controller_;
    ReferenceGenerator reference_generator_;

    // 状态
    State current_state_{0.0, 0.0, 0.0};
    bool has_odom_ = false;
    double sim_time_ = 0.0;
    nav_msgs::msg::Path robot_path_msg_;

    // 参数
    std::string controller_type_;
    TrajectoryType trajectory_type_ = TrajectoryType::Circle;
    double dt_;
    int horizon_steps_;
    std::string frame_id_;
    std::string child_frame_id_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffDriveNmpcNode>());
    rclcpp::shutdown();
    return 0;
}
