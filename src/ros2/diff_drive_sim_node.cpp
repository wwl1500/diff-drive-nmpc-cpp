#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "common_types.hpp"
#include "diff_drive_model.hpp"

#include <cmath>
#include <memory>
#include <string>

class DiffDriveSimNode : public rclcpp::Node {
public:
    explicit DiffDriveSimNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("diff_drive_sim_node", options)
    {
        // 声明参数
        declare_parameter("dt", 0.05);
        declare_parameter("initial_x", 0.0);
        declare_parameter("initial_y", 0.0);
        declare_parameter("initial_theta", 0.0);
        declare_parameter("frame_id", "odom");
        declare_parameter("child_frame_id", "base_link");

        const double dt = get_parameter("dt").as_double();
        frame_id_ = get_parameter("frame_id").as_string();
        child_frame_id_ = get_parameter("child_frame_id").as_string();

        // 初始化模型和状态
        model_ = std::make_unique<DiffDriveModel>(dt);
        state_ = {
            get_parameter("initial_x").as_double(),
            get_parameter("initial_y").as_double(),
            get_parameter("initial_theta").as_double()
        };

        // 订阅 /cmd_vel
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                latest_cmd_vel_ = *msg;
            });

        // 发布 /odom
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS());

        // TF 广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 仿真定时器
        sim_timer_ = create_wall_timer(
            std::chrono::duration<double>(dt),
            std::bind(&DiffDriveSimNode::sim_timer_callback, this));

        RCLCPP_INFO(get_logger(), "仿真节点已启动 (dt=%.3f, 初始位姿=[%.2f, %.2f, %.2f])",
                     dt, state_.x, state_.y, state_.theta);
    }

private:
    void sim_timer_callback()
    {
        Control control{latest_cmd_vel_.linear.x, latest_cmd_vel_.angular.z};
        state_ = model_->step(state_, control);

        const auto stamp = now();

        // 发布 Odometry
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = stamp;
        odom_msg.header.frame_id = frame_id_;
        odom_msg.child_frame_id = child_frame_id_;
        odom_msg.pose.pose.position.x = state_.x;
        odom_msg.pose.pose.position.y = state_.y;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation.z = std::sin(state_.theta / 2.0);
        odom_msg.pose.pose.orientation.w = std::cos(state_.theta / 2.0);
        odom_msg.twist.twist.linear.x = control.v;
        odom_msg.twist.twist.angular.z = control.omega;
        odom_pub_->publish(odom_msg);

        // 广播 TF
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = frame_id_;
        tf.child_frame_id = child_frame_id_;
        tf.transform.translation.x = state_.x;
        tf.transform.translation.y = state_.y;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation = odom_msg.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr sim_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::unique_ptr<DiffDriveModel> model_;
    State state_{0.0, 0.0, 0.0};
    geometry_msgs::msg::Twist latest_cmd_vel_;
    std::string frame_id_;
    std::string child_frame_id_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffDriveSimNode>());
    rclcpp::shutdown();
    return 0;
}
