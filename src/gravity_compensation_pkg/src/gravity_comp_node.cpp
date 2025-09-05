#include <memory>
#include <string>
#include <functional>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "gravity_compensation_pkg/common_types.h"

using namespace std::chrono_literals;

RotationMatrix QuaternionToRotationMatrix(double qx, double qy, double qz, double qw) {
    RotationMatrix R;
    R.m[0][0] = 1 - 2*qy*qy - 2*qz*qz; R.m[0][1] = 2*qx*qy - 2*qz*qw; R.m[0][2] = 2*qx*qz + 2*qy*qw;
    R.m[1][0] = 2*qx*qy + 2*qz*qw; R.m[1][1] = 1 - 2*qx*qx - 2*qz*qz; R.m[1][2] = 2*qy*qz - 2*qx*qw;
    R.m[2][0] = 2*qx*qz - 2*qy*qw; R.m[2][1] = 2*qy*qz + 2*qx*qw; R.m[2][2] = 1 - 2*qx*qx - 2*qy*qy;
    return R;
}

class GravityCompNode : public rclcpp::Node
{
public:
    GravityCompNode() : Node("gravity_comp_node")
    {
        mySander_ = std::make_shared<Tool>(3.0, Vector3d(0.0, 0.0, 0.15));
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        wrench_publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("~/gravity_wrench", 10);
        
        // 타이머 주기를 1초로 늘려서 로그 확인을 쉽게 합니다.
        timer_ = this->create_wall_timer(
            1s, std::bind(&GravityCompNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Gravity Compensation Node has started. Publishing to '~/gravity_wrench' and logging to console.");
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_->lookupTransform("base_link", "link_6", tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Could not get transform: %s", ex.what());
            return;
        } 

        // Debugging information
        auto q = t.transform.rotation;
        RCLCPP_INFO(this->get_logger(), "--- Debug Start ---");
        RCLCPP_INFO(this->get_logger(), "Quaternion (x,y,z,w): (%.3f, %.3f, %.3f, %.3f)", q.x, q.y, q.z, q.w);
        
        RotationMatrix R = QuaternionToRotationMatrix(q.x, q.y, q.z, q.w);
        RCLCPP_INFO(this->get_logger(), "Rotation Matrix R: %s", RotationMatrixToString(R).c_str());

        const double g = 9.80665;
        Vector3d G_world(0.0, 0.0, -mySander_->mass * g);
        Vector3d G_sensor = MatrixVectorMultiply(R, G_world);
        RCLCPP_INFO(this->get_logger(), "G_sensor (Action): Fx=%.3f, Fy=%.3f, Fz=%.3f", G_sensor.x, G_sensor.y, G_sensor.z);

        Vector3d F_gravity(-G_sensor.x, -G_sensor.y, -G_sensor.z);
        RCLCPP_INFO(this->get_logger(), "F_gravity (Reaction): Fx=%.3f, Fy=%.3f, Fz=%.3f", F_gravity.x, F_gravity.y, F_gravity.z);
        RCLCPP_INFO(this->get_logger(), "--- Debug End ---");

        Wrench gravity_wrench = calculateGravityWrench(R, *mySander_);
        auto wrench_msg = std::make_unique<geometry_msgs::msg::WrenchStamped>();
        wrench_msg->header.stamp = t.header.stamp;
        wrench_msg->header.frame_id = "link_6";
        wrench_msg->wrench.force.x = gravity_wrench.Fx; // 최종 발행값은 F_gravity.x와 동일
        wrench_msg->wrench.force.y = gravity_wrench.Fy;
        wrench_msg->wrench.force.z = gravity_wrench.Fz;
        wrench_msg->wrench.torque.x = gravity_wrench.Tx;
        wrench_msg->wrench.torque.y = gravity_wrench.Ty;
        wrench_msg->wrench.torque.z = gravity_wrench.Tz;
        wrench_publisher_->publish(std::move(wrench_msg));
    }

    std::shared_ptr<Tool> mySander_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GravityCompNode>());
    rclcpp::shutdown();
    return 0;
}