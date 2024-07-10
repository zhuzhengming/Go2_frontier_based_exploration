#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"
#include <optional>

class VelocitySubscriber : public rclcpp::Node
{
public:
    VelocitySubscriber()
    : Node("velocity_subscriber")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&VelocitySubscriber::topic_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

        // Set a timer to publish at a higher frequency
        double publish_frequency = 0.1; // 10 times per second, adjust as necessary
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 * publish_frequency)),
            std::bind(&VelocitySubscriber::publish_request, this));
    }

private:
    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received velocity - Linear: x=%.2f, y=%.2f, z=%.2f; Angular: x=%.2f, y=%.2f, z=%.2f",
                    msg->linear.x, msg->linear.y, msg->linear.z,
                    msg->angular.x, msg->angular.y, msg->angular.z);

        // Store latest velocity command
        last_msg_ = *msg;
    }

    void publish_request()
    {
        if (!last_msg_) {
            return; 
        }

        const double threshold = 0.05;

        if (std::abs(last_msg_->linear.x) < threshold &&
            std::abs(last_msg_->linear.y) < threshold &&
            std::abs(last_msg_->angular.z) < threshold)
        {
            return;
        }
        

        SportClient sport_req;
        unitree_api::msg::Request req;

        // Transform the last received cmd_vel into request format
        sport_req.Move(req, last_msg_->linear.x, last_msg_->linear.y, last_msg_->angular.z);
        
        publisher_->publish(req);
    }

    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::optional<geometry_msgs::msg::Twist> last_msg_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocitySubscriber>());
    rclcpp::shutdown();
    return 0;
}
