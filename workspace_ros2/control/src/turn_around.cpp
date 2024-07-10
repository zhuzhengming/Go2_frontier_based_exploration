#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

class TurnPublisher : public rclcpp::Node
{
public:
    int16_t Turn_time = 10;
    int16_t count;
    TurnPublisher()
    : Node("TurnPublisher")
    {
        publisher_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

        // Set a timer to publish at a higher frequency
        double publish_frequency = 0.1; // 10 times per second, adjust as necessary
        this->count = (int16_t)(Turn_time / publish_frequency);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 * publish_frequency)),
            std::bind(&TurnPublisher::publish_request, this));
        
    }

private:

    void publish_request()
    {   
        if (this->count<=0)
        {
            rclcpp::shutdown();
        }
        
        
        SportClient sport_req;
        unitree_api::msg::Request req;

        // Transform the last received cmd_vel into request format
        sport_req.Move(req, 0.0, 0.0, 0.5);
        
        publisher_->publish(req);
        this->count--;
    }

    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurnPublisher>());
    rclcpp::shutdown();
    return 0;
}
