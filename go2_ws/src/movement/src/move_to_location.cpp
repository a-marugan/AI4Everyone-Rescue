
#include <unistd.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"
#include "interfaces/srv/position.hpp"

static const float dt = 0.5;     // Time step in seconds
static const float maxSpeed = 0.3;

class MoveToLocation : public rclcpp::Node
{
public:
    MoveToLocation() : Node("move_to_location")
    {
        // Create the sportmodestate subscriber
        stateSuber_ = this->create_subscription<unitree_go::msg::SportModeState>("sportmodestate", 10, std::bind(&MoveToLocation::stateCallback, this, std::placeholders::_1));
        // Create the reqest publisher
        reqPuber_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        // Create the position service 
        positionSrv_ = create_service<interfaces::srv::Position>("position", std::bind(&MoveToLocation::serviceCallback, this, std::placeholders::_1, std::placeholders::_2));
        // Create a timer when called
        timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(dt * 1000)), std::bind(&MoveToLocation::timerCallback, this));

        // Initialise variables
        startTimer_ = false;
        currentX_ = 0;
        currentY_ = 0;
        currentYaw_ = 0;
        targetX_ = currentX_;
        targetY_ = currentY_;
        targetYaw_ = currentYaw_;
    }

private:

    void serviceCallback(const std::shared_ptr<interfaces::srv::Position::Request> request, std::shared_ptr<interfaces::srv::Position::Response> response) 
    {
        targetX_ = request->x;
        targetY_ = request->y;
        targetYaw_ = std::atan2(targetY_ - currentY_, targetX_ - currentX_);

        startTimer_ = true;
        response->success = true;
    }

    void timerCallback() 
    {
        // If service not called, do nothing
        if (startTimer_ == false) return;

        // Calculate error
        float yawError = targetYaw_ - currentYaw_;
        while (yawError > M_PI) yawError -= 2 * M_PI;
        while (yawError < -M_PI) yawError += 2 * M_PI;
        float xError = targetX_ - currentX_;
        float yError = targetY_ - currentY_;
        vx_ = 0;
        vy_ = 0;
        vyaw_ = 0;

        if (abs(yawError) > 0.1) {
            // If yaw not reached
            if (yawError > 0) {
                vyaw_ = maxSpeed;
            }
            else {
                vyaw_ = -maxSpeed;
            }
            sportClient_.Move(reqMsg_, vx_, vy_, vyaw_);
            reqPuber_->publish(reqMsg_);
        }
        else if (abs(xError) > 0.1 || abs(yError) > 0.1) {
            // else if target not reached
            vx_ = maxSpeed;
            sportClient_.Move(reqMsg_, vx_, vy_, vyaw_);
            reqPuber_->publish(reqMsg_);
        } 
        else {
            sportClient_.StopMove(reqMsg_);
            reqPuber_->publish(reqMsg_);
        }
        
    }

    void stateCallback(unitree_go::msg::SportModeState::SharedPtr data)
    {
        // Get current real position of robot
        currentX_ = data->position[0];
        currentY_ = data->position[1];
        currentYaw_ = data->imu_state.rpy[2];
        RCLCPP_INFO(get_logger(), "Current position: %f, %f, %f", currentX_, currentY_, currentYaw_);
    }

    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr stateSuber_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr reqPuber_;
    rclcpp::Service<interfaces::srv::Position>::SharedPtr positionSrv_;
    rclcpp::TimerBase::SharedPtr timer_;

    unitree_api::msg::Request reqMsg_;
    SportClient sportClient_;

    float currentX_, currentY_, currentYaw_; 
    float targetX_, targetY_, targetYaw_;
    float vx_, vy_, vyaw_;
    
    bool startTimer_;

    float error_, prevError_, integral_, derivative_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveToLocation>());   //Run ROS2 node
    rclcpp::shutdown();
    return 0;
}
