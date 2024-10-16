
#include <unistd.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"
#include "interfaces/srv/position.hpp"

static const float dt_ = 0.5;     // Time step in seconds

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
        timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(dt_ * 1000)), std::bind(&MoveToLocation::timerCallback, this));

        // Initialise variables
        startTimer_ = false;

        //posArray_.push_back(Position(-0.5, 0));
        // Position pos;
        // pos.x = 1.3;
        // pos.y = 0;
        // pos.yaw = 0;

        // moveToPosition(pos);
    };

private:

    void serviceCallback(const std::shared_ptr<interfaces::srv::Position::Request> request, std::shared_ptr<interfaces::srv::Position::Response> response) 
    {
        targetX_ = request->x + currentX_;
        targetY_ = request->y + currentY_;
        targetYaw_ = std::atan2(targetY_ - currentY_, targetX_ - currentX_);

        startTimer_ = true;
        response->success = true;
    }

    void timerCallback() 
    {
        // If service not called, do nothing
        if (startTimer_ == false) return;
        // If target reached, do nothing
        if (abs(targetX_ - currentX_) <= 0.1 && abs(targetY_ - currentY_) <= 0.1) return;

        float speed = 0.3;
        float yaw = std::atan2(targetY_ - currentY_, targetX_ - currentX_);
        vx_ = speed * std::cos(yaw);
        vy_ = speed * std::sin(yaw);
        vyaw_ = 0;

        // vx_ = computePIDcontrol(currentX_, targetX_, 0.1, 0, 0);
        // vy_ = computePIDcontrol(currentY_, targetY_, 0.1, 0, 0);
        //vyaw_ = computePIDcontrol(currentYaw_, targetYaw_, 0.1, 0, 0);

        sportClient_.Move(reqMsg_, vx_, vy_, vyaw_);
        reqPuber_->publish(reqMsg_);
    }

    // TODO: Implement PID control on position
    float computePIDcontrol(float input, float setpoint, float kp, float ki, float kd) 
    {
        error_ = setpoint - input;
        integral_ = integral_ + error_ * dt_;
        derivative_ = (error_ - prevError_) / dt_;
        float output = kp * error_ + ki * integral_ + kd * derivative_;
        prevError_ = error_;

        // temp speed cap
        float maxSpeed = 0.3;
        if (output > maxSpeed) output = maxSpeed;
        if (output < maxSpeed) output = -maxSpeed;

        return output;
    }

    // void moveToPosition(Position pos) {
    //     int i = 0;
    //     std::vector<PathPoint> path;
    //     // for (Position& pos : posArray_) {
    //     path.clear();
    //     if (i > 0) {
    //         x0_ = posArray_[i - 1].x;
    //         y0_ = posArray_[i - 1].y;
    //         yaw0_ = posArray_[i - 1].yaw;
    //     }

    //     double gradient;

    //     if (x0_ != pos.x) {
    //         gradient = (y0_ - pos.y) / (x0_ - pos.x);
    //     } else {
    //         gradient = 1; 
    //         //
    //     }

    //     double intercept = y0_ - gradient * x0_;
    //     double horizontal_diff = pos.x - x0_;
    //     // double vertical_diff = pos.y - y0_;

    //     for (int i = 0; i < 30; ++i) {
    //         PathPoint tempPathPoint;
    //         tempPathPoint.x = i * horizontal_diff + x0_;
    //         tempPathPoint.y = gradient * tempPathPoint.x + intercept;
    //         tempPathPoint.yaw = 0;
    //         tempPathPoint.vx = 0.2;
    //         tempPathPoint.vy = 0.2;
    //         tempPathPoint.vyaw = 0;
    //         path.push_back({tempPathPoint});
    //     }

    //     while (!(abs(x_ - pos.x) < 0.2 && abs(y_ - pos.y) < 0.2)) {
    //         sportClient_.TrajectoryFollow(reqMsg_, path);
    //         reqPuber_->publish(reqMsg_);
    //     }
    // }

    void stateCallback(unitree_go::msg::SportModeState::SharedPtr data)
    {
        // Get current position of robot
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
