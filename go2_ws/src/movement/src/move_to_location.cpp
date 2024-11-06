#include <unistd.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"
#include "interfaces/srv/coordinate.hpp"

static const float dt = 0.5;     // Time step in seconds
static const float maxSpeed = 0.5;

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
        coordinateSrv_ = create_service<interfaces::srv::Coordinate>("coordinate", std::bind(&MoveToLocation::serviceCallback, this, std::placeholders::_1, std::placeholders::_2));
        // Create a timer when called
        timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(dt * 1000)), std::bind(&MoveToLocation::timerCallback, this));

        // Initialise variables
        phase_ = 0;
        currentX_ = 0;
        currentY_ = 0;
        currentYaw_ = 0;
        targetX_ = currentX_;
        targetY_ = currentY_;
        targetYaw_ = currentYaw_;
    }

private:

    void serviceCallback(const std::shared_ptr<interfaces::srv::Coordinate::Request> request, std::shared_ptr<interfaces::srv::Coordinate::Response> response) 
    {
        // If robot still moving, ignore call
        if (phase_ != 0) {
            response->success = false;
            return;
        }

        targetX_ = request->x;
        targetY_ = request->y;
        if (targetX_ == previousX_ && targetY_ == previousY_) targetYaw_ = currentYaw_;
        else if (targetY_ == previousY_ && targetX_ > previousX_) targetYaw_ = 0; 
        else if (targetY_ == previousY_ && targetX_ < previousX_) targetYaw_ = M_PI; 
        else if (targetX_ == previousX_ && targetY_ > previousY_) targetYaw_ = M_PI / 2.0; 
        else if (targetX_ == previousX_ && targetY_ < previousY_) targetYaw_ = -M_PI / 2.0; 
        else targetYaw_ = std::atan2(targetY_ - previousY_, targetX_ - previousX_);

        while (targetYaw_ > M_PI) targetYaw_ -= 2 * M_PI;
        while (targetYaw_ <= M_PI) targetYaw_ += 2 * M_PI;

        // Initialise previousDistanceError_
        previousDistanceError_ = 0;
        
        // Receive new instructions successfully
        phase_ = 1;
        response->success = true;

    }

    void timerCallback() 
    {
        if (phase_ == 1) turn();

        if (phase_ == 2) forward();

        if (phase_ == 3) {
            sportClient_.StopMove(reqMsg_);
            reqPuber_->publish(reqMsg_);
            previousX_ = targetX_;
            previousY_ = targetY_;
            previousYaw_ = targetYaw_;
            phase_ = 0;
        }
    }

    void turn() {
        float vyaw;
        // Calculate the angle error between the current and target yaw
        float angleError = targetYaw_ - currentYaw_;

        // Normalize the angle error to be between -pi and pi
        while (angleError > M_PI) angleError -= 2 * M_PI;
        while (angleError <= -M_PI) angleError += 2 * M_PI;

        // If the angle error is within the threshold, stop turning
        if (abs(angleError) < 0.01745) { // 0.01745 rad = 1 deg threshold
            // Stop rotation
            vyaw = 0;
            phase_ = 2; // Move to the next phase (moving forward)
        } else {
            // Set rotation speed proportional to angle error (basic P controller)
            float kP = 1.0;
            vyaw = kP * angleError;
            // Cap at max speed
            if (vyaw > maxSpeed) vyaw = maxSpeed;
            if (vyaw < -maxSpeed) vyaw = -maxSpeed;
        }

        sportClient_.Move(reqMsg_, 0, 0, vyaw);
        reqPuber_->publish(reqMsg_);
    }

    void forward() {
        float vx, vyaw;

        // Calculate the Euclidean distance to the target
        float distanceError = sqrt(pow(targetX_ - currentX_, 2) + pow(targetY_ - currentY_, 2));

        // Calculate the direction of the error
        if (previousDistanceError_ > 0 && distanceError > previousDistanceError_) distanceError *= -1;
        if (previousDistanceError_ < 0 && -distanceError < previousDistanceError_) distanceError *= -1;
        previousDistanceError_ = distanceError;

        // If the robot is close enough (0.1 m) to the target, stop moving
        if (distanceError < 0.1) {
            vx = 0;
            vyaw = 0;
            phase_ = 3; // Move to the stopping phase
            RCLCPP_INFO(get_logger(), "Target reached: (%f, %f)", currentX_, currentY_);
        } else {
            // Set forward velocity (considering a simple P controller)
            float kP = 0.4;
            vx = kP * distanceError;

            // Cap at max speed
            if (vx > maxSpeed) vx = maxSpeed;
            if (vx < -maxSpeed) vx = -maxSpeed;

            vyaw = 0; // Maintain current heading

            // Compensate for drift: Adjust the heading if necessary
            float headingError = targetYaw_ - currentYaw_;

            // Normalize the heading error to be between -π and π
            while (headingError > M_PI) headingError -= 2 * M_PI;
            while (headingError <= -M_PI) headingError += 2 * M_PI;

            // Correct the path if the heading error is significant (0.05 rad)
            if (abs(headingError) > 0.05) {
                vyaw = (headingError > 0) ? 0.1 : -0.1; // Apply a small rotation to correct heading
            }

            sportClient_.Move(reqMsg_, vx, 0, vyaw);
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
    rclcpp::Service<interfaces::srv::Coordinate>::SharedPtr coordinateSrv_;
    rclcpp::TimerBase::SharedPtr timer_;

    unitree_api::msg::Request reqMsg_;
    SportClient sportClient_;

    float currentX_, currentY_, currentYaw_; 
    float previousX_, previousY_, previousYaw_; 
    float targetX_, targetY_, targetYaw_;

    float previousDistanceError_;
    
    int phase_;     // 0: default do nothing, 1: start moving
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveToLocation>());
    rclcpp::shutdown();
    return 0;
}
