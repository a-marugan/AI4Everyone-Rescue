#include <termios.h>
#include <unistd.h>
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

static const float maxSpeed = 0.3;

class KeyboardControl : public rclcpp::Node
{
public:
    KeyboardControl() : Node("keyboard_control")
    {
        // Configure terminal for non-canonical mode
        tcgetattr(STDIN_FILENO, &oldt_);
        newt_ = oldt_;
        newt_.c_lflag &= ~(ICANON | ECHO); // Disable canonical mode and echo
        tcsetattr(STDIN_FILENO, TCSANOW, &newt_);

        // Create the sportmodestate subscriber
        stateSuber_ = this->create_subscription<unitree_go::msg::SportModeState>("sportmodestate", 10, std::bind(&KeyboardControl::stateCallback, this, std::placeholders::_1));
        // Create the reqest publisher
        reqPuber_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        // Create a timer when called
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&KeyboardControl::timerCallback, this));

    }

    ~KeyboardControl() {
        // Restore terminal settings
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt_);
    }

private:

    void timerCallback() 
    {
        float vx = 0;
        float vy = 0;
        float vyaw = 0;

        // Non-blocking key press check
        int ch = getKeyPress();

        if (ch == 'w') vx = maxSpeed;
        if (ch == 'a') vy = maxSpeed;
        if (ch == 's') vx = -maxSpeed;
        if (ch == 'd') vy = -maxSpeed;
        if (ch == 'n') vyaw = maxSpeed;
        if (ch == 'm') vyaw = -maxSpeed;
        if (ch == 'w' || ch == 'a' || ch == 's' || ch == 'd' || ch == 'n' || ch == 'm') {
            RCLCPP_INFO(this->get_logger(), "Key pressed: %c", ch);
            sportClient_.Move(reqMsg_, vx, vy, vyaw);
            reqPuber_->publish(reqMsg_);
        }
        else {
            sportClient_.StopMove(reqMsg_);
            reqPuber_->publish(reqMsg_);
        }
            
    }

    int getKeyPress() {
        int ch = -1;
        struct timeval tv = {0, 0};
        fd_set readfds;

        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);

        if (select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &tv) > 0) {
            ch = getchar();
        }
        return ch;
    }

    void stateCallback(unitree_go::msg::SportModeState::SharedPtr data)
    {
        // Get current real position of robot
        currentX_ = data->position[0];
        currentY_ = data->position[1];
        currentYaw_ = data->imu_state.rpy[2];
        RCLCPP_INFO(get_logger(), "Current position: %f, %f, %f", currentX_, currentY_, currentYaw_);
    }


    struct termios oldt_, newt_;

    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr stateSuber_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr reqPuber_;
    rclcpp::TimerBase::SharedPtr timer_;

    unitree_api::msg::Request reqMsg_;
    SportClient sportClient_;

    float currentX_, currentY_, currentYaw_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardControl>());
    rclcpp::shutdown();
    return 0;
}
