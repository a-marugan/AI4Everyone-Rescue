#include <unistd.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/coordinate.hpp"

struct Point {
    Point() = default;
    Point(float px, float py) : x{ px }, y{ py } {}
    float x;
    float y;
};

class PathPlanning : public rclcpp::Node
{
public:
    PathPlanning() : Node("path_planning")
    {
        // Create arm movement client
        coordinateClient_ = create_client<interfaces::srv::Coordinate>("coordinate");
        while (!coordinateClient_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(get_logger(), "Position service not available, waiting...");
        }

        // Initialise path points
        pathPoints_.push_back(Point(1.0, 0.0));
        pathPoints_.push_back(Point(1.0, -1.0));
        pathPoints_.push_back(Point(0.0, -1.0));
        pathPoints_.push_back(Point(0.0, 0.0));

        // Call the first position service
        callCoordinateService(pathPoints_[pathPointsIndex_]);
    }

private:

    void callCoordinateService(const Point& point)
    {
        auto request = std::make_shared<interfaces::srv::Coordinate::Request>();
        request->x = point.x;
        request->y = point.y;

        auto futureResult = coordinateClient_->async_send_request(
            request,
            std::bind(&PathPlanning::handleCoordinateResponse, this, std::placeholders::_1)
        );

        RCLCPP_INFO(get_logger(), "Phase: %d", pathPointsIndex_);
    }

    void handleCoordinateResponse(rclcpp::Client<interfaces::srv::Coordinate>::SharedFuture future)
    {
        if (future.get()) {
            if (future.get()->success) {    

                RCLCPP_INFO(get_logger(), "Successfully reached Points.");
                pathPointsIndex_++;
                if (pathPointsIndex_ < pathPoints_.size()) {
                    // Move to point
                    callCoordinateService(pathPoints_[pathPointsIndex_]);
                }
                else {
                    RCLCPP_INFO(get_logger(), "Path completed!");
                    // Reset node
                    pathPointsIndex_ = 0;
                }
                
            } else {
                // Keep calling until the motion is finished
                RCLCPP_INFO(get_logger(), "Executing motion...");
                callCoordinateService(pathPoints_[pathPointsIndex_]);
            }   
        } else {
            RCLCPP_ERROR(get_logger(), "Service call failed!");
        }

    }

    rclcpp::Client<interfaces::srv::Coordinate>::SharedPtr coordinateClient_;
    uint8_t pathPointsIndex_;
    std::vector<Point> pathPoints_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanning>());
    rclcpp::shutdown();
    return 0;
}
