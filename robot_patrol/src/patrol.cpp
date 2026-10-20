#include "rclcpp/qos.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <memory>

#include "rclcpp/rclcpp.hpp"
// import the sensor msgs interface
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
// import the quality of service library
#include "rclcpp/qos.hpp"

#include <cmath>
#include <limits>

using namespace std::chrono_literals;

class Patrol : public rclcpp::Node 
{
public:
    Patrol() : Node("patrol_node") {
        // create the subsciber
        // auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/fastbot_1/scan",
            rclcpp::SensorDataQoS(),
            std::bind(&Patrol::scan_callback, this, std::placeholders::_1));

        // create the publisher
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/fastbot_1/cmd_vel", 
            10);

        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&Patrol::timer_callback,this)); // 10 Hz

        RCLCPP_INFO(this->get_logger(),"NODE HAS STARTED!! ");
    }  
private: 
    // Callback 1: Only stores the latest scan
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    last_scan_ = msg;
    }

    // Callback 2 : Where the decision happens. Decides and publishes at 10Hz.
    // This is the Control Loop
    void timer_callback() {
        if (!last_scan_) {
            return; // if no scan yet, do absolutely nothing
        }

        // converting degree to radian
        // Radians = Degrees *(M_PI/180)
        const auto &scan = *last_scan_;
        const double half_pi = M_PI / 2.0;
        const double front_view = 20.0 * M_PI / 180.0; // +/- 20 deg
        const double obstacle_dist = 0.35;

        float min_dist_front = std::numeric_limits<float>::infinity(); // initialise to infinity. valid distance will be smaller than infinity.
        float best_dist = -1.0f;
        float best_angel = 0.0;

        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            // laser measurement = r = scan.ranges[i]
            float r = scan.ranges[i];

            // ignore invalid readings like inf, NaN, below the range_min and above the range_max
            if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) {
            continue;
            }

            // Converting a laser ray index into an angle
            // angle = angle_min + (angle of ray * angle of increment)
            double angle = scan.angle_min + (i * scan.angle_increment);
            angle = std::atan2(std::sin(angle),std::cos(angle)); // normalise the angle to the range |-180,180|

            // Use only the front 180 angle
            if (std::fabs(angle) > half_pi) { // |-90,90|
            continue;
            }

            // find the closest obstacle in front
            if (std::fabs(angle) <= front_view && r < min_dist_front) {
                min_dist_front = r;
            }

            // find the safest direction
            if (r > best_dist) {
                best_dist = r;
                best_angel = angle;
            }

        }
        geometry_msgs::msg::Twist cmd;
        if (min_dist_front < obstacle_dist) {
            // obstacle decision: slow ahead and turn toward direction}
            cmd.linear.x = 0.05; 
            cmd.angular.z = (best_angel >= 0.0) ? 0.5 : -0.5;  // + left , - right
        } else {
            cmd.linear.x = 0.1;
            cmd.angular.z = 0.0;
        }
            cmd_pub_->publish(cmd);
    }


    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Patrol>());
    rclcpp::shutdown();
    return 0;
}