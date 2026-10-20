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

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.1; // This is to test the platform. Just move forward for now.
        cmd.linear.z = 0.0;
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