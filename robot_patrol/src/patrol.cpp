#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <memory>

#include "rclcpp/rclcpp.hpp"
// import the LaserScan module from sensor_msgs interface
#include "sensor_msgs/msg/laser_scan.hpp"
// import the quality of service library
#include "rclcpp/qos.hpp"

class Patrol : public rclcpp::Node 
{
public:
    Patrol() : Node("patrol_node") {
        // create the subsciber
        RCLCPP_INFO(this->get_logger(),"NODE HAS STARTED!! ");
        // auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
        scan_sub_ = this -> create_subscription<sensor_msgs::msg::LaserScan>(
        "/fastbot_1/scan",
        rclcpp::SensorDataQoS(),
        std::bind(&Patrol::scan_callback, this, std::placeholders::_1));
    }  
private: 
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Only store the latest scan here.
    // Decisions happen in the timer callback
    last_scan_ = msg;
    RCLCPP_INFO_ONCE(this->get_logger(),
                "First scan received: %zu rays, angle_min=%.2f,increment=%.4f",
               msg->ranges.size(), msg->angle_min, msg->angle_increment);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Patrol>());
    rclcpp::shutdown();
    return 0;
}