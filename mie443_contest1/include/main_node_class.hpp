#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>

#include <boost/thread.hpp>

#include <stdio.h>
#include <cmath>

#include <chrono>

namespace mainSpace {

class MainNodeClass {

private:

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Timer timer_;

    ros::Subscriber bumper_sub_;
    ros::Subscriber laser_sub_;
    ros::Publisher vel_pub_;

    geometry_msgs::Twist vel_;

    float angular_;
    float linear_;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start_;

public:

    uint64_t secondsElapsed = 0;

    MainNodeClass(ros::NodeHandle &node_handle, ros::NodeHandle& private_node_handle);
    ~MainNodeClass() = default;

    void init();

    void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    void timerCallback(const ros::TimerEvent& event);

};

}