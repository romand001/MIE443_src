#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>

//#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <boost/thread.hpp>

#include <stdio.h>
#include <cmath>

#include "map_class.hpp"

#define N_BUMPER 3
#define RAD2DEG(rad) ((rad)* 180./M_PI)
#define DEG2RAD(deg) ((deg)* M_PI/180.)

// defines for velocity control system
#define QUEUE_SIZE 5 // first and last queue errors used for derivative term
#define KP 0.25 // proportional gain
#define KD 0.1 // derivative gain

#ifndef DESIRED_ANGLE
#define DESIRED_ANGLE 5
#endif

namespace mainSpace {

class MainNodeClass {

private:

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Timer timer_;

    ros::Subscriber bumper_sub_;
    ros::Subscriber laser_sub_;
    ros::Subscriber map_sub_;
    ros::Publisher vel_pub_;
    ros::Publisher vis_pub_;
    ros::Publisher smoothed_map_pub_;
    tf2_ros::StaticTransformBroadcaster static_broadcaster_;

    geometry_msgs::Twist vel_;

    float angular_, linear_;
    float posX_, posY_, yaw_;
    std::queue<float> yawErrorQueue_;

    Map map_;
    std::string base_link_frame_, map_frame_;
    tf::TransformListener listener_;

    bool receivedMap_ = false;

    uint8_t bumper_[3] = {kobuki_msgs::BumperEvent::RELEASED, 
                         kobuki_msgs::BumperEvent::RELEASED, 
                         kobuki_msgs::BumperEvent::RELEASED};

    float minLaserDist_;

    std::vector<std::pair<float, float>> path_;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start_;

public:

    uint64_t secondsElapsed = 0;

    MainNodeClass(ros::NodeHandle& node_handle, ros::NodeHandle& private_node_handle);
    ~MainNodeClass() = default;

    void init();

    void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    //void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    void timerCallback(const ros::TimerEvent& event);

    void plotFrontiers(std::vector<std::pair<float, float>> frontierTiles);

    void plotPath(std::vector<std::pair<float, float>> pathTiles);

    void publish_smoothed_map_static_transform();
};

}