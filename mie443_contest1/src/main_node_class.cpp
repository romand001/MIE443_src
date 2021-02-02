#include "main_node_class.hpp"
#include "map_class.hpp"

namespace mainSpace {

MainNodeClass::MainNodeClass(ros::NodeHandle& node_handle, ros::NodeHandle& private_node_handle)
   :nh_(node_handle),
    pnh_(private_node_handle)
{
    this->init();
}

void MainNodeClass::init() 
{
    angular_ = linear_ = 0.0;
    posX_ = posY_ = yaw_ = 0.0;
    ROS_INFO("hell");

    bumper_sub_ = nh_.subscribe("mobile_base/events/bumper", 10, &MainNodeClass::bumperCallback, this);
    laser_sub_ = nh_.subscribe("scan", 10, &MainNodeClass::laserCallback, this);
    map_sub_ = nh_.subscribe("map", 10, &MainNodeClass::mapCallback, this);
    odom_sub_ = nh_.subscribe("odom", 1, &MainNodeClass::odomCallback, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    timer_ = pnh_.createTimer(ros::Duration(0.1), &MainNodeClass::timerCallback, this);

    start_ = std::chrono::system_clock::now();

}

void MainNodeClass::bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
    // this code runs whenever new bumper collision data is published to the 
    //      mobile_base/events/bumper topic
    // updating the private bumper_ variable should be all that is needed here
    bumper_[msg->bumper] = msg->state;
}

void MainNodeClass::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // this code runs whenever new laser scan data is published to the scan topic
    // try to turn the scan array into data that the control code can easily use
    minLaserDist_ = std::numeric_limits<float>::infinity();
	int32_t nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    int32_t desiredNLasers = DEG2RAD(DESIRED_ANGLE) / msg->angle_increment;
    // ROS_INFO("Size of laser scan array: %iand size of offset: %i", nLasers, desiredNLasers);

    if (DEG2RAD(DESIRED_ANGLE) < msg->angle_max && -DEG2RAD(DESIRED_ANGLE) > msg->angle_min) {
        for (uint32_t laser_idx = nLasers/2 - desiredNLasers; laser_idx < nLasers/2 + desiredNLasers; laser_idx++) {
            minLaserDist_ = std::min(minLaserDist_, msg->ranges[laser_idx]);
            }
        }
            else {
                for (uint32_t laser_idx = 0; laser_idx < nLasers; laser_idx++) {
                    minLaserDist_ = std::min(minLaserDist_, msg->ranges[laser_idx]);
                    }
                }
}

void MainNodeClass::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    // this code runs whenever a new occupancy grid map is published to the map topic
    // suggestion: try to update a private variable indicating where to go next
    // nav_msgs::MapMetaData info = msg->info;
    mainSpace::Map map(msg->info.width, msg->info.height, msg->data);
    map.info();

}

void MainNodeClass::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	posX_ = msg->pose.pose.position.x;
    posY_ = msg->pose.pose.position.y;
    yaw_ = tf::getYaw(msg->pose.pose.orientation);
    
    // ROS_INFO("Position: (%f,%f) Orientation:%frad or%fdegrees.", posX_, posY_, yaw_, RAD2DEG(yaw_));
}

void MainNodeClass::timerCallback(const ros::TimerEvent &event)
{
    // this code runs at a regular time interval defined in the timer_ intialization
    // this is probably where we put the control code that moves the robot
    bool any_bumper_pressed = false;
    for (uint32_t b_idx = 0; b_idx < N_BUMPER; b_idx++) {
        any_bumper_pressed |= (bumper_[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
    }

    // Control logic after bumpers are being pressed.
    if (posX_<0.5 && yaw_<M_PI/12 && !any_bumper_pressed) {
        angular_ = 0.0;
        linear_ = 0.2;
    }
    else if (yaw_<M_PI/2 && posX_>0.5 && !any_bumper_pressed) {
        angular_ = M_PI/6;
        linear_ = 0.0;
    }
    else if (minLaserDist_>1. && !any_bumper_pressed) {
        linear_ = 0.1;
        if(yaw_<17/36*M_PI || posX_>0.6) {
            angular_ = M_PI/12.;
        }
        else if (yaw_<19/36*M_PI || posX_<0.4) {
            angular_ = -M_PI/12.;
        }
        else {
            angular_ = 0;
        }
    }
    else {
        angular_ = 0.0;
        linear_ = 0.0;
    } 

    vel_.angular.z = angular_;
    vel_.linear.x = linear_;
    vel_pub_.publish(vel_);

    //update the elapsed time
    secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start_).count();
}

} //namespace end