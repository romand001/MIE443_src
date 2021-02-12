#include "main_node_class.hpp"

namespace mainSpace {

MainNodeClass::MainNodeClass(ros::NodeHandle& node_handle, ros::NodeHandle& private_node_handle)
   :nh_(node_handle),
    pnh_(private_node_handle),
    map_()
{
    this->init();
}

void MainNodeClass::init() 
{
    angular_ = linear_ = 0.0;
    posX_ = posY_ = yaw_ = 0.0;

    bumper_sub_ = nh_.subscribe("mobile_base/events/bumper", 10, &MainNodeClass::bumperCallback, this);
    laser_sub_ = nh_.subscribe("scan", 10, &MainNodeClass::laserCallback, this);
    map_sub_ = nh_.subscribe("map", 10, &MainNodeClass::mapCallback, this);
    //odom_sub_ = nh_.subscribe("odom", 1, &MainNodeClass::odomCallback, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    vis_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    timer_ = pnh_.createTimer(ros::Duration(0.1), &MainNodeClass::timerCallback, this);

    nh_.param<std::string>("map_frame", map_frame_, "/map");
    nh_.param<std::string>("base_link_frame", base_link_frame_, "/base_link");

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

    // begin timing
    auto start = std::chrono::high_resolution_clock::now();

    // this code runs whenever a new occupancy grid map is published to the map topic
    
    // create new map object if received map size != current map size
    if (msg->info.width != map_.getWidth() || msg->info.height != map_.getHeight()) {
        ROS_INFO("received map of size: w=%u, h=%u, creating new map object", 
                                            msg->info.width, msg->info.height);
        map_ = Map(msg->info);
    }

    // update data array of map
    map_.update(msg->data);

    // get closest frontier
    std::vector<std::pair<float, float>> frontierList = map_.closestFrontier(posX_, posY_);

    plotMarkers(frontierList);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start); 

    ROS_INFO("Map callback exec: %li ms", duration.count());
    

}

// this code runs at a regular time interval defined in the timer_ intialization
void MainNodeClass::timerCallback(const ros::TimerEvent &event)
{
    
    // updating the x, y, and yaw of the robot
    tf::StampedTransform transform;
    try
    {
        listener_.lookupTransform(map_frame_, base_link_frame_,
                                ros::Time(0), transform);
        posX_ = transform.getOrigin().x();
        posY_ = transform.getOrigin().y();
        yaw_ = tf::getYaw(transform.getRotation());
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("[ mapCallback() ] %s", ex.what());
    }

    // getting the shortest path
    std::vector<std::pair<float, float>> pathPoints = map_.getPath(posX_, posY_);

    // converting path into velocity commands





    vel_.angular.z = angular_;
    vel_.linear.x = linear_;
    vel_pub_.publish(vel_);

    //update the elapsed time
    secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start_).count();
}

void MainNodeClass::plotMarkers(std::vector<std::pair<float, float>> frontierTiles)
{
    visualization_msgs::Marker points;
    points.header.frame_id = "/map";
    points.header.stamp = ros::Time::now();
    points.ns = "points";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.05;
    points.scale.y = 0.05;
    points.color.a = 1.0;
    points.color.r = 1.0;

    for (auto frontier: frontierTiles) {
        geometry_msgs::Point p;
        p.x = frontier.first;
        p.y = frontier.second;
        //ROS_INFO("plotting point at %f, %f", p.x, p.y);
        p.z = 0.02;
        points.points.push_back(p);
    }
    
    vis_pub_.publish(points);
}


} //namespace end