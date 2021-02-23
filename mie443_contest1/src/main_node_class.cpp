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
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    vis_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    smoothed_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("smoothed_map", 10);

    timer_ = pnh_.createTimer(ros::Duration(0.1), &MainNodeClass::timerCallback, this);

    nh_.param<std::string>("map_frame", map_frame_, "/map");
    nh_.param<std::string>("base_link_frame", base_link_frame_, "/base_link");

    start_ = std::chrono::system_clock::now();

}

void MainNodeClass::publish_smoothed_map_static_transform() {
    geometry_msgs::TransformStamped static_transformStamped;
    static_transformStamped.header.frame_id = "/map";
    static_transformStamped.child_frame_id = "/smoothed_map";
    static_transformStamped.transform.translation.x = 0;
    static_transformStamped.transform.translation.y = 0;
    static_transformStamped.transform.translation.z = 0;
    static_transformStamped.transform.rotation.x = 1;
    static_transformStamped.transform.rotation.y = 0;
    static_transformStamped.transform.rotation.z = 0;
    static_transformStamped.transform.rotation.w = 0;
    static_broadcaster_.sendTransform(static_transformStamped);
}

void MainNodeClass::bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
    // this code runs whenever new bumper collision data is published to the 
    // mobile_base/events/bumper topic
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
//    map_.update(msg->data);
//
//    // get closest frontier
//    std::vector<std::pair<float, float>> frontierList = map_.closestFrontier(posX_, posY_);
//
//    plotFrontiers(frontierList);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start); 

    receivedMap_ = true;

    ROS_INFO("Map callback exec: %li ms", duration.count());
    

}

// this code runs at a regular time interval defined in the timer_ intialization
void MainNodeClass::timerCallback(const ros::TimerEvent &event)
{
    publish_smoothed_map_static_transform();

    // updating the x, y, and yaw of the robot
    tf::StampedTransform transform;
    try
    {
        listener_.lookupTransform(map_frame_, base_link_frame_,
                                ros::Time(0), transform);
        posX_ = transform.getOrigin().x();
        posY_ = transform.getOrigin().y();
        yaw_ = tf::getYaw(transform.getRotation());

        // std::cout << "robposX:" << posX_ << std::endl;
        // std::cout << "robposY:" << posY_ << std::endl;

        map_.plotSmoothedMap(smoothed_map_pub_);

    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("[ mapCallback() ] %s", ex.what());
    }

    // use index of bumpers, to see which bumper is pressed and then based on that determine 
    // rotation amount.  

    // 5cm per tile round up, add buffer, get rob radius to determine how many tiles in between the robot centre and the wall
    // account for yaw to determine coord. 18cm radius on turtlebot. use 4 tiles as spacing 

    // Check if any of the bumpers were pressed.
    bool any_bumper_pressed = false;
    for (uint32_t b_idx = 0; b_idx < N_BUMPER; b_idx++) {
        any_bumper_pressed |= (bumper_[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
    }

    if (!receivedMap_) ROS_INFO("waiting for map to be published");

    // normal operation
    if (!any_bumper_pressed && receivedMap_) {
        std::pair<uint32_t, uint32_t> map_coords = map_.posToMap(posX_, posY_);
        uint32_t mapX = map_coords.first;
        uint32_t mapY = map_coords.second;

        if (is_start_) {
            linear_ = 0.1;
            ROS_INFO("%f, %f, %f", posX_, posY_, yaw_);
            ROS_INFO("%d, %d, %f", map_coords.first, map_coords.second, yaw_);
            for (int i = 0; i < 20; i++) {
                ROS_INFO("%d", map_.get_map_value(mapX, mapY));
            }
            is_start_ = false;
        }

        if (minLaserDist_ < 0.5) {
            linear_ = 0;
            move_goal = -0.3;
            turn_goal = ((float)rand() / (float)RAND_MAX) * 2 * M_PI - M_PI;
        }
            // getting the shortest path
//        std::vector<std::pair<float, float>> pathPoints = map_.getPath(posX_, posY_);
//        plotPath(pathPoints);
//
//        ROS_INFO("Path length: %li", pathPoints.size());
//
//        if (pathPoints.size() >= 5) {
//            std::pair<float, float> target = pathPoints[4];
//            // calculate current yaw error and push it to the queue
//            float yawError = atan2( (target.second - posY_), (target.first - posX_) ) - yaw_;
//            yawErrorQueue_.push(yawError);
//
//            // maintain queue size of QUEUE_SIZE
//            static bool filledQueue = false;
//            static uint8_t queueCount = 0;
//            if (!filledQueue) { // if queue not full, count until it is
//                queueCount++;
//                if (queueCount >= QUEUE_SIZE) filledQueue = true;
//            }
//            else yawErrorQueue_.pop(); // if the queue is full, pop off the last number
//
//            float p = - KP * yawError; // proportional component
//            float d = - KD * (yawErrorQueue_.front() - yawErrorQueue_.back()); // derivative component
//            angular_ = p + d; // set angular velocity to sum of two correcting components
//
//            // set speed based on yaw error (higher speed for less error)
//            linear_ = 0.14 * (M_PI/2 - abs(yawError)) + 0.05;
//        }
//        else {
//            //linear_ = 0.03;
//            //angular_ = 0.15;
//        }

        
    
    
    }
    
        
    else {
        // ***invis wall is hit***
        ROS_INFO("WALL");
        float bumpLocX, bumpLocY, bumpAngle;
        float radius = 0.18;  //radius of the robot [m], distance from the center of robot to bumper sensor 
        
        if (bumper_[kobuki_msgs::BumperEvent::LEFT] == kobuki_msgs::BumperEvent::PRESSED) {
            bumpAngle = yaw_ + M_PI_2;
        }

        else if (bumper_[kobuki_msgs::BumperEvent::CENTER] == kobuki_msgs::BumperEvent::PRESSED) {
            bumpAngle = yaw_; 
        }

        else if (bumper_[kobuki_msgs::BumperEvent::RIGHT] == kobuki_msgs::BumperEvent::PRESSED) {
            bumpAngle = yaw_ + 3*M_PI_2; 
        } 
        
        bumpLocX = posX_ + radius*cos(bumpAngle);
        bumpLocY = posY_ + radius*sin(bumpAngle);

        std::pair <uint32_t, uint32_t> bumpCoords = map_.posToMap(bumpLocX, bumpLocY); 
        map_.invis.push_back(bumpCoords);
        linear_ = 0;
        move_goal = -0.3;
        turn_goal = ((float)rand() / (float)RAND_MAX) * 2 * M_PI - M_PI;
        
        // std::cout << "bumpcoordx:" << bumpCoords.first << std::endl;
        // std::cout << "bumpcoordy:" <<bumpCoords.second << std::endl;
        // std::cout << "bumplocX:" << bumpLocX << std::endl;
        // std::cout << "bumplocY:" <<bumpLocY << std::endl;

    }

    if (turn_goal != 0) {
        turn_goal -= angular_ * 0.1;
        if (turn_goal < 0.1 && turn_goal > -0.1) {
            ROS_INFO("Stop turn");
            turn_goal = 0;
            angular_ = 0;
            if (move_goal == 0)
                move_goal = 1;
        } else {
            angular_ = copysign(0.1, turn_goal);
            ROS_INFO("turn_goal %f", turn_goal);
        }
    }

    if (move_goal != 0) {
        move_goal -= linear_ * 0.1;
        if (move_goal < 0.1 && move_goal > -0.1) {
            ROS_INFO("Stop move");
            move_goal = 0;
            linear_ = 0;
            move_goal = 1;
        } else {
            linear_ = copysign(0.1, move_goal);
            ROS_INFO("move_goal %f", move_goal);
        }
    }

    vel_.angular.z = angular_;
    vel_.linear.x = linear_;
    vel_pub_.publish(vel_);

    //update the elapsed time
    secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start_).count();
}

void MainNodeClass::plotFrontiers(std::vector<std::pair<float, float>> frontierTiles)
{
    visualization_msgs::Marker redPoints;
    redPoints.header.frame_id = "/map";
    redPoints.header.stamp = ros::Time::now();
    redPoints.ns = "redPoints";
    redPoints.action = visualization_msgs::Marker::ADD;
    redPoints.pose.orientation.w = 1.0;
    redPoints.id = 0;
    redPoints.type = visualization_msgs::Marker::POINTS;
    redPoints.scale.x = 0.05;
    redPoints.scale.y = 0.05;
    redPoints.color.a = 1.0;
    redPoints.color.r = 1.0;

    for (auto frontier: frontierTiles) {
        geometry_msgs::Point p;
        p.x = frontier.first;
        p.y = frontier.second;
        //ROS_INFO("plotting point at %f, %f", p.x, p.y);
        p.z = 0.02;
        redPoints.points.push_back(p);
    }
    
    vis_pub_.publish(redPoints);
}

void MainNodeClass::plotPath(std::vector<std::pair<float, float>> pathTiles)
{
    visualization_msgs::Marker greenPoints;
    greenPoints.header.frame_id = "/map";
    greenPoints.header.stamp = ros::Time::now();
    greenPoints.ns = "greenPoints";
    greenPoints.action = visualization_msgs::Marker::ADD;
    greenPoints.pose.orientation.w = 1.0;
    greenPoints.id = 1;
    greenPoints.type = visualization_msgs::Marker::POINTS;
    greenPoints.scale.x = 0.05;
    greenPoints.scale.y = 0.05;
    greenPoints.color.a = 1.0;
    greenPoints.color.g = 1.0;

    for (auto pathPoint: pathTiles) {
        geometry_msgs::Point p;
        p.x = pathPoint.first;
        p.y = pathPoint.second;
        //ROS_INFO("plotting point at %f, %f", p.x, p.y);
        p.z = 0.02;
        greenPoints.points.push_back(p);
    }
    
    vis_pub_.publish(greenPoints);
}

} //namespace end