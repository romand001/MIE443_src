#include <ros/ros.h>
#include <ros/package.h>
#include "explore.h"
//
// If you cannot find the sound play library try the following command.
// sudo apt install ros-kinetic-sound-play
#include <sound_play/sound_play.h>
#include <ros/console.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>

geometry_msgs::Pose2D current_pose;
//ros::Publisher pub_pose2d;
ros::Publisher vel_pub;

void emotionCallback(const std_msgs::Int32::ConstPtr& msg) 
{
    std::cout << "Emotion: " << msg->data << std::endl;
}

void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    // linear position
    current_pose.x = msg->pose.pose.position.x;
    current_pose.y = msg->pose.pose.position.y;
    
    // quaternion to RPY conversion
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    // angular position
    current_pose.theta = yaw;
    //pub_pose2d.publish(current_pose);
}

// void timerCallback(const ros::TimerEvent &event) 
// {
    
// }

void spinAround() 
{

    std::cout << "spinning around... ";

    double initialTime = ros::Time::now().toSec();

    do {
        
        geometry_msgs::Twist move;
        move.angular.z = 1.0;
        vel_pub.publish(move);
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    } while (ros::Time::now().toSec() - initialTime < 2 * M_PI);

    geometry_msgs::Twist stop;
    stop.angular.z = 0.0;
    vel_pub.publish(stop);

    std::cout << "done.\n";

}

int main(int argc, char** argv) 
{
    //
    // Setup ROS.
    ros::init(argc, argv, "contest3");
    ros::NodeHandle n;

    // subscribers and publishers
    ros::Subscriber emotion_sub = n.subscribe("/detected_emotion", 1, &emotionCallback);
    ros::Subscriber odom_sub = n.subscribe("odom", 1, odomCallback);
    vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    //motion checking thread
    // ros::Timer motionTimer = n.createTimer(ros::Duration(5.0), &timerCallback);

    //
    // Frontier exploration algorithm.
    explore::Explore explore;
    //
    // Class to handle sounds.
    sound_play::SoundClient sc;
    //
    // The code below shows how to play a sound.
    std::string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
    // sc.playWave(path_to_sounds + "sound.wav");
    
    double dt = 5.0; // # of seconds to wait before spinning
    double travel_thresh = 0.02; // min travel distance to see if robot moved

    auto prevPos = explore.getPosition();

    explore.stop();
    spinAround();
    explore.start();

    double prevTime = ros::Time::now().toSec();
    while(ros::ok()) {

        // code to run every dt seconds
        double curTime = ros::Time::now().toSec();
        if (curTime - prevTime > dt) {

            // get distance travelled
            auto curPos = explore.getPosition();
            double travelled = sqrt(pow(curPos.x - prevPos.x, 2) 
                                  + pow(curPos.y - prevPos.y, 2));

            prevPos = explore.getPosition();

            if (travelled < travel_thresh) {
                std::cout << "robot stuck? only travelled " << travelled << " in past " << dt << " seconds...\n";
                explore.stop();
                spinAround();
                explore.start();
            }

            prevTime = ros::Time::now().toSec();

        }


        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    return 0;
}