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


ros::Publisher vel_pub;
explore::Explore * global_explore;

int victims_found;

// move forward
void moveForward(double speed, double distance) 
{

    //std::cout << "moving forward... ";

    double duration = distance / speed;

    double initialTime = ros::Time::now().toSec();

    do {
        geometry_msgs::Twist move;
        move.linear.x = speed;
        vel_pub.publish(move);
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    } while (ros::Time::now().toSec() - initialTime < duration);

    geometry_msgs::Twist stop;
    stop.linear.x = 0.0;
    vel_pub.publish(stop);

    //std::cout << "done.\n";

}

// spin in circle
void rotate(double speed, double angle)
{
    //std::cout << "rotating on the spot... ";

    double initialTime = ros::Time::now().toSec();

    double duration = angle / speed;

    do {
        geometry_msgs::Twist move;
        move.angular.z = speed;
        vel_pub.publish(move);
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    } while (ros::Time::now().toSec() - initialTime < duration);

    geometry_msgs::Twist stop;
    stop.angular.z = 0.0;
    vel_pub.publish(stop);

    //std::cout << "done.\n";
}

// spin 360 degrees
void moveSmallCircle(double angSpeed, double linSpeed, double angle) 
{

    std::cout << "moving in circle... ";

    double initialTime = ros::Time::now().toSec();

    double duration = angle / angSpeed;

    do {
        geometry_msgs::Twist move;
        move.linear.x = linSpeed;
        move.angular.z = angSpeed;
        vel_pub.publish(move);
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    } while (ros::Time::now().toSec() - initialTime < duration);

    geometry_msgs::Twist stop;
    stop.linear.x = 0.0;
    stop.angular.z = 0.0;
    vel_pub.publish(stop);

    std::cout << "done.\n";

}


// example movement function, makes robot drive in a square
void emotEXMovement()
{
    double lineSpeed = 0.5; double turnSpeed = 1.0;
    std::cout << "moving in square shape...";
    moveForward(lineSpeed, 0.2); // move forward 0.2 m at lineSpeed m/s
    rotate(turnSpeed, M_PI_2); // turn pi/2 rad at turnSpeed rad/s
    moveForward(lineSpeed, 0.2); // move forward 0.2 m at lineSpeed m/s
    rotate(turnSpeed, M_PI_2); // turn pi/2 rad at turnSpeed rad/s
    moveForward(lineSpeed, 0.2); // move forward 0.2 m at lineSpeed m/s
    rotate(turnSpeed, M_PI_2); // turn pi/2 rad at turnSpeed rad/s
    moveForward(lineSpeed, 0.2); // move forward 0.2 m at lineSpeed m/s
    rotate(turnSpeed, M_PI_2); // turn pi/2 rad at turnSpeed rad/s
    std::cout << " done\n";
}

void emotDiscontentMovement() // agitated partial turns
{
    std::cout << "doing discontent motion...";
    double turnSpeed = 2.0;
    rotate(turnSpeed, M_PI/6);
    for (int i = 0; i < 2; i++) {
        rotate(-turnSpeed, -M_PI/3);
        rotate(turnSpeed, M_PI/3);
    }
    rotate(-turnSpeed, -M_PI/6);
    std::cout << " done\n";
}

void emotSurpriseMovement() // fast back, slow forwards
{
    std::cout << "doing surprise motion...";
    moveForward(-1, -0.2);
    moveForward(0.2, 0.2);
    std::cout << " done\n";
}

void emotSadMovement() // slow forwards
{
    std::cout << "doing sad motion...";
    moveForward(0.1, 0.1);
    std::cout << " done\n";
}

void emotRageMovement() // fast forwards, then forward/back jitter
{
    std::cout << "doing rage motion...";
    double lineSpeed = 0.5;
    moveForward(1, 0.2);
    for (int i = 0; i < 2; i++) {
        moveForward(-lineSpeed, -0.1);
        moveForward(lineSpeed, 0.1);
    }
    moveForward(-lineSpeed, -0.1);
    std::cout << " done\n";
}

void emotExcitedMovement() // 2 full spins
{
    std::cout << "doing excitement motion...";
    double turnSpeed = 4.0;
    rotate(turnSpeed, M_PI*2);
    rotate(-turnSpeed, -M_PI*2);
    std::cout << " done\n";
}

void emotProudMovement() // forward motion
{
    std::cout << "doing pride motion...";
    moveForward(0.25, 0.1);
    std::cout << " done\n";
}

void emotFearMovement() // foward and back motion, but turn when going back to show urgency
{
    std::cout << "doing fear motion...";
    double lineSpeed = 1; double turnSpeed = 2.0;
    for (int i = 0; i < 2; i++) {
        rotate(turnSpeed, M_PI);
        moveForward(lineSpeed, 0.2);
        rotate(turnSpeed, M_PI);
        moveForward(lineSpeed, 0.2);
    }
    std::cout << " done\n";
}

void secondaryDelay() {
    std::cout << "waiting for 3 seconds...";
    ros::Duration(3).sleep();
    std::cout << " done\n";
}

// gets called with result from emotion classifier (vote out of 10)
void emotionCallback(const std_msgs::Int32::ConstPtr& msg) 
{
    std::cout << "Emotion: " << (int)msg->data << std::endl;

    victims_found ++; // increment number of victims found so far

    // msg->data is a number from 0-6 corresponding to the emotion,
    // check on piazza for which emotion is which

    // implement behaviour for each emotion below
    // currently all emotions have an example movement
    global_explore->stop();
    switch((int)msg->data) {
        case 0:
            // behaviour for emotion 0: angry
            secondaryDelay();
            emotDiscontentMovement();

        break;
        case 1:
            // behaviour for emotion 1: disgust

            emotSurpriseMovement();

        break;
        case 2:
            // behaviour for emotion 2: fear
            secondaryDelay();
            emotSadMovement();

        break;
        case 3:
            // behaviour for emotion 3: happy

            emotRageMovement();

        break;
        case 4:
            // behaviour for emotion 4: sad
            secondaryDelay();
            emotExcitedMovement();

        break;
        case 5:
            // behaviour for emotion 5: surprise
            secondaryDelay();
            emotProudMovement();

        break;
        case 6:
            // behaviour for emotion 6: neutral
            secondaryDelay();
            emotFearMovement();

        break;

    }
    global_explore->start();
}


int main(int argc, char** argv) 
{
    //
    // Setup ROS.
    ros::init(argc, argv, "contest3");
    ros::NodeHandle n;

    // subscribers and publishers
    ros::Subscriber emotion_sub = n.subscribe("/detected_emotion", 1, &emotionCallback);
    vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    //motion checking thread
    // ros::Timer motionTimer = n.createTimer(ros::Duration(5.0), &timerCallback);

    //
    // Frontier exploration algorithm.
    explore::Explore explore;
    global_explore = &explore;
    //
    // Class to handle sounds.
    sound_play::SoundClient sc;
    //
    // The code below shows how to play a sound.
    std::string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
    // sc.playWave(path_to_sounds + "sound.wav");
    
    double dt = 10.0; // # of seconds to wait before spinning
    double travel_thresh = 0.05; // min travel distance to see if robot moved
    int spinCount = 0; // counter for number of consecutive spins

    victims_found = 0; // number of victims found so far

    auto prevPos = explore.getPose();

    explore.stop();
    moveSmallCircle(1.0, 0.05, 2*M_PI);
    explore.start();

    double prevTime = ros::Time::now().toSec();
    while(ros::ok()) {

        // code to run every dt seconds
        double curTime = ros::Time::now().toSec();
        if (curTime - prevTime > dt) {

            // get distance travelled, including rotation
            auto curPos = explore.getPose();
            double travelled = sqrt(pow(curPos.position.x - prevPos.position.x, 2) 
                                  + pow(curPos.position.y - prevPos.position.y, 2)
                                  + pow(curPos.orientation.z - prevPos.orientation.z, 2));

            prevPos = explore.getPose();

            if (travelled < travel_thresh) {

                // check if we are done
                if (victims_found == 7) {
                    std::cout << "All victims have been helped!\nexiting...";
                    return 0;
                }

                spinCount ++; // increment number of consecutive spins
                std::cout << "robot stuck? only travelled " << travelled << " in past " << dt << " seconds...\n";
                explore.stop();
                moveSmallCircle(1.0, 0.05, 2*M_PI);
                explore.start();
            }
            else spinCount = 0; // reset consecutive spins

            // if robot tried to get unstuck by spinning 3 times in a row
            if (spinCount >= 3) {
                spinCount = 0; // reset
                explore.stop();
                moveForward(0.25, 1.0); // move forward 1 metre
                if ( victims_found < 7 && explore.resetIfBlacklisted() ) {
                    std::cout << "emptied blacklist!\n";
                }
                explore.start();

            }

            prevTime = ros::Time::now().toSec();

        }


        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    return 0;
}
