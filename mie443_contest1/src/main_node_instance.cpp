#include "main_node_class.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_class_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    mainSpace::MainNodeClass node(nh, nh_private);

    ROS_INFO_STREAM("Main loop in thread:" << boost::this_thread::get_id());

    while(ros::ok() && node.secondsElapsed <= 900) {
        ros::spinOnce();
    }

    return 0;
}