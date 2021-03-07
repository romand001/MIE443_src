#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <visualization_msgs/Marker.h>

ros::Publisher vis_pub;

const float goalDistance = 0.4; // distance in front of boxes where robot should go


void plotMarkers(std::vector<std::vector<float>> markers)
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

    for (auto marker: markers) {
        geometry_msgs::Point p;
        p.x = marker[0];
        p.y = marker[1];
        //ROS_INFO("plotting point at %f, %f", p.x, p.y);
        p.z = 0.02;
        points.points.push_back(p);
    }
    
    vis_pub.publish(points);
}

std::vector<std::vector<float>> getGoals(std::vector<std::vector<float>> boxCoords)
{
    std::vector<std::vector<float>> goals;
    for (auto coord: boxCoords) {
        float x = coord[0] + goalDistance * cos(coord[2]);
        float y = coord[1] + goalDistance * sin(coord[2]);
        float phi = coord[2];
        std::vector<float> goalCoord = {x, y, phi};
        goals.push_back(goalCoord);
    }
    return goals;
}

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    vis_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    // Initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
                  << boxes.coords[i][2] << std::endl;
    }
    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);
    // Execute strategy.

    std::vector<std::vector<float>> goals = getGoals(boxes.coords);

    while(ros::ok()) {
        ros::spinOnce();
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi

        plotMarkers(goals);

        imagePipeline.getTemplateID(boxes);
        ros::Duration(1).sleep();
    }
    return 0;
}
