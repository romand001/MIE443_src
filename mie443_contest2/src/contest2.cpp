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
    points.ns = "points_and_lines";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.1;
    points.scale.y = 0.1;
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

void plotPath(RobotPose robotPose, std::vector<std::vector<float>> path) 
{

    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "/map";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "points_and_lines";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.id = 1;
    line_strip.scale.x = 0.1;
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    geometry_msgs::Point p;
    p.x = robotPose.x;
    p.y = robotPose.y;
    p.z = 0.02;
    line_strip.points.push_back(p);

    for (auto point: path) {
        geometry_msgs::Point p;
        p.x = point[0];
        p.y = point[1];
        p.z = 0.02;
        line_strip.points.push_back(p);
    }

    vis_pub.publish(line_strip);

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

float pathLength(RobotPose robotPose, std::vector<std::vector<float>> path)
{
    int n = path.size();
    float rob_x_dist = path[0][0] - robotPose.x;
    float rob_y_dist = path[0][1] - robotPose.y;
    float length = rob_x_dist * rob_x_dist + rob_y_dist * rob_y_dist;
    
    for (int i = 1; i < n; i++) {
        float x_dist = path[i][0] - path[i-1][0];
        float y_dist = path[i][1] - path[i-1][1];

        length += x_dist * x_dist + y_dist + y_dist; // sqrt is too slow
    }

    return length;
}

std::vector<std::vector<float>> bestPath(RobotPose robotPose, std::vector<std::vector<float>> goalCoords)
{
    std::cout << "Starting path search" << std::endl;
    float bestLength = 99999.9;
    std::vector<std::vector<float>> bestPath;

    do {
        float length = pathLength(robotPose, goalCoords);
        if (length < bestLength) {
            bestLength = length;
            bestPath = goalCoords;
        }
    } while (std::next_permutation(goalCoords.begin(), goalCoords.end()));
    std::cout << "Finished path search" << std::endl;
    return bestPath;
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
    std::vector<std::vector<float>> path = bestPath(robotPose, goals);

    plotMarkers(goals);
    plotPath(robotPose, path);

    while(ros::ok()) {
        ros::spinOnce();
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi

        

        imagePipeline.getTemplateID(boxes);
        ros::Duration(1).sleep();
    }
    return 0;
}
