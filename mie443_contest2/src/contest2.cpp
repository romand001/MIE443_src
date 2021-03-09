#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <visualization_msgs/Marker.h>

#include <iterator>
#include <algorithm>

ros::Publisher vis_pub;

const std::vector<float> phi_offsets = {0, -M_PI/4, M_PI/4};
const std::vector<float> goal_distances = {0.25, 0.35, 0.35};

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


//std::vector<std::vector<float>> getGoals(std::vector<std::vector<float>> boxCoords)
//{
//    std::vector<std::vector<float>> goals;
//    for (auto coord: boxCoords) {
//        float x = coord[0] + goalDistance * cos(coord[2]);
//        float y = coord[1] + goalDistance * sin(coord[2]);
//        float phi = coord[2];
//        std::vector<float> goalCoord = {x, y, phi};
//        goals.push_back(goalCoord);
//    }
//    return goals;
//}

std::vector<float> getGoal(float phi_offset, float goalDistance, std::vector<float> coord)
{
    float phi = coord[2] + phi_offset;
    float x = coord[0] + goalDistance * cos(phi);
    float y = coord[1] + goalDistance * sin(phi);
    std::vector<float> goalCoord = {x, y, phi + (float)M_PI};
    return goalCoord;
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

bool compareVec(const std::vector<float>& v1, const std::vector<float>& v2)
{
  return (v1[0] + v1[1] < v2[0] + v2[1]);
}

std::vector<std::vector<float>> bestPath(RobotPose robotPose, std::vector<std::vector<float>> goalCoords)
{
    ROS_INFO("starting path search");
    float bestLength = 99999.9;
    std::vector<std::vector<float>> bestPath;

    std::sort(goalCoords.begin(), goalCoords.end(), compareVec); // sort by comp function, permutations ordered lexicographically

    do {
        float length = pathLength(robotPose, goalCoords);

        if (length < bestLength) {
            bestLength = length;
            bestPath = goalCoords;
        }
        
    } while (std::next_permutation(goalCoords.begin(), goalCoords.end(), compareVec));

    return bestPath;
}

void visitBox(std::vector<float> boxCoords) {
    for (int i = 0; i < phi_offsets.size(); i++) {
        std::vector<float> coords = getGoal(phi_offsets[i], goal_distances[i], boxCoords);
        ROS_INFO("Moving to (%f, %f, %f), phi_offset %f, goal_distance %f", coords[0], coords[1], coords[2], phi_offsets[i], goal_distances[i]);
        plotMarkers({coords});
        bool success = Navigation::moveToGoal(coords[0], coords[1], coords[2]);
        if (success) {
            break;
        }
    }
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

//    std::vector<std::vector<float>> goals = getGoals(boxes.coords);
    std::vector<std::vector<float>> path = bestPath(robotPose, boxes.coords);

    plotMarkers(boxes.coords);
    plotPath(robotPose, path);

    int box_index = 0;
    while(ros::ok() && box_index < path.size()) {
        ros::spinOnce();
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi

        ROS_INFO("Moving to box index %d", box_index);
        visitBox(path[box_index]);
        imagePipeline.getTemplateID(boxes);
        ros::Duration(10).sleep();
        box_index++;
    }
    return 0;
}
