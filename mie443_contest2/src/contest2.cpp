#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/GetPlan.h>

#include <iterator>
#include <algorithm>
#include <ctime>
#include <map>
#include <ros/package.h>
#include <fstream>

ros::Publisher vis_pub;
std::map<int16_t, std::map<int16_t, float>> node_distances;

// pairs of <angle_offset, distance_offset> for the goals, relative to the box
const std::vector<std::pair<float, float>> goalOffsets = {
    std::make_pair(0.0, 0.35),
    std::make_pair(0.0, 0.30),
    std::make_pair(0.0, 0.40),

    std::make_pair(-M_PI/8, 0.35),
    std::make_pair(M_PI/8, 0.35),
    std::make_pair(-M_PI/8, 0.30),
    std::make_pair(M_PI/8, 0.30),
    std::make_pair(-M_PI/8, 0.40),
    std::make_pair(M_PI/8, 0.40),

    std::make_pair(-M_PI/6, 0.35),
    std::make_pair(M_PI/6, 0.35),
    std::make_pair(-M_PI/6, 0.30),
    std::make_pair(M_PI/6, 0.30),
    std::make_pair(-M_PI/6, 0.40),
    std::make_pair(M_PI/6, 0.40),

    std::make_pair(-M_PI/4, 0.35),
    std::make_pair(M_PI/4, 0.35),
    std::make_pair(-M_PI/4, 0.30),
    std::make_pair(M_PI/4, 0.30),
    std::make_pair(-M_PI/4, 0.40),
    std::make_pair(M_PI/4, 0.40),
    
};

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

bool checkGoal(ros::NodeHandle n, RobotPose robotPose, std::vector<float> goal)
{

    geometry_msgs::PoseStamped Start;
    Start.header.seq = 0;
    Start.header.stamp = ros::Time::now();
    Start.header.frame_id = "map";
    Start.pose.position.x = robotPose.x;
    Start.pose.position.y = robotPose.y;
    Start.pose.position.z = 0.0;
    Start.pose.orientation.x = 0.0;
    Start.pose.orientation.y = 0.0;
    Start.pose.orientation.w = 1.0;

    geometry_msgs::PoseStamped Goal;
    Goal.header.seq = 0;
    Goal.header.stamp = ros::Time::now();
    Goal.header.frame_id = "map";
    Goal.pose.position.x = goal[0];
    Goal.pose.position.y = goal[1];
    Goal.pose.position.z = 0.0;
    Goal.pose.orientation.x = 0.0;
    Goal.pose.orientation.y = 0.0;
    Goal.pose.orientation.w = 1.0;

    ros::ServiceClient check_path = n.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
    nav_msgs::GetPlan srv;
    srv.request.start = Start;
    srv.request.goal = Goal;
    srv.request.tolerance = 0;

    check_path.call(srv);
    return srv.response.plan.poses.size() == 0 ? false : true;
}

std::vector<float> getGoal(float phi_offset, float goalDistance, std::vector<float> coord)
{
    float phi = coord[2] + phi_offset;
    float x = coord[0] + goalDistance * cos(phi);
    float y = coord[1] + goalDistance * sin(phi);
    std::vector<float> goalCoord = {x, y, phi + (float)M_PI};
    return goalCoord;
}

inline float saveDistance(std::vector<float> coords1, std::vector<float> coords2, int16_t box1_num, int16_t box2_num) {
    float x_dist = coords1[0] - coords2[0];
    float y_dist = coords1[1] - coords2[1];
    float dist = x_dist * x_dist + y_dist * y_dist;
    node_distances[box1_num][box2_num] = dist;
    node_distances[box2_num][box1_num] = dist;
    ROS_INFO("Saving distances between %d and %d", box1_num, box2_num);
    return dist;
}

inline float getBoxesDistance(std::vector<float> coords1, std::vector<float> coords2) {
    int16_t box1_num = coords1[3];
    int16_t box2_num = coords2[3];
    auto it1 = node_distances.find(box1_num);
    if (it1 != node_distances.end()) {
        auto it2 = it1->second.find(box2_num);
        if (it2 != it1->second.end()) {
            return it2->second;
        } else {
            return saveDistance(coords1, coords2, box1_num, box2_num);
        }
    } else {
        return saveDistance(coords1, coords2, box1_num, box2_num);
    }
}


inline float getRobotBoxDistance(RobotPose robotPose, std::vector<float> coords) {
    int16_t box1_num = -1;
    int16_t box2_num = coords[3];
    auto it1 = node_distances.find(box1_num);
    if (it1 != node_distances.end()) {
        auto it2 = it1->second.find(box2_num);
        if (it2 != it1->second.end()) {
            return it2->second;
        } else {
            return saveDistance({robotPose.x, robotPose.y}, coords, box1_num, box2_num);
        }
    } else {
        return saveDistance({robotPose.x, robotPose.y}, coords, box1_num, box2_num);
    }
}


inline float pathLength(RobotPose robotPose, std::vector<std::vector<float>> path)
{
    int n = path.size();
    // x and y distances from robot to first goal
    float rob_x_dist = path[0][0] - robotPose.x;
    float rob_y_dist = path[0][1] - robotPose.y;

    // x and y distances from last goal to robot (has to return)
    float ret_x_dist = path[n-1][0] - robotPose.x;
    float ret_y_dist = path[n-1][1] - robotPose.y;

    // initialize length with starting and ending distances
    float length = rob_x_dist * rob_x_dist + rob_y_dist * rob_y_dist
                 + ret_x_dist * ret_x_dist + ret_y_dist * ret_y_dist;
    
    for (int i = 1; i < n; i++) {
        float x_dist = path[i][0] - path[i-1][0];
        float y_dist = path[i][1] - path[i-1][1];
        length += x_dist * x_dist + y_dist * y_dist; // sqrt is too slow
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
    //node_distances.empty();
    for (int i = 0; i < goalCoords.size(); i++) {
        goalCoords[i].push_back(i);
    }

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

bool visitBox(ros::NodeHandle n, RobotPose robotPose, std::vector<float> boxCoords) {

    std::vector<float> goalCoord;

    for (int i = 0; i < goalOffsets.size(); i++) {
        goalCoord = getGoal(goalOffsets[i].first, goalOffsets[i].second, boxCoords);
        // ROS_INFO("Checking goal (%f, %f, %f), phi_offset %f, goal_distance %f", 
        //          goalCoord[0], goalCoord[1], goalCoord[2], goalOffsets[i].first, goalOffsets[i].second);
        plotMarkers({goalCoord});

        if ( checkGoal(n, robotPose, goalCoord) ) {
            ROS_INFO("Moving to goal (%f, %f, %f)", goalCoord[0], goalCoord[1], goalCoord[2]);
            bool success = Navigation::moveToGoal(goalCoord[0], goalCoord[1], goalCoord[2]);
            if (!success) {
                ROS_INFO("Failed to reach goal :(");
            } else {
                return true;
            }
        }
        else ROS_INFO("Cannot reach this goal");
    }

    ROS_WARN("Could not find a suitable location to approach box");
    return false;

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

    if (imagePipeline.setTagDescriptors() == -1) {
        std::cout << "could not load an image\n";
        return -1;
    }

//    struct timespec start_wall, end_wall;
//    clock_gettime(CLOCK_MONOTONIC, &start_wall);
//    std::clock_t start = clock();
    std::vector<std::vector<float>> path = bestPath(robotPose, boxes.coords);
//    std::clock_t end = clock();
//    clock_gettime(CLOCK_MONOTONIC, &end_wall);
//    double elapsed_secs = double(end - start) / CLOCKS_PER_SEC;
//    double clock_elapsed = (end_wall.tv_sec - start_wall.tv_sec);
//    clock_elapsed += (end_wall.tv_nsec - start_wall.tv_nsec) / 1000000000.0;
//    ROS_INFO("Elapsed time: %fs CPU %fs clock", elapsed_secs, clock_elapsed);
//
    plotMarkers(boxes.coords);
    plotPath(robotPose, path);

    std::vector<int> tagsFound;

    std::string packagePath = ros::package::getPath("mie443_contest2");
    std::ofstream outputFile (packagePath + "/output_team_25.txt");

    int box_index = 0;
    while(ros::ok() && box_index < path.size()) {
        ros::spinOnce();

        ROS_INFO("Moving to box #%d", box_index + 1);

        // visit next box and check if successful
        if ( visitBox(n, robotPose, path[box_index]) ) {

            int tagID = imagePipeline.getTemplateID(boxes); // determine tag number of this box
            
            // check if tag has already been found
            std::string duplicateStr = "false";
            if ( std::count(tagsFound.begin(), tagsFound.end(), tagID) ) duplicateStr = "true";

            // try to open file, write into it if possible
            if (outputFile.is_open()) {

                if (tagID == -1) outputFile << "Tag: blank\t";
                else outputFile << "Tag: " << tagID << "\t";
                
                outputFile << "Location: (" << path[box_index][0] << ", " << path[box_index][1] << ", " << path[box_index][2] << ")\t";
                outputFile << "Duplicate: " << duplicateStr << std::endl;

            }
            else ROS_ERROR("could not open output file for writing!");

            tagsFound.push_back(tagID);


            //ros::Duration(10).sleep(); // REMEMBER TO REMOVE THIS!!!
        }

        box_index++;
        
    }
    return 0;
}
