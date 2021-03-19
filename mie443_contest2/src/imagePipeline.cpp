#include <imagePipeline.h>
#include <bits/stdc++.h>

using namespace cv;
using namespace cv::xfeatures2d;

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

#define NUMTAGS 16 // including blank
#define GOALAREA 300000


// darie's path: "/mnt/src/mie443_contest2/boxes_database"
// yaakob's path:"/home/yaakob613/catkin_ws/src/MIE443_src/mie443_contest2/boxes_database"
// Rohan's path: "/home/turtlebot/catkin_ws/src/MIE443_src/mie443_contest2/boxes_database"

const std::string tagPath = "/mnt/src/mie443_contest2/boxes_database"; // path to tags

std::vector<int> good_matches_vector;

ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        if(isValid) {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        isValid = true;
    } catch (cv_bridge::Exception& e) {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }    
}

int ImagePipeline::setTagDescriptors() {

    int minHessian = 400;
    Ptr<SURF> detector = SURF::create(minHessian);

    for (int i = 0; i < NUMTAGS - 1; i++) {

        std::vector<KeyPoint> keypoints;
        Mat descriptors;
        std::string imgPath = tagPath + "/tag_" + std::to_string(i+1) + ".jpg";
        Mat tagImg = imread(imgPath, IMREAD_GRAYSCALE);
        if (tagImg.empty()) {
            std::cout << "Could not open or find the image!\n";
            std::cout << imgPath << std::endl;
            return -1;
        }

        tagImgs.push_back(tagImg);

        detector->detectAndCompute(tagImg, noArray(), keypoints, descriptors);

        tagDescriptors.push_back(descriptors);

        tagKeypoints.push_back(keypoints);
    }

    std::vector<KeyPoint> keypoints;
    Mat descriptors;
    Mat tagImg = imread(tagPath + "/tag_blank.jpg", IMREAD_GRAYSCALE);

    if (tagImg.empty()) {
        std::cout << "Could not open or find the blank tag image!\n";
        return -1;
    }

    tagImgs.push_back(tagImg);

    detector->detectAndCompute(tagImg, noArray(), keypoints, descriptors);

    tagKeypoints.push_back(keypoints);

    tagDescriptors.push_back(descriptors);

    return 0;
}

/////////////////////////////////////////////////////////////////////
// Given three colinear points p, q, r, the function
// point q lies on line segment 'pr' 
bool onSegment(Point2f p, Point2f q, Point2f r) 
{ 
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) && 
        q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y)) 
    return true; 

    return false; 
} 

// To find orientation of ordered triplet (p, q, r). 
// The function returns following values 
// 0 --> p, q and r are colinear 
// 1 --> Clockwise 
// 2 --> Counterclockwise 
int orientation(Point2f p, Point2f q, Point2f r) 
{ 
    int val = (q.y - p.y) * (r.x - q.x) - 
            (q.x - p.x) * (r.y - q.y); 

    if (val == 0) return 0; // colinear 

    return (val > 0)? 1: 2; // clock or counterclock wise 
} 

// The main function that returns true if line segment 'p1q1' 
// and 'p2q2' intersect. 
bool doIntersect(Point2f p1, Point2f q1, Point2f p2, Point2f q2) 
{ 
    // Find the four orientations needed for general and 
    // special cases 
    int o1 = orientation(p1, q1, p2); 
    int o2 = orientation(p1, q1, q2); 
    int o3 = orientation(p2, q2, p1); 
    int o4 = orientation(p2, q2, q1); 

    // General case 
    if (o1 != o2 && o3 != o4) 
        return true; 

    // Special Cases 
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1 
    if (o1 == 0 && onSegment(p1, p2, q1)) return true; 

    // p1, q1 and q2 are colinear and q2 lies on segment p1q1 
    if (o2 == 0 && onSegment(p1, q2, q1)) return true; 

    // p2, q2 and p1 are colinear and p1 lies on segment p2q2 
    if (o3 == 0 && onSegment(p2, p1, q2)) return true; 

    // p2, q2 and q1 are colinear and q1 lies on segment p2q2 
    if (o4 == 0 && onSegment(p2, q1, q2)) return true; 

    return false; // Doesn't fall in any of the above cases 
} 

// C++ program to evaluate area of a polygon using
// shoelace formula

// (X[i], Y[i]) are coordinates of i'th point.
double polygonArea(std::vector<Point2f> vecPoint)
{
    int n = vecPoint.size();
	// Initialze area
	double area = 0.0;

	// Calculate value of shoelace formula
	int j = n - 1;
	for (int i = 0; i < n; i++)
	{
		area += (vecPoint[j].x + vecPoint[i].x) * (vecPoint[j].y - vecPoint[i].y);
		j = i; // j is previous vertex to i
	}

	// Return absolute value
	return abs(area / 2.0);
}


////////////////////////////////////////////////////////////////////

int ImagePipeline::getTemplateID(Boxes& boxes) {

    ros::spinOnce();

    std::vector<double> areaError;

    if(!isValid) {
        std::cout << "ERROR:INVALID IMAGE!" << std::endl;
        return -1;

    } else if (img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
        return -1;

    } else {

        int minHessian = 200;
        Ptr<SURF> detector = SURF::create(minHessian);

        std::vector<KeyPoint> keypoints_scene;
        Mat descriptors_scene;

        detector->detectAndCompute(img, noArray(), keypoints_scene, descriptors_scene);

        if(img.empty()) {
            std::cout << "Could not open or find the image!\n";
            return -1;
        }

        
        if (tagDescriptors.empty()) {
            std::cout << "Nothing in tagDescriptors" << std::endl;    //Print 2
        }
        /////////////////////////////////////////////////////


        for (int c = 0; c < NUMTAGS - 1; c++) {

            Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
            std::vector<std::vector<DMatch> >knn_matches;
            matcher->knnMatch(tagDescriptors[c], descriptors_scene, knn_matches, 2);

            if (knn_matches.empty()) {
                std::cout<<"Nothing in knnMatches"<<std::endl;    //Print 3
            }
            //--Filter matches using the Lowe's ratio test
            const float ratio_thresh = 0.75f;

            std::vector<DMatch> good_matches;
            for (size_t i = 0; i < knn_matches.size(); i++) {

                if (knn_matches[i][0].distance <ratio_thresh*knn_matches[i][1].distance) {
                    good_matches.push_back(knn_matches[i][0]);
                }
            }


            //std::cout << "check 2\n";

            ////////////////////////
            //--Draw matches
            Mat img_matches;
            drawMatches(tagImgs[c], tagKeypoints[c], img, 
                        keypoints_scene, good_matches, img_matches, Scalar::all(-1),
                        Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

            // /////////////////////////////

            // std::cout << "check 3\n";

            //--Localize the object
            std::vector<Point2f>obj;
            std::vector<Point2f>scene;
            for (size_t i = 0; i < good_matches.size(); i++) {
                //--Get the keypointsfrom the good matches
                obj.push_back(tagKeypoints[c][good_matches[i].queryIdx].pt);
                scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
            }

            //std::cout << "check 4\n";

            Mat H = findHomography(obj, scene, RANSAC);
            //--Get the corners from the image_1 ( the object to be "detected" )
            std::vector<Point2f> obj_corners(4);
            obj_corners[0] = Point2f(0, 0);
            obj_corners[1] = Point2f((float)tagImgs[c].cols, 0);
            obj_corners[2] = Point2f((float)tagImgs[c].cols, (float)tagImgs[c].rows);
            obj_corners[3] = Point2f(0, (float)tagImgs[c].rows);

            std::vector<Point2f> scene_corners(4);
            if (!H.empty()) {

                perspectiveTransform(obj_corners, scene_corners, H);

                //--Draw lines between the corners (the mapped object in the scene -image_2 )
                line(img_matches, scene_corners[0] + Point2f((float)tagImgs[c].cols, 0), scene_corners[1] + Point2f((float)tagImgs[c].cols, 0), Scalar(0, 255, 0), 4);
                line(img_matches, scene_corners[1] + Point2f((float)tagImgs[c].cols, 0), scene_corners[2] + Point2f((float)tagImgs[c].cols, 0), Scalar( 255, 0, 0), 4);
                line(img_matches, scene_corners[2] + Point2f((float)tagImgs[c].cols, 0), scene_corners[3] + Point2f((float)tagImgs[c].cols, 0), Scalar( 0, 0, 255), 4);
                line(img_matches, scene_corners[3] + Point2f((float)tagImgs[c].cols, 0), scene_corners[0] + Point2f((float)tagImgs[c].cols, 0), Scalar( 255, 0, 255), 4);
                //--Show detected matches
                //imshow("Good Matches & Object detection", img_matches);
                //waitKey(3000);

                ///////////////////////////////////////// 
                Point2f point1 = Point2f(scene_corners[0].x + tagImgs[c].cols, scene_corners[0].y);
                Point2f point2 = Point2f(scene_corners[1].x + tagImgs[c].cols, scene_corners[1].y);
                Point2f point3 = Point2f(scene_corners[2].x + tagImgs[c].cols, scene_corners[2].y);
                Point2f point4 = Point2f(scene_corners[3].x + tagImgs[c].cols, scene_corners[3].y);

                std::vector<Point2f> vecPoints;
                // The main function that returns true if line segment 'p1q1' 
                // and 'p2q2' intersect.
                //p1q1 and p2q2

                if (doIntersect(point1, point2, point3, point4)) {
                    vecPoints = {point1, point3, point2, point4};
                } 
                else if (doIntersect(point1, point3, point4, point2)) {
                    vecPoints = {point1, point2, point3, point4};
                } 
                else if (doIntersect(point1, point4, point2, point3)) {
                    vecPoints = {point1, point3, point4, point2};
                } 
                else {
                    //std::cout << "no intersection\n";
                    areaError.push_back(99999999.0);
                    continue;
                }
                        
                // pushback numerical value that represents match quality (area) into a vector 
                double area = polygonArea(vecPoints);
                areaError.push_back(std::abs(area - GOALAREA));
                //std::cout << "area for template " << c + 1 << " is " << area << std::endl;
            

                good_matches_vector.push_back(good_matches.size());

            }
            
            else {

                areaError.push_back(99999999.0);

                //std::cout << "homography matrix is empty\n";
                good_matches_vector.push_back(good_matches.size());
                //std::cout << "good matches: " << good_matches.size() << std::endl;
            }

        }


        int minIndex = (int) (std::min_element(areaError.begin(), areaError.end()) - areaError.begin());
        std::cout << "the tag closest to goal area is " << minIndex + 1 << std::endl;
        return minIndex + 1;



            // int maxIndex = std::max_element(good_matches_vector.begin(), good_matches_vector.end()) - good_matches_vector.begin();
            // std::cout<<maxIndex+1;
            // std::cout<<"\n";
            // return maxIndex + 1;




            /*        
            if(minIndex = NUMTAGS)
                {
                    std::string tagName = "tag_blank"; 
                }

            else()
                {
                    std::string tagName = "tag_" + std::to_string(maxIndex+1);
                }
            
            */

    }  
}
