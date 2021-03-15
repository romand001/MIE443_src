#include <imagePipeline.h>

using namespace cv;
using namespace cv::xfeatures2d;

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

#define NUMTAGS 16 // including blank

std::vector<float> point1(2);
std::vector<float> point2(2);
std::vector<float> point3(2);
std::vector<float> point4(2);

float line_length1;
float line_length2;
double area;
std::vector<double> areaVec;


// darie's path: "/mnt/src/mie443_contest2/boxes_database"
// yaakob's path:"/home/yaakob613/catkin_ws/src/MIE443_src/mie443_contest2/boxes_database"
// Rohan's path: "/home/turtlebot/catkin_ws/src/MIE443_src/mie443_contest2/boxes_database"

const std::string tagPath = "/home/turtlebot/catkin_ws/src/MIE443_src/mie443_contest2/boxes_database"; // path to tags

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

    std::vector<std::vector<KeyPoint>> keypointsVec;
    std::vector<Mat> descriptorVec;
    std::vector<Mat> imgTagsVec;
    std::vector<float> localtagImgAreas;

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

       

        detector->detectAndCompute(tagImg, noArray(), keypoints, descriptors);

        descriptorVec.push_back(descriptors);

        imgTagsVec.push_back(tagImg);

        keypointsVec.push_back(keypoints);

        localtagImgAreas.push_back(tagImg.rows*tagImg.cols);
    }

    std::vector<KeyPoint> keypoints;
    Mat descriptors;
    Mat tagImg = imread(tagPath + "/tag_blank.jpg", IMREAD_GRAYSCALE);

    if (tagImg.empty()) {
        std::cout << "Could not open or find the blank tag image!\n";
        return -1;
    }

    imgTagsVec.push_back(tagImg);

    detector->detectAndCompute(tagImg, noArray(), keypoints, descriptors);

    keypointsVec.push_back(keypoints);

    descriptorVec.push_back(descriptors);

    localtagImgAreas.push_back(tagImg.rows*tagImg.cols);

    tagKeypoints = keypointsVec;

    tagDescriptors = descriptorVec;

    imgTags=imgTagsVec;

    tagImgAreas = localtagImgAreas;

    return 0;
}


int ImagePipeline::getTemplateID(Boxes& boxes) {
    int template_id = -1;
    if(!isValid) {
        std::cout << "ERROR:INVALID IMAGE!" << std::endl;
    } else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    } else {
        /***YOUR CODE HERE***/
        // Use: boxes.templates
        // const char*keys = 
        // "{ help h | | Print help message. }"
        // "{ input1 | box.png| Path to input image 1. }"
        // "{ input2 | box_in_scene.png| Path to input image 2. }";
        
        // int main( int argc, char*argv[] ){
        // CommandLineParserparser( argc, argv, keys );

        int minHessian = 400;
        Ptr<SURF> detector = SURF::create(minHessian);

        std::vector<KeyPoint> keypoints_scene;
        Mat descriptors_scene;

        detector->detectAndCompute(img, noArray(), keypoints_scene, descriptors_scene);

        if(img.empty()) {
            std::cout << "Could not open or find the image!\n";
            return-1;
        }

        std::cout << "everything good so far\n";
        
        if(tagDescriptors.empty()){

            std::cout<<"Nothing in tagDescriptors"<<std::endl;    //Print 2
        }
        /////////////////////////////////////////////////////


        for (int c = 0; c < NUMTAGS -1; c++) {

        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
        std::vector<std::vector<DMatch> >knn_matches;
        matcher->knnMatch(tagDescriptors[c], descriptors_scene, knn_matches, 2);

        if(knn_matches.empty()){

            std::cout<<"Nothing in knnMatches"<<std::endl;    //Print 3
        }
        //--Filter matches using the Lowe's ratio test
        const float ratio_thresh = 0.75f;


       std::cout << "check 1\n";

        //////////////////////////

        std::vector<DMatch>good_matches;
        for(size_t i = 0; i < knn_matches.size(); i++)
        {
            if(knn_matches[i][0].distance <ratio_thresh*knn_matches[i][1].distance)
            {
                good_matches.push_back(knn_matches[i][0]);
            }
        }


        std::cout << "check 2\n";

        ////////////////////////
        //--Draw matches
        Mat img_matches;
        drawMatches(imgTags[c], tagKeypoints[c], img, 
        keypoints_scene, good_matches, img_matches, Scalar::all(-1),
        Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );


        // /////////////////////////////

        std::cout << "check 3\n";

        //--Localize the object
        std::vector<Point2f>obj;
        std::vector<Point2f>scene;
        for(size_t i = 0; i < good_matches.size(); i++)
        {
        //--Get the keypointsfrom the good matches
        obj.push_back(tagKeypoints[c] [ good_matches[i].queryIdx].pt);
        scene.push_back( keypoints_scene[ good_matches[i].trainIdx].pt);

        }

        // /////////////////////////

        std::cout << "check 4\n";

        Mat H =findHomography( obj, scene, RANSAC );
        //--Get the corners from the image_1 ( the object to be "detected" )
        std::vector<Point2f>obj_corners(4);
        obj_corners[0] =Point2f(0, 0);
        obj_corners[1] =Point2f( (float)imgTags[c].cols, 0);
        obj_corners[2] =Point2f( (float)imgTags[c].cols, (float)imgTags[c].rows);
        obj_corners[3] =Point2f( 0, (float)imgTags[c].rows);

        std::vector<Point2f>scene_corners(4);
        if(!H.empty()){
        perspectiveTransform( obj_corners, scene_corners, H);


        // ////////////////////////////

        std::cout << "check 5\n";

        //--Draw lines between the corners (the mapped object in the scene -image_2 )
        line( img_matches, scene_corners[0] +Point2f((float)imgTags[c].cols, 0),scene_corners[1] +Point2f((float)imgTags[c].cols, 0), Scalar(0, 255, 0), 4);
        line( img_matches, scene_corners[1] +Point2f((float)imgTags[c].cols, 0),scene_corners[2] +Point2f((float)imgTags[c].cols, 0), Scalar( 0, 255, 0), 4);
        line( img_matches, scene_corners[2] +Point2f((float)imgTags[c].cols, 0),scene_corners[3] +Point2f((float)imgTags[c].cols, 0), Scalar( 0, 255, 0), 4);
        line( img_matches, scene_corners[3] +Point2f((float)imgTags[c].cols, 0),scene_corners[0] +Point2f((float)imgTags[c].cols, 0), Scalar( 0, 255, 0), 4);
        //--Show detected matches
        //imshow("Good Matches & Object detection", img_matches);
        
        std::cout<<"check6\n";
       point1[0] = scene_corners[0].x + (float)imgTags[c].cols;
       point1[1] = scene_corners[0].y;

       point2[0] = scene_corners[1].x + (float)imgTags[c].cols;
       point2[1] = scene_corners[1].y;

       point3[0] = scene_corners[2].x + (float)imgTags[c].cols;
       point3[1] = scene_corners[2].y;

       point4[0] = scene_corners[3].x + (float)imgTags[c].cols;
       point4[1] = scene_corners[3].y;

       line_length1 = pow((pow((point1[0]-point2[0]),2) + pow((point1[1]-point2[1]),2)),0.5);
       line_length2 = pow((pow((point2[0]-point3[0]),2) + pow((point2[1]-point3[1]),2)),0.5);

       // pushback numerical value that represents match quality (area) into a vector 
        
        
        
         
        area = line_length1*line_length2 ; // calc area of box 
        std::cout<<c;
        std::cout<<"\n";
        areaVec.push_back(area/tagImgAreas[c]); // vector containing the area values for each of the tags  
        std::cout<<area/tagImgAreas[c];
        std::cout<<"\n";
        //cv::imshow("view", img);
        //cv::waitKey(10);
       


       good_matches_vector.push_back(good_matches.size());
       std::cout<< good_matches_vector[c];
       std::cout<<"\n";
        }
        
        else
        {
        std::cout<<"Homography filter loop\n";
        areaVec.push_back(0);
        std::cout<<"\n";
        good_matches_vector.push_back(good_matches.size());
        std::cout<<good_matches.size();

        }

        }


        /* find max inside the vector we created and that will have the correct corresponding index tag
            int     

        int maxIndex = *std::max_element(areaVec.begin(), areaVec.end());  // find the element at which the max area occurs (i.e. best matching tag)
        
        if(maxIndex = NUMTAGS)
            {
                std::string tagName = "tag_blank"; 
            }

        else()
            {
                std::string tagName = "tag_" + std::to_string(maxIndex+1);
            }


        std::vector<pair<string,????>> template_id;  ////template_id is classified as an int above???

        template_id.push_back(std::make_pair(tagName,coords???));
                    
                    // vector containing pairs made of tags that have  been  found (tag  names) as  well  as  their 
                    // respective coordinate index in the order that they have been found for evaluation
            
        */ 


    }  
    template_id = 0;
    return template_id;
}
