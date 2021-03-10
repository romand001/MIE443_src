#include <imagePipeline.h>

using namespace cv;
using namespace cv::xfeatures2d;

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

#define NUMTAGS 16 // including blank

// darie's path: "/mnt/src/mie443_contest2/boxes_database"

const std::string tagPath = "/mnt/src/mie443_contest2/boxes_database"; // path to tags

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

    std::vector<Mat> descriptorVec;

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

    }

    std::vector<KeyPoint> keypoints;
    Mat descriptors;
    Mat tagImg = imread(tagPath + "/tag_blank.jpg", IMREAD_GRAYSCALE);

    if (tagImg.empty()) {
        std::cout << "Could not open or find the blank tag image!\n";
        return -1;
    }

    detector->detectAndCompute(tagImg, noArray(), keypoints, descriptors);

    descriptorVec.push_back(descriptors);

    tagDescriptors = descriptorVec;


    return 0;
}


int ImagePipeline::getTemplateID(Boxes& boxes) {
    int template_id = -1;
    if(!isValid) {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
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

        std::vector<KeyPoint> keypoints;
        Mat descriptors;

        detector->detectAndCompute(img, noArray(), keypoints, descriptors);

        if(img.empty()) {
            std::cout << "Could not open or find the image!\n";
            return-1;
        }

        std::cout << "everything good so far\n";

        // Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
        // std::vector<std::vector<DMatch> >knn_matches;
        // matcher->knnMatch( descriptors_object, descriptors_scene, knn_matches, 2);
        // //--Filter matches using the Lowe's ratio test
        // const float ratio_thresh = 0.75f;
       

        // ////////////////////////

        // std::vector<DMatch>good_matches;
        // for(size_t i = 0; i < knn_matches.size(); i++)
        // {
        //     if(knn_matches[i][0].distance <ratio_thresh*knn_matches[i][1].distance)
        //     {
        //         good_matches.push_back(knn_matches[i][0]);
        //     }
        // }



        // ////////////////////////
        // //--Draw matches
        // Mat img_matches;
        // drawMatches( img_object, keypoints_object, img_scene, 
        // keypoints_scene, good_matches, img_matches, Scalar::all(-1),
        // Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );


        // /////////////////////////////

        // //--Localize the object
        // std::vector<Point2f>obj;
        // std::vector<Point2f>scene;
        // for(size_t i = 0; i < good_matches.size(); i++)
        // {
        // //--Get the keypointsfrom the good matches
        // obj.push_back( keypoints_object[ good_matches[i].queryIdx].pt);
        // scene.push_back( keypoints_scene[ good_matches[i].trainIdx].pt);
        // }

        // /////////////////////////

        // Mat H =findHomography( obj, scene, RANSAC );
        // //--Get the corners from the image_1 ( the object to be "detected" )
        // std::vector<Point2f>obj_corners(4);
        // obj_corners[0] =Point2f(0, 0);
        // obj_corners[1] =Point2f( (float)img_object.cols, 0);
        // obj_corners[2] =Point2f( (float)img_object.cols, (float)img_object.rows);
        // obj_corners[3] =Point2f( 0, (float)img_object.rows);
        // std::vector<Point2f>scene_corners(4);
        // perspectiveTransform( obj_corners, scene_corners, H);


        // ////////////////////////////

        // //--Draw lines between the corners (the mapped object in the scene -image_2 )
        // line( img_matches, scene_corners[0] +Point2f((float)img_object.cols, 0),scene_corners[1] +Point2f((float)img_object.cols, 0), Scalar(0, 255, 0), 4);
        // line( img_matches, scene_corners[1] +Point2f((float)img_object.cols, 0),scene_corners[2] +Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4);
        // line( img_matches, scene_corners[2] +Point2f((float)img_object.cols, 0),scene_corners[3] +Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4);
        // line( img_matches, scene_corners[3] +Point2f((float)img_object.cols, 0),scene_corners[0] +Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4);
        // //--Show detected matches
        // imshow("Good Matches & Object detection", img_matches);
        // waitKey();
        return 0;
        //}





        cv::imshow("view", img);
        cv::waitKey(10);
    }  
    return template_id;
}
