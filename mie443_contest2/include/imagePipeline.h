#pragma once

#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <opencv2/core.hpp>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <boxes.h>
#include<iostream>

#include"opencv2/calib3d.hpp"
#include"opencv2/highgui.hpp"
#include"opencv2/imgproc.hpp"
#include"opencv2/features2d.hpp"
#include"opencv2/xfeatures2d.hpp"

using namespace cv;
using namespace cv::xfeatures2d;

class ImagePipeline {
    private:
        cv::Mat img;
        bool isValid;
        image_transport::Subscriber sub;

        std::vector<cv::Mat> imgTags;
        std::vector<std::vector<KeyPoint>> tagKeypoints;
        std::vector<cv::Mat> tagDescriptors;
        std::vector<float> tagImgAreas;
        
    public:
        ImagePipeline(ros::NodeHandle& n);
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
        int setTagDescriptors();
        int getTemplateID(Boxes& boxes);
};
