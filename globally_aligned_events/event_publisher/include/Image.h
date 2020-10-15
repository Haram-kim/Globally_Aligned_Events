// class for Image
#ifndef __IMAGE_H__
#define __IMAGE_H__

// ros
#include <ros/ros.h>

// opencv
#include <opencv2/core.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// messages
#include <sensor_msgs/Image.h>

// file manager
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

// general
#include <string>

class Image{
private:
    std::vector< sensor_msgs::Image > imageData;
public:
    Image(std::string yaml);
    ~Image();
    //functions
    bool imageReader(const std::string &filename);
    void publish();
    
    void setCurrTime(double time);
    void setImagePublisher(ros::Publisher *pub);
    void reset();
    
    ros::Publisher* image_pub;
    
    //variables
    bool isEmpty;
    bool isRunning;
    
    int height, width;
    int count;
    double time_curr;

    cv::Mat last_image;
};

#endif //__IMAGE_H__