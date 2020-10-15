// class for Event
#ifndef __EVENT_H__
#define __EVENT_H__

// ros
#include <ros/ros.h>

// opencv
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>

// messages
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

// file manager
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
// general
#include <string>

class Event{
private:
    std::vector< dvs_msgs::Event > eventData;

    //temporay image
    cv::Mat last_image;
    cv_bridge::CvImage cv_image;
    dvs_msgs::EventArrayPtr msg;
    
public:
    Event(std::string yaml);
    ~Event();

    //functions
    bool eventReader(const std::string &filename);
    void publish();
    void renderer();

    void setCurrTime(double time);
    void setEventPublisher(ros::Publisher *pub);
    void setDvsPublisher(ros::Publisher *pub);
    void setImage(const cv::Mat &image);
    void reset();

    ros::Publisher* event_array_pub;
    ros::Publisher* dvs_image_pub;

    //variables
    int height, width;
    bool isEmpty;
    bool isRunning;
    int count;
    uint32_t seq;

    double delta_time;
    double next_send_time;
    int max_events;

    double time_curr;
};

#endif //__EVENT_H__