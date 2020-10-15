#ifndef EVENTBUNDLER_H
#define EVENTBUNDLER_H
// ros
#include <ros/ros.h>

// opencv
#include <opencv2/core.hpp>

// messages
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

class EventBundler
{
public:
    EventBundler(std::string yaml);

    void GrabEvent(dvs_msgs::EventArrayConstPtr msg);
    void setEventPublisher(ros::Publisher *pub);
    void setCurrTime(double time);
    void Publish();

    std::vector< dvs_msgs::Event > eventData;
    std::vector<dvs_msgs::EventArrayConstPtr> vec_msg;
    dvs_msgs::EventArrayPtr msg;

    double delta_time;
    double next_send_time;
    int max_events;

    double time_curr;

    ros::Publisher* event_array_pub;
};

#endif // EVENTBUNDLER_H