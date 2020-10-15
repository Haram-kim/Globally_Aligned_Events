#include <ros/ros.h>
#include <std_msgs/String.h>

// opencv
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

// eigen
#include <eigen_conversions/eigen_msg.h>

// messages
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "utils/System.h"

class ImageGrabber
{
public:
    ImageGrabber(System* sys) : system(sys) {}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    System* system;
};

class EventGrabber
{
public:
    EventGrabber(System* sys) : system(sys) {}

    void GrabEvent(const dvs_msgs::EventArrayConstPtr& msg);

    System* system;
};

class ImuGrabber
{
public:
    ImuGrabber(System* sys) : system(sys) {}

    void GrabImu(const sensor_msgs::ImuConstPtr& msg);

    System* system;
};


void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    static int lastseq = msg->header.seq-1;
    lastseq = msg->header.seq;
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    ImageData imageData;
    imageData.image = cv_ptr->image.clone();
    imageData.seq =  msg->header.seq;
    imageData.time_stamp = cv_ptr->header.stamp.toSec();

    system->PushImageData(imageData);
}


void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr& msg)
{
    static int lastseq = msg->header.seq - 1;
    lastseq = msg->header.seq;

    Eigen::Vector3d angular_velocity;
    tf::vectorMsgToEigen(msg->angular_velocity, angular_velocity);
 
    Eigen::Vector3d linear_acceleration;
    tf::vectorMsgToEigen(msg->linear_acceleration, linear_acceleration);

    ImuData imuData;
    imuData.angular_velocity = angular_velocity;
    imuData.linear_acceleration = linear_acceleration;
    imuData.time_stamp = msg->header.stamp.toSec();

    system->PushImuData(imuData);
}

void EventGrabber::GrabEvent(const dvs_msgs::EventArrayConstPtr& msg)
{
    static int lastseq = msg->header.seq -1;
    lastseq = msg->header.seq;

    EventData eventData;
    eventData.time_stamp = msg->header.stamp.toSec();
    eventData.event = msg->events;

    system->PushEventData(eventData);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "rotation_estimator");
    ros::start();

    ros::NodeHandle nodeHandler("~");
    std::string yaml;

    nodeHandler.param<std::string>("yaml", yaml, "");

    System Sys(yaml);

    ImageGrabber imageGrabber(&Sys);
    ImuGrabber imuGrabber(&Sys);
    EventGrabber eventGrabber(&Sys);

    
    ros::Subscriber image_sub = nodeHandler.subscribe("/dvs/image_raw", 10, &ImageGrabber::GrabImage, &imageGrabber);
    ros::Subscriber imu_sub = nodeHandler.subscribe("/dvs/imu", 100, &ImuGrabber::GrabImu, &imuGrabber);
    ros::Subscriber event_sub = nodeHandler.subscribe("/dvs/events", 10, &EventGrabber::GrabEvent, &eventGrabber);

    ros::spin();

    ros::shutdown();
    return 0;
}
