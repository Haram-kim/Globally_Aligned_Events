#include <ros/ros.h>
#include <std_msgs/String.h>

// opencv
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

// messages
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "Event.h"
#include "IMU.h"
#include "Image.h"

// file manager
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>


int main(int argc, char **argv)
{

    ros::init(argc, argv, "event_publisher");

    ros::NodeHandle nh_private("~");

    std::string filename = "filename";
    std::string yaml = "yaml";
    bool loop = false;
    nh_private.param<std::string>("filename", filename, "");
    nh_private.param<std::string>("yaml", yaml, "");
    nh_private.param<bool>("loop", loop, "");

    ros::Publisher event_array_pub = nh_private.advertise<dvs_msgs::EventArray>("/events", 10);
    ros::Publisher imu_pub = nh_private.advertise<sensor_msgs::Imu>("/imu", 10);
    ros::Publisher image_pub = nh_private.advertise<sensor_msgs::Image>("/image", 10);
    ros::Publisher dvs_image_pub = nh_private.advertise<sensor_msgs::Image>("/renderer", 10);
    
    ros::Rate loop_rate(10);

    Event event(yaml);
    IMU imu(yaml);
    Image image(yaml);

    ROS_INFO("Reading event data");
    bool isEventEmpty = event.eventReader(filename);
    ROS_INFO("Reading imu data");
    bool isImuEmpty = imu.imuReader(filename);
    ROS_INFO("Reading image data");
    bool isImageEmpty = image.imageReader(filename);

    event.setEventPublisher(&event_array_pub);
    event.setDvsPublisher(&dvs_image_pub);
    imu.setIMUPublisher(&imu_pub);
    image.setImagePublisher(&image_pub);

    // initialize
    double time_begin = double(ros::Time::now().toSec());
    double time_curr, time;

    ROS_INFO("Start publishing data");
    while (ros::ok())
    {
        time = double(ros::Time::now().toSec());
        time_curr = time - time_begin;

        event.setCurrTime(time_curr);
        imu.setCurrTime(time_curr);
        image.setCurrTime(time_curr);

        event.setImage(image.last_image);

        event.publish();
        imu.publish();
        image.publish();

        ros::spinOnce();
        
        // termination condition
        if(!(event.isRunning || imu.isRunning || image.isRunning)){
            if(loop){
                time_begin = time;
                event.reset();
                image.reset();
                imu.reset();
            }
            else{
                break;
            }            
        }
    }
    ROS_INFO("Finish publishing data");
    
    return 0;
}