// class for IMU
#ifndef __IMU_H__
#define __IMU_H__

// ros
#include <ros/ros.h>

// opencv
#include <opencv2/core.hpp>
// #include <cv_bridge/cv_bridge.h>

// messages
#include <sensor_msgs/Imu.h>

// file manager
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

// general
#include <string>

class IMU{
private:
    std::vector< sensor_msgs::Imu > imuData;
public:
    IMU(std::string yaml);
    ~IMU();
    //functions
    bool imuReader(const std::string &filename);
    void publish();
    
    void setCurrTime(double time);
    void setIMUPublisher(ros::Publisher *pub);
    void reset();

    ros::Publisher* imu_pub;
    
    //variables
    bool isEmpty;
    bool isRunning;
    int count;
    double time_curr;
};

#endif //__IMU_H__