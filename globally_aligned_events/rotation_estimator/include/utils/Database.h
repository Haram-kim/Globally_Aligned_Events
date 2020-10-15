#ifndef DATABASE_H
#define DATABASE_H

#include <eigen3/Eigen/Core>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

#include <ros/ros.h>
// messages
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// Define data structure
struct ImuData{
    double time_stamp;
    Eigen::Vector3d linear_acceleration;
    Eigen::Vector3d angular_velocity;
};

struct ImageData{
    double time_stamp;
    cv::Mat image;
    uint32_t seq;
};

struct EventData{
    double time_stamp;
    std::vector<dvs_msgs::Event> event;
};

struct CameraParam{
    float fx, fy; // focus
    float cx, cy; // center
    float rd1, rd2; // radial distortion
};

class EventBundle{
public:
    EventBundle();
    EventBundle(const EventBundle& temp);
    ~EventBundle();

    void Clear();
    void Copy(const EventBundle &ref);
    void SetCoord();
    void SetXY();
    void Projection(Eigen::Matrix3f K);
    void InverseProjection(Eigen::Matrix3f K);
    void DiscriminateInner(int width, int height, int map_sampling_rate = 1);
    void Append(const EventBundle &ref);
    
    void erase(size_t iter);
    
    // coord: 2 by N uv coord or 3 by N xyz coord
    Eigen::MatrixXf coord;
    Eigen::MatrixXf coord_3d;

    Eigen::VectorXf time_delta;
    Eigen::VectorXf time_delta_reverse;
    
    Eigen::Vector3f angular_velocity;
    Eigen::Vector3f angular_position;

    std::vector<double> time_stamp;
    std::vector<bool> polarity;
    std::vector<float> x;
    std::vector<float> y;
    std::vector<bool> isInner;
    size_t size;
};

#endif //DATABASE_H
