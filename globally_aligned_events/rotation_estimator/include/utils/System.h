#ifndef SYSTEM_H
#define SYSTEM_H

// #include <cmath>
#include <ros/ros.h>

// messages
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

// eigen
#include <eigen3/Eigen/Core>

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

#include <algorithm>

#include "Database.h"
#include "utils/numerics.h"

#include <fstream>
#include <iostream>

enum PlotOption{
    DVS_RENDERER,
    SIGNED_EVENT_IMAGE,
    SIGNED_EVENT_IMAGE_COLOR,
    UNSIGNED_EVENT_IMAGE,
    GRAY_EVENT_IMAGE
};


class System
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    System(const std::string& strSettingsFile);

    ~System();

    // Data read
    void PushImageData(ImageData imageData);
    void PushImuData(ImuData imuData);
    void PushEventData(EventData eventData);

private:
    /// functions
    void Run();
    void RunUptoIdx();
    
    void BindEvent();
    void BindMap();
    void ClearEvent();

    void UndistortEvent();
    void UndistortImage();
    void GenerateMesh();

    void EstimateMotion();
    void GetWarpedEventImage(const Eigen::Vector3f &temp_ang_vel, const PlotOption &option = UNSIGNED_EVENT_IMAGE);
    void GetWarpedEventImage(const Eigen::Vector3f &temp_ang_vel, const Eigen::Vector3f &temp_ang_pos);
    void GetWarpedEventMap(const Eigen::Vector3f &temp_ang_pos);
    void GetWarpedEventPoint(const EventBundle &eventIn, 
                            EventBundle &eventOut, 
                            const Eigen::Vector3f &temp_ang_vel, 
                            const bool &is_map_warping = false,
                            const Eigen::Vector3f &temp_ang_pos = Eigen::Vector3f::Zero());
    void DeriveErrAnalytic(const Eigen::Vector3f &temp_ang_vel);
    void DeriveErrAnalytic(const Eigen::Vector3f &temp_ang_vel, const Eigen::Vector3f &temp_ang_pos);
    void DeriveErrAnalyticMap(const Eigen::Vector3f &temp_ang_pos);
    std::vector<uint16_t> GetValidIndexFromEvent(const EventBundle &event);
    void JacobianTrim();
    void ComputeAngularPosition();
    void UpdateAngularVelocity();
    void UpdateAngularPosition();
    
    
    void Mapping();
    cv::Mat GetMapImage(const PlotOption &option = SIGNED_EVENT_IMAGE);
    cv::Mat GetMapImageNew(const PlotOption &option = SIGNED_EVENT_IMAGE);

    void Visualize();
    void Renderer();
    cv::Mat GetEventImage(const EventBundle& e, const PlotOption &option, const bool &is_large_size = 0);
    
    
    /// variables
    // data stack
    std::vector<ImuData> vec_imu_data;
    std::vector<ImageData> vec_image_data;
    std::vector<EventData> vec_event_data;
    std::vector<EventBundle> vec_event_bundle_data;

    // event points
    EventBundle eventBundle;
    EventBundle eventUndistorted;
    EventBundle eventUndistortedPrev;
    EventBundle eventWarped;
    EventBundle eventAlignedPoints;
    EventBundle eventMap;
    EventBundle eventWarpedMap;
    std::vector<EventBundle> eventMapPoints;
    
    // Image
    ImageData current_image;
    ImageData undistorted_image;
    cv::Mat undistorted_render;
    cv::Mat map_render;
    cv::Mat undist_mesh_x;
    cv::Mat undist_mesh_y;

    // time
    double next_process_time;
    double delta_time;
    double event_delta_time;
    double last_event_time;
    double estimation_time_interval;
    double estimation_time_interval_prev;

    int max_event_num;

    // counter
    uint16_t map_iter_begin;

    size_t event_data_iter;
    size_t image_data_iter;
    size_t imu_data_iter;

    // samplier
    int sampling_rate;
    int map_sampling_rate;

    // camera parameters
    Eigen::Matrix3f K;
    Eigen::Matrix3f K_map;
    CameraParam camParam;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    int width, height;
    int width_map, height_map;
    float map_scale;

    // camera pose
    std::vector<double> estimatior_time;

    std::vector<Eigen::Vector3f> vec_angular_velocity;
    std::vector<Eigen::Vector3f> vec_angular_position;

    Eigen::Vector3f angular_velocity;
    Eigen::Vector3f angular_velocity_prev;
    Eigen::Vector3f angular_position;
    Eigen::Vector3f angular_position_init;
    Eigen::Vector3f angular_position_prev;
    Eigen::Vector3f angular_position_pprev;
    Eigen::Vector3f angular_position_compensator;

    Eigen::Vector3f update_angular_velocity;
    Eigen::Vector3f update_angular_position;

    // visualize 
    float plot_denom;
    float denominator;
    /// Estimator variables
    // optimization parameters
    float nu_event; // exponential average of squares of gradients
    float mu_event; // step size | optimization rate
    float rho_event; // smoothing factor | the degree of weigthing decrease in geometric moving average
    
    float nu_map;
    float mu_map;
    float rho_map;

    int optimizer_max_iter;
    bool is_polarity_on;

    cv::Mat warped_event_image;
    cv::Mat warped_event_image_grabbed;
    cv::Mat warped_map_image;

    // derive error analytic
    cv::Mat grad_x_kernel;
    cv::Mat grad_y_kernel;

    Eigen::VectorXf xp_zp;
    Eigen::VectorXf yp_zp;

    Eigen::ArrayXf xx_zz;
    Eigen::ArrayXf yy_zz;
    Eigen::ArrayXf xy_zz;

    Eigen::VectorXf xp;
    Eigen::VectorXf yp;
    Eigen::VectorXf zp;

    cv::Mat blur_image;

    cv::Mat Ix_sum;
    cv::Mat Iy_sum;

    cv::Mat Ix;
    cv::Mat Iy;
    Eigen::VectorXf Ix_interp;
    Eigen::VectorXf Iy_interp;

    Eigen::VectorXf xp_map;
    Eigen::VectorXf yp_map;
    Eigen::VectorXf zp_map;

    cv::Mat blur_image_map;
    cv::Mat truncated_map, truncated_event, truncated_sum;
    
    cv::Mat Ix_map;
    cv::Mat Iy_map;
    Eigen::VectorXf Ix_interp_map;
    Eigen::VectorXf Iy_interp_map;

    Eigen::VectorXf warped_image_delta_t;
    Eigen::VectorXf warped_map_delta_t;

    Eigen::MatrixXf Jacobian;
    Eigen::MatrixXf Jacobian_map;

    Eigen::Vector3f Gradient;
    Eigen::Vector3f Gradient_map;

    int event_image_threshold;
    int rotation_estimation;
    int run_index;
    float mapping_interval;
    int optimization_view_index;
    bool optimization_view_flag;

    std::string filePath;
    std::ofstream writeFile;
};

#endif // SYSTEM_H