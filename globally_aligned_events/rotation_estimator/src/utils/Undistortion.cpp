#include "utils/System.h"

void System::UndistortEvent(){
    if(eventBundle.size == 0){
        return;
    }
    
    eventUndistorted.Copy(eventBundle);
    std::vector<cv::Point2f> raw_event_point(eventBundle.size),
                             undistorted_event_point(eventBundle.size);            
    for (size_t pts_iter = 0; pts_iter < eventBundle.size; pts_iter++){
        raw_event_point[pts_iter] = cv::Point2f(eventBundle.x[pts_iter], eventBundle.y[pts_iter]);
    }
    cv::undistortPoints(raw_event_point, undistorted_event_point, cameraMatrix, distCoeffs, cv::noArray(), cameraMatrix); 
    cv::Mat temp = cv::Mat(eventBundle.size, 2, CV_32FC1);
    temp.data = cv::Mat(undistorted_event_point).data;
    cv::cv2eigen(temp, eventUndistorted.coord);
    
    eventUndistorted.x.resize(eventUndistorted.coord.col(0).size());
    eventUndistorted.y.resize(eventUndistorted.coord.col(1).size());

    Eigen::VectorXf::Map(&eventUndistorted.x[0], eventUndistorted.coord.col(0).size()) = Eigen::VectorXf(eventUndistorted.coord.col(0));
    Eigen::VectorXf::Map(&eventUndistorted.y[0], eventUndistorted.coord.col(1).size()) = Eigen::VectorXf(eventUndistorted.coord.col(1));  
    
    eventUndistorted.DiscriminateInner(width, height);
    eventUndistorted.SetCoord();
}

// Generate mesh for undistortion
void System::GenerateMesh(){
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat::eye(3, 3, CV_32FC1), 
                                cameraMatrix, cv::Size(width, height), CV_32FC1, 
                                undist_mesh_x, undist_mesh_y); 
}
// Undistort image
void System::UndistortImage(){
    undistorted_image.time_stamp = current_image.time_stamp;
    undistorted_image.seq = current_image.seq;

    cv::remap(current_image.image, undistorted_image.image, undist_mesh_x, undist_mesh_y, CV_INTER_LINEAR);
}
