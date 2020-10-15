#include <eigen3/Eigen/Core>

#include "utils/System.h"
#include "numerics.h"

System::System(const std::string& yaml)
{
    std::cout << "\n" <<
            "RME-EC: Rotational Motion Estimation with Event Camera" << "\n" << "\n"
            "Copyright (C) 2020 Haram Kim" << "\n" <<
            "Lab for Autonomous Robotics Research, Seoul Nat'l Univ." << "\n" << "\n";

    // Read settings file
    cv::FileStorage fsSettings(yaml, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        ROS_ERROR("Failed to open settings file at: %s", yaml.c_str());
        exit(-1);
    }

    // cv::namedWindow("map image", CV_WINDOW_AUTOSIZE);
    // cv::namedWindow("warped event image", CV_WINDOW_AUTOSIZE);
    // cv::namedWindow("undistorted renderer", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("map image view", CV_WINDOW_AUTOSIZE);
    
    /// Set parameters   

    // general params
    plot_denom = fsSettings["visualize.denom"];
    sampling_rate = fsSettings["sampling_rate"];
    map_sampling_rate = fsSettings["map_sampling_rate"];
    event_image_threshold = fsSettings["event_image_threshold"];
    rotation_estimation = fsSettings["rotation_estimation"];
    run_index = fsSettings["run_index"];
    mapping_interval = fsSettings["mapping_interval"];
    optimization_view_index = fsSettings["optimization_view_index"];
    optimization_view_flag = false;

    // event params
    delta_time = fsSettings["delta_time"];
    max_event_num = fsSettings["max_event_num"];

    // camera params
    width = fsSettings["width"];
    height = fsSettings["height"];

    map_scale = fsSettings["map_scale"];

    camParam.fx = fsSettings["Camera.fx"];
    camParam.fy = fsSettings["Camera.fy"];
    camParam.cx = fsSettings["Camera.cx"];
    camParam.cy = fsSettings["Camera.cy"];
    camParam.rd1 = fsSettings["Camera.rd1"];
    camParam.rd2 = fsSettings["Camera.rd2"];

    cameraMatrix = 
    (cv::Mat1d(3,3) << camParam.fx, 0.0, camParam.cx,
                    0.0, camParam.fy, camParam.cy,
                    0.0, 0.0, 1.0);
    distCoeffs = 
    (cv::Mat1d(1,4) << camParam.rd1, camParam.rd2, 0.0, 0.0);

    grad_x_kernel = 
    (cv::Mat1d(1, 3) << -0.5, 0.0, 0.5); 
    grad_y_kernel = 
    (cv::Mat1d(3, 1) << -0.5, 0.0, 0.5); 

    cv::cv2eigen(cameraMatrix, K);

    width_map = map_scale * width;
    height_map = map_scale * height;
    K_map = K;
    K_map(0, 2) = width_map / 2;
    K_map(1, 2) = height_map / 2;

    // optimization parameters
    optimizer_max_iter = fsSettings["Optimizer.max_iter"];

    mu_event = fsSettings["Optimizer.mu_angular_velocity"];
    rho_event = fsSettings["Optimizer.rho_angular_velocity"];

    mu_map = fsSettings["Optimizer.mu_angular_position"];
    rho_map = fsSettings["Optimizer.rho_angular_position"];

    /// Initialization
    denominator = 1.0f / plot_denom;
    last_event_time = 0.0f;
    estimation_time_interval = -1;
    estimation_time_interval_prev = -1;

    // image
    current_image.image = cv::Mat(height, width, CV_16UC1);
    current_image.image = cv::Scalar(0);
    undistorted_image.image = cv::Mat(height, width, CV_16UC1);
    undistorted_image.image = cv::Scalar(0);
    warped_event_image = cv::Mat(height, width, CV_32FC1);
    warped_event_image = cv::Scalar(0);
    warped_map_image = cv::Mat(height, width, CV_32FC1);
    warped_map_image = cv::Scalar(0);
    undistorted_render = cv::Mat(height, width, CV_32FC3);
    undistorted_render = cv::Scalar(0, 0, 0);
    map_render = cv::Mat(height_map, width_map, CV_32FC3);
    map_render = cv::Scalar(0, 0, 0);

    Ix_map = cv::Mat(height, width, CV_32FC1);
    Iy_map = cv::Mat(height, width, CV_32FC1); 
    
    GenerateMesh();
    
    // time
    next_process_time = delta_time;

    // counter
    map_iter_begin = 0;

    event_data_iter = 0;
    image_data_iter = 0;
    imu_data_iter = 0;

    // angular velocity
    angular_velocity = Eigen::Vector3f::Zero();
    angular_position = Eigen::Vector3f::Zero();
    angular_velocity_prev = Eigen::Vector3f::Zero();
    angular_position_prev = Eigen::Vector3f::Zero();
    angular_position_init = Eigen::Vector3f::Zero();

    update_angular_velocity = Eigen::Vector3f::Zero(); 
    update_angular_position = Eigen::Vector3f::Zero(); 

    cv::waitKey(1000);
}

System::~System()
{
}

void System::PushImageData(ImageData imageData){
    vec_image_data.push_back(imageData);
    current_image = imageData;
    UndistortImage();
    
}

void System::PushImuData(ImuData imuData){
    vec_imu_data.push_back(imuData);
}

void System::PushEventData(EventData eventData){
    vec_event_data.push_back(eventData);
    if(run_index){
        RunUptoIdx();
    }
    else{
        Run();
    }
}

// bind event data into bundle as vector form
void System::BindEvent(){
    const EventData & eventData = vec_event_data[event_data_iter];
    std::vector<dvs_msgs::Event> data = eventData.event;
    int sampler = 0;
    for (std::vector<dvs_msgs::Event>::iterator it = data.begin(); it != data.end(); it++){
        if(sampler++ % sampling_rate != 0){
            continue;
        }
        eventBundle.x.push_back(it->x);
        eventBundle.y.push_back(it->y);
        eventBundle.time_stamp.push_back(it->ts.toSec()); 
        eventBundle.polarity.push_back(it->polarity); 
    }
    event_data_iter++;
}

void System::BindMap(){
    eventMap.Clear();
    EventBundle temp;
    for (uint16_t m_iter = map_iter_begin; m_iter != eventMapPoints.size(); m_iter++){
        EventBundle & curEvent = eventMapPoints.at(m_iter);
        curEvent.size = curEvent.coord_3d.rows();
        if(curEvent.size == 0 || SO3add(curEvent.angular_position, -angular_position_prev).norm() > mapping_interval){
            map_iter_begin = m_iter + 1;
            continue;
        }
        eventMap.Append(curEvent);
    }
    eventWarpedMap.Copy(eventMap);
}

void System::ClearEvent(){
    eventBundle.Clear();
    eventUndistorted.Clear();
    eventWarped.Clear(); 
}

// run the algorithm
void System::Run()
{
    double time_begin = ros::Time::now().toSec();
    BindEvent();
    if(eventBundle.time_stamp.size() == 0){
        return;
    }
    double event_interval = eventBundle.time_stamp.back() - eventBundle.time_stamp.front();

    if (eventBundle.time_stamp.back() >= next_process_time || eventBundle.x.size() > max_event_num)
    {
        next_process_time = eventBundle.time_stamp.back() + delta_time;
        eventBundle.SetCoord();
        EstimateMotion();
        Renderer();
        Visualize();
        ClearEvent();
    }
    
    // for dataset repeat 
    if (vec_event_data.back().time_stamp < delta_time)
    {
        next_process_time = delta_time;
    }
}

void System::RunUptoIdx(){
    
    BindEvent();
    if(eventBundle.time_stamp.size() == 0){
        return;
    }
    
    if (eventBundle.time_stamp.back() >= next_process_time || eventBundle.x.size() > max_event_num)
    {
        next_process_time = eventBundle.time_stamp.back() + delta_time;
        vec_event_bundle_data.push_back(eventBundle);
        ClearEvent();
    }
    
    // main loop 
    if (vec_event_bundle_data.size() > run_index)
    {
        for (size_t iter = 0 ; iter < vec_event_bundle_data.size(); iter++){
            if (iter > optimization_view_index){
                optimization_view_flag = true;
            }
            eventBundle = vec_event_bundle_data[iter];
            double time_begin = ros::Time::now().toSec();
            double event_interval = eventBundle.time_stamp.back() - eventBundle.time_stamp.front();
            eventBundle.SetCoord();
            EstimateMotion();
            Renderer();
            Visualize();
        }
        next_process_time = 0.0;
        cv::waitKey(0);
    }
}

void System::Visualize(){
    // imshow("map image", warped_map_image * denominator);
    // imshow("warped event image", warped_event_image * denominator);
    // imshow("undistorted renderer", undistorted_render);
    imshow("map image view", map_render);    
    cv::waitKey(1);    
}

void System::Renderer(){
    map_render = GetMapImageNew(SIGNED_EVENT_IMAGE_COLOR).clone();
}

// Get event image from various option 
cv::Mat System::GetEventImage(const EventBundle &event, const PlotOption &option, const bool &is_large_size /* = false*/)
{
    int height, width;
    int x, y;
    if (is_large_size){
        height = this->height_map;
        width = this->width_map;
    }
    else{
        height = this->height;
        width = this->width;
    }
    cv::Mat event_image;
    switch (option)
    {
    case DVS_RENDERER:
        if (undistorted_image.image.rows != 0 && undistorted_image.image.cols != 0)
        {
            cv::cvtColor(undistorted_image.image, event_image, CV_GRAY2BGR);
        }
        else
        {
            event_image = cv::Mat(height, width, CV_8UC3);
            event_image = cv::Scalar(0, 0, 0);
        }
        for (int i = 0; i < event.size; i++)
        {
            if (!event.isInner[i])
            {
                continue;
            }
            x = event.x[i];
            y = event.y[i];
            event_image.at<cv::Vec3b>(cv::Point(x, y)) = (event.polarity[i] == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
        }
        break;
    case SIGNED_EVENT_IMAGE:
        event_image = cv::Mat(height, width, CV_16SC1);
        event_image = cv::Scalar(0);
        for (int i = 0; i < event.size; i++)
        {
            if (!event.isInner[i])
            {
                continue;
            }
            x = event.x[i];
            y = event.y[i];
            event_image.at<short>(cv::Point(x, y)) += (event.polarity[i] == true ? 1 : -1);
        }        
        break;
    case SIGNED_EVENT_IMAGE_COLOR:
        event_image = cv::Mat(height, width, CV_16UC3);
        event_image = cv::Scalar(0, 0, 0);
        for (int i = 0; i < event.size; i++)
        {
            if (!event.isInner[i])
            {
                continue;
            }
            x = event.x[i];
            y = event.y[i];
            event_image.at<cv::Vec3w>(cv::Point(x, y)) += (event.polarity[i] == true ? cv::Vec3b(1, 0, 0) : cv::Vec3b(0, 0, 1));
        }
        break;
    case UNSIGNED_EVENT_IMAGE:
        event_image = cv::Mat(height, width, CV_16UC1);
        event_image = cv::Scalar(0);
        for (int i = 0; i < event.size; i++)
        {
            if (!event.isInner[i])
            {
                continue;
            }
            x = event.x[i];
            y = event.y[i];
            event_image.at<unsigned short>(cv::Point(x, y)) += 1;
        }
        break;
    case GRAY_EVENT_IMAGE:
        event_image = cv::Mat(height, width, CV_32FC1);
        event_image = cv::Scalar(0.5);
        for (int i = 0; i < event.size; i++)
        {
            if (!event.isInner[i])
            {
                continue;
            }
            x = event.x[i];
            y = event.y[i];
            event_image.at<float>(cv::Point(x, y)) += (event.polarity[i] == true ? 0.05 : -0.05);
        }
        break;
    }

    return event_image;
}