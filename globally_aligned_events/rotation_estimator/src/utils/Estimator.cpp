#include "utils/System.h"

void System::EstimateMotion(){
    UndistortEvent();
    eventUndistorted.InverseProjection(K);
    eventWarped.Copy(eventUndistorted);

    float event_num_weight = std::min(eventUndistorted.size * 1e-3 + 1e-4, 1.0);
    
    estimation_time_interval = eventUndistorted.time_stamp.front() - last_event_time;
    last_event_time = eventUndistorted.time_stamp.front();
    event_delta_time = eventUndistorted.time_stamp.back() - eventUndistorted.time_stamp.front();

    ComputeAngularPosition();
    BindMap();

    angular_velocity = Eigen::Vector3f::Zero();
    angular_position_compensator = Eigen::Vector3f::Zero();
    nu_event = 1.0f;
    nu_map = 1.0f;

    float map_norm = 0.0, map_norm_reciprocal = 0.0;
    if(rotation_estimation){
        GetWarpedEventMap(angular_position_init);
        cv::threshold(warped_map_image, truncated_map, event_image_threshold, 255, 2); 
        map_norm = cv::norm(truncated_map);
        map_norm_reciprocal = 1 / sqrt(map_norm);
        cv::GaussianBlur(truncated_map * map_norm_reciprocal, truncated_map, cv::Size(21, 21), 3);
        cv::Sobel(truncated_map, Ix_map, CV_32FC1, 1, 0);
        cv::Sobel(truncated_map, Iy_map, CV_32FC1, 0, 1);
    }

    for (int i = 0; i < optimizer_max_iter; i++){
        // Derive Error Analytic
        is_polarity_on = i < optimizer_max_iter / 2;
        DeriveErrAnalytic(angular_velocity, angular_position_compensator);
        Gradient = Jacobian.transpose() * warped_image_delta_t;
        // RMS-prop optimizer
        nu_event = rho_event * nu_event
                + (1.0f - rho_event) * (float)(Gradient.transpose() * Gradient);
        update_angular_velocity = - event_num_weight * mu_event / std::sqrt(nu_event + 1e-8) * Gradient;
        angular_velocity = SO3add(update_angular_velocity, angular_velocity, true);
        // R_upd
        if(map_norm != 0 && rotation_estimation){
            Gradient_map = Jacobian_map.transpose() * warped_map_delta_t;
            //RMS-prop optimizer
            nu_map = rho_map * nu_map
                    + (1.0f - rho_map) * (float)(Gradient_map.transpose() * Gradient_map);
            update_angular_position = - event_num_weight * mu_map / std::sqrt(nu_map + 1e-8) * Gradient_map;
            angular_position_compensator = SO3add(update_angular_position, angular_position_compensator);
        }
    }
    angular_position = SO3add(angular_position_init, angular_position_compensator);
    GetWarpedEventImage(angular_velocity, SIGNED_EVENT_IMAGE_COLOR);
}

void System::ComputeAngularPosition(){
    // Global events alignment
    eventAlignedPoints.Copy(eventUndistortedPrev);
    GetWarpedEventPoint(eventUndistortedPrev, eventAlignedPoints, angular_velocity, false, angular_position);
    eventAlignedPoints.angular_velocity = angular_velocity_prev;
    eventAlignedPoints.angular_position = angular_position;
    eventMapPoints.push_back(eventAlignedPoints);

    // initial angular position with angular velocity
    if(!std::isfinite(estimation_time_interval_prev) || estimation_time_interval_prev <= 0){
        std::cout << "estimation_time_interval is nan" << std::endl;
    }
    else{
        angular_position_init = SO3add(angular_position, -angular_velocity * estimation_time_interval);
    }

    eventUndistortedPrev = eventUndistorted;
    estimation_time_interval_prev = estimation_time_interval;
    angular_velocity_prev = angular_velocity;
    angular_position_pprev = angular_position_prev;    
    angular_position_prev = angular_position;
}

// Compute Jacobian matrix
// update Ix, Iy, Jacobian, warped_image_valid
void System::DeriveErrAnalytic(const Eigen::Vector3f &temp_ang_vel, const Eigen::Vector3f &temp_ang_pos)
{
    GetWarpedEventImage(temp_ang_vel, temp_ang_pos);
    cv::threshold(warped_event_image, truncated_event, event_image_threshold, 255, 2); 
  
    cv::GaussianBlur(truncated_event, blur_image, cv::Size(5, 5), 1);
    
    cv::Sobel(blur_image, Ix, CV_32FC1, 1, 0);
    cv::Sobel(blur_image, Iy, CV_32FC1, 0, 1);

    if(rotation_estimation){
        float event_norm = 2.0f / cv::norm(truncated_event);
        truncated_sum = truncated_map + truncated_event * event_norm;
        Ix_sum = Ix_map + Ix * event_norm;
        Iy_sum = Iy_map + Iy * event_norm;
        if(optimization_view_flag){
            imshow("truncated_evt", truncated_event * 20);
            imshow("truncated_sum", truncated_sum * 5);
            cv::waitKey(10);
        }
    }

    std::vector<uint16_t> e_valid = GetValidIndexFromEvent(eventWarped);
    
    uint16_t valid_size = e_valid.size();
    xp.resize(valid_size);
    yp.resize(valid_size);
    zp.resize(valid_size);
    Ix_interp.resize(valid_size);
    Iy_interp.resize(valid_size);
    warped_image_delta_t.resize(valid_size);
    Jacobian.resize(valid_size, 3);

    Ix_interp_map.resize(valid_size);
    Iy_interp_map.resize(valid_size);
    warped_map_delta_t.resize(valid_size);
    Jacobian_map.resize(valid_size, 3);
    
    int16_t coord_x;
    int16_t coord_y;
    uint16_t e_valid_v_iter;
    for (uint16_t v_iter = 0; v_iter < valid_size; v_iter++)
    {
        e_valid_v_iter = e_valid[v_iter];

        coord_x = std::round(eventWarped.coord(e_valid_v_iter, 0));
        coord_y = std::round(eventWarped.coord(e_valid_v_iter, 1)); 

        xp(v_iter) = eventWarped.coord_3d(e_valid_v_iter, 0);
        yp(v_iter) = eventWarped.coord_3d(e_valid_v_iter, 1);
        zp(v_iter) = eventWarped.coord_3d(e_valid_v_iter, 2);

        Ix_interp(v_iter) = Ix.at<float>(coord_y, coord_x);
        Iy_interp(v_iter) = Iy.at<float>(coord_y, coord_x);

        warped_image_delta_t(v_iter) = - eventWarped.time_delta(e_valid_v_iter) 
                                * warped_event_image.at<float>(coord_y, coord_x);
        if(rotation_estimation){
            Ix_interp_map(v_iter) = Ix_sum.at<float>(coord_y, coord_x);
            Iy_interp_map(v_iter) = Iy_sum.at<float>(coord_y, coord_x);

            warped_map_delta_t(v_iter) = - truncated_sum.at<float>(coord_y, coord_x);
        }
    }
    Ix_interp *= camParam.fx;
    Iy_interp *= camParam.fy;

    xp_zp = xp.array() / zp.array();
    yp_zp = yp.array() / zp.array();
    Eigen::ArrayXf xx_zz = xp_zp.array() * xp_zp.array();
    Eigen::ArrayXf yy_zz = yp_zp.array() * yp_zp.array();
    Eigen::ArrayXf xy_zz = xp_zp.array() * yp_zp.array();
    Jacobian.col(0) = -(Ix_interp.array() * xy_zz) 
                    - (Iy_interp.array() * (1 + yy_zz));
    Jacobian.col(1) = (Ix_interp.array() * (1 + xx_zz)) 
                    + (Iy_interp.array() * xy_zz);
    Jacobian.col(2) = -Ix_interp.array() * yp_zp.array() + Iy_interp.array() * xp_zp.array();

    if(rotation_estimation){
        Ix_interp_map *= camParam.fx;
        Iy_interp_map *= camParam.fy;

        Jacobian_map.col(0) = -(Ix_interp_map.array() * xy_zz) 
                            - (Iy_interp_map.array() * (1 + yy_zz));
        Jacobian_map.col(1) = (Ix_interp_map.array() * (1 + xx_zz)) 
                            + (Iy_interp_map.array() * xy_zz);
        Jacobian_map.col(2) = -Ix_interp_map.array() * yp_zp.array() + Iy_interp_map.array() * xp_zp.array();
    }
}

// warping function
void System::GetWarpedEventPoint(const EventBundle &eventIn, EventBundle &eventOut, const Eigen::Vector3f &temp_ang_vel, const bool &is_map_warping /*= false*/, const Eigen::Vector3f &temp_ang_pos /*= Eigen::Vector3f::Zero()*/){
    if(eventIn.size == 0){
        std::cout << "EventIn size is zero" << std::endl;
        return;
    }
    float ang_vel_norm = temp_ang_vel.norm();
    float ang_pos_norm = temp_ang_pos.norm();
    Eigen::MatrixXf x_ang_vel_hat, x_ang_vel_hat_square;
    x_ang_vel_hat.resize(eventIn.size, 3);
    x_ang_vel_hat_square.resize(eventIn.size, 3);
    
    x_ang_vel_hat.col(0) = - temp_ang_vel(2) * eventIn.coord_3d.col(1) + temp_ang_vel(1) * eventIn.coord_3d.col(2);
    x_ang_vel_hat.col(1) = + temp_ang_vel(2) * eventIn.coord_3d.col(0) - temp_ang_vel(0) * eventIn.coord_3d.col(2);
    x_ang_vel_hat.col(2) = - temp_ang_vel(1) * eventIn.coord_3d.col(0) + temp_ang_vel(0) * eventIn.coord_3d.col(1);

    x_ang_vel_hat_square.col(0) = - temp_ang_vel(2) * x_ang_vel_hat.col(1) + temp_ang_vel(1) * x_ang_vel_hat.col(2);
    x_ang_vel_hat_square.col(1) = + temp_ang_vel(2) * x_ang_vel_hat.col(0) - temp_ang_vel(0) * x_ang_vel_hat.col(2);
    x_ang_vel_hat_square.col(2) = - temp_ang_vel(1) * x_ang_vel_hat.col(0) + temp_ang_vel(0) * x_ang_vel_hat.col(1);

    // Element-wise
    // points warping via second order approximation with Rodrigue's formular
    if(is_map_warping){
        if(ang_vel_norm < 1e-8){
            eventOut.coord_3d = eventIn.coord_3d;
        }
        else{
            auto delta_t = eventIn.time_delta_reverse.array();
            eventOut.coord_3d = eventIn.coord_3d
                            + Eigen::MatrixXf(x_ang_vel_hat.array().colwise()
                            * delta_t.array()
                            + x_ang_vel_hat_square.array().colwise() 
                            * (0.5f * delta_t.array().square()) );
        }
    }
    else{
        if(ang_vel_norm < 1e-8){
            eventOut.coord_3d = eventIn.coord_3d;
        }
        else{            
            auto delta_t = eventIn.time_delta.array();
            eventOut.coord_3d = eventIn.coord_3d
                            + Eigen::MatrixXf( x_ang_vel_hat.array().colwise()
                            * delta_t.array()
                            + x_ang_vel_hat_square.array().colwise() 
                            * (0.5f * delta_t.array().square()) );
        }
    }  
    if(ang_pos_norm > 1e-8){
        eventOut.coord_3d = eventOut.coord_3d * SO3(temp_ang_pos).transpose();
    }
}

std::vector<uint16_t> System::GetValidIndexFromEvent(const EventBundle & event){
    std::vector<uint16_t> valid;
    uint16_t valid_counter = 0;
    for (auto e_iter = event.isInner.begin(); e_iter != event.isInner.end(); e_iter++){
        if(*e_iter){
            valid.push_back(valid_counter);
        }
        valid_counter++;
    }
    return valid;
}

void System::GetWarpedEventImage(const Eigen::Vector3f &temp_ang_vel, const PlotOption &option /*= UNSIGNED_EVENT_IMAGE*/){
    GetWarpedEventPoint(eventUndistorted, eventWarped, temp_ang_vel);
    eventWarped.Projection(K);
    eventWarped.SetXY();
    eventWarped.DiscriminateInner(width - 1, height - 1);
    GetEventImage(eventWarped, option).convertTo(warped_event_image, CV_32FC1);
}

void System::GetWarpedEventImage(const Eigen::Vector3f &temp_ang_vel, const Eigen::Vector3f &temp_ang_pos){
    GetWarpedEventPoint(eventUndistorted, eventWarped, temp_ang_vel, false, temp_ang_pos);
    eventWarped.Projection(K);
    eventWarped.SetXY();
    eventWarped.DiscriminateInner(width - 1, height - 1);
    if(is_polarity_on){
        GetEventImage(eventWarped, SIGNED_EVENT_IMAGE).convertTo(warped_event_image, CV_32FC1);
    }
    else{
        GetEventImage(eventWarped, UNSIGNED_EVENT_IMAGE).convertTo(warped_event_image, CV_32FC1);
    }
}

void System::GetWarpedEventMap(const Eigen::Vector3f &temp_ang_pos){
    if( eventMap.size == 0 ){
        return;
    }
    GetWarpedEventPoint(eventMap, eventWarpedMap, Eigen::Vector3f::Zero(), true, -temp_ang_pos);
    eventWarpedMap.Projection(K);
    eventWarpedMap.SetXY();
    eventWarpedMap.DiscriminateInner(width - 1, height - 1, map_sampling_rate);
    GetEventImage(eventWarpedMap, UNSIGNED_EVENT_IMAGE, false).convertTo(warped_map_image, CV_32FC1);
}

cv::Mat System::GetMapImage(const PlotOption &option /*= UNSIGNED_EVENT_IMAGE*/){
    cv::Mat map_image = cv::Mat(height_map, width_map, CV_16UC3);
    map_image = cv::Scalar(0, 0, 0);
    cv::Mat map_image_temp = cv::Mat(height_map, width_map, CV_16UC3);
    EventBundle temp;
    for (uint16_t m_iter = map_iter_begin; m_iter != eventMapPoints.size(); m_iter++){
        EventBundle & curEvent = eventMapPoints.at(m_iter);
        curEvent.size = curEvent.coord_3d.rows();
        if(curEvent.size == 0 || SO3add(curEvent.angular_position, -angular_position).norm() > mapping_interval){
            // map_iter_begin = m_iter + 1;
            continue;
        }
        temp.Copy(curEvent);
        GetWarpedEventPoint(curEvent, temp, Eigen::Vector3f::Zero(), true, -angular_position);
        temp.Projection(K_map);
        temp.SetXY();
        temp.DiscriminateInner(width_map - 1, height_map - 1, map_sampling_rate);
        GetEventImage(temp, option, true).convertTo(map_image_temp, CV_16UC3);
        map_image += map_image_temp;
    }
    cv::Mat result;
    map_image.convertTo(result, CV_32FC3);
    return result;
}

cv::Mat System::GetMapImageNew(const PlotOption &option /*= UNSIGNED_EVENT_IMAGE*/){
    cv::Mat result = cv::Mat(height_map, width_map, CV_32FC3);
    result = cv::Scalar(0, 0, 0);
    EventBundle eventMapTemp;
    eventMapTemp.Copy(eventMap);
    if( eventMap.size == 0 ){
        return result;
    }
    GetWarpedEventPoint(eventMap, eventMapTemp, Eigen::Vector3f::Zero(), true, -angular_position);
    eventMapTemp.Projection(K_map);
    eventMapTemp.SetXY();
    eventMapTemp.DiscriminateInner(width_map - 1, height_map - 1, map_sampling_rate);
    GetEventImage(eventMapTemp, option, true).convertTo(result, CV_32FC3);
    return result;
}

