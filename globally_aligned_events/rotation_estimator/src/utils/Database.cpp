#include "Database.h"
EventBundle::EventBundle(){
    size = 0;
    coord.resize(size, 2);
    coord_3d.resize(size, 3);
}

EventBundle::EventBundle(const EventBundle& temp){
    coord = temp.coord;
    coord_3d = temp.coord_3d;

    angular_velocity = temp.angular_velocity;
    angular_position = temp.angular_position;

    time_delta = temp.time_delta;
    time_delta_reverse = temp.time_delta_reverse;

    time_stamp = temp.time_stamp;
    polarity = temp.polarity;
    x = temp.x;
    y = temp.y;
    isInner = temp.isInner;
    size = temp.size;
}
EventBundle::~EventBundle(){

}

void EventBundle::Append(const EventBundle &ref){
    
    size += ref.size;
    
    Eigen::MatrixXf temp_coord = coord;
    Eigen::MatrixXf temp_coord_3d = coord_3d;
    Eigen::VectorXf temp_time_delta = time_delta;
    Eigen::VectorXf temp_time_delta_reverse = time_delta_reverse;

    coord.resize(size, 2);
    coord_3d.resize(size, 3);

    time_delta.resize(size);
    time_delta_reverse.resize(size);

    coord << temp_coord, ref.coord;
    coord_3d << temp_coord_3d, ref.coord_3d;
    time_delta << temp_time_delta, ref.time_delta;
    time_delta_reverse << temp_time_delta_reverse, ref.time_delta_reverse;

    time_stamp.insert(time_stamp.end(), ref.time_stamp.begin(), ref.time_stamp.end());
    polarity.insert(polarity.end(), ref.polarity.begin(), ref.polarity.end());


}

void EventBundle::Clear(){
    size = 0;

    coord.resize(size, 2);
    coord_3d.resize(size, 3);

    time_delta.resize(size);
    time_delta_reverse.resize(size);

    angular_velocity = Eigen::Vector3f::Zero();
    angular_position = Eigen::Vector3f::Zero();

    time_stamp.clear();
    polarity.clear();
    x.clear();
    y.clear();
    isInner.clear();
}

void EventBundle::Copy(const EventBundle &ref){
    time_delta = ref.time_delta;
    time_delta_reverse = ref.time_delta_reverse;

    time_stamp = ref.time_stamp;
    polarity = ref.polarity;
    size = ref.size;

    angular_velocity = ref.angular_velocity;
    angular_position = ref.angular_position;

    coord.resize(size,2);
    coord_3d.resize(size,3);
}

void EventBundle::SetCoord(){
    size = x.size();
    if(size != 0){
        Eigen::Map<Eigen::VectorXf> eigen_x(x.data(), size);
        Eigen::Map<Eigen::VectorXf> eigen_y(y.data(), size);
        coord = Eigen::MatrixXf(size, 2);
        coord << eigen_x, eigen_y;
    }
}

void EventBundle::erase(size_t iter){
    time_stamp.erase(time_stamp.begin() + iter);
    polarity.erase(polarity.begin() + iter);
    x.erase(x.begin() + iter);
    y.erase(y.begin() + iter);
}

// Project coord_3d into coord
void EventBundle::Projection(Eigen::Matrix3f K){
    coord.col(0) = coord_3d.col(0).array()/coord_3d.col(2).array() * K(0, 0) + K(0, 2); 
    coord.col(1) = coord_3d.col(1).array()/coord_3d.col(2).array() * K(1, 1) + K(1, 2);
    coord = coord.array().round();
}

// Backproject coord into coord_3d
void EventBundle::InverseProjection(Eigen::Matrix3f K){
    Eigen::Map<Eigen::VectorXd> time_delta_(time_stamp.data(), size);
    time_delta = (time_stamp.front()- time_delta_.array()).cast<float>();
    time_delta_reverse = (time_stamp.back()- time_delta_.array()).cast<float>();
    coord_3d.col(0) = (coord.col(0).array() - K(0, 2))/K(0, 0);
    coord_3d.col(1) = (coord.col(1).array() - K(1, 2))/K(1, 1);
    coord_3d.col(2) = Eigen::MatrixXf::Ones(size, 1);
}

// Discriminate whether the coord is inside the image window or not
void EventBundle::DiscriminateInner(int width, int height, int map_sampling_rate /*= 1*/)
{
    isInner.resize(size);
    if (x.size() != size)
    {
        SetXY();
    }
    for (uint32_t pts_iter = 0; pts_iter < size; pts_iter++)
    {
        if (x[pts_iter] <= 0 || x[pts_iter] >= width || y[pts_iter] <= 0 || y[pts_iter] >= height || pts_iter % map_sampling_rate != 0)
        {
            isInner[pts_iter] = false;
        }
        else
        {
            isInner[pts_iter] = true;
        }
    }
}

// Synchronize the coord and the x, y
void EventBundle::SetXY(){
    x = std::vector<float>(coord.col(0).data(), coord.col(0).data() + size);
    y = std::vector<float>(coord.col(1).data(), coord.col(1).data() + size);
}