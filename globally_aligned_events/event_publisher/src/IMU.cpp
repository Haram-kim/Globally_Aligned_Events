#include "IMU.h"

IMU::IMU(std::string yaml)
{
    isEmpty = false;
    isRunning = true;
    count = 0;
    // read yaml file
    cv::FileStorage fSettings(yaml, cv::FileStorage::READ);
    if (!fSettings.isOpened())
    {
        throw std::runtime_error(std::string("Could not open file: ") + yaml);
    }

}

IMU::~IMU(){
}

bool IMU::imuReader(const std::string &filename)
{
    std::vector< sensor_msgs::Imu > Data;
    std::string file_;
    file_.append(filename);
    file_.append("/imu.txt");

    std::ifstream openFile(file_.c_str());
    if (!openFile.is_open())
    {
        std::cout << "text file open error" << std::endl;
    }
    else
    {
        std::string line;
        std::string token;
        std::vector< std::string > vToken;
        uint32_t seq = 0;
        char *sz;

        while (getline(openFile, line))
        {
            sensor_msgs::Imu msg;
            std::stringstream ss(line);
            while (getline(ss, token, '\t'))
                vToken.push_back(token);
            
            if(vToken.size() == 7){
                msg.orientation_covariance[0] = -1.0;
                msg.header.frame_id = "base_link";
                msg.header.seq = seq++;
                msg.header.stamp           = ros::Time(std::strtod(vToken[0].c_str(), &sz));
                msg.linear_acceleration.x  = std::strtod(vToken[1].c_str(), &sz);
                msg.linear_acceleration.y  = std::strtod(vToken[2].c_str(), &sz);
                msg.linear_acceleration.z  = std::strtod(vToken[3].c_str(), &sz);
                msg.angular_velocity.x     = std::strtod(vToken[4].c_str(), &sz);
                msg.angular_velocity.y     = std::strtod(vToken[5].c_str(), &sz);
                msg.angular_velocity.z     = std::strtod(vToken[6].c_str(), &sz);

                Data.push_back(msg);
            }
            vToken.clear();
        }

        openFile.close();
    }
    this->imuData = Data;
    this->isEmpty = this->imuData.size() == 0;

    return this->isEmpty;
}

void IMU::publish()
{
    if (!isEmpty)
    {
        while (imuData.at(count).header.stamp.toSec() <= time_curr)
        {
            imu_pub->publish(imuData.at(count));
            count++;
            if (count >= imuData.size())
            {
                isRunning = false;
                count = imuData.size() - 1;
                break;
            }
        }
    }
    else
    {
        isRunning = false;
    }
    
}

void IMU::setCurrTime(double time){
    this->time_curr = time;
}

void IMU::setIMUPublisher(ros::Publisher *pub){
    this->imu_pub = pub;
}

void IMU::reset(){
    count = 0;
    isRunning = true;
}