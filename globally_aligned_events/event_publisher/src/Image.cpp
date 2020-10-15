#include "Image.h"

Image::Image(std::string yaml)
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
    height = fSettings["height"];
    width = fSettings["width"];
}

Image::~Image(){
}

bool Image::imageReader(const std::string &filename)
{
    std::vector< sensor_msgs::Image > Data;
    std::string file_;
    file_.append(filename);
    file_.append("/images.txt");

    std::ifstream openFile(file_.c_str());;
    if (!openFile.is_open())
    {
        std::cout << "text file open error" << std::endl;
    }
    else
    {
        std::string line;
        std::string token;
        std::vector< std::string > vToken;
        char *sz;
        uint32_t seq = 0;
        while (getline(openFile, line))
        {
            sensor_msgs::Image msg;
            std::stringstream ss(line);
            while (getline(ss, token, '\t'))
                vToken.push_back(token);
            
            if(vToken.size() == 2){
                cv_bridge::CvImage cv_msg;
                cv_msg.header.seq = seq++;
                cv_msg.header.stamp = ros::Time(std::strtod(vToken[0].c_str(), &sz));
                cv_msg.encoding = "mono8";
                std::string image_name = filename;
                image_name.append("/");
                image_name.append(vToken[1].c_str());
                cv_msg.image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
                msg = *cv_msg.toImageMsg();
                Data.push_back(msg);
            }
            vToken.clear();
        }

        openFile.close();
    }
    this->imageData = Data;
    this->isEmpty = this->imageData.size() == 0;

    return this->isEmpty;
}

void Image::publish()
{
    if (!isEmpty)
    {
        while (imageData.at(count).header.stamp.toSec() <= time_curr)
        {
            cv_bridge::CvImagePtr temp = cv_bridge::toCvCopy(imageData.at(count));
            last_image = temp->image;
            image_pub->publish(imageData.at(count));
            count++;
            if (count >= imageData.size())
            {
                isRunning = false;
                count = imageData.size() - 1;
                break;
            }
        }
    }
    else{
        isRunning = false;
    }
}

void Image::setCurrTime(double time){
    this->time_curr = time;
}

void Image::setImagePublisher(ros::Publisher *pub){
    this->image_pub = pub;
}

void Image::reset(){
    count = 0;
    isRunning = true;
}