#include "Event.h"

Event::Event(std::string yaml)
{
    msg = dvs_msgs::EventArrayPtr(new dvs_msgs::EventArray());
    isEmpty = false;
    isRunning = true;
    count = 0;
    seq = 0;
    next_send_time = 0.0;
    // read yaml file
    cv::FileStorage fSettings(yaml, cv::FileStorage::READ);
    if (!fSettings.isOpened())
    {
        throw std::runtime_error(std::string("Could not open file: ") + yaml);
    }

    height = fSettings["height"];
    width = fSettings["width"];

    delta_time = fSettings["Event.delta_time"];
    max_events = fSettings["Event.max_event"];

}

Event::~Event(){
}

bool Event::eventReader(const std::string &filename)
{
    std::vector<dvs_msgs::Event> Data;
    std::string file_;
    file_.append(filename);
    file_.append("/events.txt");

    std::ifstream openFile(file_.c_str());
    if (!openFile.is_open())
    {
        std::cout << "text file open error" << std::endl;
    }
    else
    {
        std::string line;
        std::string token;
        std::vector<std::string> vToken;
        char *sz;

        while (getline(openFile, line))
        {
            dvs_msgs::Event msg;
            std::stringstream ss(line);
            while (getline(ss, token, '\t'))
                vToken.push_back(token);

            if (vToken.size() == 4)
            {
                msg.ts = ros::Time(std::strtod(vToken[0].c_str(), &sz));
                msg.x = uint16_t(std::strtod(vToken[1].c_str(), &sz));
                msg.y = uint16_t(std::strtod(vToken[2].c_str(), &sz));
                msg.polarity = uint8_t(std::strtod(vToken[3].c_str(), &sz));

                // std::cout << "EVT::  "<< vToken[0]<< ",  "<< vToken[1]<< ",  "<< vToken[2]<< ",  "<< vToken[3] << std::endl;
                Data.push_back(msg);
            }
            vToken.clear();
        }

        openFile.close();
    }
    this->eventData = Data;
    this->isEmpty = this->eventData.size() == 0;

    return this->isEmpty;
}

void Event::publish()
{
    if (!msg)
    {
        msg = dvs_msgs::EventArrayPtr(new dvs_msgs::EventArray());
    }
    msg->height = height;
    msg->width = width;
    // add events on event array until the ts of an event is less than the current time.
    while (eventData.at(count).ts.toSec() <= time_curr)
    {
        msg->events.push_back(eventData.at(count));
        msg->header.seq = seq++;
        msg->header.stamp = eventData.at(count).ts;
        count++;
        if (count >= eventData.size())
        {
            isRunning = false;
            count = eventData.size() - 1;
            break;
        }
    }

    if (time_curr > next_send_time || msg->events.size() > max_events)
    {
        // publish renderer
        renderer();
        this->dvs_image_pub->publish(cv_image.toImageMsg());

        // publish event
        this->event_array_pub->publish(msg);
        next_send_time = time_curr + delta_time;
        msg.reset();
    }    
}

void Event::renderer()
{
    // use color renderer
    cv_image.encoding = "bgr8";

    if (last_image.rows == height && last_image.cols == width)
    {
        cv::Mat last_image_;
        cv::cvtColor(last_image, last_image_, CV_GRAY2BGR);
        last_image_.copyTo(cv_image.image);
    }
    else
    {
        cv_image.image = cv::Mat(height, width, CV_8UC3);
        cv_image.image = cv::Scalar(0, 0, 0);
    }

    for (int i = 0; i < msg->events.size(); ++i)
    {
        const int x = msg->events[i].x;
        const int y = msg->events[i].y;

        cv_image.image.at<cv::Vec3b>(cv::Point(x, y)) = (msg->events[i].polarity == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
    }
}

void Event::setCurrTime(double time){
    this->time_curr = time;
}

void Event::setEventPublisher(ros::Publisher *pub){
    this->event_array_pub = pub;
}

void Event::setDvsPublisher(ros::Publisher *pub){
    this->dvs_image_pub = pub;
}

void Event::setImage(const cv::Mat &image){
    image.copyTo(this->last_image);
}

void Event::reset(){
    next_send_time = 0.0;
    count = 0;
    seq = 0;
    isRunning = true;
}