#include "EventBundler.h"
EventBundler::EventBundler(std::string yaml)
{
    // read yaml file
    cv::FileStorage fSettings(yaml, cv::FileStorage::READ);
    if (!fSettings.isOpened())
    {
        throw std::runtime_error(std::string("Could not open file: ") + yaml);
    }

    delta_time = fSettings["delta_time"];
    max_events = fSettings["max_events"];

    msg = dvs_msgs::EventArrayPtr(new dvs_msgs::EventArray());
    next_send_time = delta_time;
}

void EventBundler::GrabEvent(dvs_msgs::EventArrayConstPtr msg)
{
    vec_msg.push_back(msg);
}

void EventBundler::setEventPublisher(ros::Publisher *pub)
{
    this->event_array_pub = pub;
}

void EventBundler::setCurrTime(double time){
    this->time_curr = time;
}

void EventBundler::Publish(){
    if (!msg)
    {
        msg = dvs_msgs::EventArrayPtr(new dvs_msgs::EventArray());
    }
    size_t event_num = 0;
    for(size_t e_iter = 0; e_iter < vec_msg.size(); e_iter++){
        event_num += vec_msg[e_iter]->events.size();
    } 
    if ((time_curr > next_send_time || event_num > max_events )&& vec_msg.size() != 0)
    {
        // publish event
        msg->header = vec_msg[0]->header;
        msg->height = vec_msg[0]->height;
        msg->width = vec_msg[0]->width;
        for(size_t e_iter = 0; e_iter < vec_msg.size(); e_iter++){
            auto cur_msg = vec_msg[e_iter];
            msg->events.insert(msg->events.end(), cur_msg->events.begin(), cur_msg->events.end());
        } 
        this->event_array_pub->publish(msg);
        next_send_time = time_curr + delta_time;
        vec_msg.clear();
        msg.reset();

    }
}