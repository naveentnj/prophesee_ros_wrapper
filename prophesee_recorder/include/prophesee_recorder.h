#pragma once

#include <string>
#include <deque>
#include <fstream>
#include <mutex>

#include <prophesee_event_msgs/Event.h>
#include <prophesee_event_msgs/EventArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <rosbag/bag.h>
#include <iostream>



namespace prophesee_recorder
{
class PropheseeRecorder
{
public:
    PropheseeRecorder(ros::NodeHandle & nh,
                    const ros::NodeHandle& nh_private,
                    const std::string& imu_topic,
                    const std::string& trigger_topic,
                    const std::string& event_topic);
    ~PropheseeRecorder();
    
private:
    void initFoldersAndFiles(const std::string& imu_topic,
                             const std::string& trigger_topic,
                             const std::string& event_topic);

    void run();
    int processImuMsgs(ros::Time t0, ros::Time t1);
    int processEvents(ros::Time t0, ros::Time t1);
    
    void imuCallback(const sensor_msgs::ImuConstPtr& m_imu);
    void eventsCallback(const prophesee_event_msgs::EventArrayConstPtr& m_dvs);
    void triggerCallback(const boost::shared_ptr<prophesee_event_msgs::Event>& m_trigger);

    std::string imu_data_folder_;
    std::string event_data_folder_;
    std::string event_subroot_folder_;
    std::string imu_subroot_folder_;

    std::deque<prophesee_event_msgs::Event> events_;
    std::deque<prophesee_event_msgs::Event> trigger_events_;
    std::deque<sensor_msgs::Imu> imu_msgs_;

    ros::NodeHandle nh_;
    ros::Time first_trigger_timestamp_;
    
    ros::Subscriber event_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber trigger_sub_;

    std::mutex trigger_mutex_, event_mutex_, imu_mutex_;

    int event_counter_;
    int imu_counter_;
    int trigger_counter_;
};
} // prophesee_recorder
