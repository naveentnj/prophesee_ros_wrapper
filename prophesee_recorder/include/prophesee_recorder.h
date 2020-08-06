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
                    const std::string& input_rosbag,
                    const std::string& imu_topic,
                    const std::string& trigger_topic,
                    const std::string& event_topic);
    ~PropheseeRecorder();
    
private:
    static std::string replaceSlashWithUnderscore(std::string topic)
    {
        // remove first /
        topic.replace(0,1,"");
        // replace others with _0
        int pos;
        while ((pos = topic.find("/")) != std::string::npos) {
            topic.replace(pos,1,"_");
        }

        return topic;
    };

    void initFoldersAndFiles(const std::string& input_rosbag,
                             const std::string& imu_topic,
                             const std::string& trigger_topic,
                             const std::string& event_topic);

    void processRosbag(const std::string& input_bag_path,
                       const std::string& imu_topic,
                       const std::string& trigger_topic,
                       const std::string& event_topic);

    void run();
    void processImuMsgs();
    void processTriggers();
    void processEvents();
    
    void imuCallback(const sensor_msgs::ImuConstPtr& m_imu);
    void eventsCallback(const prophesee_event_msgs::EventArrayConstPtr& m_dvs);
    void triggerCallback(const boost::shared_ptr<prophesee_event_msgs::Event>& m_trigger);

    std::string imu_data_folder_;
    std::string event_data_folder_;

    std::ofstream event_timestamps_file_;
    std::ofstream imu_timestamps_file_;
    std::ofstream trigger_file_;

    std::deque<prophesee_event_msgs::Event> events_;
    std::deque<prophesee_event_msgs::Event> trigger_events_;
    std::deque<sensor_msgs::Imu> imu_msgs_;

    ros::NodeHandle nh_;

    ros::Subscriber event_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber trigger_sub_;

    std::mutex trigger_mutex_, event_mutex_, imu_mutex_;

    int event_counter_;
    int imu_counter_;
};
} // prophesee_recorder
