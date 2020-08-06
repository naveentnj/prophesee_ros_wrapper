#include <chrono>  // for high_resolution_clock
#include <thread>
#include <boost/filesystem.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <cnpy.h>

#include "prophesee_recorder.h"


DEFINE_uint64(event_packet_size, 100000, "");
DEFINE_uint64(imu_packet_size, 1000, "");
DEFINE_string(output_root, "", "");


namespace prophesee_recorder
{

PropheseeRecorder::~PropheseeRecorder()
{
    // process rest of imu data
    std::vector<uint32_t> time_data(imu_msgs_.size());
    std::vector<float> orientation_data(4*imu_msgs_.size());
    std::vector<float> linear_velocity_data(3*imu_msgs_.size());
    std::vector<float> angular_velocity_data(3*imu_msgs_.size());

    int64_t time = imu_msgs_[0].header.stamp.toNSec();
    imu_timestamps_file_ << time << std::endl;
    
    for (int i=0; i<imu_msgs_.size(); i++)
    {
        // write data into array
        time_data[i+0] = imu_msgs_[i].header.stamp.toNSec()-time;

        orientation_data[4*i+0] = imu_msgs_[i].orientation.x;
        orientation_data[4*i+1] = imu_msgs_[i].orientation.y;
        orientation_data[4*i+2] = imu_msgs_[i].orientation.z;
        orientation_data[4*i+3] = imu_msgs_[i].orientation.w;

        angular_velocity_data[3*i+0] = imu_msgs_[i].angular_velocity.x;
        angular_velocity_data[3*i+1] = imu_msgs_[i].angular_velocity.y;
        angular_velocity_data[3*i+2] = imu_msgs_[i].angular_velocity.z;

        linear_velocity_data[3*i+0] = imu_msgs_[i].linear_acceleration.x;
        linear_velocity_data[3*i+1] = imu_msgs_[i].linear_acceleration.y;
        linear_velocity_data[3*i+2] = imu_msgs_[i].linear_acceleration.z;

        std::string output_file_name = imu_data_folder_ + "imu" + std::to_string(imu_counter_) + ".npz"; 
        cnpy::npz_save(output_file_name, "t", &time_data[0], {imu_msgs_.size()}, "w");
        cnpy::npz_save(output_file_name, "orientation", &orientation_data[0], {imu_msgs_.size(),4}, "a");
        cnpy::npz_save(output_file_name, "angular_velocity", &angular_velocity_data[0], {imu_msgs_.size(), 3}, "a");
        cnpy::npz_save(output_file_name, "linear_velocity", &linear_velocity_data[0], {imu_msgs_.size(), 3}, "a");
    }

    // trigger data
    for (int i=0; i<trigger_events_.size(); i++)
    {
        // do stuff about files 
        trigger_file_ << trigger_events_[i].ts.toNSec() << " " 
                      << trigger_events_[i].x << " " 
                      << trigger_events_[i].y << " " 
                      << int(trigger_events_[i].polarity) << std::endl;
    } 

    // event data
    std::vector<uint32_t> event_t_data(events_.size());
    std::vector<uint8_t> event_p_data(events_.size());
    std::vector<uint16_t> event_xy_data(2*events_.size());
    
    VLOG(3) << "Processing event packets found " << events_.size() << " events.";

    time = events_[0].ts.toNSec();
    event_timestamps_file_ << time << std::endl;

    for (int i=0; i<events_.size(); i++)
    {
        // write data into array
        event_t_data[i+0] = events_[i].ts.toNSec()-time;

        event_xy_data[2*i+0] = events_[i].x;
        event_xy_data[2*i+1] = events_[i].y;

        event_p_data[i+0] = int(events_[i].polarity);
    } 

    std::string output_file_name = event_data_folder_ + "events" + std::to_string(event_counter_) + ".npz"; 
    cnpy::npz_save(output_file_name, "t", &event_t_data[0], {events_.size()}, "w");
    cnpy::npz_save(output_file_name, "xy", &event_xy_data[0], {events_.size(),2}, "a");
    cnpy::npz_save(output_file_name, "p", &event_p_data[0], {events_.size()}, "a");
}


PropheseeRecorder::PropheseeRecorder(ros::NodeHandle & nh,
                                 const ros::NodeHandle& nh_private,
                                 const std::string& input_rosbag,
                                 const std::string& imu_topic,
                                 const std::string& trigger_topic,
                                 const std::string& event_topic)
                                 : nh_(nh), imu_counter_(0), event_counter_(0)
{
    // init files 
    initFoldersAndFiles(input_rosbag,
                        imu_topic,
                        trigger_topic,
                        event_topic);

    if (input_rosbag.empty())
    {
        VLOG(1) << "No rosbag was given. listening to the following topic:";
        VLOG(1) << "\tevemts: " << event_topic;
        VLOG(1) << "\ttriggers: " << trigger_topic ;
        VLOG(1) << "\timus: " << imu_topic;

        event_sub_ = nh_.subscribe(event_topic, 100, &PropheseeRecorder::eventsCallback, this);
        trigger_sub_ = nh_.subscribe(trigger_topic, 100, &PropheseeRecorder::triggerCallback, this);
        imu_sub_ = nh_.subscribe(imu_topic, 100, &PropheseeRecorder::imuCallback, this);

        std::thread worker(&PropheseeRecorder::run, this);
        worker.detach();
        return;
    }

    processRosbag(input_rosbag, imu_topic, trigger_topic, event_topic);
}

void PropheseeRecorder::initFoldersAndFiles(const std::string& input_rosbag,
                                          const std::string& imu_topic,
                                          const std::string& trigger_topic,
                                          const std::string& event_topic)
{
    // create folders
    std::string root_folder;
    if (FLAGS_output_root.empty())
    {
        CHECK(!input_rosbag.empty()) << "Either --output_root or --input_rosbag need to be non-empty.";
        root_folder = input_rosbag.substr(0,input_rosbag.find(".bag"));
    }
    else
    {
        root_folder = FLAGS_output_root;
    }

    std::string event_root_folder = root_folder + "/" + replaceSlashWithUnderscore(event_topic);
    std::string imu_root_folder = root_folder + "/" + replaceSlashWithUnderscore(imu_topic);
    std::string trigger_root_folder = root_folder + "/" + replaceSlashWithUnderscore(trigger_topic);

    imu_data_folder_ = imu_root_folder + "/data/";
    event_data_folder_ = event_root_folder + "/data/";

    boost::filesystem::create_directory(root_folder);
    boost::filesystem::create_directory(event_root_folder);
    boost::filesystem::create_directory(imu_root_folder);
    boost::filesystem::create_directory(trigger_root_folder);
    boost::filesystem::create_directory(imu_data_folder_);
    boost::filesystem::create_directory(event_data_folder_);

    // create files
    std::string event_timestamps_file_name = event_root_folder + "/timestamps.txt";
    std::string imu_timestamps_file_name = imu_root_folder + "/timestamps.txt";
    std::string trigger_file_name = trigger_root_folder + "/triggers.txt";

    event_timestamps_file_.open(event_timestamps_file_name);
    imu_timestamps_file_.open(imu_timestamps_file_name);
    trigger_file_.open(trigger_file_name);

    VLOG(1) << "Creating folders and files in " << root_folder;

    CHECK(event_timestamps_file_.is_open());
    CHECK(imu_timestamps_file_.is_open());
    CHECK(trigger_file_.is_open());
}

void PropheseeRecorder::processRosbag(const std::string& input_bag_path,
                                    const std::string& imu_topic,
                                    const std::string& trigger_topic,
                                    const std::string& event_topic)
{
    // input bag open
    rosbag::Bag input_bag;
    try
    {
        input_bag.open(input_bag_path, rosbag::bagmode::Read);
    }
    catch(rosbag::BagIOException &e)
    {
        std::cerr << "Error: could not open rosbag: " << input_bag_path << std::endl;
    }
    
    rosbag::View view(input_bag);    

    const uint32_t num_messages = view.size();
    uint32_t message_index = 0;
    
    for (rosbag::MessageInstance const m : view)
    {
        VLOG(5) << "Message from topic " << m.getTopic();

        // trigger events
        if (m.getTopic() == trigger_topic)
        {
            CHECK(m.getDataType() == "prophesee_event_msgs/Event") << "Trigger Topic message must be of type prophesee_event_msgs/EventArray and is " << m.getDataType();

            boost::shared_ptr<prophesee_event_msgs::Event> m_trigger = m.instantiate<prophesee_event_msgs::Event>();
            triggerCallback(m_trigger);
            processTriggers();
        }

        if (m.getTopic() == event_topic)
        {
            CHECK(m.getDataType() == "prophesee_event_msgs/EventArray") << "Trigger Topic message must be of type prophesee_event_msgs/EventArray and is " << m.getDataType();            
            try
            {
                prophesee_event_msgs::EventArrayConstPtr m_events = m.instantiate<prophesee_event_msgs::EventArray>();
                eventsCallback(m_events);
                processEvents();
            }
            catch(const std::exception& e)
            {
                std::cerr << "Error instantiating events. Topic is " << m.getTopic();
            }            
        }

        if (m.getTopic() == imu_topic)
        {
            const sensor_msgs::ImuConstPtr m_imu = m.instantiate<sensor_msgs::Imu>();
            imuCallback(m_imu);
            processImuMsgs();
        }
    }

    VLOG(1) << "Finished processing!";
}

void PropheseeRecorder::run()
{
    ros::Rate rate(100);
    while (ros::ok())
    {
        processImuMsgs();
        processEvents();
        processTriggers();
        rate.sleep();
    }
}

void PropheseeRecorder::processImuMsgs()
{
    static std::vector<uint32_t> time_data(FLAGS_imu_packet_size);
    static std::vector<float> orientation_data(4*FLAGS_imu_packet_size);
    static std::vector<float> linear_velocity_data(3*FLAGS_imu_packet_size);
    static std::vector<float> angular_velocity_data(3*FLAGS_imu_packet_size);
    
    VLOG(4) << "Processing IMU packets found " << imu_msgs_.size() << " messages.";

    if (imu_msgs_.size()<FLAGS_imu_packet_size)
        return;

    std::lock_guard<std::mutex> lock(imu_mutex_);

    while (imu_msgs_.size()>=FLAGS_imu_packet_size)
    {
        int64_t time = imu_msgs_[0].header.stamp.toNSec();
        imu_timestamps_file_ << time << std::endl;
    
        for (int i=0; i<FLAGS_imu_packet_size; i++)
        {
            sensor_msgs::Imu m = imu_msgs_[i];
                    // write data into array
            time_data[i+0] = m.header.stamp.toNSec()-time;

            orientation_data[4*i+0] = m.orientation.x;
            orientation_data[4*i+1] = m.orientation.y;
            orientation_data[4*i+2] = m.orientation.z;
            orientation_data[4*i+3] = m.orientation.w;

            angular_velocity_data[3*i+0] = m.angular_velocity.x;
            angular_velocity_data[3*i+1] = m.angular_velocity.y;
            angular_velocity_data[3*i+2] = m.angular_velocity.z;

            linear_velocity_data[3*i+0] = m.linear_acceleration.x;
            linear_velocity_data[3*i+1] = m.linear_acceleration.y;
            linear_velocity_data[3*i+2] = m.linear_acceleration.z;
        }

        imu_msgs_.erase(imu_msgs_.begin(), imu_msgs_.begin() + FLAGS_imu_packet_size);

        std::string output_file_name = imu_data_folder_ + "imu" + std::to_string(imu_counter_) + ".npz"; 
        VLOG(2) << "Creating imu packet at " << output_file_name;
        cnpy::npz_save(output_file_name, "t", &time_data[0], {FLAGS_imu_packet_size}, "w");
        cnpy::npz_save(output_file_name, "orientation", &orientation_data[0], {FLAGS_imu_packet_size,4}, "a");
        cnpy::npz_save(output_file_name, "angular_velocity", &angular_velocity_data[0], {FLAGS_imu_packet_size, 3}, "a");
        cnpy::npz_save(output_file_name, "linear_velocity", &linear_velocity_data[0], {FLAGS_imu_packet_size, 3}, "a");

        imu_counter_++;
    } 
}

void PropheseeRecorder::processTriggers()
{
    if (trigger_events_.empty()>0)
        return;

    std::lock_guard<std::mutex> lock(trigger_mutex_);

    while (trigger_events_.size()>0)
    {
        prophesee_event_msgs::Event trigger_event = trigger_events_.front();

        // do stuff about files 
        trigger_file_ << trigger_event.ts.toNSec() << " " 
                      << trigger_event.x << " " 
                      << trigger_event.y << " " 
                      << int(trigger_event.polarity) << std::endl;

        trigger_events_.pop_front();
    } 
}

void PropheseeRecorder::processEvents()
{
    static std::vector<uint32_t> event_t_data(FLAGS_event_packet_size);
    static std::vector<uint8_t> event_p_data(FLAGS_event_packet_size);
    static std::vector<uint16_t> event_xy_data(2*FLAGS_event_packet_size);
    
    VLOG(3) << "Processing event packets found " << events_.size() << " events.";

    if (events_.size()<FLAGS_event_packet_size)
        return;
    
    std::lock_guard<std::mutex> lock(event_mutex_);

    while (events_.size()>=FLAGS_event_packet_size)
    {
        auto start = std::chrono::high_resolution_clock::now();

        int64_t time = events_[0].ts.toNSec();
        event_timestamps_file_ << time << std::endl;
        
        for (int i=0; i<FLAGS_event_packet_size; i++)
        {
            prophesee_event_msgs::Event e = events_[i];

            // write data into array
            event_t_data[i+0] = e.ts.toNSec()-time;

            event_xy_data[2*i+0] = e.x;
            event_xy_data[2*i+1] = e.y;

            event_p_data[i+0] = int(e.polarity);
        }

        float data_seconds = (events_[FLAGS_event_packet_size-1].ts.toNSec() - time)/float(1e9);
        
        events_.erase(events_.begin(), events_.begin() + FLAGS_event_packet_size);

        // do stuff about saving file
        std::string output_file_name = event_data_folder_ + "events" + std::to_string(event_counter_) + ".npz"; 
        VLOG(2) << "Creating event packet at " << output_file_name;
        cnpy::npz_save(output_file_name, "t", &event_t_data[0], {FLAGS_event_packet_size}, "w");
        cnpy::npz_save(output_file_name, "xy", &event_xy_data[0], {FLAGS_event_packet_size,2}, "a");
        cnpy::npz_save(output_file_name, "p", &event_p_data[0], {FLAGS_event_packet_size}, "a");

        event_counter_++;

        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        auto elapsed_seconds = elapsed.count();

        VLOG(1) << "Processed " << FLAGS_event_packet_size / elapsed_seconds / 1000 << " kEv/s";
        VLOG(1) << "Processing at " << data_seconds / elapsed_seconds << " data seconds/second.";
    } 
}

void PropheseeRecorder::imuCallback(const sensor_msgs::ImuConstPtr& m_imu)
{
    std::lock_guard<std::mutex> lock(imu_mutex_);
    sensor_msgs::Imu imu = *m_imu;
    imu_msgs_.push_back(imu);
}

void PropheseeRecorder::eventsCallback(const prophesee_event_msgs::EventArrayConstPtr& m_dvs)
{
    std::lock_guard<std::mutex> lock(event_mutex_);
    prophesee_event_msgs::EventArray event_array = *m_dvs;
    for (const prophesee_event_msgs::Event &e : event_array.events)
        events_.push_back(e);
}

void PropheseeRecorder::triggerCallback(const boost::shared_ptr<prophesee_event_msgs::Event>& m_trigger)
{
    std::lock_guard<std::mutex> lock(trigger_mutex_);
    trigger_events_.push_back(*m_trigger);
}

};
