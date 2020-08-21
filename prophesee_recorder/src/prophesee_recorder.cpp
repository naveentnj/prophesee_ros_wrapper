#include <chrono>  // for high_resolution_clock
#include <thread>
#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <hdf5/serial/H5Cpp.h>

#include "prophesee_recorder.h"


DEFINE_string(output_root, "", "");
DEFINE_string(subfolder, "", "");


namespace prophesee_recorder
{

PropheseeRecorder::~PropheseeRecorder()
{
    // trigger data
    std::vector<uint32_t> trigger_t_data(trigger_events_.size());
    std::vector<uint8_t> trigger_p_data(trigger_events_.size());

    for (int i=0; i<trigger_events_.size(); i++)
    {
        // do stuff about files
        trigger_t_data[i+0] = (trigger_events_[i].ts.toSec()-first_trigger_timestamp_.toSec())*1e6;
        trigger_p_data[i+0] = int(trigger_events_[i].polarity);
    }

    // write data into hdf5
    auto* file = new H5::H5File(event_subroot_folder_ + "/triggers.h5", H5F_ACC_TRUNC);
    hsize_t dimsf[1] = {trigger_events_.size()};

    // write t
    auto* dataspace = new H5::DataSpace(1, dimsf);
    auto* dataset = new H5::DataSet(file->createDataSet("t", H5::PredType::NATIVE_UINT32, *dataspace));
    dataset->write(&trigger_t_data[0], H5::PredType::NATIVE_UINT32);

    // write p
    dataspace = new H5::DataSpace(1, dimsf);
    dataset = new H5::DataSet(file->createDataSet("p", H5::PredType::NATIVE_UINT8, *dataspace));
    dataset->write(&trigger_p_data[0], H5::PredType::NATIVE_UINT8);

    delete dataset;
    delete dataspace;
    file->close();
    delete file;
}


PropheseeRecorder::PropheseeRecorder(ros::NodeHandle & nh,
                                     const ros::NodeHandle& nh_private,
                                     const std::string& imu_topic,
                                     const std::string& trigger_topic,
                                     const std::string& event_topic)
                                     : nh_(nh), imu_counter_(0), event_counter_(0), trigger_counter_(0)
{
    // init files
    initFoldersAndFiles(imu_topic,
                        trigger_topic,
                        event_topic);

    event_sub_ = nh_.subscribe(event_topic, 100, &PropheseeRecorder::eventsCallback, this);
    trigger_sub_ = nh_.subscribe(trigger_topic, 100, &PropheseeRecorder::triggerCallback, this);
    imu_sub_ = nh_.subscribe(imu_topic, 100, &PropheseeRecorder::imuCallback, this);

    std::thread worker(&PropheseeRecorder::run, this);
    worker.detach();
}

void PropheseeRecorder::initFoldersAndFiles(const std::string& imu_topic,
                                            const std::string& trigger_topic,
                                            const std::string& event_topic)
{
    // create folders
    std::string root_folder = FLAGS_output_root;

    std::string event_root_folder = root_folder + "/events/";
    std::string imu_root_folder = root_folder + "/imu/";
    
    event_subroot_folder_ = event_root_folder + FLAGS_subfolder;
    imu_subroot_folder_ = imu_root_folder + FLAGS_subfolder;

    event_data_folder_ = event_subroot_folder_ + "/data/";
    imu_data_folder_ = imu_subroot_folder_ + "/data/";

    boost::filesystem::create_directory(root_folder);
    boost::filesystem::create_directory(event_root_folder);
    boost::filesystem::create_directory(imu_root_folder);
    boost::filesystem::create_directory(event_subroot_folder_);
    boost::filesystem::create_directory(imu_subroot_folder_);
    boost::filesystem::create_directory(imu_data_folder_);
    boost::filesystem::create_directory(event_data_folder_);

    VLOG(1) << "Created folders in " << root_folder;
}

void PropheseeRecorder::run()
{
    ros::Rate rate(100);
    while (ros::ok())
    {
        rate.sleep();
        // if trigger_events_[trigger_counter_] is no longer 
        // the last trigger, package and increment counter
        if (trigger_events_.size()<2)
            continue;

        // for the first trigger, set the first timestamp
        // and measure all subsequent timestamps relative to this
        if (trigger_counter_ == 0)
            first_trigger_timestamp_ = trigger_events_.front().ts;

        while (!(events_.empty() || imu_msgs_.empty() ||
               events_[events_.size()-1].ts < trigger_events_[trigger_counter_+1].ts ||
               imu_msgs_[imu_msgs_.size()-1].header.stamp < trigger_events_[trigger_counter_+1].ts||
               trigger_counter_+1>=trigger_events_.size()))
        {
            ros::Time trigger_timestamp_first = trigger_events_[trigger_counter_].ts;
            ros::Time trigger_timestamp_last = trigger_events_[trigger_counter_+1].ts;

            VLOG(2) << "Processing data between t0=" << trigger_timestamp_first.toNSec() - first_trigger_timestamp_.toNSec()
                               << " and t1=" << trigger_timestamp_last.toNSec() - first_trigger_timestamp_.toNSec();

            int num_imu_msgs = processImuMsgs(trigger_timestamp_first, trigger_timestamp_last);
            int num_event_msgs = processEvents(trigger_timestamp_first, trigger_timestamp_last);

            VLOG(1) << "Processed " << num_imu_msgs << " imu messages and " << num_event_msgs << " events.";

            trigger_counter_++;
        }
    }
}

int PropheseeRecorder::processImuMsgs(ros::Time t0, ros::Time t1)
{    
    std::lock_guard<std::mutex> lock(imu_mutex_);

    // if the most recent imu msg is before t1, wait for more messages
    if (imu_msgs_.empty() || imu_msgs_[imu_msgs_.size()-1].header.stamp < t1)
        return 0;

    // remove all msgs that are before t0 (needs to be done once in the beginning)
    while (!imu_msgs_.empty() && imu_msgs_.front().header.stamp <= t0)
        imu_msgs_.pop_front();

    VLOG(3) << "Imu messages range from t0=" << imu_msgs_.front().header.stamp.toNSec() - first_trigger_timestamp_.toNSec()
            << " and t1=" << imu_msgs_[imu_msgs_.size()-1].header.stamp.toNSec() - first_trigger_timestamp_.toNSec();

    std::vector<uint32_t> time_data;
    std::vector<double> linear_acceleration_data;
    std::vector<double> angular_velocity_data;

    // process all imu msgs uf to t1
    unsigned long num_imu_msgs = 0;

    while (!imu_msgs_.empty() && imu_msgs_.front().header.stamp < t1)
    {
        sensor_msgs::Imu& m = imu_msgs_.front();

        // write data into array
        uint32_t timestamp = (m.header.stamp.toSec() - first_trigger_timestamp_.toSec())*1e6;
        time_data.push_back(timestamp);

        angular_velocity_data.push_back(m.angular_velocity.x);
        angular_velocity_data.push_back(m.angular_velocity.y);
        angular_velocity_data.push_back(m.angular_velocity.z);

        linear_acceleration_data.push_back(m.linear_acceleration.x);
        linear_acceleration_data.push_back(m.linear_acceleration.y);
        linear_acceleration_data.push_back(m.linear_acceleration.z);

        imu_msgs_.pop_front();

        num_imu_msgs++;
    }
    // write data into hdf5
    char buffer[11];
    std::snprintf(buffer, sizeof(buffer), "%010d", imu_counter_);
    auto* file = new H5::H5File(imu_data_folder_ + std::string(buffer) + ".h5", H5F_ACC_TRUNC);
    imu_counter_++;

    // write t
    hsize_t dimsf[1] = {num_imu_msgs};
    auto* dataspace1 = new H5::DataSpace(1, dimsf);
    auto* dataset1 = new H5::DataSet(file->createDataSet("t", H5::PredType::NATIVE_UINT32, *dataspace1));
    dataset1->write(&time_data[0], H5::PredType::NATIVE_UINT32);

    dataset1->close();
    dataspace1->close();
    delete dataspace1;
    delete dataset1;

    // write angular velocity
    hsize_t dimsf3[2] = {num_imu_msgs, 3};
    auto* dataspace3 = new H5::DataSpace(2, dimsf3);
    auto* dataset3 = new H5::DataSet(file->createDataSet("angular_velocity", H5::PredType::NATIVE_DOUBLE, *dataspace3));
    dataset3->write(&angular_velocity_data[0], H5::PredType::NATIVE_DOUBLE);
    dataspace3->close();
    dataset3->close();

    // write angular velocity
    dataspace3 = new H5::DataSpace(2, dimsf3);
    dataset3 = new H5::DataSet(file->createDataSet("linear_acceleration", H5::PredType::NATIVE_DOUBLE, *dataspace3));
    dataset3->write(&linear_acceleration_data[0], H5::PredType::NATIVE_DOUBLE);

    file->close();
    dataspace3->close();
    dataset3->close();

    delete dataspace3;
    delete file;

    return num_imu_msgs;
}

int PropheseeRecorder::processEvents(ros::Time t0, ros::Time t1)
{ 
    std::lock_guard<std::mutex> lock(event_mutex_);

    // if the most recent event before t1, wait for more events
    if (events_.empty() || events_[events_.size()-1].ts < t1)
        return 0;

    // remove all events that are before t0 (needs to be done once in the beginning)
    while (!events_.empty() && events_.front().ts <= t0)
        events_.pop_front();

    VLOG(3) << "Events range from t0=" << events_.front().ts.toNSec() - first_trigger_timestamp_.toNSec()
            << " and t1=" << events_[events_.size()-1].ts.toNSec() - first_trigger_timestamp_.toNSec();

    std::vector<uint32_t> event_t_data;
    std::vector<uint8_t> event_p_data;
    std::vector<uint16_t> event_xy_data;
    
    unsigned long num_events = 0;

    while (!events_.empty() && events_.front().ts < t1)
    {
        prophesee_event_msgs::Event& e = events_.front();

        // write data into array
        uint32_t timestamp = (e.ts.toSec() - first_trigger_timestamp_.toSec())*1e6;
        event_t_data.push_back(timestamp);

        event_xy_data.push_back(e.x);
        event_xy_data.push_back(e.y);

        event_p_data.push_back(int(e.polarity));

        events_.pop_front();

        num_events++;
    }

    // do stuff about saving file
    char buffer[11];
    std::snprintf(buffer, sizeof(buffer), "%010d", event_counter_);
    auto* file = new H5::H5File(event_data_folder_ + std::string(buffer) + ".h5", H5F_ACC_TRUNC);
    event_counter_++;

    // write t
    hsize_t dimsf[1] = {num_events};
    auto* dataspace1 = new H5::DataSpace(1, dimsf);
    auto* dataset1 = new H5::DataSet(file->createDataSet("t", H5::PredType::NATIVE_UINT32, *dataspace1));
    dataset1->write(&event_t_data[0], H5::PredType::NATIVE_UINT32);

    dataspace1->close();
    dataset1->close();
    delete dataspace1;
    delete dataset1;

    // write xy
    hsize_t dimsf2[2] = {num_events, 2};
    auto* dataspace2 = new H5::DataSpace(2, dimsf2);
    auto* dataset2 = new H5::DataSet(file->createDataSet("xy", H5::PredType::NATIVE_UINT16, *dataspace2));
    dataset2->write(&event_xy_data[0], H5::PredType::NATIVE_UINT16);

    dataspace2->close();
    dataset2->close();
    delete dataspace2;
    delete dataset2;

    // write p
    auto* dataspace3 = new H5::DataSpace(2, dimsf);
    auto* dataset3 = new H5::DataSet(file->createDataSet("p", H5::PredType::NATIVE_UINT8, *dataspace3));
    dataset3->write(&event_p_data[0], H5::PredType::NATIVE_UINT8);

    dataspace3->close();
    dataset3->close();
    file->close();

    delete dataspace3;
    delete dataset3;
    delete file;

    return num_events;
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
