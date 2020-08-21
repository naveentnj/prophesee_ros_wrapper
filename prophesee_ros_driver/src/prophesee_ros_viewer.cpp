/*******************************************************************
 * File : prophesee_ros_viewer.cpp                                 *
 *                                                                 *
 * Copyright: (c) 2015-2019 Prophesee                              *
 *******************************************************************/

#include "prophesee_ros_viewer.h"

typedef const boost::function< void(const prophesee_event_msgs::EventArray::ConstPtr& msgs)> callback;

PropheseeWrapperViewer::PropheseeWrapperViewer():
    nh_("~"),
    it_(nh_),
    cd_window_name_("CD Events"),
    gl_window_name_("GrayLevel Data"),
    display_acc_time_(5000),
    initialized_(false)
{
    std::string camera_name("");

    // Load Parameters
    nh_.getParam("camera_name", camera_name);
    nh_.getParam("show_cd", show_cd_);
    nh_.getParam("show_graylevels", show_graylevels_);
    nh_.getParam("display_accumulation_time", display_acc_time_);
    const std::string topic_cam_info = "/prophesee/" + camera_name + "/camera_info";
    const std::string topic_cd_event_buffer = "/prophesee/" + camera_name + "/cd_events_buffer";
    const std::string topic_graylevel_buffer = "/prophesee/" + camera_name + "/graylevel_image";

    const std::string publish_topic_rendering = "/prophesee/" + camera_name + "/rendered_events";
    image_transport::ImageTransport it_(nh_);
    rendering_pub_ = it_.advertise(publish_topic_rendering, 1);

    // Subscribe to camera info topic
    sub_cam_info_ = nh_.subscribe(topic_cam_info, 1, &PropheseeWrapperViewer::cameraInfoCallback, this);

    // Subscribe to CD buffer topic
    if (show_cd_) {
        sub_cd_events_ = nh_.subscribe(topic_cd_event_buffer, 2, &PropheseeWrapperViewer::eventsCallback, this);
    }

    // Subscribe to gray-level event buffer topic
    if (show_graylevels_)
        sub_gl_frame_ = it_.subscribe(topic_graylevel_buffer, 1, &PropheseeWrapperViewer::glFrameCallback, this);
}

void PropheseeWrapperViewer::eventsCallback(const prophesee_event_msgs::EventArray::ConstPtr &msgs)
{
    if (rendering_pub_.getNumSubscribers() == 0 || !initialized_)
        return;

    std::unique_lock<std::mutex> lock(event_buffer_mutex_);
    for (auto &e : msgs->events)
    {
        event_buffer_.push_back(e);
        cv::Point pt(e.x,e.y);
        display_image_.at<int>(pt) +=  40*(2*int(e.polarity)-1);
        cv_image_.image.at<uint8_t>(pt) = std::min(display_image_.at<int>(pt), 255);
        

        if (event_buffer_.size()>display_acc_time_)
        {    
            auto &e_last = event_buffer_[0];
            cv::Point pt_last(e_last.x, e_last.y);
            display_image_.at<int>(pt_last) -=  40*(2*int(e_last.polarity)-1);
            cv_image_.image.at<uint8_t>(pt_last) = std::max(display_image_.at<int>(pt_last), 0);
            event_buffer_.pop_front();
        }
    }
    cv_image_.header.stamp = event_buffer_[event_buffer_.size()-1].ts;
}

void PropheseeWrapperViewer::publishCDEventsRendering()
{
    if (event_buffer_.size()<display_acc_time_)
        return;

    std::unique_lock<std::mutex> lock(event_buffer_mutex_);
    rendering_pub_.publish(cv_image_.toImageMsg());
}

PropheseeWrapperViewer::~PropheseeWrapperViewer() {
    nh_.shutdown();
    rendering_pub_.shutdown();

    if (!initialized_)
        return;
}

bool PropheseeWrapperViewer::isInitialized() {
    return initialized_;
}

void PropheseeWrapperViewer::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    if (initialized_)
        return;

    if ((msg->width != 0) && (msg->height != 0))
        init(msg->width, msg->height);
}

void PropheseeWrapperViewer::glFrameCallback(const sensor_msgs::ImageConstPtr& msg) {
    if (!initialized_)
        return;

    try {
        cv::imshow(gl_window_name_, cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_8UC1)->image);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

bool PropheseeWrapperViewer::init(const unsigned int& sensor_width, const unsigned int& sensor_height) {
    cv_image_.image = cv::Mat(sensor_height, sensor_width, CV_8U);
    cv_image_.image = cv::Scalar(127);
    cv_image_.encoding = "mono8";
    display_image_ = cv::Mat(sensor_height, sensor_width, CV_32S);;
    display_image_ = cv::Scalar(127);

    if (show_graylevels_) {
        // Define the display window for gray-level frame
        create_window(gl_window_name_, sensor_width, sensor_height, 0, sensor_height + 50);
    }

    initialized_ = true;

    return true;
}

void PropheseeWrapperViewer::create_window(const std::string &window_name, const unsigned int& sensor_width,
                                           const unsigned int& sensor_height, const int &shift_x, const int &shift_y) {
    cv::namedWindow(window_name, CV_GUI_EXPANDED);
    cv::resizeWindow(window_name, sensor_width, sensor_height);
    // move needs to be after resize on apple, otherwise the window stacks
    cv::moveWindow(window_name, shift_x, shift_y);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "prophesee_ros_viewer");
    PropheseeWrapperViewer wv;

    while(ros::ok() && !wv.isInitialized()) {
        ros::spinOnce();
    }

    ros::Rate loop_rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        wv.publishCDEventsRendering();

        loop_rate.sleep();

    }

    ros::shutdown();

    return 0;
}
