#include <ros/ros.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include "prophesee_recorder.h"

DEFINE_string(imu_topic, "/prophesee/camera/left/imu", 
              "Topic of imu msgs.");
DEFINE_string(event_topic, "/prophesee/camera/left/cd_events_buffer", 
              "Topic of event msgs.");
DEFINE_string(trigger_topic, "/prophesee/camera/left/extTrigger", 
              "Topic of trigger msgs.");


int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "prophesee_recorder");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
 
  prophesee_recorder::PropheseeRecorder prophesee_recorder(nh, nh_private, 
                                                           FLAGS_imu_topic, 
                                                           FLAGS_trigger_topic,
                                                           FLAGS_event_topic);

  ros::spin();

  return 0;
}
