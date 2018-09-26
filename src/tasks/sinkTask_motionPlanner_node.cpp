#include "ros/ros.h"
#include "sinkTaskMotionPlanner.h"
#include <vector>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "sinkTask_motionPlanner_node");

  ros::NodeHandle nh;
  double frequency = 500.0;

  // Parameters
  std::string input_pose_topic_name;
  std::string input_ds1_topic_name;
  std::string input_ds2_topic_name;
  std::string output_vel_topic_name;
  

  if (!nh.getParam("input_pose_topic_name", input_pose_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the input. ");
    // return -1;
  }

  if (!nh.getParam("input_ds1_topic_name", input_ds1_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the input. ");
    // return -1;
  }

  if (!nh.getParam("input_ds2_topic_name", input_ds2_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the input. ");
    // return -1;
  }

  if (!nh.getParam("output_vel_topic_name", output_vel_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the output. ");
    // return -1;
  }


  sinkTaskMotionPlanner sinkTask_MotionPlanner_(nh, frequency,
                                         input_pose_topic_name,
                                         input_ds1_topic_name,
                                         input_ds2_topic_name,
                                         output_vel_topic_name);
  
  if (!sinkTask_MotionPlanner_.Init()) 
    return -1;
  else 
    sinkTask_MotionPlanner_.Run();

  return 0;
}
