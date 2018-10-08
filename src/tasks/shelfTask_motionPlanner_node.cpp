/*
 * Copyright (C) 2018 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author:  Nadia Figueroa
 * email:   nadia.figueroafernandez@epfl.ch
 * website: lasa.epfl.ch
 *
 * This work was supported by the EU project Cogimon H2020-ICT-23-2014.
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "ros/ros.h"
#include "shelfTaskMotionPlanner.h"
#include "lwr_ros_client/action_client_cmd_interface.h"
#include <vector>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "shelfTask_motionPlanner_node");

  ros::NodeHandle nh;
  double frequency = 500.0;
  ros::ServiceClient joint_cmd_client = nh.serviceClient<lwr_ros_client::String_cmd>("/action_client/kuka_action_cmd");

  // Parameters
  std::string          input_pose_topic_name;
  std::string          input_ds1_topic_name;
  std::string          input_ds2_topic_name;
  std::string          output_vel_topic_name;
  std::string          output_pick_topic_name;
  std::string          output_place_topic_name;
  std::vector<double>  attractors_pick;
  std::vector<double>  attractors_place;
  bool                 sim(false);


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
    ROS_ERROR("Couldn't retrieve the topic name for the velocity output. ");
    // return -1;
  }

  if (!nh.getParam("output_pick_topic_name", output_pick_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the output of pick topic. ");
    // return -1;
  }

  if (!nh.getParam("output_place_topic_name", output_place_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the output of place topic. ");
    // return -1;
  }

  if (!nh.getParam("attractors_pick", attractors_pick))   {
    ROS_ERROR("Couldn't retrieve the topic name for the output. ");
    // return -1;
  }

  if (!nh.getParam("attractors_place", attractors_place))   {
    ROS_ERROR("Couldn't retrieve the topic name for the output. ");
    // return -1;
  }

  if (!nh.getParam("sim", sim))   {
    ROS_ERROR("Couldn't retrieve sim value. ");
    // return -1;
  }

  /* Before starting the node go to good joint-space task */
  lwr_ros_client::String_cmd joint_srv;
  joint_srv.request.cmd = "go_center";
  joint_cmd_client.call(joint_srv);
  ros::Duration(5.0).sleep(); // wait


  int num_picks = (int)(attractors_pick.size()/3);
  shelfTaskMotionPlanner* shelfTaskMotionPlanner_;
  std::vector<double> attractor_pick; attractor_pick.resize(3);
  std::vector<double> attractor_place; attractor_place.resize(3);
  for(int p=0; p<num_picks; p++){
      for(unsigned int i=0; i<3; i++){
          attractor_pick[i] = attractors_pick[i + p*3];
          attractor_place[i] = attractors_place[i + p*3];
      }

      shelfTaskMotionPlanner_ = new shelfTaskMotionPlanner(nh, frequency,
                                                           input_pose_topic_name,
                                                           input_ds1_topic_name,
                                                           input_ds2_topic_name,
                                                           output_vel_topic_name,
                                                           output_pick_topic_name,
                                                           output_place_topic_name,
                                                           attractor_pick,
                                                           attractor_place,
                                                           sim);
      /* Run Motion Planning Class */
      if (!shelfTaskMotionPlanner_->Init())
          return -1;
      else
          shelfTaskMotionPlanner_->Run();

      /* Send robot to go_center joint command */
      joint_srv.request.cmd = "go_center";
      joint_cmd_client.call(joint_srv);
      ros::Duration(5.0).sleep(); // wait

      /* Send robot to go_center joint command */
      joint_srv.request.cmd = "go_center_up";
      joint_cmd_client.call(joint_srv);
      ros::Duration(5.0).sleep(); // wait

      /* Send robot to go_center joint command */
      joint_srv.request.cmd = "go_center_up";
      joint_cmd_client.call(joint_srv);
      ros::Duration(5.0).sleep(); // wait
  }



  /* Shutdown node after doing all this */
  ros::shutdown();

  return 0;
}
