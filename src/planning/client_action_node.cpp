/*
 * Copyright (C) 2016 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author:  Guillaume De Chambrier
 * email:   guillaume.dechambrier@epfl.ch
 * website: lasa.epfl.ch
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


#include <ros/ros.h>

#include "lwr_ros_client/action_client_cmd_interface.h"
#include "lwr_ros_client/kuka_action_client.h"
#include "lwr_ros_client/ros_param_parser.h"
#include "lwr_ros_action/joint_action.h"
#include "planning/simple_actions/linear_cart_action.h"

/**
  *     Client Action node (simple example)
  *
  *     This .cpp file encodes the client ROS node of the action server. It is in this node
  *     that you the name of your actions with associated goal parameters. This list of
  *     of tuples [name,goal] is can the be called via three methods;
  *         1) service       : rosservice call /control_cmd_interface/kuka_cmd 'name'"
  *         2) voice         :
  *         3) cmd interface : terminal
  *
  *     When a name is selected the client contacts the Action server node with [goal] information.
  *     The action server will then proceed to run the action with the goal parameters specified in
  *     [goal].
 */

int main(int argc, char** argv)
{

    ros::init(argc, argv,"action_client");
    ros::NodeHandle nh("action_client");

    std::string node_name = ros::this_node::getName();

    std::map<std::string,std::string> param_name_value;
    param_name_value[node_name + "/speech_topic"]           = "";
    param_name_value[node_name + "/action_service_name"]    = "";
    param_name_value[node_name + "/cmd_service_name"]       = "";
    param_name_value[node_name + "/action_server_name"]     = "";

    if(!pps::Parser::parser_string(nh,param_name_value)){
        ROS_ERROR("failed to parse all parameters!");
        return -1;
    }

    std::string speech_topic          =  param_name_value[node_name + "/speech_topic"];
    std::string action_serivce_name   =  param_name_value[node_name + "/action_service_name"];
    std::string cmd_service_name      =  param_name_value[node_name + "/cmd_service_name"];
    std::string action_server_name    =  param_name_value[node_name + "/action_server_name"];


    /** ------------- Initialise Action Client & Set Action-Goals -------------

      The Simple_action client is initialsed. A set of actions and goals are defined
      add added to the action clients container which is a map. The key of
      the map is the name of the action and the value is the Goal.

    **/

    ac::Kuka_action_client kuka_action_client;
    std::map<std::string,ac::Base_action*> actions;


    /** ------------- Defining goals -------------
     *
     *  The action client registers in a map, goals["action_name"] = goal.
     *  The goal object holds sepcific variables for the policy that will
     *  execut this goal.
     *
     *  For instance goal can hold a target cartesian position, a target
     *  joint position, target stiffness values, etc..
     *
     *  It is important that in the action server node (server_action_node.cpp) there exists a
     *  a policy which has been registered with a type matching that of goal.action_type.
     *
     */
      std::array<double,7> des_position;
    
      ac::Joint_action joint_go_right(nh);      
      des_position  =  {{-1.323, 0.797, 0.990, -1.423, -0.464, 1.469, -0.0944}};
      joint_go_right.set_joint_values(des_position,ac::Joint_action::MESSAGE_TYPE::JOINT_POSITION);
      joint_go_right.debug_print = true;
      actions["go_right"] = &joint_go_right;

      ac::Joint_action joint_go_write(nh);
      des_position  =  {{-2.223, 0.597, 0.990, -1.423, -0.464, 1.469, -0.0944}};
      joint_go_write.set_joint_values(des_position,ac::Joint_action::MESSAGE_TYPE::JOINT_POSITION);
      joint_go_write.debug_print = true;
      actions["go_write"] = &joint_go_write;
      
      ac::Joint_action joint_go_left(nh);   
      des_position  =  {{0.570, 0.693, 0.265, -0.784, -0.257, 1.557, 0.768}};
      joint_go_left.set_joint_values(des_position,ac::Joint_action::MESSAGE_TYPE::JOINT_POSITION);
      joint_go_left.debug_print = true;
      actions["go_left"]        = &joint_go_left;  
      
      ac::Joint_action joint_go_home(nh);
      des_position  =  {{-0.102, 0.226, 0.14, -1.456, -0.025, 1.407, -0.001}};
      joint_go_home.set_joint_values(des_position,ac::Joint_action::MESSAGE_TYPE::JOINT_POSITION);
      joint_go_home.debug_print = true;
      actions["go_home"]        = &joint_go_home;

      ac::Joint_action go_candle(nh);
      des_position  =  {{0,0,0,0,0,0,0}};
      go_candle.set_joint_values(des_position,ac::Joint_action::MESSAGE_TYPE::JOINT_POSITION);
      go_candle.debug_print = true;
      actions["go_candle"]         = &go_candle;

      ac::Joint_action joint_go_center(nh);          
      des_position  =  {{-1.472, 0.651, 0.94, -1.84, -1.9, -0.98, -0.72}};
      joint_go_center.set_joint_values(des_position,ac::Joint_action::MESSAGE_TYPE::JOINT_POSITION);
      joint_go_center.debug_print = true;
      actions["go_center"]        = &joint_go_center;  


      ac::Joint_action joint_go_center_up(nh);
      des_position  =  {{-2.104, 0.679, 1.5903, -1.781, -2.229, -1.0975, -0.106}};
      joint_go_center_up.set_joint_values(des_position,ac::Joint_action::MESSAGE_TYPE::JOINT_POSITION);
      joint_go_center_up.debug_print = true;
      actions["go_center_up"]        = &joint_go_center_up;

      ac::Joint_action joint_go_top_center(nh);
      des_position  =  {{-0.763, 1.00, 1.140, -0.730, -2.225, -0.939, -0.027}};
      joint_go_top_center.set_joint_values(des_position,ac::Joint_action::MESSAGE_TYPE::JOINT_POSITION);
      joint_go_top_center.debug_print = true;
      actions["go_top_center"]        = &joint_go_top_center;


      simple_actions::Linear_cart_action linear_cart_action(nh);
      actions["linear"]         = &linear_cart_action;

    /**
      * Here we register all the goals with the action client. This wil make them available to
      * be called with the three different methods mentioned above (service,voice,cmd interface)
      *
      **/

      kuka_action_client.push_back(actions);


    /**  ------------- Initialise Service, Voice & Cmd interface  -------------
     *  The control command interface is an interface to the action client.
     *  It provied a ros service and a voice command interface such to
     *  command the client server to send desired action requests to the action server.
     */
     ac::Action_client_cmd_interface action_cmd_interface(nh,kuka_action_client,action_serivce_name,cmd_service_name);
     action_cmd_interface.init_nl_subscriber(speech_topic);

     ROS_INFO("action CLIENT started!");
     ros::spin();

    return 0;
}
