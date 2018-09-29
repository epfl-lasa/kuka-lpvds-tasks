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
      des_position  =  {{-0.838, 0.614, 0.388, -1.072, -0.1988, 1.478, -0.606}};
      joint_go_right.set_joint_values(des_position,ac::Joint_action::MESSAGE_TYPE::JOINT_POSITION);
      joint_go_right.debug_print = true;
      actions["go_right"] = &joint_go_right;
      
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

      ac::Joint_action joint_go_top_center(nh);         
      des_position  =  {{-0.842, 0.903, 1.479, -1.180, 0.218, 0.375, -1.11}};            
      joint_go_top_center.set_joint_values(des_position,ac::Joint_action::MESSAGE_TYPE::JOINT_POSITION);
      joint_go_top_center.debug_print = true;
      actions["go_top_center"]        = &joint_go_top_center;  
      
      ac::Joint_action joint_go_top_farleft(nh);         
      des_position  =  {{-0.706, 0.49, 1.509, -1.622, 0.551, -0.075, -1.04}};                
      joint_go_top_farleft.set_joint_values(des_position,ac::Joint_action::MESSAGE_TYPE::JOINT_POSITION);
      joint_go_top_farleft.debug_print = true;
      actions["go_top_farleft"]        = &joint_go_top_farleft;  

      ac::Joint_action joint_go_top_left(nh);         
      des_position  =  {{-0.896, 0.63, 1.536, -1.562, 0.773, 0.137, -1.426}};            
      joint_go_top_left.set_joint_values(des_position,ac::Joint_action::MESSAGE_TYPE::JOINT_POSITION);
      joint_go_top_left.debug_print = true;
      actions["go_top_left"]        = &joint_go_top_left;  

      ac::Joint_action joint_go_center(nh);          
      des_position  =  {{-1.677, 1.385, 1.771, -1.923, -0.20, 0.585, -1.0438}};            
      joint_go_center.set_joint_values(des_position,ac::Joint_action::MESSAGE_TYPE::JOINT_POSITION);
      joint_go_center.debug_print = true;
      actions["go_center"]        = &joint_go_center;  

      ac::Joint_action joint_go_bottom_center(nh);         
      des_position  =  {{0.217, 1.649, 0.485, -0.707, -1.801, -1.0479, 0.713}};
      joint_go_bottom_center.set_joint_values(des_position,ac::Joint_action::MESSAGE_TYPE::JOINT_POSITION);
      joint_go_bottom_center.debug_print = true;
      actions["go_bottom_center"]        = &joint_go_bottom_center;  


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
