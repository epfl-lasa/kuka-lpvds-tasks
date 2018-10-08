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

#include "kuka_action_server/action_server.h"
#include "kuka_action_client/ros_param_parser.h"

#include "kuka_common_action_server/kuka_goto_cart_as.h"
#include "kuka_common_action_server/kuka_grav_as.h"

#include "planning/simple_actions/linear_cart_action.h"


/**
 *  Server Action node
 *
 *  The server action node holds the non-real time controller implementations.
 *  These monitor and send commands to the real time controllers.
 *
 *  The action servers (control policies) might send desired cartesian positions,
 *  velocity, joint positions, joint stiffness values etc.. to real time controllers.
 *
 *
 */

int main(int argc, char** argv)
{

    ros::init(argc, argv, "kuka_server");
    ros::NodeHandle nh("kuka_server");

    // ----------- Get parameters (parameter sever) ------

    std::string node_name = ros::this_node::getName();
    std::map<std::string,std::string> param_name_value;

    if(!pps::parser_string(nh,param_name_value)){
        ROS_ERROR("failed to parse all parameters!");
        return -1;
    }
    pps::parser_print(param_name_value);
    std::string action_server_name  =  param_name_value[node_name + "/action_server_name"];


    /**  ------------- Initialise Action Server ------------- **/
    ///  The action server class handles the changes between different action servers.
    asrv::Action_server action_server(nh,action_server_name);


    /**  ------------- Initialise your action  -------------
    *
    * This is an action server which encodes a simple policy. It performs
    * a left to right translational motion.
    * You can define your own policies (action servers) and make sure
    * to inherit Base_action_server.h.
    *
    */
    simple_actions::Linear_cart_action linear_cart_action(nh);



    /**  ------------- Push back policies -------------
     *
     *  Here we regiser a previously initialised polices with the action server.
     *  The second argument, string, is the TYPE tag of this policy and should
     *  match goal.action_type defined in the action client (see goal definitions
     *  in client_action_node.cpp).
     *
     **/
    action_server.push_back(&linear_cart_action,"linear");


    ROS_INFO("action SERVER started");
    ros::spin();

	return 0;
}
