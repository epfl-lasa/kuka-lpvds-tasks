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
#include "lwr_ros_client/kuka_action_console.h"


int main(int argc, char** argv)
{

    ros::init(argc, argv,"action_console");
    ros::NodeHandle nh("action_console_node");

    ac::Action_client_console client_console(nh);


    client_console.AddConsoleCommand("go_home");
    client_console.AddConsoleCommand("go_right");
    client_console.AddConsoleCommand("go_write");
    client_console.AddConsoleCommand("go_left");
    client_console.AddConsoleCommand("go_candle");
    client_console.AddConsoleCommand("go_center");
    client_console.AddConsoleCommand("go_center_up");
    client_console.AddConsoleCommand("go_top_center");

    client_console.AddConsoleCommand("linear");

    client_console.start();

      ros::Rate rate(50);
      while(ros::ok()){

          client_console.ConsoleUpdate();

          rate.sleep();
          ros::spinOnce();
      }

    return 0;
}
