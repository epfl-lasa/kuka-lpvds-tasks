#include <ros/ros.h>
#include "lwr_ros_client/kuka_action_console.h"


int main(int argc, char** argv)
{

    ros::init(argc, argv,"action_console");
    ros::NodeHandle nh("action_console_node");

    ac::Action_client_console client_console(nh);


    client_console.AddConsoleCommand("go_home");
    client_console.AddConsoleCommand("go_right");
    client_console.AddConsoleCommand("go_left");
    client_console.AddConsoleCommand("go_candle");
    client_console.AddConsoleCommand("go_top_center");
    client_console.AddConsoleCommand("go_top_left");
        client_console.AddConsoleCommand("go_top_farleft");
    client_console.AddConsoleCommand("go_center");
    client_console.AddConsoleCommand("go_bottom_center");
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
