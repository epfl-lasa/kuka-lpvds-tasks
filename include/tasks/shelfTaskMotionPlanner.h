#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

#include <std_msgs/Bool.h>
#include <vector>
#include <mutex>
#include <string>
#include "eigen3/Eigen/Dense"

#include <grasp_interface/rs_gripper_interface.h>

using namespace Eigen;

enum string_code_motion {
    ePick,
    ePlace,
};

string_code_motion hashit_ (std::string const inString) {
    if (inString == "pick") return ePick;
    if (inString == "place") return ePlace;
};


class shelfTaskMotionPlanner {

private:

    // ROS system variables
    ros::NodeHandle           nh_;
    ros::Rate                 loop_rate_;    

    // Publishers/Subscriber/Service Calls
    ros::Subscriber           sub_real_pose_;
    ros::Subscriber           sub_ds1_twist_;
    ros::Subscriber           sub_ds2_twist_;
    ros::Publisher            pub_desired_twist_;
    ros::Publisher            pub_desired_pick_target_;

    // Topic Names
    std::string               input_pose_topic_name_;
    std::string               input_ds1_topic_name_;
    std::string               input_ds2_topic_name_;
    std::string               output_vel_topic_name_;
    std::string               output_pick_topic_name_;
    std::string               motion_phase_name_;

    // Messages
    geometry_msgs::Pose       msg_real_pose_;
    geometry_msgs::Twist      msg_ds1_twist_;
    geometry_msgs::Twist      msg_ds2_twist_;
    geometry_msgs::Twist      msg_desired_velocity_;
    geometry_msgs::Point      msg_desired_pick_target_;

	// Class variables
    std::mutex                mutex_;
    VectorXd                  ds1_target;
    VectorXd                  ds2_target;
    VectorXd                  real_pose_;
    VectorXd                  ds1_velocity_;
    VectorXd                  ds2_velocity_;
    VectorXd                  desired_velocity_;
    std::vector<double>       attractors_pick_;
    std::vector<double>       attractor_place_;
    VectorXd                  *targets_pick_;
    VectorXd                  target_place_;
    bool                      bFirst_;
    bool                      bEnd_;
    int                       num_picks_;
    int                       picks_;
    double                    thres_;
    unsigned int              M_;
    bool                      sim_;

    // Gripper Controller
    RSGripperInterface*        gripper_;


public:
	shelfTaskMotionPlanner(ros::NodeHandle &n,
	                  double frequency,
	                  std::string input_pose_topic_name,
                      std::string input_ds1_topic_name,
                      std::string input_ds2_topic_name,
                      std::string output_vel_topic_name,
                      std::string output_pick_topic_name,
                      std::vector<double> &attractors_pick,
                      std::vector<double> &attractor_place,
                      bool sim);

    ~shelfTaskMotionPlanner(void);

	bool Init();

	void Run();

private:

	bool InitializeROS();

	void UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg);
    
    void UpdateDS1Velocity(const geometry_msgs::Twist::ConstPtr& msg);
    
    void UpdateDS2Velocity(const geometry_msgs::Twist::ConstPtr& msg);

	void ComputeDesiredVelocity();

	void PublishDesiredVelocity();

    void PublishDesiredPickingTarget();

};
