#include "sinkTaskMotionPlanner.h"

sinkTaskMotionPlanner::sinkTaskMotionPlanner(ros::NodeHandle &n,
                                     double frequency,
                                     std::string input_pose_topic_name,
                                     std::string input_ds1_topic_name,
                                     std::string input_ds2_topic_name,
                                     std::string output_vel_topic_name,
                                     std::string motion_phase_name)
	: nh_(n),
	  loop_rate_(frequency),
	  input_pose_topic_name_(input_pose_topic_name),
	  input_ds1_topic_name_ (input_ds1_topic_name),
	  input_ds2_topic_name_ (input_ds2_topic_name),
      output_vel_topic_name_ (output_vel_topic_name),
      motion_phase_name_(motion_phase_name){

	ROS_INFO_STREAM("Sink-Task Motion Planning node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}

sinkTaskMotionPlanner::~sinkTaskMotionPlanner(){
    ROS_INFO_STREAM("In destructor.. motion generator was killed! ");

}
bool sinkTaskMotionPlanner::Init() {

    real_pose_.resize(3);    real_pose_.setZero();
    ds1_velocity_.resize(3); ds1_velocity_.setZero();
    ds2_velocity_.resize(3); ds1_velocity_.setZero();
    desired_velocity_.resize(3); desired_velocity_.setZero();

	if (!InitializeROS()) {
		ROS_ERROR_STREAM("ERROR intializing the DS");
		return false;
	}

	return true;
}

bool sinkTaskMotionPlanner::InitializeROS() {

	/* Initialize subscribers */
    sub_real_pose_              = nh_.subscribe( input_pose_topic_name_ , 1000, &sinkTaskMotionPlanner::UpdateRealPosition, this, ros::TransportHints().reliable().tcpNoDelay());
    sub_ds1_twist_              = nh_.subscribe( input_ds1_topic_name_  , 1000, &sinkTaskMotionPlanner::UpdateDS1Velocity, this, ros::TransportHints().reliable().tcpNoDelay());
    sub_ds2_twist_              = nh_.subscribe( input_ds2_topic_name_, 1000, &sinkTaskMotionPlanner::UpdateDS2Velocity, this, ros::TransportHints().reliable().tcpNoDelay());


	/* Initialize publishers */
    pub_desired_twist_          = nh_.advertise<geometry_msgs::Twist>(output_vel_topic_name_, 1);    

	if (nh_.ok()) { // Wait for poses being published
		ros::spinOnce();
		ROS_INFO("The Sink Task planner is ready.");
		return true;
	}
	else {
		ROS_ERROR("The ros node has a problem.");
		return false;
	}
}


void sinkTaskMotionPlanner::Run() {

	while (nh_.ok()) {

        ComputeDesiredVelocity();
        PublishDesiredVelocity();
		ros::spinOnce();

		loop_rate_.sleep();
	}
    nh_.shutdown();
}

void sinkTaskMotionPlanner::UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg) {

	msg_real_pose_ = *msg;

	real_pose_(0) = msg_real_pose_.position.x;
	real_pose_(1) = msg_real_pose_.position.y;
	real_pose_(2) = msg_real_pose_.position.z;
}

void sinkTaskMotionPlanner::UpdateDS1Velocity(const geometry_msgs::Twist::ConstPtr& msg) {

	msg_ds1_twist_ = *msg;

    ds1_velocity_(0) = msg_ds1_twist_.linear.x;
    ds1_velocity_(1) = msg_ds1_twist_.linear.y;
    ds1_velocity_(2) = msg_ds1_twist_.linear.z;

}

void sinkTaskMotionPlanner::UpdateDS2Velocity(const geometry_msgs::Twist::ConstPtr& msg) {

	msg_ds2_twist_ = *msg;

    ds2_velocity_(0) = msg_ds2_twist_.linear.x;
    ds2_velocity_(1) = msg_ds2_twist_.linear.y;
    ds2_velocity_(2) = msg_ds2_twist_.linear.z;
}


/* This function does the planning logic for the task!! */
void sinkTaskMotionPlanner::ComputeDesiredVelocity() {

	mutex_.lock();

	// if (std::isnan(desired_velocity_.Norm2())) {
	// 	ROS_WARN_THROTTLE(1, "DS is generating NaN. Setting the output to zero.");
	// 	desired_velocity_.Zero();
	// }

	// desired_velocity_ = desired_velocity_ * scaling_factor_;

	// if (desired_velocity_.Norm() > ds_vel_limit_) {
	// 	desired_velocity_ = desired_velocity_ / desired_velocity_.Norm() * ds_vel_limit_;
	// }

 //    pos_error_ = (real_pose_ - target_pose_ - target_offset_).Norm2();
 //    ROS_WARN_STREAM_THROTTLE(1, "Distance to attractor:" << pos_error_);
 //    if (pos_error_ < 1e-4){
 //        ROS_WARN_STREAM_THROTTLE(1, "[Attractor REACHED] Distance to attractor:" << pos_error_);
 //    }

    switch (hashit(motion_phase_name_)){
    case ePick:
        ROS_WARN_STREAM_THROTTLE(1, "Doing PICKING motion");
        desired_velocity_ = ds1_velocity_;
        break;

    case eSink:
        ROS_WARN_STREAM_THROTTLE(1, "Doing SINK motion");
        desired_velocity_ = ds2_velocity_;
        break;
    }

    /* Filling in desired velocity message */
    msg_desired_velocity_.linear.x  = desired_velocity_(0);
    msg_desired_velocity_.linear.y  = desired_velocity_(1);
    msg_desired_velocity_.linear.z  = desired_velocity_(2);
    msg_desired_velocity_.angular.x = 0;
    msg_desired_velocity_.angular.y = 0;
    msg_desired_velocity_.angular.z = 0;
    ROS_WARN_STREAM_THROTTLE(1, "Desired Velocities:" << desired_velocity_(0) << " " << desired_velocity_(1) << " " << desired_velocity_(2));

    mutex_.unlock();

}


void sinkTaskMotionPlanner::PublishDesiredVelocity() {
    ROS_WARN_STREAM_THROTTLE(1,"PUBLISIHIIIING");
	pub_desired_twist_.publish(msg_desired_velocity_);

}
