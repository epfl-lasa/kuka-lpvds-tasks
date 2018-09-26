#include "sinkTaskMotionPlanner.h"

sinkTaskMotionPlanner::sinkTaskMotionPlanner(ros::NodeHandle &n,
                                           double frequency,
                                           std::string input_pose_topic_name,
                                           std::string input_ds1_topic_name,
                                           std::string input_ds2_topic_name,
                                           std::string output_vel_topic_name,
                                           std::string motion_phase_name,
                                           std::vector<double> &attractor_pick,
                                           std::vector<double> &attractor_sink)
	: nh_(n),
	  loop_rate_(frequency),
	  input_pose_topic_name_(input_pose_topic_name),
	  input_ds1_topic_name_ (input_ds1_topic_name),
	  input_ds2_topic_name_ (input_ds2_topic_name),
      output_vel_topic_name_ (output_vel_topic_name),
      motion_phase_name_(motion_phase_name),
      attractor_pick_(attractor_pick),
      attractor_sink_(attractor_sink),
      num_picks_(2),
      thres_(1e-3){

	ROS_INFO_STREAM("Sink-Task Motion Planning node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}

sinkTaskMotionPlanner::~sinkTaskMotionPlanner(){
    ROS_INFO_STREAM("In destructor.. motion generator was killed! ");

}
bool sinkTaskMotionPlanner::Init() {

    real_pose_.resize(3);        real_pose_.setZero();
    ds1_velocity_.resize(3);     ds1_velocity_.setZero();
    ds2_velocity_.resize(3);     ds1_velocity_.setZero();
    desired_velocity_.resize(3); desired_velocity_.setZero();
    target_pick_.resize(3);      target_pick_.setZero();
    target_sink_.resize(3);      target_sink_.setZero();


    /* Fill in target Vectors*/
    for (unsigned int i=0;i < attractor_pick_.size(); i++){
        target_pick_(i) = attractor_pick_[i];
        target_sink_(i) = attractor_sink_[i];
    }


    ROS_INFO_STREAM("Doing  " << num_picks_ << " picks!" );
    bFirst_ = true;
    picks_ = 0;
    bEnd_ = false;

	if (!InitializeROS()) {
        ROS_ERROR_STREAM("ERROR intializing the ROS NODE");
		return false;
	}

    /* Initializing Gripper */
    gripper_ = new RSGripperInterface(false);
    ROS_INFO("[RSGripperInterfaceTest] resetting");
    gripper_->reset();
    ROS_INFO("[RSGripperInterfaceTest] activating");
    gripper_->activate();
    ros::Duration(1.0).sleep();
    gripper_->setSpeed(250);
    gripper_->setPosition(64);
    ros::Duration(1.0).sleep();

	return true;
}

bool sinkTaskMotionPlanner::InitializeROS() {

	/* Initialize subscribers */
    sub_real_pose_              = nh_.subscribe( input_pose_topic_name_ , 1000, &sinkTaskMotionPlanner::UpdateRealPosition, this, ros::TransportHints().reliable().tcpNoDelay());
    sub_ds1_twist_              = nh_.subscribe( input_ds1_topic_name_  , 1000, &sinkTaskMotionPlanner::UpdateDS1Velocity, this, ros::TransportHints().reliable().tcpNoDelay());
    sub_ds2_twist_              = nh_.subscribe( input_ds2_topic_name_,   1000, &sinkTaskMotionPlanner::UpdateDS2Velocity, this, ros::TransportHints().reliable().tcpNoDelay());


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

    while (nh_.ok() && !bEnd_) {

        ComputeDesiredVelocity();
        PublishDesiredVelocity();
        ros::spinOnce();

        loop_rate_.sleep();

        if (picks_ == num_picks_)
            bEnd_ = true;
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

    Eigen::VectorXd pos_error_;pos_error_.resize(3);
    double target_error_(0.0);


    /* Start with a picking task*/
    if (bFirst_)
        motion_phase_name_ = "pick";

    /* Do either picking or sink */
    switch (hashit(motion_phase_name_)){
    case ePick:
        ROS_WARN_STREAM_THROTTLE(1, "Doing PICKING motion");
        desired_velocity_ = ds1_velocity_;
        pos_error_ = real_pose_ - target_pick_;
        target_error_ = pos_error_.squaredNorm();
        ROS_WARN_STREAM_THROTTLE(1, "Distance to PICKING TARGET" << picks_+1 << ": " << target_error_);

        /* Check of robot has reached target and swicth to next motion*/
        if (target_error_ < thres_){
            ROS_WARN_STREAM_THROTTLE(1, "PICKING TARGET REACHED!!!!.. switching to sink!");

            /* Grasp Cube*/
            gripper_->setPosition(250); // to close

            /*Switch to eSink*/
            motion_phase_name_ = "sink";

            /* Reset initial task boolean */
            if (bFirst_)
                bFirst_=false;
        }

        break;

    case eSink:
        ROS_WARN_STREAM_THROTTLE(1, "Doing SINK motion");
        desired_velocity_ = ds2_velocity_;
        pos_error_ = real_pose_ - target_sink_;
        target_error_ = pos_error_.squaredNorm();
        ROS_WARN_STREAM_THROTTLE(1, "Distance to SINK TARGET:" << target_error_);

        /* Check of robot has reached target and swicth to next motion*/
        if (target_error_ < thres_){
            ROS_WARN_STREAM_THROTTLE(1, "Sink TARGET REACHED!!!!.. switching to pick!");

            /* Release Cube*/
            ros::Duration(0.8).sleep(); // wait
            gripper_->setPosition(0); // to close

            /*Switch to eSink*/
            motion_phase_name_ = "pick";

            picks_++;
        }
        break;
    }

    /* Number of Picks has been achieved, send 0 velocities*/
    if (picks_ == num_picks_)
            desired_velocity_.setZero();

    /* Filling in desired velocity message */
    msg_desired_velocity_.linear.x  = desired_velocity_(0);
    msg_desired_velocity_.linear.y  = desired_velocity_(1);
    msg_desired_velocity_.linear.z  = desired_velocity_(2);
    ROS_WARN_STREAM_THROTTLE(1, "Sent Velocities:" << desired_velocity_(0) << " " << desired_velocity_(1) << " " << desired_velocity_(2));


    mutex_.unlock();

}


void sinkTaskMotionPlanner::PublishDesiredVelocity() {
    msg_desired_velocity_.angular.x = 0;
    msg_desired_velocity_.angular.y = 0;
    msg_desired_velocity_.angular.z = 0;
	pub_desired_twist_.publish(msg_desired_velocity_);

}
