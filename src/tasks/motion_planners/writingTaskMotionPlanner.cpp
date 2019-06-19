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

#include "writingTaskMotionPlanner.h"

writingTaskMotionPlanner::writingTaskMotionPlanner(ros::NodeHandle &n,
                                           double frequency,
                                           std::string input_pose_topic_name,
                                           std::string input_ds1_topic_name,
                                           std::string input_ds2_topic_name,
                                           std::string input_target_topic_name,
                                           std::string output_vel_topic_name,
                                           std::string output_pick_topic_name,
                                           std::string output_write_topic_name,
                                           std::vector<double> &attractors_pick,
                                           std::vector<double> &attractor_write,
                                           bool sim)
	: nh_(n),
	  loop_rate_(frequency),
	  input_pose_topic_name_(input_pose_topic_name),
	  input_ds1_topic_name_ (input_ds1_topic_name),
	  input_ds2_topic_name_ (input_ds2_topic_name),
      input_target_topic_name_(input_target_topic_name),
      output_vel_topic_name_ (output_vel_topic_name),
      output_pick_topic_name_ (output_pick_topic_name),
      output_write_topic_name_ (output_write_topic_name),
      attractors_pick_(attractors_pick),
      attractor_write_(attractor_write),
      thres_(1e-3),
      M_(3), sim_(sim){

    ROS_INFO_STREAM("Writing-Task Motion Planning node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}

writingTaskMotionPlanner::~writingTaskMotionPlanner(){
    ROS_INFO_STREAM("In destructor.. motion generator was killed! ");

}
bool writingTaskMotionPlanner::Init() {

    real_pose_.resize(M_);        real_pose_.setZero();
    ds1_velocity_.resize(M_);     ds1_velocity_.setZero();
    ds2_velocity_.resize(M_);     ds1_velocity_.setZero();
    desired_velocity_.resize(M_); desired_velocity_.setZero();
    target_write_.resize(M_);      target_write_.setZero();
    learned_att_write_.resize(M_); learned_att_write_.setZero();

    /* Fill in target Vector for sink motion*/
    for (unsigned int i=0;i < M_; i++){
        learned_att_write_(i) = attractor_write_[i];
        target_write_(i) = attractor_write_[i];
    }

    /* Fill in targets Vector* for sink motion*/
    num_picks_ = (int) attractors_pick_.size()/M_;
    ROS_INFO_STREAM("Doing  " << num_picks_ << " picks!" );
    targets_pick_  = new VectorXd[num_picks_]; for(int s=0; s<num_picks_; s++ ){targets_pick_[s].resize(M_);	}

    for(int p=0; p < num_picks_; p++ ){
        VectorXd target_pick_; target_pick_.resize(M_); target_pick_.setZero();
        for (unsigned int m = 0; m < M_; m++)
            target_pick_[m] = attractors_pick_[p * 3 + m];
        targets_pick_[p] = target_pick_;
    }

    bFirst_ = true;
    picks_ = 0;
    bEnd_ = false;

	if (!InitializeROS()) {
        ROS_ERROR_STREAM("ERROR intializing the ROS NODE");
		return false;
	}

    /* Initializing Gripper */
    if (!sim_){
        gripper_ = new RSGripperInterface(false);
        ROS_INFO("[RSGripperInterfaceTest] resetting");
        gripper_->reset();
        ROS_INFO("[RSGripperInterfaceTest] activating");
        gripper_->activate();
        ros::Duration(1.0).sleep();
        gripper_->setSpeed(300);
        gripper_->setPosition(64);
        ros::Duration(1.0).sleep();
    }

	return true;
}

bool writingTaskMotionPlanner::InitializeROS() {

	/* Initialize subscribers */
    sub_real_pose_              = nh_.subscribe( input_pose_topic_name_ , 1000, &writingTaskMotionPlanner::UpdateRealPosition, this, ros::TransportHints().reliable().tcpNoDelay());
    sub_ds1_twist_              = nh_.subscribe( input_ds1_topic_name_  , 1000, &writingTaskMotionPlanner::UpdateDS1Velocity, this, ros::TransportHints().reliable().tcpNoDelay());
    sub_ds2_twist_              = nh_.subscribe( input_ds2_topic_name_,   1000, &writingTaskMotionPlanner::UpdateDS2Velocity, this, ros::TransportHints().reliable().tcpNoDelay());
    sub_desired_target_         = nh_.subscribe( input_target_topic_name_ , 1000, &writingTaskMotionPlanner::UpdateDynamicTarget, this, ros::TransportHints().reliable().tcpNoDelay());

	/* Initialize publishers */
    pub_desired_twist_          = nh_.advertise<geometry_msgs::Twist>(output_vel_topic_name_, 1);    
    pub_desired_pick_target_    = nh_.advertise<geometry_msgs::Point>(output_pick_topic_name_, 1);
    pub_desired_write_target_   = nh_.advertise<geometry_msgs::Point>(output_write_topic_name_, 1);

    if (nh_.ok()) {
		ros::spinOnce();
		ROS_INFO("The Writing Task planner is ready.");
		return true;
	}
	else {
		ROS_ERROR("The ros node has a problem.");
		return false;
	}
}


void writingTaskMotionPlanner::Run() {

    while ((nh_.ok()) && (picks_ < num_picks_)){

        ComputeDesiredVelocity();
        PublishDesiredVelocity();
        PublishDesiredPickingTarget();
        PublishDesiredWritingTarget();

        ros::spinOnce();
        loop_rate_.sleep();

    }

    ros::spinOnce();

    loop_rate_.sleep();

    nh_.shutdown();
}

void writingTaskMotionPlanner::UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg) {

	msg_real_pose_ = *msg;

	real_pose_(0) = msg_real_pose_.position.x;
	real_pose_(1) = msg_real_pose_.position.y;
	real_pose_(2) = msg_real_pose_.position.z;
}

void writingTaskMotionPlanner::UpdateDS1Velocity(const geometry_msgs::Twist::ConstPtr& msg) {

	msg_ds1_twist_ = *msg;

    ds1_velocity_(0) = msg_ds1_twist_.linear.x;
    ds1_velocity_(1) = msg_ds1_twist_.linear.y;
    ds1_velocity_(2) = msg_ds1_twist_.linear.z;

}

void writingTaskMotionPlanner::UpdateDS2Velocity(const geometry_msgs::Twist::ConstPtr& msg) {

	msg_ds2_twist_ = *msg;

    ds2_velocity_(0) = msg_ds2_twist_.linear.x;
    ds2_velocity_(1) = msg_ds2_twist_.linear.y;
    ds2_velocity_(2) = msg_ds2_twist_.linear.z;
}


void writingTaskMotionPlanner::UpdateDynamicTarget(const geometry_msgs::Point::ConstPtr& msg) {

    msg_desired_target_ = *msg;
    target_write_(0) = msg_desired_target_.x;
    target_write_(1) = msg_desired_target_.y;
    if (M_== 3)
        target_write_(2) = msg_desired_target_.z;
}



/* This function does the planning logic for the task!! */
void writingTaskMotionPlanner::ComputeDesiredVelocity() {

	mutex_.lock();

    Eigen::VectorXd pos_error_;pos_error_.resize(3);
    double target_error_(0.0);


    /* Start with a picking task*/
    if (bFirst_)
        motion_phase_name_ = "pick";

    /* Do either picking or sink */
    switch (hashit_write(motion_phase_name_)){
    case ePick:
            ROS_WARN_STREAM_THROTTLE(1, "Doing PICKING motion");
            desired_velocity_ = ds1_velocity_;
            pos_error_ = real_pose_ - targets_pick_[picks_];
            target_error_ = pos_error_.squaredNorm();
            ROS_WARN_STREAM_THROTTLE(1, "Distance to PICKING TARGET" << picks_ << ": " << target_error_);

            /* Check if robot has reached target and switch to next motion*/
            if (target_error_ < thres_){
                ROS_WARN_STREAM_THROTTLE(1, "PICKING TARGET REACHED!!!!.. switching to write motion!");


                /*Switch to eWrite*/
                motion_phase_name_ = "write";

                /* Reset initial task boolean */
                if (bFirst_)
                    bFirst_=false;
            }
            break;

    case eWrite:
        ROS_WARN_STREAM_THROTTLE(1, "Doing Writing motion");
        desired_velocity_ = ds2_velocity_;

        /* Artificial Z-velocity to adapt height */
        desired_velocity_(2) = -0.5 * (real_pose_(2) - target_write_(2));

        pos_error_ = real_pose_ - (target_write_);
        target_error_ = pos_error_.squaredNorm();
        ROS_WARN_STREAM_THROTTLE(1, "Distance to Writing TARGET:" << target_error_);

        /* For viapoint motion */
        if (target_error_ < thres_){
            ROS_WARN_STREAM_THROTTLE(1, "Writing TARGET REACHED!!!!.. switching to pick!");

            /*Switch to eWrite*/
            motion_phase_name_ = "pick";
            picks_ = picks_ + 1;

        }
        break;
    }

    /* Filling in desired velocity message */
    msg_desired_velocity_.linear.x  = desired_velocity_(0);
    msg_desired_velocity_.linear.y  = desired_velocity_(1);
    msg_desired_velocity_.linear.z  = desired_velocity_(2);
    ROS_WARN_STREAM_THROTTLE(1, "Sent Velocities:" << desired_velocity_(0) << " " << desired_velocity_(1) << " " << desired_velocity_(2));

    mutex_.unlock();


}


void writingTaskMotionPlanner::PublishDesiredVelocity() {
    msg_desired_velocity_.angular.x = 0;
    msg_desired_velocity_.angular.y = 0;
    msg_desired_velocity_.angular.z = 0;
    pub_desired_twist_.publish(msg_desired_velocity_);

}


void writingTaskMotionPlanner::PublishDesiredPickingTarget(){

    if(picks_ < num_picks_){
        msg_desired_pick_target_.x = targets_pick_[picks_](0);
        msg_desired_pick_target_.y = targets_pick_[picks_](1);
        msg_desired_pick_target_.z = targets_pick_[picks_](2);
        pub_desired_pick_target_.publish(msg_desired_pick_target_);

    }
}



void writingTaskMotionPlanner::PublishDesiredWritingTarget(){

        msg_desired_write_target_.x = target_write_(0);
        msg_desired_write_target_.y = target_write_(1);
        msg_desired_write_target_.z = target_write_(2);
        pub_desired_write_target_.publish(msg_desired_write_target_);
}
