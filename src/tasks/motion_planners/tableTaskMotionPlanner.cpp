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

#include "tableTaskMotionPlanner.h"

tableTaskMotionPlanner::tableTaskMotionPlanner(ros::NodeHandle &n,
                                           double frequency,
                                           std::string input_pose_topic_name,
                                           std::string input_ds1_topic_name,
                                           std::string input_ds2_topic_name,
                                           std::string input_target_topic_name,
                                           std::string output_vel_topic_name,
                                           std::string output_pick_topic_name,
                                           std::vector<double> &attractors_pick,
                                           std::vector<double> &attractor_sink,
                                           bool sim)
	: nh_(n),
	  loop_rate_(frequency),
	  input_pose_topic_name_(input_pose_topic_name),
	  input_ds1_topic_name_ (input_ds1_topic_name),
	  input_ds2_topic_name_ (input_ds2_topic_name),
      input_target_topic_name_(input_target_topic_name),
      output_vel_topic_name_ (output_vel_topic_name),
      output_pick_topic_name_ (output_pick_topic_name),
      attractors_pick_(attractors_pick),
      attractor_sink_(attractor_sink),
      thres_(1e-3),
      M_(3), sim_(sim){

	ROS_INFO_STREAM("Sink-Task Motion Planning node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}

tableTaskMotionPlanner::~tableTaskMotionPlanner(){
    ROS_INFO_STREAM("In destructor.. motion generator was killed! ");

}
bool tableTaskMotionPlanner::Init() {

    real_pose_.resize(M_);        real_pose_.setZero();
    ds1_velocity_.resize(M_);     ds1_velocity_.setZero();
    ds2_velocity_.resize(M_);     ds1_velocity_.setZero();
    desired_velocity_.resize(M_); desired_velocity_.setZero();
    target_sink_.resize(M_);      target_sink_.setZero();
    learned_att_sink_.resize(M_); learned_att_sink_.setZero();

    /* Fill in target Vector for sink motion*/
    for (unsigned int i=0;i < M_; i++){
        learned_att_sink_(i) = attractor_sink_[i];
        target_sink_(i) = attractor_sink_[i];
    }

    /* Fill in targets Vector* for sink motion*/
    num_picks_ = (int) attractors_pick_.size()/M_;
    ROS_INFO_STREAM("Doing  " << num_picks_ << " picks!" );
    targets_pick_  = new VectorXd[num_picks_]; for(unsigned int s=0; s<num_picks_; s++ ){targets_pick_[s].resize(M_);	}

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

bool tableTaskMotionPlanner::InitializeROS() {

	/* Initialize subscribers */
    sub_real_pose_              = nh_.subscribe( input_pose_topic_name_ , 1000, &tableTaskMotionPlanner::UpdateRealPosition, this, ros::TransportHints().reliable().tcpNoDelay());
    sub_ds1_twist_              = nh_.subscribe( input_ds1_topic_name_  , 1000, &tableTaskMotionPlanner::UpdateDS1Velocity, this, ros::TransportHints().reliable().tcpNoDelay());
    sub_ds2_twist_              = nh_.subscribe( input_ds2_topic_name_,   1000, &tableTaskMotionPlanner::UpdateDS2Velocity, this, ros::TransportHints().reliable().tcpNoDelay());
    sub_desired_target_         = nh_.subscribe( input_target_topic_name_ , 1000, &tableTaskMotionPlanner::UpdateDynamicTarget, this, ros::TransportHints().reliable().tcpNoDelay());

	/* Initialize publishers */
    pub_desired_twist_          = nh_.advertise<geometry_msgs::Twist>(output_vel_topic_name_, 1);    
    pub_desired_pick_target_    = nh_.advertise<geometry_msgs::Point>(output_pick_topic_name_, 1);

    if (nh_.ok()) {
		ros::spinOnce();
		ROS_INFO("The Sink Task planner is ready.");
		return true;
	}
	else {
		ROS_ERROR("The ros node has a problem.");
		return false;
	}
}


void tableTaskMotionPlanner::Run() {

    while ((nh_.ok()) && (picks_ < num_picks_)){

        ComputeDesiredVelocity();
        PublishDesiredVelocity();
        PublishDesiredPickingTarget();

        ros::spinOnce();
        loop_rate_.sleep();

    }

    ros::spinOnce();

    loop_rate_.sleep();

    nh_.shutdown();
}

void tableTaskMotionPlanner::UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg) {

	msg_real_pose_ = *msg;

	real_pose_(0) = msg_real_pose_.position.x;
	real_pose_(1) = msg_real_pose_.position.y;
	real_pose_(2) = msg_real_pose_.position.z;
}

void tableTaskMotionPlanner::UpdateDS1Velocity(const geometry_msgs::Twist::ConstPtr& msg) {

	msg_ds1_twist_ = *msg;

    ds1_velocity_(0) = msg_ds1_twist_.linear.x;
    ds1_velocity_(1) = msg_ds1_twist_.linear.y;
    ds1_velocity_(2) = msg_ds1_twist_.linear.z;

}

void tableTaskMotionPlanner::UpdateDS2Velocity(const geometry_msgs::Twist::ConstPtr& msg) {

	msg_ds2_twist_ = *msg;

    ds2_velocity_(0) = msg_ds2_twist_.linear.x;
    ds2_velocity_(1) = msg_ds2_twist_.linear.y;
    ds2_velocity_(2) = msg_ds2_twist_.linear.z;
}


void tableTaskMotionPlanner::UpdateDynamicTarget(const geometry_msgs::Point::ConstPtr& msg) {

    msg_desired_target_ = *msg;
    target_sink_(0) = msg_desired_target_.x;
    target_sink_(1) = msg_desired_target_.y;
    if (M_== 3)
        target_sink_(2) = msg_desired_target_.z;
}



/* This function does the planning logic for the task!! */
void tableTaskMotionPlanner::ComputeDesiredVelocity() {

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
            pos_error_ = real_pose_ - targets_pick_[picks_];
            target_error_ = pos_error_.squaredNorm();
            ROS_WARN_STREAM_THROTTLE(1, "Distance to PICKING TARGET" << picks_ << ": " << target_error_);

            /* Check if robot has reached target and swicth to next motion*/
            if (target_error_ < thres_){
                ROS_WARN_STREAM_THROTTLE(1, "PICKING TARGET REACHED!!!!.. switching to sink!");

                if (!sim_){
                    /* Grasp Cube*/
                    ros::Duration(0.1).sleep(); // wait
                    gripper_->setSpeed(250);
                    gripper_->setPosition(255); // to close
                    ros::Duration(0.1).sleep();
                }

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
        pos_error_ = real_pose_ - (target_sink_);
        target_error_ = pos_error_.squaredNorm();
        ROS_WARN_STREAM_THROTTLE(1, "Distance to SINK TARGET:" << target_error_);

        /* Check if robot has reached target and swicth to next motion*/
        /* For sink motion */
//        if (target_error_ < 2*thres_){

        /* For viapoint motion */
        if (target_error_ < 5*thres_){
            ROS_WARN_STREAM_THROTTLE(1, "Sink TARGET REACHED!!!!.. switching to pick!");

            if (!sim_){
                /* Release Cube*/
                gripper_->setPosition(60); // to open
                ros::Duration(0.2).sleep(); // wait
            }

            /*Switch to eSink*/
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


void tableTaskMotionPlanner::PublishDesiredVelocity() {
    msg_desired_velocity_.angular.x = 0;
    msg_desired_velocity_.angular.y = 0;
    msg_desired_velocity_.angular.z = 0;
    pub_desired_twist_.publish(msg_desired_velocity_);

}


void tableTaskMotionPlanner::PublishDesiredPickingTarget(){

    if(picks_ < num_picks_){
        msg_desired_pick_target_.x = targets_pick_[picks_](0);
        msg_desired_pick_target_.y = targets_pick_[picks_](1);
        msg_desired_pick_target_.z = targets_pick_[picks_](2);
        pub_desired_pick_target_.publish(msg_desired_pick_target_);

    }
}
