/*
    Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

/*
 * \file  gazebo_ros_diff_drive.cpp
 *
 * \brief A differential drive plugin for gazebo. Based on the diffdrive plugin
 * developed for the erratic robot (see copyright notice above). The original
 * plugin can be found in the ROS package gazebo_erratic_plugins.
 *
 * \author  Piyush Khandelwal (piyushk@gmail.com)
 *
 * $ Id: 06/21/2013 11:23:40 AM piyushk $
 */


/*
 *
 * The support of acceleration limit was added by
 * \author   George Todoran <todorangrg@gmail.com>
 * \author   Markus Bader <markus.bader@tuwien.ac.at>
 * \date 22th of May 2014
 */

#include <algorithm>
#include <assert.h>

#include "gazebo-7/gazebo/math/gzmath.hh"
#include "sdf/sdf.hh"

#include "ros/ros.h"
#include "wheel_plugin.h"

namespace gazebo {

    WheelMove::WheelMove() {}

// Destructor
    WheelMove::~WheelMove() {}

// Load the controller
    void WheelMove::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

        this->parent = _parent;
        gazebo_ros_ = GazeboRosPtr(new GazeboRos(_parent, _sdf, "WheelMove"));
        // Make sure the ROS node for Gazebo has already been initialized
        gazebo_ros_->isInitialized();

        gazebo_ros_->getParameter<std::string>(command_topic_, "commandTopic", "cmd_vel");
        gazebo_ros_->getParameter<std::string>(robot_base_frame_, "robotBaseFrame", "base_footprint");
        gazebo_ros_->getParameterBoolean(publishWheelTF_, "publishWheelTF", false);
        gazebo_ros_->getParameterBoolean(publishWheelJointState_, "publishWheelJointState", false);

        gazebo_ros_->getParameter<double>(wheel_diameter_, "wheelDiameter", 0.15);
        gazebo_ros_->getParameter<double>(wheel_accel, "wheelAcceleration", 0.0);
        gazebo_ros_->getParameter<double>(wheel_torque, "wheelTorque", 5.0);
        gazebo_ros_->getParameter<double>(update_rate_, "updateRate", 100.0);

        joint_ = gazebo_ros_->getJoint(parent, "jointName", "left_wheel_hinge");

        // Initialize update rate stuff
        if (this->update_rate_ > 0.0) this->update_period_ = 1.0 / this->update_rate_;
        else this->update_period_ = 0.0;
        last_update_time_ = parent->GetWorld()->GetSimTime();

        wheel_speed_ = 0;

        x_ = 0;
        rot_ = 0;
        alive_ = true;

        ros::NodeHandle* node_handle_ = new ros::NodeHandle("lego_robot");

        joint_state_publisher_ = node_handle_->advertise<sensor_msgs::JointState>("joint_states", 1000);

        ros::SubscribeOptions so =
                ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
                                                                    boost::bind(&WheelMove::cmdVelCallback,
                                                                                this, _1),
                                                                    ros::VoidPtr(), &queue_);

        cmd_vel_subscriber_ = node_handle_->subscribe(so);

        // start custom queue for wheel move
        this->callback_queue_thread_ =
                boost::thread(boost::bind(&WheelMove::QueueThread, this));

        // listen to the update event (broadcast every simulation iteration)
        this->update_connection_ =
                event::Events::ConnectWorldUpdateBegin(boost::bind(&WheelMove::UpdateChild, this));

    }

    void WheelMove::Reset() {
        last_update_time_ = parent->GetWorld()->GetSimTime();
        pose_encoder_.x = 0;
        pose_encoder_.y = 0;
        pose_encoder_.theta = 0;
        x_ = 0;
        rot_ = 0;
    }

    void WheelMove::publishWheelJointState() {
        ros::Time current_time = ros::Time::now();

        joint_state_.header.stamp = current_time;
        joint_state_.name.resize(1);
        joint_state_.position.resize(1);

        physics::JointPtr joint = joint_;
        double velocity = joint->GetVelocity(0);
        joint_state_.name[0] = joint->GetName();
        joint_state_.position[0] = velocity;

        joint_state_publisher_.publish(joint_state_);
    }

// Update the controller
    void WheelMove::UpdateChild() {
        common::Time current_time = parent->GetWorld()->GetSimTime();
        double seconds_since_last_update = (current_time - last_update_time_).Double();

        if (seconds_since_last_update > update_period_) {
            if (publishWheelTF_) publishWheelTF();
            publishWheelJointState();

            // Update robot in case new velocities have been requested
            getWheelVelocities();

            last_update_time_ += common::Time(update_period_);

            //joint_->SetVelocity (0, wheel_speed_);
            joint_->SetVelocity(0, wheel_angular_speed);
        }
    }

// Finalize the controller
    void WheelMove::FiniChild() {
        alive_ = false;
        queue_.clear();
        queue_.disable();
        gazebo_ros_->node()->shutdown();
        callback_queue_thread_.join();
    }

    void WheelMove::getWheelVelocities() {
        boost::mutex::scoped_lock scoped_lock(lock);

        double vr = x_;
        double va = rot_;

        wheel_speed_ = vr;
        wheel_angular_speed = va;
    }

    void WheelMove::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &cmd_msg) {
        boost::mutex::scoped_lock scoped_lock(lock);
        x_ = cmd_msg->linear.x;
        rot_ = cmd_msg->angular.x;
    }

    void WheelMove::QueueThread() {
        static const double timeout = 0.01;

        while (alive_ && gazebo_ros_->node()->ok()) {
            queue_.callAvailable(ros::WallDuration(timeout));
        }
    }

    GZ_REGISTER_MODEL_PLUGIN ( WheelMove );
}
