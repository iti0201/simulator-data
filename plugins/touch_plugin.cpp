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

/*
  * This plugin was modified by Rasmus Iila for personal use.
  * April 2017
  *
  */

#include <algorithm>
#include <assert.h>

#include "gazebo-7/gazebo/math/gzmath.hh"
#include "sdf/sdf.hh"

#include "ros/ros.h"
#include "touch_plugin.h"

namespace gazebo {

    TouchPlugin::TouchPlugin() {}

// Destructor
    TouchPlugin::~TouchPlugin() {}

// Load the controller
    void TouchPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

        this->parent = _parent;
        gazebo_ros_ = GazeboRosPtr(new GazeboRos(_parent, _sdf, "TouchPlugin"));
        // Make sure the ROS node for Gazebo has already been initialized
        gazebo_ros_->isInitialized();

        gazebo_ros_->getParameter<double>(update_rate_, "updateRate", 100.0);

        joint_ = gazebo_ros_->getJoint(parent, "jointName", "palm_riser");
        link_ = _parent->GetLink("palm");
        sensor_ = gazebo_ros_->getJoint(parent, "jointName", "touch_hinge");

        // Initialize update rate stuff
        if (this->update_rate_ > 0.0) this->update_period_ = 1.0 / this->update_rate_;
        else this->update_period_ = 0.0;
        last_update_time_ = parent->GetWorld()->GetSimTime();

        joint_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState>("touch_joint_states", 1000);

        // start custom queue for wheel move
        this->callback_queue_thread_ =
                boost::thread(boost::bind(&TouchPlugin::QueueThread, this));

        // listen to the update event (broadcast every simulation iteration)
        this->update_connection_ =
                event::Events::ConnectWorldUpdateBegin(boost::bind(&TouchPlugin::UpdateChild, this));

    }

    void TouchPlugin::Reset() {
        last_update_time_ = parent->GetWorld()->GetSimTime();
    }

    void TouchPlugin::publishTouchJointState() {
        ros::Time current_time = ros::Time::now();

        joint_state_.header.stamp = current_time;
        joint_state_.name.resize(1);
        joint_state_.position.resize(4);

        physics::JointPtr joint = joint_;
        physics::LinkPtr link = link_;
        physics::JointPtr sensor = sensor_;
        joint_state_.name[0] = joint->GetName();
        joint_state_.position[0] = link->GetWorldPose().pos.x;
        joint_state_.position[1] = link->GetWorldPose().pos.y;
        joint_state_.position[2] = sensor->GetParentWorldPose().pos.x;
        joint_state_.position[3] = sensor->GetParentWorldPose().pos.y;

        joint->SetForce(0, 1);
        joint_state_publisher_.publish(joint_state_);
    }

// Update the controller
    void TouchPlugin::UpdateChild() {
        common::Time current_time = parent->GetWorld()->GetSimTime();
        double seconds_since_last_update = (current_time - last_update_time_).Double();

        if (seconds_since_last_update > update_period_) {
            publishTouchJointState();

            last_update_time_ += common::Time(update_period_);

        }
    }

// Finalize the controller
    void TouchPlugin::FiniChild() {
        alive_ = false;
        queue_.clear();
        queue_.disable();
        gazebo_ros_->node()->shutdown();
        callback_queue_thread_.join();
    }

    void TouchPlugin::QueueThread() {
        static const double timeout = 0.01;

        while (alive_ && gazebo_ros_->node()->ok()) {
            queue_.callAvailable(ros::WallDuration(timeout));
        }
    }

    GZ_REGISTER_MODEL_PLUGIN ( TouchPlugin );
}