/*
 * Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/

/*
 * \file  gazebo_ros_diff_drive.h
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
  * This plugin was modified by Rasmus Iila for personal use.
  * April 2017
  *
  */

#ifndef GYRO_PLUGIN_HH
#define GYRO_PLUGIN_HH

#include <map>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_plugins/gazebo_ros_utils.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/JointState.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {

    class Joint;

    class Entity;

    class GyroPlugin : public ModelPlugin {

        enum OdomSource {
            ENCODER = 0,
            WORLD = 1,
        };
    public:
        GyroPlugin();

        ~GyroPlugin();

        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

        void Reset();

    protected:
        virtual void UpdateChild();

        virtual void FiniChild();

    private:

        void publishGyroJointState();



        GazeboRosPtr gazebo_ros_;
        physics::ModelPtr parent;
        event::ConnectionPtr update_connection_;

        std::vector<physics::JointPtr> joints_;

        // ROS STUFF
        sensor_msgs::JointState joint_state_;
        ros::Publisher joint_state_publisher_;

        boost::mutex lock;

        // Custom Callback Queue
        ros::CallbackQueue queue_;
        boost::thread callback_queue_thread_;

        void QueueThread();

        bool alive_;

        // Update Rate
        double update_rate_;
        double update_period_;
        common::Time last_update_time_;

        physics::JointPtr joint_;
        ros::Subscriber cmd_vel_subscriber_;
        double wheel_angular_speed;
        physics::JointPtr sensor_;
    };

}

#endif

