
#include <algorithm>
#include <assert.h>

#include "gazebo-7/gazebo/math/gzmath.hh"
#include "sdf/sdf.hh"

#include "ros/ros.h"
#include "sonar_wheel_plugin.h"

namespace gazebo {

    SonarWheelMove::SonarWheelMove() {}

// Destructor
    SonarWheelMove::~SonarWheelMove() {}

// Load the controller
    void SonarWheelMove::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

        this->parent = _parent;
        gazebo_ros_ = GazeboRosPtr(new GazeboRos(_parent, _sdf, "SonarWheelMove"));
        // Make sure the ROS node for Gazebo has already been initialized
        gazebo_ros_->isInitialized();

        gazebo_ros_->getParameter<std::string>(command_topic_, "commandTopic", "sonar_wheel_pose");
        gazebo_ros_->getParameter<std::string>(robot_base_frame_, "robotBaseFrame", "base_footprint");
        gazebo_ros_->getParameterBoolean(publishWheelJointState_, "publishWheelJointState", false);

        gazebo_ros_->getParameter<double>(update_rate_, "updateRate", 25.0);

        joint_ = gazebo_ros_->getJoint(parent, "jointName", "sonar_wheel_hinge");

        // Initialize update rate stuff
        if (this->update_rate_ > 0.0) this->update_period_ = 1.0 / this->update_rate_;
        else this->update_period_ = 0.0;
        last_update_time_ = parent->GetWorld()->GetSimTime();

        wheel_speed_ = 0;
        wheel_angular_goal_ = 0;

        speed_ = 0;
        goal_ = 0;
        alive_ = true;

        joint_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState>("sonar_joint_states", 1000);

        ros::SubscribeOptions so =
                ros::SubscribeOptions::create<geometry_msgs::Pose>(command_topic_, 1,
                                                                    boost::bind(&SonarWheelMove::cmdVelCallback,
                                                                                this, _1),
                                                                    ros::VoidPtr(), &queue_);

        cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);

        // start custom queue for wheel move
        this->callback_queue_thread_ =
                boost::thread(boost::bind(&SonarWheelMove::QueueThread, this));

        // listen to the update event (broadcast every simulation iteration)
        this->update_connection_ =
                event::Events::ConnectWorldUpdateBegin(boost::bind(&SonarWheelMove::UpdateChild, this));

    }

    void SonarWheelMove::Reset() {
        last_update_time_ = parent->GetWorld()->GetSimTime();
        pose_encoder_.x = 0;
        pose_encoder_.y = 0;
        pose_encoder_.theta = 0;
        speed_ = 0;
        goal_ = 0;
    }

    void SonarWheelMove::publishWheelJointState() {
        ros::Time current_time = ros::Time::now();

        joint_state_.header.stamp = current_time;
        joint_state_.name.resize(1);
        joint_state_.position.resize(1);

        physics::JointPtr joint = joint_;
        joint_state_.name[0] = joint->GetName();
        joint_state_.position[0] = joint->GetWorldPose().rot.GetAsEuler().z - parent->GetWorldPose().rot.GetAsEuler().z;

        joint_state_publisher_.publish(joint_state_);
    }

// Update the controller
    void SonarWheelMove::UpdateChild() {
        common::Time current_time = parent->GetWorld()->GetSimTime();
        double seconds_since_last_update = (current_time - last_update_time_).Double();

        if (seconds_since_last_update > update_period_) {
            publishWheelJointState();

            // Update robot in case new velocities have been requested
            getWheelVelocities();

            last_update_time_ += common::Time(update_period_);

            double currentYaw = joint_->GetWorldPose().rot.GetAsEuler().z - parent->GetWorldPose().rot.GetAsEuler().z;

//            ROS_INFO("yaw goal is: %4f", wheel_angular_goal_);
            double yawDelta = wheel_angular_goal_ - currentYaw;
//            ROS_INFO("yaw delta is: %4f", yawDelta);

            double potential_distance = wheel_speed_ * seconds_since_last_update * yawDelta / fabs(yawDelta);
//            ROS_INFO("potential_distance is: %4f", potential_distance);

            double new_yaw = currentYaw + potential_distance;

            if (fabs(yawDelta) < fabs(potential_distance)) {
                new_yaw = wheel_angular_goal_;
            }
//            ROS_INFO("new yaw is: %4f", new_yaw);
            //joint_->SetVelocity (0, wheel_speed_);
            joint_->SetPosition(0, new_yaw);
        }
    }

// Finalize the controller
    void SonarWheelMove::FiniChild() {
        alive_ = false;
        queue_.clear();
        queue_.disable();
        gazebo_ros_->node()->shutdown();
        callback_queue_thread_.join();
    }

    void SonarWheelMove::getWheelVelocities() {
        boost::mutex::scoped_lock scoped_lock(lock);

        wheel_speed_ = speed_;
        wheel_angular_goal_ = goal_;
    }

    void SonarWheelMove::cmdVelCallback(const geometry_msgs::Pose::ConstPtr &cmd_msg) {
        boost::mutex::scoped_lock scoped_lock(lock);
        speed_ = cmd_msg->position.x;

        goal_ = cmd_msg->orientation.z;
    }

    void SonarWheelMove::QueueThread() {
        static const double timeout = 0.01;

        while (alive_ && gazebo_ros_->node()->ok()) {
            queue_.callAvailable(ros::WallDuration(timeout));
        }
    }

    GZ_REGISTER_MODEL_PLUGIN ( SonarWheelMove );
}