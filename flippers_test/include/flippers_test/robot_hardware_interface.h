#ifndef ROBOT_HARDWARE_INTERFACE_H_
#define ROBOT_HARDWARE_INTERFACE_H_

// ros_control
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>

// ROS
#include <ros/ros.h>
#include <rospy_tutorials/Floats.h>
#include <angles/angles.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
//#include <std_msgs/Float64.h>

// CTRE
#include "Platform-linux-socket-can.h"
#include "ctre/Phoenix.h"
#include "ctre/phoenix/motorcontrol/can/TalonSRX.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

// STD
#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/scoped_ptr.hpp>
#include <stdlib.h>


class ROBOTHardwareInterface : public hardware_interface::RobotHW 
{
	public:
        ROBOTHardwareInterface(ros::NodeHandle& nh); // Constructor
        ~ROBOTHardwareInterface();  // Deconstructor 
        /*
            Initialization in constructor
        */
        void init_interface();
        void init_sub();
        void init_pub(ros::NodeHandle& nh);
        void init_client(ros::NodeHandle& nh);
        void init_control_loop();
        void init_ctre_driver();
        /* 
            Main routine 
        */
        void update(const ros::TimerEvent& e, ros::NodeHandle& nh);
        void read();
        void write(ros::Duration elapsed_time, ros::NodeHandle& nh);
        /* 
            Setup pub, sub and client object
        */
        //ros::Publisher pub;
        //ros::Publisher flipper_fl_pub;
        //ros::Publisher flipper_fr_pub;
        //ros::Publisher flipper_rl_pub;
        //ros::Publisher flipper_rr_pub;

        //ros::Subscriber sub;
        //ros::ServiceClient client;
        /*
            Setup ros msg for joints 
        */
        //rospy_tutorials::Floats joints_pub;
        //ros_control_example::Floats_array joint_read;
        
    protected:
        /*
            Setup joints interfaces
        */
        hardware_interface::JointStateInterface        joint_state_interface_;
        hardware_interface::PositionJointInterface  position_joint_interface_;
        hardware_interface::EffortJointInterface      effort_joint_interface_;
        joint_limits_interface::EffortJointSaturationInterface effortJointSaturationInterface;
        
        /* 
            Setup var
        */
        int current_limit_ = 30;
        const int timeout_ms_ = 30;
        int drive_fl_id_, drive_fr_id_ = 0;
        int drive_rl_id_, drive_rr_id_ = 0;
        int num_joints_ = 4;
        double pos = 0;
        double vel = 0;
        double output = 0;
        double tmp = 0;
        //long double encoderPos;
        //long double lastPos;

        std::vector<std::string> joint_names_;  

        std::vector<double> joint_position_;
        std::vector<double> joint_velocity_;
        std::vector<double> joint_effort_;

        std::vector<double> joint_position_command_;
        std::vector<double> joint_velocity_command_;
        std::vector<double> joint_effort_command_;

        std::unique_ptr<TalonSRX> front_left_drive_;
        std::unique_ptr<TalonSRX> front_right_drive_;
        std::unique_ptr<TalonSRX> rear_left_drive_;
        std::unique_ptr<TalonSRX> rear_right_drive_;

        //float front_left_drive_upper_limit_;
        //float front_left_drive_lower_limit_;     
        //float front_right_drive_upper_limit_;
        //float front_right_drive_lower_limit_;
        //float rear_left_drive_upper_limit_;
        //float rear_left_drive_lower_limit_;
        //float rear_right_drive_upper_limit_;
        //float rear_right_drive_lower_limit_;

        float front_left_drive_base_position_;
        float front_right_drive_base_position_;
        float rear_left_drive_base_position_;
        float rear_right_drive_base_position_;
        bool has_reset_event_occured_ = false;

        
        ros::NodeHandle nh_;    // passed between "main" and constructor

        ros::Timer non_realtime_loop_;
        ros::Duration elapsed_time_;
        double loop_hz_;

        /* 
            boost::shared_ptr is a smart pointer: No need to call new and delete
            Also, shared_ptr mean reference counting (delete when ref=0)

        */
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
        //std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};

#endif 