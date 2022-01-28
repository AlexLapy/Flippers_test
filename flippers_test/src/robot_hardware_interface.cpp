#include "flippers_test/robot_hardware_interface.h"
#include <ctre/phoenix/motorcontrol/SupplyCurrentLimitConfiguration.h>

// Constructor
ROBOTHardwareInterface::ROBOTHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {

  init_interface();
  init_control_loop();
  init_ctre_driver();
	init_pub(nh);
	init_client(nh);
	
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
}

// Deconstructor
ROBOTHardwareInterface::~ROBOTHardwareInterface() {
}

/************************************************************/

// Initialize joints interface
void ROBOTHardwareInterface::init_interface() {
    
    
  joint_names_.push_back("flipper_fl_j");
  joint_names_.push_back("flipper_fr_j");
  joint_names_.push_back("flipper_rl_j");
  joint_names_.push_back("flipper_rr_j");

// Status
  joint_position_.resize(num_joints_, 0.0);
  joint_velocity_.resize(num_joints_, 0.0);
  joint_effort_.resize(num_joints_, 0.0);

// Command
  joint_position_command_.resize(num_joints_, 0.0);
  joint_velocity_command_.resize(num_joints_, 0.0);
  joint_effort_command_.resize(num_joints_, 0.0);
    


  // connect and register the joint state and effort interfaces
  for (unsigned int joint_id = 0; joint_id < num_joints_; ++joint_id)
  {
    // Create joint state interface
    hardware_interface::JointStateHandle jointStateHandle(joint_names_[joint_id], &joint_position_[joint_id],
                                                      &joint_velocity_[joint_id], &joint_effort_[joint_id]);
    joint_state_interface_.registerHandle(jointStateHandle);

    // Create effort joint interface
    hardware_interface::JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_[joint_id]);
	effort_joint_interface_.registerHandle(jointEffortHandle);

    // Create Joint Limit interface   
    joint_limits_interface::JointLimits limits;
    joint_limits_interface::getJointLimits(joint_names_[joint_id], nh_, limits);
	joint_limits_interface::EffortJointSaturationHandle jointLimitsHandle(jointEffortHandle, limits);
	effortJointSaturationInterface.registerHandle(jointLimitsHandle);
  }

// Register all joints interfaces    
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&effort_joint_interface_);
    registerInterface(&effortJointSaturationInterface);
}

// Initialize non real time control loop
void ROBOTHardwareInterface::init_control_loop() {

    loop_hz_ = 100;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
    non_realtime_loop_ = nh_.createTimer(update_freq, &ROBOTHardwareInterface::update, this);
}

// Initialize ctre drive
void ROBOTHardwareInterface::init_ctre_driver() {

    std::string interface = "can0";
    ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

    SupplyCurrentLimitConfiguration current_limit_config;
    current_limit_config.enable = true;
    current_limit_config.currentLimit = current_limit_;

    float kP, kI, kD = 0.0;
    nh_.getParam("/markhor/markhor_flippers_node/kP", kP);
    nh_.getParam("/markhor/markhor_flippers_node/kI", kI);
    nh_.getParam("/markhor/markhor_flippers_node/kD", kD);

    if (nh_.getParam("/markhor/markhor_flippers_node/front_left", drive_fl_id_) == true)
    {
        front_left_drive_ = std::make_unique<TalonSRX>(drive_fl_id_);
        front_left_drive_->ConfigFactoryDefault();
        front_left_drive_->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, timeout_ms_);
        front_left_drive_->SetSensorPhase(true);
        front_left_drive_->ConfigSupplyCurrentLimit(current_limit_config);
        front_left_drive_->ConfigNominalOutputForward(0, timeout_ms_);
        front_left_drive_->ConfigNominalOutputReverse(0, timeout_ms_);
        front_left_drive_->ConfigAllowableClosedloopError(0, 100, timeout_ms_);

        double front_left_peak_output_forward, front_left_peak_output_reverse = 0;
        nh_.getParam("/markhor/markhor_flippers_node/front_left_drive_peak_output_forward", front_left_peak_output_forward);
        nh_.getParam("/markhor/markhor_flippers_node/front_left_drive_peak_output_reverse", front_left_peak_output_reverse);

        front_left_drive_->ConfigPeakOutputForward(front_left_peak_output_forward, timeout_ms_);
        front_left_drive_->ConfigPeakOutputReverse(front_left_peak_output_reverse, timeout_ms_);

        front_left_drive_->SelectProfileSlot(0, 0);
        front_left_drive_->Config_kF(0, 0, timeout_ms_);
        front_left_drive_->Config_kP(0, kP, timeout_ms_);
        front_left_drive_->Config_kI(0, kI, timeout_ms_);
        front_left_drive_->Config_kD(0, kD, timeout_ms_);

       
    }
    if (nh_.getParam("/markhor/markhor_flippers_node/front_right", drive_fr_id_) == true)
    {
        front_right_drive_ = std::make_unique<TalonSRX>(drive_fr_id_);
        front_right_drive_->ConfigFactoryDefault();
        front_right_drive_->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 50);
        front_right_drive_->SetSensorPhase(true);
        front_right_drive_->ConfigSupplyCurrentLimit(current_limit_config);
        front_right_drive_->ConfigNominalOutputForward(0, timeout_ms_);
        front_right_drive_->ConfigNominalOutputReverse(0, timeout_ms_);
        front_right_drive_->ConfigAllowableClosedloopError(0, 100, timeout_ms_);

        double front_right_peak_output_forward, front_right_peak_output_reverse = 0;
        nh_.getParam("/markhor/markhor_flippers_node/front_right_drive_peak_output_forward", front_right_peak_output_forward);
        nh_.getParam("/markhor/markhor_flippers_node/front_right_drive_peak_output_reverse", front_right_peak_output_reverse);

        front_right_drive_->ConfigPeakOutputForward(front_right_peak_output_forward, timeout_ms_);
        front_right_drive_->ConfigPeakOutputReverse(front_right_peak_output_reverse, timeout_ms_);

        front_right_drive_->SelectProfileSlot(0, 0);
        front_right_drive_->Config_kF(0, 0, timeout_ms_);
        front_right_drive_->Config_kP(0, kP, timeout_ms_);
        front_right_drive_->Config_kI(0, kI, timeout_ms_);
        front_right_drive_->Config_kD(0, kD, timeout_ms_);

    }
    if (nh_.getParam("/markhor/markhor_flippers_node/rear_left", drive_rl_id_) == true)
    {
        rear_left_drive_ = std::make_unique<TalonSRX>(drive_rl_id_);
        rear_left_drive_->ConfigFactoryDefault();
        rear_left_drive_->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 50);
        rear_left_drive_->SetSensorPhase(true);
        rear_left_drive_->ConfigSupplyCurrentLimit(current_limit_config);
        rear_left_drive_->ConfigNominalOutputForward(0, timeout_ms_);
        rear_left_drive_->ConfigNominalOutputReverse(0, timeout_ms_);
        rear_left_drive_->ConfigAllowableClosedloopError(0, 100, timeout_ms_);


        double rear_left_peak_output_forward, rear_left_peak_output_reverse = 0;
        nh_.getParam("/markhor/markhor_flippers_node/rear_left_drive_peak_output_forward", rear_left_peak_output_forward);
        nh_.getParam("/markhor/markhor_flippers_node/rear_left_drive_peak_output_reverse", rear_left_peak_output_reverse);

        rear_left_drive_->ConfigPeakOutputForward(rear_left_peak_output_forward, timeout_ms_);
        rear_left_drive_->ConfigPeakOutputReverse(rear_left_peak_output_reverse, timeout_ms_);

        rear_left_drive_->SelectProfileSlot(0, 0);
        rear_left_drive_->Config_kF(0, 0, timeout_ms_);
        rear_left_drive_->Config_kP(0, kP, timeout_ms_);
        rear_left_drive_->Config_kI(0, kI, timeout_ms_);
        rear_left_drive_->Config_kD(0, kD, timeout_ms_);


    }
    if (nh_.getParam("/markhor/markhor_flippers_node/rear_right", drive_rr_id_) == true)
    {
        rear_right_drive_ = std::make_unique<TalonSRX>(drive_rr_id_);
        rear_right_drive_->ConfigFactoryDefault();
        rear_right_drive_->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 50);
        rear_right_drive_->SetSensorPhase(true);
        rear_right_drive_->ConfigSupplyCurrentLimit(current_limit_config);
        rear_right_drive_->ConfigNominalOutputForward(0, timeout_ms_);
        rear_right_drive_->ConfigNominalOutputReverse(0, timeout_ms_);
        rear_right_drive_->ConfigAllowableClosedloopError(0, 100, timeout_ms_);

        double rear_right_peak_output_forward, rear_right_peak_output_reverse = 0;
        nh_.getParam("/markhor/markhor_flippers_node/rear_right_peak_output_forward", rear_right_peak_output_forward);
        nh_.getParam("/markhor/markhor_flippers_node/rear_right_peak_output_reverse", rear_right_peak_output_reverse);          

        rear_right_drive_->ConfigPeakOutputForward(rear_right_peak_output_forward, timeout_ms_);
        rear_right_drive_->ConfigPeakOutputReverse(rear_right_peak_output_reverse, timeout_ms_);

        rear_right_drive_->SelectProfileSlot(0, 0);
        rear_right_drive_->Config_kF(0, 0, timeout_ms_);
        rear_right_drive_->Config_kP(0, kP, timeout_ms_);
        rear_right_drive_->Config_kI(0, kI, timeout_ms_);
        rear_right_drive_->Config_kD(0, kD, timeout_ms_);

  }
}

// Initialize pub objects
void ROBOTHardwareInterface::init_pub(ros::NodeHandle& nh) {

    //flipper_fl_pub = nh.advertise<std_msgs::Float64>("flipper_fl_position_controller/command", 1000);
    //flipper_fr_pub = nh.advertise<std_msgs::Float64>("flipper_fr_position_controller/command", 1000);
    //flipper_rl_pub = nh.advertise<std_msgs::Float64>("flipper_rl_position_controller/command", 1000);
    //flipper_rr_pub = nh.advertise<std_msgs::Float64>("flipper_rr_position_controller/command", 1000);
}

// Initialize client objects
void ROBOTHardwareInterface::init_client(ros::NodeHandle& nh) {

   // client = nh_.serviceClient<ros_control_example::Floats_array>("/read_joint_state");
}

/**********************************************************************/

// update for controller manager read/write
void ROBOTHardwareInterface::update(const ros::TimerEvent& e, ros::NodeHandle& nh) {

    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_, nh);
}

// read sensor
void ROBOTHardwareInterface::read() {

  if (front_left_drive_->GetDeviceID() == 11)
  {
  joint_position_[0] = front_left_drive_->GetSensorCollection().GetPulseWidthPosition();
  joint_velocity_[0] = front_left_drive_->GetSensorCollection().GetPulseWidthVelocity();
  }
  else
  {
    ROS_FATAL("Could not find drive %d in drives list", front_left_drive_->GetDeviceID());
    ros::shutdown();
  }

  if (front_right_drive_->GetDeviceID() == 12)
  {
  joint_position_[1] = front_right_drive_->GetSensorCollection().GetPulseWidthPosition();
  joint_velocity_[1] = front_right_drive_->GetSensorCollection().GetPulseWidthVelocity();
  }
  else
  {
    ROS_FATAL("Could not find drive %d in drives list", front_right_drive_->GetDeviceID());
    ros::shutdown();
  }

  if (rear_left_drive_->GetDeviceID() == 13)
  {
  joint_position_[2] = rear_left_drive_->GetSensorCollection().GetPulseWidthPosition();
  joint_velocity_[2] = rear_left_drive_->GetSensorCollection().GetPulseWidthVelocity();
  }
  else
  {

    ROS_FATAL("Could not find drive %d in drives list", rear_left_drive_->GetDeviceID());
    ros::shutdown();
  }
  if (rear_right_drive_->GetDeviceID() == 14)
  {
  joint_position_[3] = rear_right_drive_->GetSensorCollection().GetPulseWidthPosition();
  joint_velocity_[3] = rear_right_drive_->GetSensorCollection().GetPulseWidthVelocity();
  }
  else
  {
    ROS_FATAL("Could not find drive %d in drives list", rear_right_drive_->GetDeviceID());
    ros::shutdown();
  }

}  

// write to drive
void ROBOTHardwareInterface::write(ros::Duration elapsed_time, ros::NodeHandle& nh) {
   
   ctre::phoenix::unmanaged::FeedEnable(100);

    effortJointSaturationInterface.enforceLimits(elapsed_time);  

    int position;
    nh.getParam("position", position);
    int velocity;
    nh.getParam("velocity", velocity);

    if (position)
    {
	front_left_drive_->Set(ControlMode::Position, joint_position_command_[0]);	
	front_right_drive_->Set(ControlMode::Position, joint_position_command_[1]);	
	rear_left_drive_->Set(ControlMode::Position, joint_position_command_[2]);	
	rear_right_drive_->Set(ControlMode::Position, joint_position_command_[3]);	
    }
    else if (velocity)
    {
    front_left_drive_->Set(ControlMode::Velocity, joint_velocity_command_[0]);	
	front_right_drive_->Set(ControlMode::Velocity, joint_velocity_command_[1]);	
	rear_left_drive_->Set(ControlMode::Velocity, joint_velocity_command_[2]);	
	rear_right_drive_->Set(ControlMode::Velocity, joint_velocity_command_[3]);
    }
		
}



