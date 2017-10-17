/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Helper ros_control hardware interface that loads configurations
*/

#include <ros_control_boilerplate/generic_hw_interface.h>
#include <limits>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace ros_control_boilerplate
{
FRCRobotInterface::FRCRobotInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
  : name_("generic_hw_interface")
  , nh_(nh)
  , use_rosparam_joint_limits_(false)
  , use_soft_limits_if_available_(false)
{
  // Check if the URDF model needs to be loaded
  if (urdf_model == NULL)
    loadURDF(nh, "robot_description");
  else
    urdf_model_ = urdf_model;

  // Load rosparams
  ros::NodeHandle rpnh(nh_, "hardware_interface"); // TODO(davetcoleman): change the namespace to "generic_hw_interface" aka name_

  // Read a list of joint information from ROS parameters.  Each entry in the list
  // specifies a name for the joint and a hardware ID corresponding
  // to that value.  Joint types and locations are specified (by name)
  // in a URDF file loaded along with the controller.
  //std::size_t error = 0;
  //error += !rosparam_shortcuts::get(name_, rpnh, "joints", joint_names_);
  XmlRpc::XmlRpcValue joint_param_list;
  if (!rpnh.getParam("joints", joint_param_list))
	  throw std::runtime_error("No joints were specified.");
  for (size_t i = 0; i < joint_param_list.size(); i++)
  {
	  XmlRpc::XmlRpcValue &joint_params = joint_param_list[i];
	  if (!joint_params.hasMember("name"))
		  throw std::runtime_error("A joint name was not specified");
	  XmlRpc::XmlRpcValue& xml_joint_name = joint_params["name"];
	  if (!xml_joint_name.valid() ||
		   xml_joint_name.getType() != XmlRpc::XmlRpcValue::TypeString)
		  throw std::runtime_error("An invalid joint name was specified.");
	  const std::string joint_name_str = xml_joint_name;
	  joint_names_.push_back(joint_name_str);

	  if (!joint_params.hasMember("hw_id"))
		  throw std::runtime_error("A joint hw_id was not specified");
	  XmlRpc::XmlRpcValue& xml_joint_hw_id = joint_params["hw_id"];
	  if (!xml_joint_hw_id.valid() ||
		   xml_joint_hw_id.getType() != XmlRpc::XmlRpcValue::TypeInt)
		  throw std::runtime_error("An invalid joint hw_id was specified.");
	  const int joint_hw_id_str = xml_joint_hw_id;
	  joint_hw_ids_.push_back(joint_hw_id_str);
	  // Eventually add a list of valid modes for this joint?
  }
  
  //rosparam_shortcuts::shutdownIfError(name_, error);
}

void FRCRobotInterface::init()
{
	num_joints_ = joint_names_.size();
	// Create vectors of the correct size for
	// talon HW state and commands
	talon_state_.resize(num_joints_);
	talon_command_.resize(num_joints_);

	// Loop through the list of joint names
	// specified as params for the hardware_interface.
	// For each of them, create a Talon object. This
	// object is used to send and recieve commands
	// and status to/from the physical Talon motor
	// controller on the robot.  Use this pointer
	// to initialize each Talon with various params
	// set for that motor controller in config files.
	for (size_t i = 0; i < num_joints_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotHWInterface: Loading joint name: " << joint_names_[i] << " at hw ID " << joint_hw_ids_[i]);

		// Create joint state interface
		// Register as JointStateInterface so that legacy
		// ROS code which uses that object type can 
		// access basic state info from the talon
		// Code which needs more specific status should
		// get a TalonStateHandle instead.
		hardware_interface::TalonStateHandle tsh(joint_names_[i], &talon_state_[i]);
		joint_state_interface_.registerHandle(*(dynamic_cast<hardware_interface::JointStateHandle *>(&tsh)));
		talon_state_interface_.registerHandle(tsh);

		// Add command interfaces to joints
		// TODO: decide based on transmissions?
		hardware_interface::TalonCommandHandle talon_command_handle(tsh, &talon_command_[i]);
		talon_command_interface_.registerHandle(talon_command_handle);
	}
	registerInterface(&talon_state_interface_);
	registerInterface(&joint_state_interface_);
	registerInterface(&talon_command_interface_);

#if 0
  // Status
  joint_position_.resize(num_joints_, 0.0);
  joint_velocity_.resize(num_joints_, 0.0);
  joint_effort_.resize(num_joints_, 0.0);

  // Command
  joint_position_command_.resize(num_joints_, 0.0);
  joint_velocity_command_.resize(num_joints_, 0.0);
  joint_effort_command_.resize(num_joints_, 0.0);
#endif

  // Limits
  joint_position_lower_limits_.resize(num_joints_, 0.0);
  joint_position_upper_limits_.resize(num_joints_, 0.0);
  joint_velocity_limits_.resize(num_joints_, 0.0);
  joint_effort_limits_.resize(num_joints_, 0.0);

#if 0
  // Initialize interfaces for each joint
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
  {
    ROS_DEBUG_STREAM_NAMED(name_, "Loading joint name: " << joint_names_[joint_id]);

    // Create joint state interface
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
        joint_names_[joint_id], &joint_position_[joint_id], &joint_velocity_[joint_id], &joint_effort_[joint_id]));

    // Add command interfaces to joints
    // TODO: decide based on transmissions?
    hardware_interface::JointHandle joint_handle_position = hardware_interface::JointHandle(
        joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_position_command_[joint_id]);
    position_joint_interface_.registerHandle(joint_handle_position);

    hardware_interface::JointHandle joint_handle_velocity = hardware_interface::JointHandle(
        joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_velocity_command_[joint_id]);
    velocity_joint_interface_.registerHandle(joint_handle_velocity);

    hardware_interface::JointHandle joint_handle_effort = hardware_interface::JointHandle(
        joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_effort_command_[joint_id]);
#if 0 // KCJ : we don't have effort interface ability (yet)
    effort_joint_interface_.registerHandle(joint_handle_effort);
#endif

    // Load the joint limits
    registerJointLimits(joint_handle_position, joint_handle_velocity, joint_handle_effort, joint_id);
  }  // end for each joint


  registerInterface(&joint_state_interface_);     // From RobotHW base class.
  registerInterface(&position_joint_interface_);  // From RobotHW base class.
  registerInterface(&velocity_joint_interface_);  // From RobotHW base class.
  registerInterface(&effort_joint_interface_);    // From RobotHW base class.
#endif

  ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface Ready.");
}

void FRCRobotInterface::registerJointLimits(const hardware_interface::JointHandle &joint_handle_position,
                                             const hardware_interface::JointHandle &joint_handle_velocity,
                                             const hardware_interface::JointHandle &joint_handle_effort,
                                             std::size_t joint_id)
{
  // Default values
  joint_position_lower_limits_[joint_id] = -std::numeric_limits<double>::max();
  joint_position_upper_limits_[joint_id] = std::numeric_limits<double>::max();
  joint_velocity_limits_[joint_id] = std::numeric_limits<double>::max();
  joint_effort_limits_[joint_id] = std::numeric_limits<double>::max();

  // Limits datastructures
  joint_limits_interface::JointLimits joint_limits;     // Position
  joint_limits_interface::SoftJointLimits soft_limits;  // Soft Position
  bool has_joint_limits = false;
  bool has_soft_limits = false;

  // Get limits from URDF
  if (urdf_model_ == NULL)
  {
    ROS_WARN_STREAM_NAMED(name_, "No URDF model loaded, unable to get joint limits");
    return;
  }

  // Get limits from URDF
  urdf::JointConstSharedPtr urdf_joint = urdf_model_->getJoint(joint_names_[joint_id]);

  // Get main joint limits
  if (urdf_joint == NULL)
  {
    ROS_ERROR_STREAM_NAMED(name_, "URDF joint not found " << joint_names_[joint_id]);
    return;
  }

  // Get limits from URDF
  if (joint_limits_interface::getJointLimits(urdf_joint, joint_limits))
  {
    has_joint_limits = true;
    ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " has URDF position limits ["
                                                            << joint_limits.min_position << ", "
                                                            << joint_limits.max_position << "]");
    if (joint_limits.has_velocity_limits)
      ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " has URDF velocity limit ["
                                                              << joint_limits.max_velocity << "]");
  }
  else
  {
    if (urdf_joint->type != urdf::Joint::CONTINUOUS)
      ROS_WARN_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " does not have a URDF "
                            "position limit");
  }

  // Get limits from ROS param
  if (use_rosparam_joint_limits_)
  {
    if (joint_limits_interface::getJointLimits(joint_names_[joint_id], nh_, joint_limits))
    {
      has_joint_limits = true;
      ROS_DEBUG_STREAM_NAMED(name_,
                             "Joint " << joint_names_[joint_id] << " has rosparam position limits ["
                                      << joint_limits.min_position << ", " << joint_limits.max_position << "]");
      if (joint_limits.has_velocity_limits)
        ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id]
                                                                << " has rosparam velocity limit ["
                                                                << joint_limits.max_velocity << "]");
    }  // the else debug message provided internally by joint_limits_interface
  }

  // Get soft limits from URDF
  if (use_soft_limits_if_available_)
  {
    if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
    {
      has_soft_limits = true;
      ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " has soft joint limits.");
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " does not have soft joint "
                             "limits");
    }
  }

  // Quit we we haven't found any limits in URDF or rosparam server
  if (!has_joint_limits)
  {
    return;
  }

  // Copy position limits if available
  if (joint_limits.has_position_limits)
  {
    // Slighly reduce the joint limits to prevent floating point errors
    joint_limits.min_position += std::numeric_limits<double>::epsilon();
    joint_limits.max_position -= std::numeric_limits<double>::epsilon();

    joint_position_lower_limits_[joint_id] = joint_limits.min_position;
    joint_position_upper_limits_[joint_id] = joint_limits.max_position;
  }

  // Copy velocity limits if available
  if (joint_limits.has_velocity_limits)
  {
    joint_velocity_limits_[joint_id] = joint_limits.max_velocity;
  }

  // Copy effort limits if available
  if (joint_limits.has_effort_limits)
  {
    joint_effort_limits_[joint_id] = joint_limits.max_effort;
  }

  if (has_soft_limits)  // Use soft limits
  {
    ROS_DEBUG_STREAM_NAMED(name_, "Using soft saturation limits");
    const joint_limits_interface::PositionJointSoftLimitsHandle soft_handle_position(joint_handle_position,
                                                                                       joint_limits, soft_limits);
    pos_jnt_soft_limits_.registerHandle(soft_handle_position);
    const joint_limits_interface::VelocityJointSoftLimitsHandle soft_handle_velocity(joint_handle_velocity,
                                                                                       joint_limits, soft_limits);
    vel_jnt_soft_limits_.registerHandle(soft_handle_velocity);
    const joint_limits_interface::EffortJointSoftLimitsHandle soft_handle_effort(joint_handle_effort, joint_limits,
                                                                                   soft_limits);
    eff_jnt_soft_limits_.registerHandle(soft_handle_effort);
  }
  else  // Use saturation limits
  {
    ROS_DEBUG_STREAM_NAMED(name_, "Using saturation limits (not soft limits)");

    const joint_limits_interface::PositionJointSaturationHandle sat_handle_position(joint_handle_position, joint_limits);
    pos_jnt_sat_interface_.registerHandle(sat_handle_position);

    const joint_limits_interface::VelocityJointSaturationHandle sat_handle_velocity(joint_handle_velocity, joint_limits);
    vel_jnt_sat_interface_.registerHandle(sat_handle_velocity);

    const joint_limits_interface::EffortJointSaturationHandle sat_handle_effort(joint_handle_effort, joint_limits);
    eff_jnt_sat_interface_.registerHandle(sat_handle_effort);
  }
}

void FRCRobotInterface::reset()
{
  // Reset joint limits state, in case of mode switch or e-stop
  pos_jnt_sat_interface_.reset();
  pos_jnt_soft_limits_.reset();
}

void FRCRobotInterface::printState()
{
  // WARNING: THIS IS NOT REALTIME SAFE
  // FOR DEBUGGING ONLY, USE AT YOUR OWN ROBOT's RISK!
  ROS_INFO_STREAM_THROTTLE(1, 
		  std::endl << "State" << 
		  std::endl << printStateHelper());
}

std::string FRCRobotInterface::printStateHelper()
{
  std::stringstream ss;
  std::cout.precision(15);

  ss << "    position     velocity         effort  " << std::endl;
  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    ss << "j" << i << ": " << std::fixed << talon_state_[i].getPosition() << "\t ";
    ss << std::fixed << talon_state_[i].getSpeed() << "\t ";
    ss << std::fixed << talon_state_[i].getOutputVoltage() << std::endl;
  }
  return ss.str();
}

std::string FRCRobotInterface::printCommandHelper()
{
  std::stringstream ss;
  std::cout.precision(15);
  ss << "    setpoint" << std::endl;
  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    ss << "j" << i << ": " << std::fixed << talon_command_[i].get() << std::endl;
  }
  return ss.str();
}

void FRCRobotInterface::loadURDF(ros::NodeHandle &nh, std::string param_name)
{
  std::string urdf_string;
  urdf_model_ = new urdf::Model();

  // search and wait for robot_description on param server
  while (urdf_string.empty() && ros::ok())
  {
    std::string search_param_name;
    if (nh.searchParam(param_name, search_param_name))
    {
      ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
                            nh.getNamespace() << search_param_name);
      nh.getParam(search_param_name, urdf_string);
    }
    else
    {
      ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
                            nh.getNamespace() << param_name);
      nh.getParam(param_name, urdf_string);
    }

    usleep(100000);
  }

  if (!urdf_model_->initString(urdf_string))
    ROS_ERROR_STREAM_NAMED(name_, "Unable to load URDF model");
  else
    ROS_DEBUG_STREAM_NAMED(name_, "Received URDF from param server");
}

}  // namespace
