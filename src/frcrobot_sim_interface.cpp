/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Original Author: Dave Coleman
Desc:   Example ros_control hardware interface blank template for the FRCRobot
For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include <ros_control_boilerplate/frcrobot_sim_interface.h>

namespace frcrobot_control
{

FRCRobotSimInterface::FRCRobotSimInterface(ros::NodeHandle &nh, 
		                                   urdf::Model *urdf_model)
	: ros_control_boilerplate::FRCRobotInterface(nh, urdf_model)
{
	// Loop through the list of joint names
	// specified as params for the hardware_interface.
	// For each of them, create a Talon object. This
	// object is used to send and recieve commands
	// and status to/from the physical Talon motor
	// controller on the robot.  Use this pointer
	// to initialize each Talon with various params
	// set for that motor controller in config files.
	for (size_t i = 0; i < can_talon_srx_can_ids_.size(); i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_sim_interface", 
				"Loading joint " << can_talon_srx_names_[i] << 
				" as CAN id " << can_talon_srx_can_ids_[i]);

		// Need config information for each talon
		// Should probably be part of YAML params for controller
		// set initial mode
		// set PIDF constants for both slots (read from nh params)
		// set close loop ramp rate
		// set voltage compensation rate
		// set soft limits - forward/reverse limits and enables
		// set limit switch config - enable, NO/NC
		// set encoder config / reverse

		//can_talons_[i]->Set(0.0); // Make sure motor is stopped
		// TODO : Grab initial mode from config file?
		// Or maybe set it to disabled and require the higher
		// level controller to request a mode on init?
		//int rc = can_talons_[i]->SetModeSelect(CanTalonSRX::kMode_DutyCycle);
		//if (rc != CTR_OKAY)
		//ROS_WARN("*** setModeSelect() failed with %d", rc);
	}
	ROS_INFO_NAMED("frcrobot_sim_interface", "FRCRobotSimInterface Ready.");
}

void FRCRobotSimInterface::read(ros::Duration &/*elapsed_time*/)
{
	// Simulated state is updated in write, so just
	// display it here for debugging

	printState();
}

void FRCRobotSimInterface::write(ros::Duration &elapsed_time)
{
	// Safety - should be using Talon Sim to control this
	// Maybe do a eStop / enabled check instead?
	//enforceLimits(elapsed_time);

	ROS_INFO_STREAM_THROTTLE(1, 
			std::endl << std::string(__FILE__) << ":" << __LINE__ << 
			std::endl << "Command" << std::endl << printCommandHelper());

	for (std::size_t joint_id = 0; joint_id < num_talon_srxs_; ++joint_id)
	{
		// Assume instant acceleration for now
		double speed = talon_command_[joint_id].get();
		talon_state_[joint_id].setPosition(talon_state_[joint_id].getPosition() + speed * elapsed_time.toSec());
		talon_state_[joint_id].setSpeed(speed);
	}
}

void FRCRobotSimInterface::enforceLimits(ros::Duration &period)
{
	// ----------------------------------------------------
	// ----------------------------------------------------
	// ----------------------------------------------------
	//
	// CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
	// YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
	// DEPENDING ON YOUR CONTROL METHOD
	//
	// EXAMPLES:
	//
	// Saturation Limits ---------------------------
	//
	// Enforces position and velocity
	pos_jnt_sat_interface_.enforceLimits(period);
	//
	// Enforces velocity and acceleration limits
	// vel_jnt_sat_interface_.enforceLimits(period);
	//
	// Enforces position, velocity, and effort
	// eff_jnt_sat_interface_.enforceLimits(period);

	// Soft limits ---------------------------------
	//
	// pos_jnt_soft_limits_.enforceLimits(period);
	// vel_jnt_soft_limits_.enforceLimits(period);
	// eff_jnt_soft_limits_.enforceLimits(period);
	//
	// ----------------------------------------------------
	// ----------------------------------------------------
	// ----------------------------------------------------
}

}  // namespace
