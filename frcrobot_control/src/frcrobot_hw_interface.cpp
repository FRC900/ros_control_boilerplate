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

#include <iostream>
#include <thread>

#include <frcrobot_control/frcrobot_hw_interface.h>
#include "HAL/DriverStation.h"
#include "HAL/HAL.h"
#include "Joystick.h"

namespace frcrobot_control
{

FRCRobotHWInterface::FRCRobotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
  : ros_control_boilerplate::FRCRobotInterface(nh, urdf_model),
	run_hal_thread_(true)
{
}

FRCRobotHWInterface::~FRCRobotHWInterface()
{
	run_hal_thread_ = false;
	hal_thread_.join();
}

void FRCRobotHWInterface::hal_keepalive_thread(void) {
	// Just throw a basic IterativeRobot in here instead?
	run_hal_thread_ = true;
	Joystick joystick(0);
	while (run_hal_thread_) {
		robot_.OneIteration();
		// Things to keep track of
		//    Alliance Station Id
		//    Robot / match mode (auto, teleop, test, disabled)
		//    Match time
		match_time_state_ = DriverStation::GetInstance().GetMatchTime();
		//    Joystick state
		//    This is for testing. Need to expand this
		//    to include buttons, the ability to set
		//    rumble state via an external joystick
		//    controller, etc.  Need to set via config
		//    files.
		joystick_state_[0].x = joystick.GetX();
		joystick_state_[0].y = joystick.GetY();
		joystick_state_[0].z = joystick.GetZ();
		std::cout << "Joystick_state = " << joystick_state_[0].x << " "
			<< joystick_state_[0].y << " "
			<< joystick_state_[0].z << " " << std::endl;
		//    Maybe e-stop, FMS attached, ds attached
		// Could do most of these via dummy joint handles. Since
		// they read-only, create bogus state handles for them
		// pointing to member vars in the FRCRobotInterface / FRCRobotHWInterface
	}
}

void FRCRobotHWInterface::init(void)
{
	// Do base class init. This loads common interface info
	// used by both the real and sim interfaces
	FRCRobotInterface::init();

	// Make sure to initialize WPIlib code before creating
	// a CAN Talon object to avoid NIFPGA: Resource not initialized
	// errors? See https://www.chiefdelphi.com/forums/showpost.php?p=1640943&postcount=3
	robot_.StartCompetition();
	hal_thread_ = std::thread(&FRCRobotHWInterface::hal_keepalive_thread, this);

	for (size_t i = 0; i < num_joints_; i++)
	{
		can_talons_.push_back(std::make_shared<CanTalonSRX>(joint_hw_ids_[i] /*, CAN update rate*/ ));

		// Need config information for each talon
		// Should probably be part of YAML params for controller
		// set initial mode
		// set PIDF constants for both slots (read from nh params)
		// set close loop ramp rate
		// set voltage compensation rate
		// set soft limits - forward/reverse limits and enables
		// set limit switch config - enable, NO/NC
		// set encoder config / reverse

		can_talons_[i]->Set(0.0); // Make sure motor is stopped
		// TODO : Grab initial mode from config file?
		// Or maybe set it to disabled and require the higher
		// level controller to request a mode on init?
		int rc = can_talons_[i]->SetModeSelect(CanTalonSRX::kMode_DutyCycle);
		if (rc != CTR_OKAY)
			ROS_WARN("*** setModeSelect() failed with %d", rc);
		// Keep internal hw state in sync with cached hw state?
		//talon_command_[i].SetMode(foo);
	}
	ROS_INFO_NAMED("frcrobot_hw_interface", "FRCRobotHWInterface Ready.");
}

void FRCRobotHWInterface::read(ros::Duration &/*elapsed_time*/)
{
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
  {
	  // read position and velocity from can_talons_[joint_id]
	  // convert to whatever units make sense
	  //
	  // TODO : convert to units which make sense
	  // for rest of code?
	  int pos;
	  int vel;
	  if ((can_talons_[joint_id]->GetEncPosition(pos) == CTR_OKAY) &&
		  (can_talons_[joint_id]->GetEncVel(vel)      == CTR_OKAY ))
	  {
		  // Save these for now so joint_state_publisher works?
		  // Check for a better way to do this
		  //joint_position_[joint_id] = pos;
		  //joint_velocity_[joint_id] = vel;
		  talon_state_[joint_id].setPosition(pos);
		  talon_state_[joint_id].setSpeed(vel);
	  }
  }
}

void FRCRobotHWInterface::write(ros::Duration &elapsed_time)
{
  // Safety - should be using Talon HW to control this
  // Maybe do a eStop / enabled check instead?
  //enforceLimits(elapsed_time);

  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
//	ROS_INFO_STREAM_THROTTLE(1, std::endl << std::string(__FILE__) << ":" << __LINE__ << 
//			                    std::endl << printCommandHelper());

  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
  {
	  // Set talon control mode if it has changed 
	  // but only if it has changed since the last
	  // pass through write()
	  hardware_interface::TalonMode mode;
	  if (talon_command_[joint_id].newMode(mode))
	  {
		can_talons_[joint_id]->Set(0.0); // Make sure motor is stopped 
		int rc = can_talons_[joint_id]->SetModeSelect(int (mode));
		if (rc != CTR_OKAY)
			ROS_WARN("*** setModeSelect() failed with %d", rc);
	  }

	  // TODO : check that mode has been initialized, if not
	  // skip over writing command since the higher
	  // level code hasn't requested we do anything with
	  // the motor yet?

	  // Read current commanded setpoint and write it
	  // to the actual robot HW
	  can_talons_[joint_id]->Set(talon_command_[joint_id].get());
  }
  // END DUMMY CODE
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

void FRCRobotHWInterface::enforceLimits(ros::Duration &period)
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
