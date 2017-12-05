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

#include <ros_control_boilerplate/frcrobot_hw_interface.h>
#include "HAL/DriverStation.h"
#include "HAL/HAL.h"
#include "Joystick.h"
#include "math.h"

//TODO Make nativeU configurable
int nativeU = 4096;   //native units of ctre magnetic encoders

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

		// Maybe e-stop, FMS attached, ds attached
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

	for (size_t i = 0; i < num_can_talon_srxs_; i++)
	{
		can_talons_.push_back(std::make_shared<CTRE::MotorControl::SmartMotorController>(can_talon_srx_can_ids_[i] /*, CAN update rate*/ ));

		// Need config information for each talon
		// Should probably be part of YAML params for controller
		// set initial mode - no, depend on controller
		// set PIDF constants for both slots (read from nh params) - no, set from controller if needed
		// set close loop ramp rate - same as above
		// set voltage compensation rate
		// set soft limits - forward/reverse limits and enables - yes
		// set limit switch config - enable, NO/NC  - probably yes
		// set encoder config / reverse  - yes

		can_talons_[i]->Disable(); // Make sure motor is stopped
		can_talon_enabled_.push_back(false);
	}
	for (size_t i = 0; i < num_nidec_brushlesses_; i++)
	{
		nidec_brushlesses_.push_back(std::make_shared<frc::NidecBrushless>(nidec_brushless_pwm_channels_[i], nidec_brushless_dio_channels_[i]));
	}
	ROS_INFO_NAMED("frcrobot_hw_interface", "FRCRobotHWInterface Ready.");
}

void FRCRobotHWInterface::read(ros::Duration &/*elapsed_time*/)
{
  for (std::size_t joint_id = 0; joint_id < num_can_talon_srxs_; ++joint_id)
  {
	  // read position and velocity from can_talons_[joint_id]
	  // convert to whatever units make sense
	  //
	  // TODO : convert to units which make sense
	  // for rest of code?
	  talon_state_[joint_id].setPosition(can_talons_[joint_id]->GetPosition()/4096*2*M_PI);
	  talon_state_[joint_id].setSpeed(can_talons_[joint_id]->GetSpeed()/4096*2*M_PI/.1);
	  talon_state_[joint_id].setOutputVoltage(can_talons_[joint_id]->GetOutputVoltage());
	  talon_state_[joint_id].setOutputCurrent(can_talons_[joint_id]->GetOutputCurrent());
	  talon_state_[joint_id].setBusVoltage(can_talons_[joint_id]->GetBusVoltage());
	  talon_state_[joint_id].setClosedLoopError(can_talons_[joint_id]->GetClosedLoopError());
	  talon_state_[joint_id].setFwdLimitSwitch(can_talons_[joint_id]->IsFwdLimitSwitchClosed());
	  talon_state_[joint_id].setRevLimitSwitch(can_talons_[joint_id]->IsRevLimitSwitchClosed());
  }
  for (size_t i = 0; i < num_nidec_brushlesses_; i++)
  {
	  // TODO : Figure out which of these the setpoint
	  // actually is...
	  brushless_pos_[i] = 
	  brushless_vel_[i] = 
	  brushless_eff_[i] = nidec_brushlesses_[i]->Get();
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

  for (std::size_t joint_id = 0; joint_id < num_can_talon_srxs_; ++joint_id)
  {
	  // Set talon control mode if it has changed 
	  // but only if it has changed since the last
	  // pass through write()
	  hardware_interface::TalonMode in_mode;
	  CTRE::MotorControl::ControlMode::SmartControlMode out_mode;
	  if (talon_command_[joint_id].newMode(in_mode) &&
		  convertControlMode(in_mode, out_mode))
	  {
		can_talons_[joint_id]->SetControlMode(out_mode);
		// Enable motor first time mode is set
		// and leave it enabled after that
		if (can_talon_enabled_[joint_id] == false)
		{
			can_talons_[joint_id]->Enable();
			can_talon_enabled_[joint_id] = true;
		}
		talon_state_[joint_id].setTalonMode(in_mode);
	  }
	  int slot;
	  if(talon_command_[joint_id].slotChanged(slot))
	  {
		  can_talons_[joint_id]->SelectProfileSlot(slot);
		  talon_state_[joint_id].setSlot(slot);
	  }
	  int lastSlot = -1;

	  double p;
	  double i;
	  double d;
	  double f;
	  unsigned iz;
	  for (int j = 0; j < 2; j++) {
		  if(talon_command_[joint_id].pidfChanged(p, i, d, f, iz, j))
		  {
			can_talons_[joint_id]->SelectProfileSlot(j);
			lastSlot = j;
			can_talons_[joint_id]->SetPID(p, i, d, f);
			can_talons_[joint_id]->SetIzone(iz);
	
			talon_state_[joint_id].setPidfP(p, j);
			talon_state_[joint_id].setPidfI(i, j);
			talon_state_[joint_id].setPidfD(d, j);
			talon_state_[joint_id].setPidfF(f, j);
			talon_state_[joint_id].setPidfIzone(iz, j);
	  	}
	  }
	  if (slot != lastSlot) can_talons_[joint_id]->SelectProfileSlot(slot);	  

	  bool invert;
	  bool invert_sensor_direction;
	  if(talon_command_[joint_id].invertChanged(invert, invert_sensor_direction))
	  {
		  can_talons_[joint_id]->SetInverted(invert);
		  can_talons_[joint_id]->SetSensorDirection(invert_sensor_direction);
		  talon_state_[joint_id].setInvert(invert);
		  talon_state_[joint_id].setInvertSensorDirection(invert_sensor_direction);
	  }

	  // TODO : check that mode has been initialized, if not
	  // skip over writing command since the higher
	  // level code hasn't requested we do anything with
	  // the motor yet?

	  // Read current commanded setpoint and write it
	  // to the actual robot HW
	  double command;
	  if (talon_command_[joint_id].get(command))
	  {
          switch (out_mode) {
            case CTRE::MotorControl::ControlMode::kSpeed:
                command = command/2/M_PI*nativeU*.1; //assumes input value is position for 100ms there is a chance it is supposed to be 10ms
                break;
            case CTRE::MotorControl::ControlMode::kPosition:
                command = command/2/M_PI*nativeU;
                break;
          }
		  can_talons_[joint_id]->Set(command);
		  talon_state_[joint_id].setSetpoint(command);
          
	  }
  }
  for (size_t i = 0; i < num_nidec_brushlesses_; i++)
  {
	  nidec_brushlesses_[i]->Set(brushless_command_[i]);
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

// Convert from internal version of hardware mode ID
// to one to write to actual Talon hardware
// Return true if conversion is OK, false if
// an unknown mode is hit.
bool FRCRobotHWInterface::convertControlMode(
		const hardware_interface::TalonMode input_mode, 
		CTRE::MotorControl::ControlMode::SmartControlMode &output_mode)
{
	switch (input_mode)
	{
		case hardware_interface::TalonMode_PercentVbus:
			output_mode = CTRE::MotorControl::ControlMode::kPercentVbus;
			break;
		case hardware_interface::TalonMode_Position:      // CloseLoop
			output_mode = CTRE::MotorControl::ControlMode::kPosition;
			break;
		case hardware_interface::TalonMode_Speed:         // CloseLoop
			output_mode = CTRE::MotorControl::ControlMode::kSpeed;
			break;
		case hardware_interface::TalonMode_Current:       // CloseLoop
			output_mode = CTRE::MotorControl::ControlMode::kCurrent;
			break;
		case hardware_interface::TalonMode_Voltage:
			output_mode = CTRE::MotorControl::ControlMode::kVoltage;
			break;
		case hardware_interface::TalonMode_Follower:
			output_mode = CTRE::MotorControl::ControlMode::kFollower;
			break;
		case hardware_interface::TalonMode_MotionProfile:
			output_mode = CTRE::MotorControl::ControlMode::kMotionProfile;
			break;
		case hardware_interface::TalonMode_MotionMagic:
			output_mode = CTRE::MotorControl::ControlMode::kMotionMagic;
			break;
		default:
			output_mode = CTRE::MotorControl::ControlMode::kDisabled;
			ROS_WARN("Unknown mode seen in HW interface");
			return false;
	}

	return true;
}

}  // namespace
