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
//#include "GenericHID.h"
#include "math.h"

//TODO Make nativeU configurable
int nativeU = 4096;   //native units of ctre magnetic encoders
//RG: More than just making nativeU configurable, we should consider a much more automated system
//i.e. set conversion factor based on specified feedback sensor
//note that you can add a conversion factors that will automatically be applied for speed and position

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
		
		//TODO: match gets with correct labels
		joystick_state_[0].leftStickX  = joystick.GetRawAxis(0);
		joystick_state_[0].leftStickY  = joystick.GetRawAxis(1);
		joystick_state_[0].rightStickX = joystick.GetRawAxis(4);
		joystick_state_[0].rightStickY = joystick.GetRawAxis(5);
		joystick_state_[0].leftTrigger = joystick.GetRawAxis(2);
		joystick_state_[0].rightTrigger= joystick.GetRawAxis(3);
		
		joystick_state_[0].buttonA.button   	 = joystick.GetRawButton(1);
		joystick_state_[0].buttonA.press    	 = joystick.GetRawButtonPressed(1);
		joystick_state_[0].buttonA.release  	 = joystick.GetRawButtonReleased(1);
		
		joystick_state_[0].buttonB.button   	 = joystick.GetRawButton(2);
		joystick_state_[0].buttonB.press    	 = joystick.GetRawButtonPressed(2);
		joystick_state_[0].buttonB.release  	 = joystick.GetRawButtonReleased(2);
		
		joystick_state_[0].buttonX.button   	 = joystick.GetRawButton(3);
		joystick_state_[0].buttonX.press    	 = joystick.GetRawButtonPressed(3);
		joystick_state_[0].buttonX.release  	 = joystick.GetRawButtonReleased(3);
		
		joystick_state_[0].buttonY.button   	 = joystick.GetRawButton(4);
		joystick_state_[0].buttonY.press    	 = joystick.GetRawButtonPressed(4);
		joystick_state_[0].buttonY.release  	 = joystick.GetRawButtonReleased(4);
		
		joystick_state_[0].bumperLeft.button   	 = joystick.GetRawButton(5);
		joystick_state_[0].bumperLeft.press    	 = joystick.GetRawButtonPressed(5);
		joystick_state_[0].bumperLeft.release  	 = joystick.GetRawButtonReleased(5);
		
		joystick_state_[0].bumperRight.button    = joystick.GetRawButton(6);
		joystick_state_[0].bumperRight.press     = joystick.GetRawButtonPressed(6);
		joystick_state_[0].bumperRight.release   = joystick.GetRawButtonReleased(6);

		joystick_state_[0].buttonBack.button   	 = joystick.GetRawButton(7);
		joystick_state_[0].buttonBack.press    	 = joystick.GetRawButtonPressed(7);
		joystick_state_[0].buttonBack.release  	 = joystick.GetRawButtonReleased(7);
		
		joystick_state_[0].buttonStart.button    = joystick.GetRawButton(8);
		joystick_state_[0].buttonStart.press   	 = joystick.GetRawButtonPressed(8);
		joystick_state_[0].buttonStart.release 	 = joystick.GetRawButtonReleased(8);
		
		joystick_state_[0].stickLeft.button	     = joystick.GetRawButton(9);
		joystick_state_[0].stickLeft.press   	 = joystick.GetRawButtonPressed(9);
		joystick_state_[0].stickLeft.release 	 = joystick.GetRawButtonReleased(9);
		
		joystick_state_[0].stickRight.button     = joystick.GetRawButton(10);
		joystick_state_[0].stickRight.press   	 = joystick.GetRawButtonPressed(10);
		joystick_state_[0].stickRight.release 	 = joystick.GetRawButtonReleased(10);
		
		//TODO: Add direction buttons
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
		can_talons_.push_back(std::make_shared<CTRE::MotorControl::CAN::TalonSRX>(can_talon_srx_can_ids_[i] /*, CAN update rate*/ ));

		// Need config information for each talon
		// Should probably be part of YAML params for controller
		// set close loop ramp rate - same as above
		// set voltage compensation rate
		// set soft limits - forward/reverse limits and enables - yes
		// set limit switch config - enable, NO/NC  - probably yes

		can_talons_[i]->Set(ControlMode::Disabled, 0); // Make sure motor is stopped
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
	  // for rest of code? Add a method which takes current
	  // mode, encoder choice and maybe a user-configurable ticks/rotation
	  // setting and converts from native units to radians (for position)
	  // and radians/src (for velocity)
	  talon_state_[joint_id].setPosition(can_talons_[joint_id]->GetSelectedSensorPosition()/4096.*2*M_PI);
	  talon_state_[joint_id].setSpeed(can_talons_[joint_id]->GetSelectedSensorVelocity()/4096.*2*M_PI/.1);

	  float bus_voltage;
	  can_talons_[joint_id]->GetBusVoltage(bus_voltage);
	  talon_state_[joint_id].setBusVoltage(bus_voltage);
	  float motor_output_percent;
	  can_talons_[joint_id]->GetMotorOutputPercent(motor_output_percent);
	  talon_state_[joint_id].setMotorOutputPercent(motor_output_percent);
	  float output_voltage;
	  can_talons_[joint_id]->GetMotorOutputVoltage(output_voltage);
	  talon_state_[joint_id].setOutputVoltage(output_voltage);
	  float output_current;
	  can_talons_[joint_id]->GetOutputCurrent(output_current);
	  talon_state_[joint_id].setOutputCurrent(output_current);

	  // Scale this from native units 
	  int closed_loop_error;
	  can_talons_[joint_id]->GetClosedLoopError(closed_loop_error);
	  talon_state_[joint_id].setClosedLoopError(closed_loop_error);

	  float integral_accumulator;
	  can_talons_[joint_id]->GetIntegralAccumulator(integral_accumulator);
	  talon_state_[joint_id].setIntegralAccumulator(integral_accumulator);

	  float error_derivative;
	  can_talons_[joint_id]->GetErrorDerivative(error_derivative);
	  talon_state_[joint_id].setErrorDerivative(error_derivative);

	  // TODO :: Fix me
	  //talon_state_[joint_id].setFwdLimitSwitch(can_talons_[joint_id]->IsFwdLimitSwitchClosed());
	  //talon_state_[joint_id].setRevLimitSwitch(can_talons_[joint_id]->IsRevLimitSwitchClosed());
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

//get rid of magic numbers
float FRCRobotHWInterface::convertPosition(FeedbackDevice encoder_feedback, int joint_id) //convert to radians //how to include talon_mode?
{
	float sensor_position = can_talons_[joint_id]->GetSelectedSensorPosition();
	switch(encoder_feedback)
	{
		case FeedbackDevice_QuadEncoder:
		case FeedbackDevice_PulseWidthEncodedPosition:
			return sensor_position * 2*M_PI/4056; //4056 = 4* encoder cycles per revolution
		case FeedbackDevice_Analog: //depends on the encoder voltage //this actually seems like it outputs voltage? //wraps around after 1023
			return (sensor_position - 1024*floor(sensor_position/1024)) * 2*M_PI; //also this gives percent of full voltage instead of position?
		case FeedbackDevice_Tachometer:
		case FeedbackDevice_SensorSum:
		case FeedbackDevice_SensorDifference:
		case FeedbackDevice_Inertial:
		case FeedbackDevice_RemoteSensor:
		case FeedbackDevice_SoftwareEmulatedSensor:
			ROS_WARN_STREAM("Unable to convert units. Native units returned.");
			return sensor_position;
		default:
			ROS_WARN_STREAM("Invalid encoder feedback device. Native units returned.");
			return sensor_position;
	}
}

float FRCRobotHWInterface::convertVelocity(FeedbackDevice encoder_feedback, int joint_id) //convert to radians/sec from native units/.1sec
{
	float sensor_velocity = can_talons_[joint_id]->GetSelectedSensorVelocity();
	switch(encoder_feedback)
	{
		case FeedbackDevice_QuadEncoder:
		case FeedbackDevice_PulseWidthEncodedPosition:
			return sensor_velocity * 2*M_PI/4056/.1; //4056 = 4* encoder cycles per revolution
		case FeedbackDevice_Analog: //depends on the encoder voltage //this actually seems like it outputs voltage? //wraps around after 1023
			return (sensor_velocity - 1024*floor(sensor_velocity/1024)) * 2*M_PI/.1;
		case FeedbackDevice_Tachometer:
		case FeedbackDevice_SensorSum:
		case FeedbackDevice_SensorDifference:
		case FeedbackDevice_Inertial:
		case FeedbackDevice_RemoteSensor:
		case FeedbackDevice_SoftwareEmulatedSensor:
			ROS_WARN_STREAM("Unable to convert units. Native units returned.");
			return sensor_velocity;
		default:
			ROS_WARN_STREAM("Invalid encoder feedback device. Native units returned.");
			return sensor_velocity;
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
	  int slot;
	  if(talon_command_[joint_id].slotChanged(slot))
	  {
		  can_talons_[joint_id]->SelectProfileSlot(slot);
		  talon_state_[joint_id].setSlot(slot);
	  }

	  float p;
	  float i;
	  float d;
	  float f;
	  int   iz;
	  int   allowable_closed_loop_error;
	  float max_integral_accumulator;
	  for (int j = 0; j < 2; j++) {
		  if(talon_command_[joint_id].pidfChanged(p, i, d, f, iz, allowable_closed_loop_error, max_integral_accumulator, j))
		  {
			can_talons_[joint_id]->Config_kP(j, p, 0);
			can_talons_[joint_id]->Config_kI(j, i, 0);
			can_talons_[joint_id]->Config_kD(j, d, 0);
			can_talons_[joint_id]->Config_kF(j, f, 0);
			can_talons_[joint_id]->Config_IntegralZone(j, iz, 0);
			can_talons_[joint_id]->ConfigAllowableClosedloopError(j, allowable_closed_loop_error, 0);
			can_talons_[joint_id]->ConfigMaxIntegralAccumulator(j, max_integral_accumulator, 0);

			talon_state_[joint_id].setPidfP(p, j);
			talon_state_[joint_id].setPidfI(i, j);
			talon_state_[joint_id].setPidfD(d, j);
			talon_state_[joint_id].setPidfF(f, j);
			talon_state_[joint_id].setPidfIzone(iz, j);
			talon_state_[joint_id].setAllowableClosedLoopError(allowable_closed_loop_error, j);
			talon_state_[joint_id].setMaxIntegralAccumulator(max_integral_accumulator, j);
	  	}
	  }

	  bool invert;
	  bool sensor_phase;
	  if(talon_command_[joint_id].invertChanged(invert, sensor_phase))
	  {
		  can_talons_[joint_id]->SetInverted(invert);
		  can_talons_[joint_id]->SetSensorPhase(sensor_phase);
		  talon_state_[joint_id].setInvert(invert);
		  talon_state_[joint_id].setSensorPhase(sensor_phase);
	  }
	  
	  hardware_interface::NeutralMode neutral_mode;
	  NeutralMode ctre_neutral_mode;

	  if(talon_command_[joint_id].neutralModeChanged(neutral_mode) && 
		 convertNeutralMode(neutral_mode, ctre_neutral_mode))
	  {
		  can_talons_[joint_id]->SetNeutralMode(ctre_neutral_mode);
		  talon_state_[joint_id].setNeutralMode(neutral_mode);
	  }

	  if (talon_command_[joint_id].neutralOutputChanged())
	  {
		  can_talons_[joint_id]->NeutralOutput();
		  talon_state_[joint_id].setNeutralOutput(true);
	  }

	  float iaccum;
	  if (talon_command_[joint_id].integralAccumulatorChanged(iaccum))
	  {
		  can_talons_[joint_id]->SetIntegralAccumulator(iaccum, 0);
		  // Do not set talon state - this changes
		  // dynamically so read it in read() above instead
	  }


	  float closed_loop_ramp;
	  float open_loop_ramp;
	  float peak_output_forward;
	  float peak_output_reverse;
	  float nominal_output_forward;
	  float nominal_output_reverse;
	  float neutral_deadband;
	  if (talon_command_[joint_id].outputShapingChanged(closed_loop_ramp,
														open_loop_ramp,
														peak_output_forward,
														peak_output_reverse,
														nominal_output_forward,
														nominal_output_reverse,
														neutral_deadband))
	  {
		  can_talons_[joint_id]->ConfigOpenloopRamp(open_loop_ramp, 0);
		  can_talons_[joint_id]->ConfigClosedloopRamp(closed_loop_ramp, 0);
		  can_talons_[joint_id]->ConfigPeakOutputForward(peak_output_forward, 0);
		  can_talons_[joint_id]->ConfigPeakOutputReverse(peak_output_reverse, 0);
		  can_talons_[joint_id]->ConfigNominalOutputForward(nominal_output_forward, 0);
		  can_talons_[joint_id]->ConfigNominalOutputReverse(nominal_output_reverse, 0);
		  can_talons_[joint_id]->ConfigNeutralDeadband(neutral_deadband, 0);

		  talon_state_[joint_id].setOpenloopRamp(open_loop_ramp);
		  talon_state_[joint_id].setClosedloopRamp(closed_loop_ramp);
		  talon_state_[joint_id].setPeakOutputForward(peak_output_forward);
		  talon_state_[joint_id].setPeakOutputReverse(peak_output_reverse);
		  talon_state_[joint_id].setNominalOutputForward(nominal_output_forward);
		  talon_state_[joint_id].setNominalOutputReverse(nominal_output_reverse);
	  }
	  float v_c_saturation;
	  int v_measurement_filter;
	  bool v_c_enable;
	  if (talon_command_[joint_id].VoltageCompensationChanged(v_c_saturation,
															  v_measurement_filter,
															  v_c_enable))
	  {
		  can_talons_[joint_id]->ConfigVoltageCompSaturation(v_c_saturation, 0);
		  can_talons_[joint_id]->ConfigVoltageMeasurementFilter(v_measurement_filter, 0);
		  can_talons_[joint_id]->EnableVoltageCompensation(v_c_enable);

		  talon_state_[joint_id].setVoltageCompensationSaturation(v_c_saturation);
		  talon_state_[joint_id].setVoltageMeasurementFilter(v_measurement_filter);
		  talon_state_[joint_id].setVoltageCompensationEnable(v_c_enable);

	  }


	  // Set new motor setpoint if either the mode or
	  // the setpoint has been changed 
	  float command;
	  hardware_interface::TalonMode in_mode;
	  ControlMode out_mode;
	  if ((talon_command_[joint_id].newMode(in_mode) || 
	       talon_command_[joint_id].get(command) ) &&
	      convertControlMode(in_mode, out_mode))
	  {
		  switch (out_mode) {
			  case ControlMode::Velocity:
				  command = command/2./M_PI*nativeU*.1; //assumes input value is velocity per 100ms there is a chance it is supposed to be 10ms
				  //RG: I am almost certain that it isn't 10 ms. However, if you configure some of the units
				  //using one of the talon functions,  the units are RPM and Rotations
				  break;
			  case ControlMode::Position:
				  command = command/2./M_PI*nativeU;
				  break;
		  }
		  can_talons_[joint_id]->Set(out_mode, command);
		  talon_state_[joint_id].setTalonMode(in_mode);
		  talon_state_[joint_id].setSetpoint(command);
		  talon_state_[joint_id].setNeutralOutput(false); // maybe make this a part of setSetpoint?
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
		ControlMode &output_mode)
{
	switch (input_mode)
	{
		case hardware_interface::TalonMode_PercentOutput:
			output_mode = ControlMode::PercentOutput;
			break;
		case hardware_interface::TalonMode_Position:      // CloseLoop
			output_mode = ControlMode::Position;
			break;
		case hardware_interface::TalonMode_Velocity:      // CloseLoop
			output_mode = ControlMode::Velocity;
			break;
		case hardware_interface::TalonMode_Current:       // CloseLoop
			output_mode = ControlMode::Current;
			break;
		case hardware_interface::TalonMode_Follower:
			output_mode = ControlMode::Follower;
			break;
		case hardware_interface::TalonMode_MotionProfile:
			output_mode = ControlMode::MotionProfile;
			break;
		case hardware_interface::TalonMode_MotionMagic:
			output_mode = ControlMode::MotionMagic;
			break;
		case hardware_interface::TalonMode_TimedPercentOutput:
			output_mode = ControlMode::TimedPercentOutput;
			break;
		case hardware_interface::TalonMode_Disabled:
			output_mode = ControlMode::Disabled;
			break;
		default:
			output_mode = ControlMode::Disabled;
			ROS_WARN("Unknown mode seen in HW interface");
			return false;
	}

	return true;
}

bool FRCRobotHWInterface::convertNeutralMode(
		const hardware_interface::NeutralMode input_mode, 
		NeutralMode &output_mode)
{
	switch (input_mode)
	{
		case hardware_interface::NeutralMode_EEPROM_Setting:
			output_mode = EEPROMSetting;
			break;
		case hardware_interface::NeutralMode_Coast:
			output_mode = Coast;
			break;
		case hardware_interface::NeutralMode_Brake:
			output_mode = Brake;
			break;
		default:
			output_mode = EEPROMSetting;
			ROS_WARN("Unknown neutral mode seen in HW interface");
			return false;
	}

	return true;
}

}  // namespace
