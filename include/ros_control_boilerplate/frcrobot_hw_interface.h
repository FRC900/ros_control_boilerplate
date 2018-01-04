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

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the FRCRobot
           For a more detailed simulation example, see sim_hw_interface.h
*/

#pragma once

#include <thread>
#include <ros_control_boilerplate/frc_robot_interface.h>
#include <ctre/phoenix/MotorControl/CAN/TalonSRX.h>
#include <IterativeRobot.h>
#include <DriverStation.h>
#include <realtime_tools/realtime_publisher.h>
#include <NidecBrushless.h>

namespace frcrobot_control
{
class ROSIterativeRobot : public frc::IterativeRobot
{
public: 
	void StartCompetition(void) 
	{
		RobotInit();
		HAL_ObserveUserProgramStarting();
	}
	void OneIteration(void)
	{
		// wait for driver station data so the loop doesn't hog the CPU
		DriverStation::GetInstance().WaitForData();
		LoopFunc(); //-- added for 2018, code from that copied below
#if 0
		// Call the appropriate function depending upon the current robot mode
		if (IsDisabled()) {
			// call DisabledInit() if we are now just entering disabled mode from
			// either a different mode or from power-on
			if (m_lastMode != Mode::kDisabled) {
				LiveWindow::GetInstance()->SetEnabled(false);
				DisabledInit();
				m_lastMode = Mode::kDisabled;
			}
			HAL_ObserveUserProgramDisabled();
			DisabledPeriodic();
		} else if (IsAutonomous()) {
			// call AutonomousInit() if we are now just entering autonomous mode from
			// either a different mode or from power-on
			if (m_lastMode != Mode::kAutonomous) {
				LiveWindow::GetInstance()->SetEnabled(false);
				AutonomousInit();
				m_lastMode = Mode::kAutonomous;
			}
			HAL_ObserveUserProgramAutonomous();
			AutonomousPeriodic();
		} else if (IsOperatorControl()) {
			// call TeleopInit() if we are now just entering teleop mode from
			// either a different mode or from power-on
			if (m_lastMode != Mode::kTeleop) {
				LiveWindow::GetInstance()->SetEnabled(false);
				TeleopInit();
				m_lastMode = Mode::kTeleop;
				Scheduler::GetInstance()->SetEnabled(true);
			}
			HAL_ObserveUserProgramTeleop();
			TeleopPeriodic();
		} else {
			// call TestInit() if we are now just entering test mode from
			// either a different mode or from power-on
			if (m_lastMode != Mode::kTest) {
				LiveWindow::GetInstance()->SetEnabled(true);
				TestInit();
				m_lastMode = Mode::kTest;
			}
			HAL_ObserveUserProgramTest();
			TestPeriodic();
		}
		RobotPeriodic();
#endif
	}
private:
  //enum class Mode { kNone, kDisabled, kAutonomous, kTeleop, kTest };

  //Mode m_lastMode = Mode::kNone;

};

/// \brief Hardware interface for a robot
class FRCRobotHWInterface : public ros_control_boilerplate::FRCRobotInterface
{
public:
  enum FeedbackDevice
	{
		FeedbackDevice_Uninitialized,
		FeedbackDevice_QuadEncoder,
		FeedbackDevice_Analog,
		FeedbackDevice_Tachometer,
		FeedbackDevice_PulseWidthEncodedPosition,
		FeedbackDevice_SensorSum,
		FeedbackDevice_SensorDifference,
		FeedbackDevice_Inertial,
		FeedbackDevice_RemoteSensor,
		FeedbackDevice_SoftwareEmulatedSensor, 
		FeedbackDevice_CTRE_MagEncoder_Absolute = FeedbackDevice_PulseWidthEncodedPosition,
		FeedbackDevice_CTRE_MagEncoder_Relative = FeedbackDevice_QuadEncoder,
		FeedbackDevice_Last
	};
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  FRCRobotHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);
  ~FRCRobotHWInterface();

  /** \brief Initialize the hardware interface */
  virtual void init(void);

  /** \brief Read the state from the robot hardware. */
  virtual void read(ros::Duration &elapsed_time);

  virtual float convertPosition(FeedbackDevice encoder_feedback, int joint_id);
  virtual float convertVelocity(FeedbackDevice encoder_feedback, int joint_id);

  /** \brief Write the command to the robot hardware. */
  virtual void write(ros::Duration &elapsed_time);

  /** \breif Enforce limits for all values before writing */
  virtual void enforceLimits(ros::Duration &period);
 
protected:
  void hal_keepalive_thread(void);

private:
  bool convertControlMode(const hardware_interface::TalonMode input_mode,
						  ctre::phoenix::motorcontrol::ControlMode &output_mode);
  bool convertNeutralMode(const hardware_interface::NeutralMode input_mode, 
		  ctre::phoenix::motorcontrol::NeutralMode &output_mode);

  std::vector<std::shared_ptr<ctre::phoenix::motorcontrol::can::TalonSRX>> can_talons_;
  std::vector<std::shared_ptr<frc::NidecBrushless>> nidec_brushlesses_;

  std::thread hal_thread_;
  bool        run_hal_thread_;

  ROSIterativeRobot robot_;

};  // class

}  // namespace

