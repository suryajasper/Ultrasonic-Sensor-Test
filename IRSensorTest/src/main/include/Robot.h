/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/PIDController.h>
#include <frc/PIDOutput.h>
#include <frc/PIDSource.h>

class Robot : public frc::TimedRobot, public frc::PIDOutput {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

  frc::PIDSource* srcIR;

  double kP = 0; // proportional coefficient
  double kI = 0; // integral coefficient
  double kD = 0; // derivative coefficient
  double period = 0.05;
  
  double minDistance = 0;
  double maxDistance = 10;

  frc::PIDController* sensorIR = new frc::PIDController(kP, kI, kD, *srcIR, *this, period);

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
