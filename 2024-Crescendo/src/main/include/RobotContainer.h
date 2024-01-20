// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc/GenericHID.h>
#include <frc2/command/Command.h>

#include "Constants.h"
#include "subsystems/SwerveDrive.h" 

using namespace OIConstants;

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  frc::GenericHID m_driverController{kDriverControllerPort};
  // frc::GenericHID m_operatorController{kOperatorControllerPort};

  SwerveDrive m_drive;


  void ConfigureBindings();
};
