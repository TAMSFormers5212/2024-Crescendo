// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc/GenericHID.h>
#include <frc2/command/Command.h>

#include "Constants.h"
#include "subsystems/SwerveDrive.h" 
#include "subsystems/VisionSubsystem.h"
#include "subsystems/Superstructure.h"
#include "subsystems/Winch.h"
#include <frc/geometry/Rotation2d.h>

using namespace OIConstants;

class RobotContainer {
 public:
  RobotContainer();
    double RotAxis;
    double speedMultiplier;
    double XAxis;
    double YAxis;
  frc2::CommandPtr GetAutonomousCommand();
  // frc2::Rotation2d getRotated();
  // frc2::CommandPtr getAutonomousCommand();
Superstructure m_superstructure;
 private:
  frc::GenericHID m_driverController{kDriverControllerPort};
  frc::GenericHID m_operatorController{kOperatorControllerPort};

  // std::unique_ptr<frc2::Command> exampleAuto;
  // std::unique_ptr<frc2::Command> pathfindToPickup;
  // std::unique_ptr<frc2::Command> pathfindToScore;
  // std::unique_ptr<frc2::Command> onTheFly;
  // std::unique_ptr<frc2::Command> followOnTheFly;

  SwerveDrive m_drive;


  VisionSubsystem m_vision;
  //Superstructure m_superstructure;
  void ConfigureBindings();
};
