// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc/GenericHID.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <frc2/command/Command.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc2/command/Commands.h>
#include "commands/Auto.h"

#include <commands/ReverseShooter.h>
#include "Constants.h"
#include "subsystems/SwerveDrive.h" 
#include "subsystems/VisionSubsystem.h"
#include "subsystems/Superstructure.h"
#include "subsystems/Winch.h"
#include "subsystems/LEDController.h"

#include <frc/geometry/Rotation2d.h>

using namespace OIConstants;
using namespace pathplanner;
class RobotContainer {
 public:
  RobotContainer();
    double RotAxis;
    double speedMultiplier;
    double XAxis;
    double YAxis;
    
  frc2::Command* GetAutonomousCommand();
  // frc2::Rotation2d getRotated();
  // frc2::CommandPtr getAutonomousCommand();
Superstructure m_superstructure;
 LEDController m_LEDs;
 SwerveDrive m_drive;
 void Periodic();
 private:
  frc::GenericHID m_driverController{kDriverControllerPort};
  frc::GenericHID m_operatorController{kOperatorControllerPort};

  // std::unique_ptr<frc2::Command> exampleAuto;
  // std::unique_ptr<frc2::Command> pathfindToPickup;
  // std::unique_ptr<frc2::Command> pathfindToScore;
  // std::unique_ptr<frc2::Command> onTheFly;
  // std::unique_ptr<frc2::Command> followOnTheFly;

  
    bool partyLights = false;
    
    frc2::CommandPtr m_simpleAuto = PathPlannerAuto("Test Auto").ToPtr();
    frc2::CommandPtr m_RotationAuto = AutoBuilder::buildAuto("Rotation Auto");
    frc2::CommandPtr m_twonote = PathPlannerAuto("2 Note Auton").ToPtr();
    frc2::CommandPtr m_threenote = PathPlannerAuto("3 Note Auton").ToPtr();
    frc2::CommandPtr m_mobilityAuto = PathPlannerAuto("Mobility auto").ToPtr();
    frc2::CommandPtr m_threenotebottom = PathPlannerAuto("3 Note Bottom Auton").ToPtr();
    frc2::CommandPtr m_fournote = PathPlannerAuto("4 Note Auton").ToPtr();
    frc2::CommandPtr m_bottompreload = PathPlannerAuto("Bottom Preload Auton").ToPtr();
    frc2::CommandPtr m_toppreload = PathPlannerAuto("Top Preload Auton").ToPtr();
    frc::SendableChooser<frc2::Command*> m_chooser;
    
  // The chooser for the autonomous routines
  
  //LEDController m_LEDs;


  VisionSubsystem m_vision;
  //Superstructure m_superstructure;
  void ConfigureBindings();
};
