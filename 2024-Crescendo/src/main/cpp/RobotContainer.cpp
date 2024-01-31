// Copyright (c) FIRST and other WPILib contributors. 
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/StartEndCommand.h>

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <frc2/command/button/Trigger.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <frc/controller/PIDController.h>

using namespace std;
using namespace frc2;
using namespace OIConstants;

RobotContainer::RobotContainer() {
  // m_autoBuilder{
  //     [this]() { return m_drive.OdometryPose(); }, // Function to supply current robot pose
  //     [this](frc::Pose2d initPose) { m_drive.resetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
  //     PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
  //     PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
  //     [this](frc::ChassisSpeeds speeds) { m_drive.swerveDrive(speeds.vx, speeds.vy, speeds.omega, true, true); }, // Output function that accepts field relative ChassisSpeeds
  //     m_eventMap, // Our event map
  //     { &m_drive }, // Drive requirements, usually just a single drive subsystem
  //     true // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
  // }
  
  ConfigureBindings();

  m_drive.SetDefaultCommand(RunCommand(
    [this] {

        double speedMultiplier = (1-m_driverController.GetRawAxis(Joystick::ThrottleSlider))*0.5;
        double XAxis = -m_driverController.GetRawAxis(Joystick::XAxis)*speedMultiplier;
        double YAxis = m_driverController.GetRawAxis(Joystick::YAxis)*speedMultiplier;
        double RotAxis = -m_driverController.GetRawAxis(Joystick::RotAxis)*speedMultiplier*4;

        frc::SmartDashboard::PutNumber("speed", speedMultiplier*100);
        int rotDeadband = Joystick::deadband;
        if (abs(XAxis)<rotDeadband){ XAxis = 0;}
        if (abs(YAxis)<rotDeadband){ YAxis=0;}
        if (abs(RotAxis)<(rotDeadband)){ RotAxis=0;}
        
        frc::SmartDashboard::PutNumber("x",XAxis);
        frc::SmartDashboard::PutNumber("y",YAxis);
        frc::SmartDashboard::PutNumber("rot",RotAxis);

        if(m_driverController.GetRawButton(11)){  
          m_drive.moveToAngle(XAxis, YAxis);
        }else if(m_driverController.GetRawButton(12)){
          m_drive.moveToAngle(0, 0.3);
        }
        std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
            frc::PIDController pid(kvP, kvI, kvD);
            
            double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
            double heading_error = targetOffsetAngle_Horizontal;
            // pid.SetSetpoint(0);
            frc::SmartDashboard::PutNumber("heading", heading_error);
            double output = pid.Calculate(heading_error, 0);   
            // pid.Calculate();
        if (m_driverController.GetRawButton(7)){
            
            
            RotAxis += output * speedMultiplier;
        }
        if (m_driverController.GetRawButton(6)){
          m_drive.tankDrive(XAxis,YAxis);
        }
        frc::SmartDashboard::PutNumber("pid", output);  
        // else {
        m_drive.swerveDrive(XAxis, YAxis, RotAxis, true);
        // }

        
        // frc::SmartDashboard::PutNumber("x axis", XAxis);
        
        // frc::SmartDashboard::PutNumber("theta", RotAxis);
        if(m_driverController.GetRawButtonPressed(9)){
          m_drive.toggleOffset();
        }
        frc::SmartDashboard::PutBoolean("toggle offset", m_drive.getOffsetToggle());
    },
    {&m_drive}
  ));
}

void RobotContainer::ConfigureBindings() {

  JoystickButton joystickTrigger{&m_driverController, Joystick::Trigger};
  
  joystickTrigger.OnTrue((
    InstantCommand(
      [this](){
        return m_drive.resetAbsoluteEncoders();
      }
    )
  ).ToPtr());

  JoystickButton joystickThree{&m_driverController, Joystick::ButtonThree};

  joystickThree.OnTrue((
    InstantCommand(
      [this](){
        return m_drive.resetHeading();
      }
    )
  ).ToPtr());

  JoystickButton joystickFour{&m_driverController, Joystick::ButtonFour};

  joystickFour.OnTrue((
    InstantCommand(
      [this](){
        return m_drive.resetOdometry(m_drive.AveragePose());
      }
    )
  ).ToPtr());  


}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
