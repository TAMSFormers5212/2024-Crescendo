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

using namespace std;
using namespace frc2;
using namespace OIConstants;

RobotContainer::RobotContainer() {
  ConfigureBindings();

  m_drive.SetDefaultCommand(RunCommand(
    [this] {
      // double check these for genericHID
        // Right stick up on xbox is negative, right stick down is postive.
        // Right stick right on xbox is negative, right stick left is postive.
        // Left stick right is positive, left stick left is negative.
        double speedMultiplier = (1-m_driverController.GetRawAxis(Joystick::ThrottleSlider))*0.5;
        double XAxis = -m_driverController.GetRawAxis(Joystick::XAxis)*speedMultiplier;
        double YAxis = m_driverController.GetRawAxis(Joystick::YAxis)*speedMultiplier;
        double RotAxis = -m_driverController.GetRawAxis(Joystick::RotAxis)*speedMultiplier;

        if(m_driverController.GetRawButton(11)){
          m_drive.moveToAngle(XAxis, YAxis);
        }else if(m_driverController.GetRawButton(12)){
          m_drive.moveToAngle(0, 0.3);
        }
        m_drive.swerveDrive(XAxis, YAxis, RotAxis, false);
        // frc::SmartDashboard::PutNumber("x axis", XAxis);
        // frc::SmartDashboard::PutNumber("y axis", YAxis);
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
