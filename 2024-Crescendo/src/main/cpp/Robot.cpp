// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <RobotContainer.h>
#include <cameraserver/CameraServer.h>
#include <cscore_oo.h>
#include <cscore_cpp.h>

//#include <LEDController.h>



void Robot::RobotInit() {

    cs::UsbCamera usbCam = frc::CameraServer::StartAutomaticCapture(); //usb back camera
    usbCam.SetResolution(640, 480);
    
    usbCam.SetExposureManual(10);
    usbCam.SetWhiteBalanceManual(50);

    

}

void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
    if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kRed){
        //m_container.m_drive.resetHeading();
    }
    m_autonomousCommand = m_container.GetAutonomousCommand();

    if (m_autonomousCommand != nullptr) {
        (*m_autonomousCommand)->Schedule();
    }
    
}

void Robot::AutonomousPeriodic() {
    
}

void Robot::AutonomousExit() {
     m_container.m_superstructure.m_shooter.exitAuto();
}

void Robot::TeleopInit() {
    if (m_autonomousCommand) {
       
       (*m_autonomousCommand)->Cancel();
    }
}

void Robot::TeleopPeriodic() {
    //m_container.m_LEDs.setColor();
}

void Robot::TeleopExit() {}

void Robot::TestInit() { frc2::CommandScheduler::GetInstance().CancelAll(); }

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
