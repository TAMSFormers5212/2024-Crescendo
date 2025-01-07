// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <RobotContainer.h>
#include <cameraserver/CameraServer.h>
#include "frc/DataLogManager.h"
#include "wpi/DataLog.h"
#include "frc/DriverStation.h"
#include <cscore_oo.h>

#include <cscore_cpp.h>

//#include <LEDController.h>

wpi::log::StringLogEntry myStringLog; 

void Robot::RobotInit() {
    frc::DataLogManager::Start();
    DriverStation::StartDataLog(DataLogManager::GetLog());
    wpi::log::DataLog& log = frc::DataLogManager::GetLog();
    myStringLog = wpi::log::StringLogEntry(log, "/my/string");
    usbCam = frc::CameraServer::StartAutomaticCapture("depth perceptionator 1", 0); //usb back camera
    usbCam.SetResolution(160, 120);
    usbCam.SetFPS(10);
    usbCam.SetPixelFormat(cs::VideoMode::kMJPEG);
    
    
    

    // usbCam2 = frc::CameraServer::StartAutomaticCapture("depth perceptionator 2", 0); //usb back camera
    // usbCam2.SetResolution(160, 120);
    // usbCam2.SetFPS(8);
   
   

    

}

void Robot::RobotPeriodic() { 
    usbCam.SetFPS(15);
    frc2::CommandScheduler::GetInstance().Run();
    //frc::SmartDashboard::PutString("cam name", usbCam.GetName());
}

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
    m_container.m_superstructure.m_shooter.exitAuto();
}

void Robot::TeleopPeriodic() {
    //m_container.m_LEDs.setColor();
    myStringLog.Append("i was testing");
}

void Robot::TeleopExit() {}

void Robot::TestInit() { frc2::CommandScheduler::GetInstance().CancelAll(); }

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
