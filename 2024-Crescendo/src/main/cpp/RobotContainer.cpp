// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/StartEndCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <frc2/command/button/Trigger.h>
#include <math.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableValue.h"
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include "commands/Auto.h"

#include <iostream>
#include <frc2/command/Command.h>
using namespace pathplanner;

using namespace std;
using namespace frc2;
using namespace OIConstants;

RobotContainer::RobotContainer() {
    // m_autoBuilder{
    //     [this]() { return m_drive.OdometryPose(); }, // Function to supply
    //     current robot pose [this](frc::Pose2d initPose) {
    //     m_drive.resetOdometry(initPose); }, // Function used to reset
    //     odometry at the beginning of auto PIDConstants(5.0, 0.0, 0.0), // PID
    //     constants to correct for translation error (used to create the X and
    //     Y PID controllers) PIDConstants(0.5, 0.0, 0.0), // PID constants to
    //     correct for rotation error (used to create the rotation controller)
    //     [this](frc::ChassisSpeeds speeds) { m_drive.swerveDrive(speeds.vx,
    //     speeds.vy, speeds.omega, true, true); }, // Output function that
    //     accepts field relative ChassisSpeeds m_eventMap, // Our event map {
    //     &m_drive }, // Drive requirements, usually just a single drive
    //     subsystem true // Should the path be automatically mirrored depending
    //     on alliance color. Optional, defaults to true
    // }
    NamedCommands::registerCommand("drive", frc2::cmd::Print("passed marker 1"));
    ConfigureBindings();

    m_drive.SetDefaultCommand(RunCommand(
        [this] {
            speedMultiplier = (1 - m_driverController.GetRawAxis(Joystick::ThrottleSlider)) * 0.5;
            XAxis = -m_driverController.GetRawAxis(Joystick::XAxis) * speedMultiplier;
            YAxis = m_driverController.GetRawAxis(Joystick::YAxis) * speedMultiplier;
            RotAxis = -m_driverController.GetRawAxis(Joystick::RotAxis) * speedMultiplier * 4;

            frc::SmartDashboard::PutNumber("speed", speedMultiplier * 100);
            // int rotDeadband = Joystick::deadband;
            if (abs(XAxis) < (Joystick::deadband*speedMultiplier)) {
                XAxis = 0;
            }
            if (abs(YAxis) < (Joystick::deadband*speedMultiplier)) {
                YAxis = 0;
            }
            if (abs(RotAxis) < (Joystick::deadband*speedMultiplier)) {
                RotAxis = 0;
            }

            frc::SmartDashboard::PutNumber("x", XAxis);
            frc::SmartDashboard::PutNumber("y", YAxis);
            frc::SmartDashboard::PutNumber("rot", RotAxis);

            if (m_driverController.GetRawButton(11)) {
                m_drive.moveToAngle(XAxis, YAxis);
            } else if (m_driverController.GetRawButton(12)) {
                m_drive.moveToAngle(0, 0.3);
            }
            // std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

            // if (m_driverController.GetRawButton(7)) {
               
            // }
            // if (m_driverController.GetRawButton(8)) {
            //     //if (m_vision.getDistanceError() > 0 &&
            //         //m_vision.getDistanceError() < 25) {
            //          RotAxis += m_superstructure.m_vision.getOutput()* speedMultiplier;
            //          YAxis += m_superstructure.m_vision.getDistanceError() * speedMultiplier;
            //          //}
            // }
            if (m_driverController.GetRawButton(6)) {
                m_drive.tankDrive(XAxis, YAxis);
            }


            m_drive.swerveDrive(XAxis, YAxis, RotAxis, true);

        },  
        {&m_drive}
    ));

    // m_superstructure.m_vision.SetDefaultCommand(RunCommand(
    //     [this] {

    //         // Led toggle
    //         if (m_driverController.GetRawButtonPressed(9)) {
    //             // if (m_vision.getLedOn() == 3) {
    //             //     m_vision.setLedOn(1);
    //             // } else if (m_vision.getLedOn() == 1) {
    //             //     m_vision.setLedOn(3);
    //             // }
    //             m_drive.toggleOffset();
    //         }
            
    //         frc::SmartDashboard::PutBoolean("toggle offset", m_drive.getOffsetToggle());
    //     },
    //     {&m_superstructure.m_vision}));

    m_superstructure.SetDefaultCommand(RunCommand(
        [this] {
            
            // if(m_operatorController.getrawb)

            //some prespin up code or something maybe
            //like have the driver hold the shoot button, and show a boolean on the driverstation
            //then, if the calcs are good, the operator can press a button to shoot it
        },
        {&m_superstructure}
    )); 
    m_superstructure.m_arm.SetDefaultCommand(RunCommand(
        [this] {
            if(abs(m_operatorController.GetRawAxis(Controller::leftYAxis))>0.05){
                m_superstructure.m_arm.setPosition(m_superstructure.m_arm.getRelativePosition() + m_operatorController.GetRawAxis(Controller::leftYAxis));
                // m_superstructure.m_arm.set(m_operatorController.GetRawAxis(Controller::leftYAxis));
            }else{
            if(m_operatorController.GetRawButton(Controller::B)){ // down
                // m_superstructure.setArm(0.73);
                m_superstructure.m_arm.setPosition(m_superstructure.m_arm.getRelativePosition()+(-0.73+m_superstructure.m_arm.getRawPosition()));
                // frc::SmartDashboard::PutNumber("armVal", m_superstructure.m_arm.getRawPosition());
            }else if(m_operatorController.GetRawButton(Controller::Y)){ // amp
               m_superstructure.m_arm.setPosition(m_superstructure.m_arm.getRelativePosition()+(-0.50+m_superstructure.m_arm.getRawPosition()));
            }
            else{
                // m_superstructure.m_arm. (0);
                m_superstructure.m_arm.setPosition(m_superstructure.m_arm.getRelativePosition());
                // m_superstructure.m_arm.set(sin((m_superstructure.m_arm.getPosition()-0.5)*MathConstants::pi2)*0.02);
            }
            }
            // if(m_operatorController.GetRawButton(Controller))
        },
        {&m_superstructure.m_arm}
    ));
    m_superstructure.m_shooter.SetDefaultCommand(RunCommand(
        [this] {
            if(m_operatorController.GetRawAxis(Controller::rightTrigger)>0.05){
                m_superstructure.m_shooter.setPercent(m_operatorController.GetRawAxis(Controller::rightTrigger));
            }
            else{
                m_superstructure.m_shooter.setPercent(0);
            }
            //  frc::SmartDashboard::PutNumber("rightTrigger",m_operatorController.GetRawAxis(Controller::rightTrigger));
        },
        {&m_superstructure.m_shooter}   
    ));
    m_superstructure.m_intake.SetDefaultCommand(RunCommand(
        [this] {
            if(m_operatorController.GetRawButton(Controller::rightBumper)){
                m_superstructure.m_intake.setSpeed(0.4);
            } 
            if(m_operatorController.GetRawButton(Controller::leftBumper)){
                m_superstructure.m_intake.setSpeed(-0.4);
            } 
            if(!m_operatorController.GetRawButton(Controller::leftBumper)&&!m_operatorController.GetRawButton(Controller::rightBumper)){
                m_superstructure.m_intake.stopIntake();
            }
            // if(m_operatorController.GetR)
            // m_operatorController.
        },
        {&m_superstructure.m_intake}
    ));

    m_superstructure.m_rightWinch.SetDefaultCommand(RunCommand(
        [this] {
            if(m_operatorController.GetRawButton(Controller::X)){
                m_superstructure.m_rightWinch.setWinchPosition(m_superstructure.m_rightWinch.getWinchPosition()+10);
            } else if (m_operatorController.GetRawButton(Controller::A)){
                m_superstructure.m_rightWinch.setWinchPosition(m_superstructure.m_rightWinch.getWinchPosition()-10);
            } else{
                m_superstructure.m_rightWinch.setWinchPosition(m_superstructure.m_rightWinch.getWinchPosition());
            }
        //    cout << "left winch" << m_superstructure.m_rightWinch.getWinchPosition() << endl;
            // if(m_operatorController.getrawb)
        },
        {&m_superstructure.m_rightWinch}
    ));

    m_superstructure.m_leftWinch.SetDefaultCommand(RunCommand(
        [this] {
            if(m_operatorController.GetRawButton(Controller::X)){
                m_superstructure.m_leftWinch.setWinchPosition(m_superstructure.m_leftWinch.getWinchPosition()-10);
            } else
            if (m_operatorController.GetRawButton(Controller::A)){
                m_superstructure.m_leftWinch.setWinchPosition(m_superstructure.m_leftWinch.getWinchPosition()+10);
            }else{
                m_superstructure.m_leftWinch.setWinchPosition(m_superstructure.m_leftWinch.getWinchPosition());
            }
            // cout << "right winch"<< m_superstructure.m_leftWinch.getWinchPosition() << endl;
            // if(m_operatorController.getrawb)
        },
        {&m_superstructure.m_leftWinch}
    ));
}

void RobotContainer::ConfigureBindings() {
    // exampleAuto = PathPlannerAuto("Example Auto").ToPtr().Unwrap();
    // frc::SmartDashboard::PutData("Example Auto", exampleAuto.get());
    JoystickButton joystickTrigger{&m_driverController, Joystick::Trigger};
    JoystickButton joystickThree{&m_driverController, Joystick::ButtonThree};
    JoystickButton joystickFour{&m_driverController, Joystick::ButtonFour};
    JoystickButton joystickFive(&m_driverController, Joystick::ButtonFive);
    JoystickButton joystickSix(&m_driverController, Joystick::ButtonSix);
    JoystickButton joystickSeven(&m_driverController, Joystick::ButtonSeven);
    JoystickButton joystickEight(&m_driverController, Joystick::ButtonEight);
    JoystickButton joystickNine(&m_driverController, Joystick::ButtonNine);
    JoystickButton joystickTen(&m_driverController, Joystick::ButtonTen);
    JoystickButton joystickEleven(&m_driverController, Joystick::ButtonEleven);

    // joystickTrigger.WhileTrue(RunCommand([this](){}).ToPtr());

    joystickTrigger.OnTrue((InstantCommand([this]() { /*shoot*/ })).ToPtr());
    joystickThree.OnTrue((InstantCommand([this]() { return m_drive.resetHeading(); })).ToPtr());
    joystickFour.OnTrue((InstantCommand([this]() { return m_drive.resetOdometry(m_drive.AveragePose()); })).ToPtr());
    joystickFive.OnTrue((InstantCommand([this] { return m_drive.resetAbsoluteEncoders(); })).ToPtr());

    JoystickButton controllerLeftTrigger(&m_operatorController, Controller::leftTrigger);
    JoystickButton controllerRightTrigger(&m_operatorController, Controller::rightTrigger);
    JoystickButton controllerLeftBumper(&m_operatorController, Controller::leftBumper);
    JoystickButton controllerRightBumper(&m_operatorController, Controller::rightBumper);
    JoystickButton controllerA(&m_operatorController, Controller::A);
    JoystickButton controllerB(&m_operatorController, Controller::B);
    JoystickButton controllerX(&m_operatorController, Controller::X);
    JoystickButton controllerY(&m_operatorController, Controller::Y);
    JoystickButton controllerMenu(&m_operatorController, Controller::Menu);
    // JoystickButton controllerLeft(&m_operatorController, Controller::);
    POVButton controllerLeft(&m_operatorController, Controller::left); // will have to check later
    POVButton controllerRight(&m_operatorController, Controller::right);
    POVButton controllerDown(&m_operatorController, Controller::down);
    POVButton controllerUp(&m_operatorController, Controller::up);

    controllerMenu.OnTrue((InstantCommand([this]{ return m_superstructure.m_arm.resetEncoder(); })).ToPtr());
    
    //controllerB.OnTrue((InstantCommand([this] { return m_superstructure.setArm(0); })).ToPtr());
    //controllerRightBumper.OnTrue((InstantCommand([this] {return m_superstructure.m_intake.shootNote(); })).ToPtr());
    //controllerLeftBumper.OnTrue((InstantCommand([this] {return m_superstructure.m_intake.intakeNote(); })).ToPtr());
    //controllerRightBumper.OnFalse((InstantCommand([this] {return m_superstructure.m_intake.setSpeed(0); })).ToPtr());
    //controllerY.OnTrue((InstantCommand([this] { return m_superstructure.intakeNote();})).ToPtr());
    // controllerRightBumper.WhileHeld((InstantCommand([this] {return m_superstructure.m_rightWinch.setWinchSpeed(0.4);})).ToPtr());
    // Joystick

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
    // return Auto(&m_drive).ToPtr();
    return Auto((&m_drive), (&(m_superstructure.m_arm)), (&(m_superstructure.m_intake)), (&(m_superstructure.m_shooter))).ToPtr();
    // return frc2::cmd::Print("No autonomous command configured");
}

// frc2::CommandPtr RobotContainer::getAutonomousCommand(){
//     auto path = PathPlannerPath::fromPathFile("Test Path");
//     return AutoBuilder::followPath(path);//PathPlannerAuto("Top pos 3 note auto preload + a1+ m1").ToPtr();
// }
