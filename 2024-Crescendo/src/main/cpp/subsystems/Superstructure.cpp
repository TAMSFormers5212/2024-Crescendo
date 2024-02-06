#include "subsystems/Superstructure.h"

using namespace PoseConstants;

Superstructure::Superstructure() 
: m_arm(ArmConstants::leftMotor, ArmConstants::rightMotor, ArmConstants::encoder, ArmConstants::encoderOffset),
    m_intake(IntakeConstants::motor, IntakeConstants::beamBreakIO),
    m_shooter(ShooterConstants::topMotor, ShooterConstants::bottomMotor),
    m_leftWinch(WinchConstants::leftWinchMotor),
    m_rightWinch(WinchConstants::rightWinchMotor)
{  // arm, intake, shooter, winch
    // this->m_arm = arm;
    // this->m_intake = intake;
    // this->m_shooter = shooter;
    resetSuperstructure();
}

void Superstructure::resetSuperstructure(){
    m_arm.resetMotors();
    m_intake.resetMotor();
    m_shooter.resetMotors();
    m_leftWinch.resetMotor();
    m_rightWinch.resetMotor();
}

// set positions and state control
void Superstructure::setToIntake(){
    m_intake.intakeNote();
    m_arm.setPosition(0);
}
void Superstructure::intakeNote(){
    m_intake.intakeNote();
}
void Superstructure::indexNote(){
    m_intake.indexNote();
}
void Superstructure::aimShooter(double armPosition, double shooterRpm){
    m_arm.setPosition(armPosition);
    m_shooter.setSpeed(shooterRpm);
}
void Superstructure::aimShooter(double distance){
    //interpolate an armposition and shooterRpm based on distance
}
void Superstructure::speakerShot(){
    m_arm.setPosition(armAngle3);
    m_shooter.setSpeed(shooterRPM3);
}
void Superstructure::ampShot(){
    m_arm.setPosition(armAmp); // just manually shoot the note once the arm is in position
}
void Superstructure::raiseToClimb(){
    m_arm.setPosition(armClimb); // just manually shoot the note once the arm is in position
}
void Superstructure::climb(){
    m_leftWinch.setWinchPosition(WinchConstants::heightToTravel.value());
}

// utility control
void Superstructure::setIntake(double speed){
    m_intake.setSpeed(speed);
}
void Superstructure::setShooter(double speed){
    m_shooter.setSpeed(speed);
}
void Superstructure::setArm(double position){
    m_arm.setPosition(position);
}
void Superstructure::setLeftWinchPosition(double position){
    m_leftWinch.setWinchPosition(position);
}
void Superstructure::setRightWinchPosition(double position){
    m_rightWinch.setWinchPosition(position);
}
void Superstructure::setLeftWinchSpeed(double speed){
    m_leftWinch.setWinchSpeed(speed);
}
void Superstructure::setRightWinchSpeed(double speed){
    m_rightWinch.setWinchSpeed(speed);
}

void Superstructure::Periodic() {
}