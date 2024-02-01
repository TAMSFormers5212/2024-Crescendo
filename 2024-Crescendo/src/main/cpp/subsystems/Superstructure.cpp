#include "subsystems/Superstructure.h"

Superstructure::Superstructure() 
: m_arm(ArmConstants::leftMotor, ArmConstants::rightMotor, ArmConstants::encoder, ArmConstants::encoderOffset),
    m_intake(IntakeConstants::motor, PortConstants::beamBreakIO),
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
    // m_shooter.resetMotors();
    m_leftWinch.resetMotor();
    m_rightWinch.resetMotor();
}

// set positions and state control
void Superstructure::setToIntake(){

}
void Superstructure::intakeNote(){

}
void Superstructure::indexNote(){

}
void Superstructure::aimShooter(){

}
void Superstructure::speakerShot(){

}
void Superstructure::ampShot(){

}
void Superstructure::raiseToClimb(){

}
void Superstructure::climb(){

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
}
void Superstructure::setRightWinchSpeed(double speed){

}

void Superstructure::Periodic() {
}