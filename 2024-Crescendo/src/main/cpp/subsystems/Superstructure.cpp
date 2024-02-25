#include "subsystems/Superstructure.h"

using namespace PoseConstants;

Superstructure::Superstructure() 
: m_arm(ArmConstants::leftMotor, ArmConstants::rightMotor, ArmConstants::encoder, ArmConstants::encoderOffset),
    m_intake(IntakeConstants::motor, IntakeConstants::beamBreakIO),
    m_shooter(ShooterConstants::topMotor, ShooterConstants::bottomMotor),
    m_leftWinch(WinchConstants::leftWinchMotor),
    m_rightWinch(WinchConstants::rightWinchMotor),
    m_limelight()
{  
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
    m_arm.setPosition(m_shooter.calculateAngle(distance, m_limelight.getX(), m_limelight.getY()));
}
void Superstructure::speakerShot(){
    double angle = m_shooter.calculateAngle(m_limelight.getZ(), m_limelight.getX(), m_limelight.getY());
    double speed = m_shooter.calculateSpeed(m_limelight.getZ(), m_limelight.getX(), m_limelight.getY());
    m_arm.setPosition(angle);
    m_shooter.setSpeed(speed);
    if(m_shooter.getSpeed()>speed*0.95&&m_shooter.getSpeed()<speed*1.05){
        m_intake.shootNote();
    }
}
void Superstructure::ampShot(){
    m_arm.setPosition(armAmp); 
    if(m_arm.getPosition()>armAmp*0.95&&m_arm.getPosition()<1.05*armAmp){
        m_shooter.setSpeed(ampSpeed);
    }
}
void Superstructure::raiseToClimb(){
    m_arm.setPosition(armClimb);
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