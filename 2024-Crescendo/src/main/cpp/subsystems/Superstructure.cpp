#include "subsystems/Superstructure.h"

using namespace PoseConstants;

Superstructure::Superstructure() 
: m_arm(ArmConstants::leftMotor, ArmConstants::rightMotor, ArmConstants::encoder, ArmConstants::encoderOffset),
    m_intake(IntakeConstants::motor, IntakeConstants::beamBreakIO),
    m_shooter(ShooterConstants::leftMotor, ShooterConstants::rightMotor),
    m_leftWinch(WinchConstants::leftWinchMotor),
    m_rightWinch(WinchConstants::rightWinchMotor),
    m_limelight(),
    rollPID(0,0,0)
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

double Superstructure::calculateSpeed(double distance, double x, double y) {  // calculate the needed speed based on current speed
    if(distance<distances.at(0).value()){
        return speeds.at(0);
    }
    for (int i = 1; i < distances.size();i++){
        if(distance<distances.at(i).value()){
            return speeds.at(i-1) + ((speeds.at(i)-speeds.at(i-1))/(distances.at(i)-distances.at(i-1)))*(distance-distances.at(i-1).value());
        }
    }
    return speeds.at(speeds.size()-1);
}
double Superstructure::calculateAngle(double distance, double x, double y) {  // angle to feed to arm
    if (distance < distances.at(0).value()) {
        return angles.at(0);
    }
    for (int i = 1; i < distances.size(); i++) {
        if (distance < distances.at(i).value()) {
            return angles.at(i - 1) + ((angles.at(i) - angles.at(i - 1)) / (distances.at(i) - distances.at(i - 1))) * (distance - distances.at(i - 1).value());
        }
    }
    return speeds.at(speeds.size() - 1);
}

void Superstructure::aim(double angle, double speed){
    m_arm.setPosition(angle);
    m_shooter.setSpeed(speed);
}
void Superstructure::aim(double distance, double x, double y) {
    m_arm.setPosition(calculateAngle(distance, x, y));
    m_shooter.setSpeed(calculateSpeed(distance, x, y));
}
void Superstructure::speakerShot(){
    double angle = calculateAngle(m_limelight.getZ(), m_limelight.getX(), m_limelight.getY());
    double speed = calculateSpeed(m_limelight.getZ(), m_limelight.getX(), m_limelight.getY());
    aim(angle, speed);
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
    m_rightWinch.setWinchPosition(WinchConstants::heightToTravel.value());
}
void Superstructure::climb(double roll) {
    rollPID.SetSetpoint(0.0);
    m_leftWinch.setWinchPosition(WinchConstants::heightToTravel.value()+rollPID.Calculate(roll));
    m_rightWinch.setWinchPosition(WinchConstants::heightToTravel.value()-rollPID.Calculate(roll)); // flip directions if wrong
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