#include "subsystems/Superstructure.h"

using namespace PoseConstants;

Superstructure::Superstructure() 
: m_arm(ArmConstants::leftMotor, ArmConstants::rightMotor, ArmConstants::encoder, ArmConstants::encoderOffset),
    m_intake(IntakeConstants::motor, IntakeConstants::beamBreakIO),
    m_shooter(ShooterConstants::leftMotor, ShooterConstants::rightMotor),
    m_leftWinch(WinchConstants::leftWinchMotor),
    m_rightWinch(WinchConstants::rightWinchMotor),
    m_vision(),
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
void Superstructure::indexNote(){ // indexes note
    m_intake.indexNote();
}

double Superstructure::calculateSpeed(double distance, double x, double y) {  // calculate the needed speed based on current speed
    if(distance<distances.at(0)){
        return speeds.at(0);
    }
    for (int i = 1; i < distances.size();i++){
        if(distance<distances.at(i)){
            return speeds.at(i-1) + ((speeds.at(i)-speeds.at(i-1))*(distance-distances.at(i-1)))/(distances.at(i)-distances.at(i-1));
        }
    }
    return speeds.at(speeds.size()-1);
}
double Superstructure::calculateAngle(double distance, double x, double y) {  // angle to feed to arm
    if (distance < distances.at(0)) {
        return angles.at(0);
    }
    for (int i = 1; i < distances.size(); i++) {
        if (abs(distance-distances.at(i))<1){
            return angles.at(i);
        }
        if (distance < distances.at(i)) {
            return angles.at(i - 1) + ((angles.at(i) - angles.at(i - 1)) * ((distance - distances.at(i - 1))/(distances.at(i) - distances.at(i - 1))));
        }
    }
    return slope*distance + intercept; //angles.at(angles.size() - 1);
}

void Superstructure::aim(double angle, double speed){ // simple aiming with preset angle and speed 
    m_arm.setNeoPosition(angle);
    m_shooter.setSpeed(speed);
}
void Superstructure::aim(double distance, double x, double y) { // raises arm and spins up shooter to calcuated values based on distance, x, y
    m_arm.setNeoPosition(calculateAngle(distance, x, y));
    m_shooter.setSpeed(calculateSpeed(distance, x, y));
    // frc::SmartDashboard::PutNumber("armAn",calculateAngle(distance,x,y));
}
void Superstructure::autonAim(double distance) { // raises arm and spins up shooter to calcuated values based on distance, x, y
    m_arm.setNeoPosition(calculateAngle(distance, 0, 0));
    
    
}
void Superstructure::speakerShot(){ // speaker shot based on limelight tag position
    double angle = 0;//calculateAngle(m_vision.getZ(), m_vision.getX(), m_vision.getY());
    double speed = 0;//calculateSpeed(m_vision.getZ(), m_vision.getX(), m_vision.getY());
    aim(angle, speed);
    if(m_shooter.getSpeed()>speed*0.95&&m_shooter.getSpeed()<speed*1.05){
        m_intake.shootNote();
    }
}
void Superstructure::ampShot(){ // raises arm to amp position, if in position shoots note
    m_arm.setPosition(armAmp); 
    if(m_arm.getPosition()>armAmp*0.95&&m_arm.getPosition()<1.05*armAmp){
        m_shooter.setSpeed(ampSpeed);
    }
}
void Superstructure::raiseToClimb(){ // raises arm to climbing position
    m_arm.setPosition(armClimb);
}

void Superstructure::climb(){ // simple climb, just goes straight up to height to travel
    m_leftWinch.setWinchPosition(WinchConstants::heightToTravel.value());
    m_rightWinch.setWinchPosition(WinchConstants::heightToTravel.value());
}
void Superstructure::climb(double roll) { // attempts climb with gyro to balance both sides
    rollPID.SetSetpoint(0.0);
    m_leftWinch.setWinchPosition(WinchConstants::heightToTravel.value()+rollPID.Calculate(roll));
    m_rightWinch.setWinchPosition(WinchConstants::heightToTravel.value()-rollPID.Calculate(roll)); // flip directions if wrong
}

// utility control
void Superstructure::setIntake(double speed){ // sets intake to speed
    m_intake.setSpeed(speed);
}
void Superstructure::setShooter(double speed){ // sets both shooter wheels to speed
    m_shooter.setSpeed(speed);
}
void Superstructure::setArm(double position){ // sets arm to position
    m_arm.setPosition(position);
}
void Superstructure::setLeftWinchPosition(double position){ // sets left winch to position
    m_leftWinch.setWinchPosition(position);
}
void Superstructure::setRightWinchPosition(double position){ // sets right winch to position
    m_rightWinch.setWinchPosition(position);
}
void Superstructure::setLeftWinchSpeed(double speed){ // sets left winch to speed
    m_leftWinch.setWinchSpeed(speed);
}
void Superstructure::setRightWinchSpeed(double speed){ // sets right winch to speed
    m_rightWinch.setWinchSpeed(speed);
}

void Superstructure::Periodic() {
}

double Superstructure::getArmPosition(){
    return m_arm.getPosition();
}
double Superstructure::getLeftWinchPosition(){
    return m_leftWinch.getPosition();
}
double Superstructure::getRightWinchPosition(){
    return m_rightWinch.getPosition();
}
double Superstructure::getleftShooterSpeed(){
    return m_shooter.getleftSpeed();
}
double Superstructure::getrightShooterSpeed(){
    return m_shooter.getrightSpeed();
}
double Superstructure::getShooterSpeed(){
    return m_shooter.getSpeed();
}