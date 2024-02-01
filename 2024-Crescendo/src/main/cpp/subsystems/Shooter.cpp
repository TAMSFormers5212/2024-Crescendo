#include "subsystems/Shooter.h"
using namespace ShooterConstants;

Shooter::Shooter(int topMotor, int bottomMotor)
: m_topMotor(topMotor, CANSparkLowLevel::MotorType::kBrushless),
    m_bottomMotor(bottomMotor, CANSparkLowLevel::MotorType::kBrushless)
{
    resetMotors();
}

void Shooter::resetMotors(){
    
}

double Shooter::calculateSpeed(double distance, double x, double y){  // calculate the needed speed based on current speed
    return 0;
}
double Shooter::calculateAngle(double distance, double x, double y) {  // angle to feed to arm
    return 0;
}

void Shooter::setSpeed(double speed){

}
double Shooter::getSpeed(){
    return m_topEncoder.GetVelocity();
}

void Shooter::Periodic(){

}