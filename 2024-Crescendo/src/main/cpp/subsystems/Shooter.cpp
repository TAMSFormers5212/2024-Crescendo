#include "subsystems/Shooter.h"
using namespace ShooterConstants;
using namespace PoseConstants;

Shooter::Shooter(int leftMotor, int rightMotor)
: m_leftMotor(leftMotor, CANSparkLowLevel::MotorType::kBrushless),
    m_rightMotor(rightMotor, CANSparkLowLevel::MotorType::kBrushless)
{
    resetMotors();
}

void Shooter::resetMotors(){
    m_leftMotor.RestoreFactoryDefaults();

    m_leftController.SetP(ksP);
    m_leftController.SetI(ksI);
    m_leftController.SetD(ksD);
    m_leftController.SetFF(ksFF);

    m_leftMotor.SetIdleMode(CANSparkBase::IdleMode::kCoast);
    m_leftMotor.EnableVoltageCompensation(12.0);
    m_leftMotor.SetSmartCurrentLimit(20, 40);

    m_leftEncoder.SetPositionConversionFactor(((wheelDiameter*MathConstants::pi) * pulleyRatio).value());
    //m_leftEncoder.SetVelocityConversionFactor(((wheelDiameter * MathConstants::pi) * pulleyRatio / 60_s).value());

    m_rightMotor.RestoreFactoryDefaults();

    m_rightController.SetP(ksP);
    m_rightController.SetI(ksI);
    m_rightController.SetD(ksD);
    m_rightController.SetFF(ksFF);

    m_rightEncoder.SetPositionConversionFactor(((wheelDiameter * MathConstants::pi) * pulleyRatio).value());
    //m_rightEncoder.SetVelocityConversionFactor(((wheelDiameter * MathConstants::pi) * pulleyRatio / 60_s).value());

    m_rightMotor.SetIdleMode(CANSparkBase::IdleMode::kCoast);
    m_rightMotor.EnableVoltageCompensation(12.0);
    m_rightMotor.SetSmartCurrentLimit(20, 40);
    m_rightMotor.SetInverted(true);

    m_rightMotor.Follow(m_leftMotor, true);
    // resetEncoder();
}


void Shooter::setPercent(double percent){
    //percent pid control
    m_leftMotor.Set(percent/100);

}
void Shooter::setSpeed(double speed){ // velocity PID control
    m_goalSpeed = speed;
    m_leftController.SetReference(speed, CANSparkMaxLowLevel::ControlType::kVelocity);
}

double Shooter::getSpeed(){
    return (m_leftEncoder.GetVelocity()+m_rightEncoder.GetVelocity())/2.0;
}

double Shooter::getrightSpeed(){
    return m_rightEncoder.GetVelocity();
}

double Shooter::getleftSpeed(){
    return m_leftEncoder.GetVelocity();
}
void Shooter::Periodic(){

}