#include "subsystems/Shooter.h"
using namespace ShooterConstants;
using namespace PoseConstants;

Shooter::Shooter(int topMotor, int bottomMotor)
: m_topMotor(topMotor, CANSparkLowLevel::MotorType::kBrushless),
    m_bottomMotor(bottomMotor, CANSparkLowLevel::MotorType::kBrushless)
{
    resetMotors();
}

void Shooter::resetMotors(){
    m_topMotor.RestoreFactoryDefaults();

    m_topController.SetP(ksP);
    m_topController.SetI(ksI);
    m_topController.SetD(ksD);
    m_topController.SetFF(ksFF);

    m_topMotor.SetIdleMode(CANSparkBase::IdleMode::kCoast);
    m_topMotor.EnableVoltageCompensation(12.0);
    m_topMotor.SetSmartCurrentLimit(20, 40);

    m_topEncoder.SetPositionConversionFactor(((wheelDiameter*MathConstants::pi) / pulleyRatio).value());
    m_topEncoder.SetVelocityConversionFactor(((wheelDiameter * MathConstants::pi) / pulleyRatio / 60_s).value());

    m_bottomMotor.RestoreFactoryDefaults();

    m_bottomController.SetP(ksP);
    m_bottomController.SetI(ksI);
    m_bottomController.SetD(ksD);
    m_bottomController.SetFF(ksFF);

    m_bottomEncoder.SetPositionConversionFactor(((wheelDiameter * MathConstants::pi) / pulleyRatio).value());
    m_bottomEncoder.SetVelocityConversionFactor(((wheelDiameter * MathConstants::pi) / pulleyRatio / 60_s).value());

    m_bottomMotor.SetIdleMode(CANSparkBase::IdleMode::kCoast);
    m_bottomMotor.EnableVoltageCompensation(12.0);
    m_bottomMotor.SetSmartCurrentLimit(20, 40);
    m_bottomMotor.SetInverted(true);

    m_bottomMotor.Follow(m_topMotor, true);
    // resetEncoder();
}



void Shooter::setSpeed(double speed){ // velocity PID control
    m_goalSpeed = speed;
    m_topController.SetReference(speed, CANSparkMaxLowLevel::ControlType::kVelocity);
}

double Shooter::getSpeed(){
    return (m_topEncoder.GetVelocity()+m_bottomEncoder.GetVelocity())/2.0;
}

double Shooter::getBottomSpeed(){
    return m_bottomEncoder.GetVelocity();
}

double Shooter::getTopSpeed(){
    return m_topEncoder.GetVelocity();
}
void Shooter::Periodic(){

}