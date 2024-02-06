#include "subsystems/Shooter.h"
using namespace ShooterConstants;

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
    // m_topController.SetIZone(kaIz);
    // m_topController.SetOutputRange(kMinOutput, kMaxOutput);

    // m_topController.SetSmartMotionMaxAccel(maxAccel);
    // m_topController.SetSmartMotionMaxVelocity(maxVelo);
    // m_topController.SetSmartMotionMinOutputVelocity(0);
    // m_topController.SetSmartMotionAllowedClosedLoopError(allowedError);
    // m_topController.

    m_topMotor.SetIdleMode(CANSparkBase::IdleMode::kBrake);
    m_topMotor.EnableVoltageCompensation(12.0);
    m_topMotor.SetSmartCurrentLimit(20, 40);

    m_topEncoder.SetPositionConversionFactor((wheelDiameter / pulleyRatio).value());
    m_topEncoder.SetPositionConversionFactor((wheelDiameter / pulleyRatio / 60_s).value());

    m_bottomMotor.RestoreFactoryDefaults();

    m_bottomController.SetP(ksP);
    m_bottomController.SetI(ksI);
    m_bottomController.SetD(ksD);
    m_bottomController.SetFF(ksFF);
    // m_bottomController.SetIZone(kaIz);
    // m_bottomController.SetOutputRange(kMinOutput, kMaxOutput);

    // m_bottomController.SetSmartMotionMaxAccel(maxAccel);
    // m_bottomController.SetSmartMotionMaxVelocity(maxVelo);
    // m_bottomController.SetSmartMotionMinOutputVelocity(0);
    // m_bottomController.SetSmartMotionAllowedClosedLoopError(allowedError);

    m_bottomMotor.SetIdleMode(CANSparkBase::IdleMode::kBrake);
    m_bottomMotor.EnableVoltageCompensation(12.0);
    m_bottomMotor.SetSmartCurrentLimit(20, 40);
    m_bottomMotor.SetInverted(true);

    // m_bottomEncoder.SetPositionConversionFactor(1.0 / armRatio);

    m_bottomMotor.Follow(m_topMotor, true);
    // resetEncoder();
}

double Shooter::calculateSpeed(double distance, double x, double y){  // calculate the needed speed based on current speed
    return 0;
}
double Shooter::calculateAngle(double distance, double x, double y) {  // angle to feed to arm
    return 0;
}

void Shooter::setSpeed(double speed){
    m_topController.SetReference(speed, CANSparkMaxLowLevel::ControlType::kVelocity);
}
double Shooter::getSpeed(){
    return m_topEncoder.GetVelocity();
}

void Shooter::Periodic(){

}