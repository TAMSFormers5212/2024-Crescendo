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
    // m_topController.SetIZone(kaIz);
    // m_topController.SetOutputRange(kMinOutput, kMaxOutput);

    // m_topController.SetSmartMotionMaxAccel(maxAccel);
    // m_topController.SetSmartMotionMaxVelocity(maxVelo);
    // m_topController.SetSmartMotionMinOutputVelocity(0);
    // m_topController.SetSmartMotionAllowedClosedLoopError(allowedError);
    // m_topController.

    m_topMotor.SetIdleMode(CANSparkBase::IdleMode::kCoast);
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

    m_bottomMotor.SetIdleMode(CANSparkBase::IdleMode::kCoast);
    m_bottomMotor.EnableVoltageCompensation(12.0);
    m_bottomMotor.SetSmartCurrentLimit(20, 40);
    m_bottomMotor.SetInverted(true);

    // m_bottomEncoder.SetPositionConversionFactor(1.0 / armRatio);

    m_bottomMotor.Follow(m_topMotor, true);
    // resetEncoder();
}

double Shooter::calculateSpeed(double distance, double x, double y){  // calculate the needed speed based on current speed
    double speed = 0;
    if(distance<=distance3.value()){
        speed = shooterRPM3;
    }else if(distance<=distance5.value()){
        speed = shooterRPM3+(distance-distance3.value())*(shooterRPM5-shooterRPM3)/(distance5.value()-distance3.value());
    }else if(distance<=distance7.value()){
        speed = shooterRPM5 + (distance - distance5.value()) * (shooterRPM7 - shooterRPM5) / (distance7.value() - distance5.value());
    }else if(distance<=distance9.value()){
        speed = shooterRPM7 + (distance - distance7.value()) * (shooterRPM9 - shooterRPM7) / (distance9.value() - distance7.value());
    }else if(distance<=distance11.value()){
        speed = shooterRPM9 + (distance - distance9.value()) * (shooterRPM11 - shooterRPM9) / (distance11.value() - distance9.value());
    }else{
        speed = shooterRPM13;
    }
    return speed;
}
double Shooter::calculateAngle(double distance, double x, double y) {  // angle to feed to arm
    double angle = 0;
    if (distance <= distance3.value()) {
        angle = armAngle0 + (distance - distance0.value()) * (armAngle3 - armAngle0) / (distance - distance0.value());
    } else if (distance <= distance5.value()) {
        angle = armAngle3 + (distance - distance3.value()) * (armAngle5 - armAngle3) / (distance5.value() - distance3.value());
    } else if (distance <= distance7.value()) {
        angle = armAngle5 + (distance - distance5.value()) * (armAngle7 - armAngle5) / (distance7.value() - distance5.value());
    } else if (distance <= distance9.value()) {
        angle = armAngle7 + (distance - distance7.value()) * (armAngle9 - armAngle7) / (distance9.value() - distance7.value());
    } else if (distance <= distance11.value()) {
        angle = armAngle9 + (distance - distance9.value()) * (armAngle11 - armAngle9) / (distance11.value() - distance9.value());
    } else {
        angle = armAngle13;
    }
    return angle;
}

void Shooter::setSpeed(double speed){ // velocity PID control
    m_topController.SetReference(speed, CANSparkMaxLowLevel::ControlType::kVelocity);
}

double Shooter::getSpeed(){
    return m_topEncoder.GetVelocity();
}

void Shooter::Periodic(){

}