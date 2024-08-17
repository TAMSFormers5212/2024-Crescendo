#include "subsystems/Shooter.h"
#include <frc/smartdashboard/SmartDashboard.h>
using namespace ShooterConstants;
using namespace PoseConstants;

Shooter::Shooter(int leftMotor, int rightMotor)
: m_leftMotor(leftMotor, CANSparkLowLevel::MotorType::kBrushless),
    m_rightMotor(rightMotor, CANSparkLowLevel::MotorType::kBrushless),
    m_leftFF{ShooterConstants::KlsS, ShooterConstants::KlsV},
    m_rightFF{ShooterConstants::KrsS, ShooterConstants::KrsV}
{
    resetMotors();
    inAuto = false;
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

    m_leftEncoder.SetPositionConversionFactor(((wheelDiameter*MathConstants::pi) / pulleyRatio).value());
    m_leftEncoder.SetVelocityConversionFactor((pulleyRatio*wheelDiameter/ 60_s).value());
    // m_leftMotor.SetInverted
    m_rightMotor.RestoreFactoryDefaults();

    m_rightController.SetP(ksP);
    m_rightController.SetI(ksI);
    m_rightController.SetD(ksD);
    m_rightController.SetFF(ksFF);

    m_rightEncoder.SetPositionConversionFactor(((wheelDiameter * MathConstants::pi) * pulleyRatio).value());
    m_rightEncoder.SetVelocityConversionFactor((pulleyRatio *wheelDiameter/ 60_s).value());

    m_rightMotor.SetIdleMode(CANSparkBase::IdleMode::kCoast);
    m_rightMotor.EnableVoltageCompensation(12.0);
    m_rightMotor.SetSmartCurrentLimit(20, 40);
    // m_rightMotor.SetInverted(true);

    // m_rightMotor.Follow(m_leftMotor, false);
    // resetEncoder();
}


void Shooter::setPercent(double percent){
    //percent pid control 0 to 1
    m_leftMotor.Set(percent);
    m_rightMotor.Set(percent);

}
void Shooter::exitAuto() {
    inAuto = false;
}
void Shooter::enterAuto() {
    inAuto = true;
}
void Shooter::setSpeed(double speed){ // velocity PID control
    m_goalSpeed = units::meters_per_second_t{speed};
}
// void Shooter::setAutoSpeed(double speed){
//     m_leftController.SetReference
// }
double Shooter::getSpeed(){
    return (m_leftEncoder.GetVelocity()+m_rightEncoder.GetVelocity())/2.0;
}

double Shooter::getrightSpeed(){
    return m_rightEncoder.GetVelocity();
}

double Shooter::getleftSpeed(){
    return m_leftEncoder.GetVelocity();
}
frc2::CommandPtr Shooter::SysIdQuasistatic(frc2::sysid::Direction direction) {
  return m_sysIdRoutine.Quasistatic(direction);
}

frc2::CommandPtr Shooter::SysIdDynamic(frc2::sysid::Direction direction) {
  return m_sysIdRoutine.Dynamic(direction);
}
void Shooter::Periodic(){
     if(!inAuto) {   
        m_leftController.SetReference(m_goalSpeed.value(), CANSparkMaxLowLevel::ControlType::kVelocity, 0, m_leftFF.Calculate(m_goalSpeed).value());
        m_rightController.SetReference(m_goalSpeed.value(), CANSparkLowLevel::ControlType::kVelocity, 0, m_rightFF.Calculate(m_goalSpeed).value());
    }
    avgSho=(getleftSpeed()+getrightSpeed())/2;
    if(avgSho > 0.95*m_goalSpeed.value()){
        shooterGood=true;
    }
    else{
        shooterGood=false;
    }
    frc::SmartDashboard::PutNumber("l speed", getleftSpeed());
    frc::SmartDashboard::PutNumber("r speed", getrightSpeed());
    frc::SmartDashboard::PutNumber("goal speed", m_goalSpeed.value());
    frc::SmartDashboard::PutBoolean("inAuto", inAuto);
    frc::SmartDashboard::PutBoolean("ShooterReady", shooterGood);
    // frc::SmartDashboard::PutNumber("l output", m_leftMotor.GetOutputCurrent());
    // frc::SmartDashboard::PutNumber("l volt", m_leftMotor.GetAppliedOutput());
}