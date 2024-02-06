#include "subsystems/Arm.h"

#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <cmath>
#include <iostream>

using namespace ArmConstants;
using namespace rev;
using namespace std;
using namespace MathConstants;

Arm::Arm(int leftMotor, int rightMotor, int encoder, double encoderOffset)
    : m_leftMotor(leftMotor, CANSparkLowLevel::MotorType::kBrushless),
      m_rightMotor(rightMotor, CANSparkLowLevel::MotorType::kBrushless),
      m_absoluteEncoder(encoder) {
    m_absoluteEncoder.SetPositionOffset(encoderOffset);
    resetMotors();
}

void Arm::resetMotors() {
    m_leftMotor.RestoreFactoryDefaults();

    m_leftController.SetP(kaP);
    m_leftController.SetI(kaI);
    m_leftController.SetD(kaD);
    m_leftController.SetFF(kaFF);
    m_leftController.SetIZone(kaIz);
    m_leftController.SetOutputRange(kMinOutput, kMaxOutput);

    m_leftController.SetSmartMotionMaxAccel(maxAccel.value());
    m_leftController.SetSmartMotionMaxVelocity(maxVelo.value());
    m_leftController.SetSmartMotionMinOutputVelocity(0);
    m_leftController.SetSmartMotionAllowedClosedLoopError(allowedError);
    // m_leftController.

    m_leftMotor.SetIdleMode(CANSparkBase::IdleMode::kBrake);
    m_leftMotor.EnableVoltageCompensation(12.0);
    m_leftMotor.SetSmartCurrentLimit(20, 40);

    m_leftEncoder.SetPositionConversionFactor(1.0 / armRatio);

    m_rightMotor.RestoreFactoryDefaults();

    m_rightController.SetP(kaP);
    m_rightController.SetI(kaI);
    m_rightController.SetD(kaD);
    m_rightController.SetFF(kaFF);
    m_rightController.SetIZone(kaIz);
    m_rightController.SetOutputRange(kMinOutput, kMaxOutput);

    m_rightController.SetSmartMotionMaxAccel(maxAccel.value());
    m_rightController.SetSmartMotionMaxVelocity(maxVelo.value());
    m_rightController.SetSmartMotionMinOutputVelocity(0);
    m_rightController.SetSmartMotionAllowedClosedLoopError(allowedError);

    m_rightMotor.SetIdleMode(CANSparkBase::IdleMode::kBrake);
    m_rightMotor.EnableVoltageCompensation(12.0);
    m_rightMotor.SetSmartCurrentLimit(20, 40);
    m_rightMotor.SetInverted(true);

    m_rightEncoder.SetPositionConversionFactor(1.0 / armRatio);

    m_rightMotor.Follow(m_leftMotor, true);
    resetEncoder();
}

void Arm::resetEncoder() {
    m_leftEncoder.SetPosition(getPosition());
    m_rightEncoder.SetPosition(getPosition());
}

double Arm::getPosition() {
    return m_absoluteEncoder.GetAbsolutePosition() - m_absoluteEncoder.GetPositionOffset();
}

double Arm::getRawPosition() { return m_absoluteEncoder.GetAbsolutePosition(); }

void Arm::setPosition(double pose) {
    //smart motion implementation
    // m_rightController.SetReference(pose,
    //                                CANSparkLowLevel::ControlType::kSmartMotion);

    // m_goal = {units::meter_t(pose), units::meters_per_second_t(0)};
    // frc::SmartDashboard::PutNumber("arm ", getPosition());
    // frc::SmartDashboard::PutNumber("left output ", m_leftMotor.GetAppliedOutput());
    // frc::SmartDashboard::PutNumber("right output ", m_rightMotor.GetAppliedOutput());
}

void Arm::Periodic() {
    //trapezoid profile implementation
    // m_setpoint = profile.Calculate(kaT, m_setpoint, m_goal);

    // m_rightController.SetReference(m_setpoint.position.value(), CANSparkLowLevel::ControlType::kPosition);
}