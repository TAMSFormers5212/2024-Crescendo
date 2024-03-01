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
      m_rightMotor(rightMotor, CANSparkLowLevel::MotorType::kBrushless)
      {
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
    // m_leftEncoder.SetVelocityConversionFactor((1.0/armRatio)/60);

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

void Arm::resetEncoder() { // sets neo encoders to absolute encoder position
    m_leftEncoder.SetPosition(getPosition());
    m_rightEncoder.SetPosition(getPosition());
}

double Arm::getPosition() { // returns the absolute encoder position with offset
    return m_absoluteEncoder.GetAbsolutePosition();
}

double Arm::getVelocity(){ // returns left motor's velocity
    return m_leftEncoder.GetVelocity();
}

double Arm::getRawPosition() { // returns the absolute encoder position minus offset
    return m_absoluteEncoder.GetAbsolutePosition() - m_absoluteEncoder.GetPositionOffset(); 
}

void Arm::setPosition(double pose) { // sets the goal pose to given parameter
    //smart motion implementation
    // m_rightController.SetReference(pose,
    //                                CANSparkLowLevel::ControlType::kSmartMotion);
    
    // TrapezoidProfile<units::meters>::State m_goal = {units::meter_t(pose), units::meters_per_second_t(0)};
    // m_goalDistance = units::meter_t(pose);
    // m_goalSpeed = units::meters_per_second_t(0); // this should be zero because we don't want the arm to move 


    m_leftController.SetReference(pose, CANSparkLowLevel::ControlType::kPosition);
}

void Arm::Periodic() {
    // frc 4481 found that profiling yieled jittering when close to goal/small distance to cover, maybe switch to normal pid when close to goal

    //trapezoid profile implementation
    // m_setpointDistance = units::meter_t(getPosition());
    // m_setpointSpeed = units::meters_per_second_t(getVelocity()); // make sure unit conversion is done correctly
    // m_setpoint = m_profile.Calculate(kaT, {m_setpointDistance, m_setpointSpeed}, {m_goalDistance, m_goalSpeed});
    // currently pretty inefficiently made but we'll see if it works

    // m_rightController.SetReference(m_setpoint.position.value(), CANSparkLowLevel::ControlType::kPosition);
    
    frc::SmartDashboard::PutNumber("arm ", getPosition());
    frc::SmartDashboard::PutNumber("armRaw ", getRawPosition());
    frc::SmartDashboard::PutNumber("left output ", m_leftMotor.GetAppliedOutput());
    frc::SmartDashboard::PutNumber("right output ", m_rightMotor.GetAppliedOutput());
    frc::SmartDashboard::PutNumber("left position ", m_leftEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("right position ", m_rightEncoder.GetPosition());
    frc::SmartDashboard::PutBoolean("arm encoder", m_absoluteEncoder.IsConnected());
}