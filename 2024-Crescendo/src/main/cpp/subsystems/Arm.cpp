#include "subsystems/Arm.h"

#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <cmath>
#include <iostream>
#include <frc/shuffleboard/Shuffleboard.h>

using namespace ArmConstants;
using namespace rev;
using namespace std;
using namespace MathConstants;

Arm::Arm(int leftMotor, int rightMotor, int encoder, double encoderOffset)
    : m_leftMotor(leftMotor, CANSparkLowLevel::MotorType::kBrushless),
      m_rightMotor(rightMotor, CANSparkLowLevel::MotorType::kBrushless),
      m_armFF(ArmConstants::kaS, ArmConstants::kaG, ArmConstants::kaV)
      {
    resetMotors();
    m_absoluteEncoder.SetPositionOffset(encoderOffset);
    // m_absoluteEncoder.SetVelocityConversionFactor();
    initalPosition = getPosition();
    position = getRelativePosition();
    // cout<<"arm abs "<<getPosition()<<" right pos "<<m_rightEncoder.GetPosition()<<" inital pos "<<initalPosition<<endl;
}

void Arm::resetMotors() {
    m_leftMotor.RestoreFactoryDefaults();

    m_leftController.SetP(kaP);
    m_leftController.SetI(kaI);
    m_leftController.SetD(kaD);
    m_leftController.SetFF(kaFF);
    m_leftController.SetIZone(kaIz);
    m_leftController.SetOutputRange(kMinOutput, kMaxOutput);

    // m_leftController.SetSmartMotionMaxAccel(maxAccel.value());
    // m_leftController.SetSmartMotionMaxVelocity(maxVelo.value());
    // m_leftController.SetSmartMotionMinOutputVelocity(0);
    // m_leftController.SetSmartMotionAllowedClosedLoopError(allowedError);
    // m_leftController.

    m_leftMotor.SetIdleMode(CANSparkBase::IdleMode::kBrake);
    m_leftMotor.EnableVoltageCompensation(12.0);
    m_leftMotor.SetSmartCurrentLimit( 40);

    m_leftEncoder.SetPositionConversionFactor(pi2 / armRatio);
    // m_leftEncoder.SetVelocityConversionFactor((1/armRatio)/60);
    // m_leftEncoder.SetInverted(true);

    m_rightMotor.RestoreFactoryDefaults();

    m_rightController.SetP(kaP);
    m_rightController.SetI(kaI);
    m_rightController.SetD(kaD);
    m_rightController.SetFF(kaFF);
    m_rightController.SetIZone(kaIz);
    m_rightController.SetOutputRange(kMinOutput, kMaxOutput);

    // m_rightController.SetSmartMotionMaxAccel(maxAccel.value());
    // m_rightController.SetSmartMotionMaxVelocity(maxVelo.value());
    // m_rightController.SetSmartMotionMinOutputVelocity(0);
    // m_rightController.SetSmartMotionAllowedClosedLoopError(allowedError);

    m_rightMotor.SetIdleMode(CANSparkBase::IdleMode::kBrake);
    m_rightMotor.EnableVoltageCompensation(12.0);
    m_rightMotor.SetSmartCurrentLimit( 40);
    m_rightMotor.SetInverted(true);

    m_rightEncoder.SetPositionConversionFactor(pi2 / armRatio);
    // m_rightEncoder.SetInverted(true);

    m_rightMotor.Follow(m_leftMotor, true);
    m_leftEncoder.SetPosition(getPosition());
    m_rightEncoder.SetPosition(getPosition());
    // m_leftMotor.EnableSoftLimit(CANSparkBase::SoftLimitDirection::kReverse, false);
    // m_rightMotor.EnableSoftLimit(CANSparkBase::SoftLimitDirection::kReverse, false);
    // m_leftMotor.EnableSoftLimit(CANSparkBase::SoftLimitDirection::kForward, false);
    // m_rightMotor.EnableSoftLimit(CANSparkBase::SoftLimitDirection::kForward, false);
    resetEncoder();
}

void Arm::resetEncoder() { // sets neo encoders to absolute encoder position
    m_leftEncoder.SetPosition(getPosition());
    m_rightEncoder.SetPosition(getPosition());
    initalPosition = getRawPosition();
}

double Arm::getPosition() { // returns the absolute encoder position with offset
    return abs(m_absoluteEncoder.GetAbsolutePosition()-0.75)*pi2;

}

double Arm::getVelocity(){ // returns left motor's velocity
    return m_leftEncoder.GetVelocity();
}

double Arm::getRawPosition() { // returns the absolute encoder position minus offset
    return (abs(m_absoluteEncoder.GetAbsolutePosition()-0.75) - m_absoluteEncoder.GetPositionOffset())*pi2; 
}

void Arm::setPosition(double pose) { // sets the goal pose to given parameter
    commandGiven=true;
    position = pose;
    //smart motion implementation
    // m_rightController.SetReference(pose,
    //                                CANSparkLowLevel::ControlType::kSmartMotion);

    // double ff = -sin((getPosition()-0.5)*MathConstants::pi2)*0.1;
    // m_leftController.SetFF(ff);
    // m_leftController.SetReference(pose, CANSparkLowLevel::ControlType::kPosition);
}
void Arm::setNeoPosition(double pose){
    commandGiven=true;
    position = getRelativePosition()+pose-getRawPosition();
}
double Arm::getRelativePosition(){
    return m_leftEncoder.GetPosition();
}

void Arm::set(double value){
    m_leftMotor.Set(value);
    m_rightMotor.Set(value);
}

void Arm::setInitialPosition(){
    initalPosition = getPosition();
}

double Arm::ampPreset(){
    commandGiven=true;
    return getRelativePosition()+(1.64  -getRawPosition());
}
double Arm::speakerPreset(){
    commandGiven=true;
    return getRelativePosition()+(0.27-getRawPosition());
}
double Arm::groundPreset(){
    //used to be a negative
    commandGiven=true;
    return getRelativePosition()+(0.03-getRawPosition());

}
void Arm::Periodic() {
    // frc 4481 found that profiling yieled jittering when close to goal/small distance to cover, maybe switch to normal pid when close to goal

    //trapezoid profile implementation
    // units::radian_t m_goal{position};
    // units::radians_per_second_t zeroV{0};
    // TrapezoidProfile<units::radian_t>::State m_goalState = {m_goal, zeroV};
    // units::radian_t m_setpointDistance = units::radian_t(getPosition());
    // units::radians_per_second_t m_setpointVelocity = units::radians_per_second_t(getVelocity()); // make sure unit conversion is done correctly
    // TrapezoidProfile<units::radian_t>::State m_current = {m_setpointDistance, m_setpointVelocity};
    // TrapezoidProfile<units::radian_t>::State m_setpoint = m_profile.Calculate(kaT, m_current, m_goalState);
    // currently pretty inefficiently made but we'll see if it works

    // m_rightController.SetReference(m_setpoint.position.value(), CANSparkLowLevel::ControlType::kPosition);

    units::radian_t ffP{position+getRawPosition()-getRelativePosition()};
    units::radians_per_second_t ffV{0};
    units::radians_per_second_squared_t ffA(0);
    // if(getRawPosition() >= 1.65) {
    //     setPosition(position - 0.01);
    // }

    if(commandGiven){
        m_leftController.SetReference(position, CANSparkLowLevel::ControlType::kPosition, 0, m_armFF.Calculate(ffP, ffV, ffA).value());
    }
    else{
        m_leftController.SetReference(m_armFF.Calculate(ffP, ffV, ffA).value(),CANSparkLowLevel::ControlType::kVoltage);
    }
        
    // m_leftController.SetReference(position, CANSparkLowLevel::ControlType::kPosition);

    armP->SetDouble(getPosition());
    armPosition->SetDouble(position);
    arm->SetDouble(getRelativePosition());
    armRaw->SetDouble(getRelativePosition());
    armOffset->SetDouble(m_absoluteEncoder.GetPositionOffset());
    
    // frc::SmartDashboard::PutNumber("armPos", position);
    // frc::SmartDashboard::PutNumber("armNoRotation ", abs(m_absoluteEncoder.GetAbsolutePosition()-0.75)*pi2);
 
    //frc::SmartDashboard::PutNumber("left output ", m_leftMotor.GetAppliedOutput());

    // frc::SmartDashboard::PutNumber("right output ", m_rightMotor.GetAppliedOutput());

     //frc::SmartDashboard::PutNumber("left position ", m_leftEncoder.GetPosition());

    // frc::SmartDashboard::PutNumber("right position ", m_rightEncoder.GetPosition());
    // frc::SmartDashboard::PutBoolean("arm encoder", m_absoluteEncoder.IsConnected());
    // frc::SmartDashboard::PutNumber("inital position", initalPosition);
    // frc::SmartDashboard::PutNumber("autoArm", getRelativePosition()-(-0.73+getRawPosition()));
    //  frc::SmartDashboard::PutNumber("feedforward",  m_armFF.Calculate(ffP, ffV, ffA).value());
}