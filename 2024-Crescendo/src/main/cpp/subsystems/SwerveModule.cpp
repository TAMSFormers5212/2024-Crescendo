#include "subsystems/SwerveModule.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>
#include <iostream>

using namespace SwerveModuleConstants;
using namespace rev;
using namespace std;
using namespace MathConstants;

SwerveModule::SwerveModule(int driveMotor, int steerMotor, int absEncoder, double offset)
  : encoderOffset(offset),
    m_driveMotor(driveMotor, CANSparkLowLevel::MotorType::kBrushless),
    m_steerMotor(steerMotor, CANSparkLowLevel::MotorType::kBrushless),
    m_absoluteEncoder(absEncoder),
    m_moduleName(getName(driveMotor))
    {
      m_absoluteEncoder.SetPositionOffset(encoderOffset);
      resetModule();
      //tony: im not quite sure of the behavior of this function, but it sounds like it just offsets the 
      //value returned by get absolute position
      cout<<"Swerve Module "<<getName(driveMotor)<<" initialized correctly"<<endl;
    }

frc::SwerveModuleState SwerveModule::getState(){
  return {units::meters_per_second_t{m_driveEncoder.GetVelocity()}, units::radian_t{getAbsolutePosition()}};
}

frc::SwerveModulePosition SwerveModule::getPosition(){
  return {units::meter_t{m_driveEncoder.GetPosition()}, units::radian_t{getAbsolutePosition()}};
}

void SwerveModule::resetModule(){
  resetDriveMotor();
  resetSteerMotor();
}

void SwerveModule::resetDriveMotor(){
  m_driveMotor.RestoreFactoryDefaults();

  m_driveController.SetP(kdP);
  m_driveController.SetI(kdI);
  m_driveController.SetD(kdD);
  m_driveController.SetFF(kdFF);

  m_driveMotor.SetIdleMode(CANSparkBase::IdleMode::kBrake);
  m_driveMotor.EnableVoltageCompensation(12.0);
  m_driveMotor.SetSmartCurrentLimit(25, 40);

  m_driveEncoder.SetPositionConversionFactor((SwerveModuleConstants::wheelCircumfrence/SwerveModuleConstants::driveRatio).value());
  m_driveEncoder.SetVelocityConversionFactor((SwerveModuleConstants::wheelCircumfrence/ SwerveModuleConstants::driveRatio / 60_s).value());
    
  resetDriveEncoder();
}

void SwerveModule::resetSteerMotor(){
  m_steerMotor.RestoreFactoryDefaults();
  resetSteerEncoder();
  
  m_steerController.SetP(ktP);
  m_steerController.SetI(ktI);
  m_steerController.SetD(ktD);
  m_steerController.SetFF(ktFF);
  m_steerController.SetPositionPIDWrappingEnabled(true);
  m_steerController.SetPositionPIDWrappingMaxInput(2*pi); // use pi if radians, 1 if 0-1
  m_steerController.SetPositionPIDWrappingMinInput(0); // idk if 0-2pi or -pi - pi 


  m_steerMotor.SetIdleMode(CANSparkBase::IdleMode::kBrake);
  m_steerMotor.EnableVoltageCompensation(12.0);
  m_steerMotor.SetSmartCurrentLimit(20, 30);

  m_steerEncoder.SetPositionConversionFactor(2*pi/ SwerveModuleConstants::steerRatio);
  //tony: mmm kinda sus, the steer encoder returns 0-1, so that leads me to believe that the 
  //pid controller is working on 0-1 as well. this current one makes one wheel rotation, 2 pi 
  //rotations in the neo's encoder/controller. i think the abs encoder also works on 0-1, so 
  //what if we just kept everything at 0-1?

}

void SwerveModule::resetDriveEncoder(){
  m_driveEncoder.SetPosition(0.0);
}

void SwerveModule::resetSteerEncoder(){
  m_steerEncoder.SetPosition(getAbsolutePosition());
}

double SwerveModule::getAbsolutePosition(){
  // return m_absoluteEncoder.GetAbsolutePosition(); // use this if 0-1 
  frc::SmartDashboard::PutNumber(getName(m_driveMotor.GetDeviceId())+" abs", m_absoluteEncoder.GetAbsolutePosition());
  return m_absoluteEncoder.GetAbsolutePosition() * 2 * pi; // shouldn't need to add an offset value because 
                                                  // position offset was set in constructor
}

double SwerveModule::getDrivePosition(){
  return m_driveEncoder.GetPosition();
}

double SwerveModule::getSteerPosition(){
  return m_steerEncoder.GetPosition();
}

double SwerveModule::getDriveVelocity(){
  return m_driveEncoder.GetVelocity();
}

std::string SwerveModule::getName(int driveMotorID){
  // return this->m_moduleName;
  if(driveMotorID == topleft::driveMotor){
    return "top left";
  }else if(driveMotorID == topright::driveMotor){
    return "top right";
  }else if(driveMotorID == bottomleft::driveMotor){
    return "bottom left";
  }else if(driveMotorID == bottomright::driveMotor){
    return "bottom right";
  }else{
    return ""+driveMotorID;
  }
}

void SwerveModule::setState(const frc::SwerveModuleState state){ 
  frc::SwerveModuleState optimizedState = frc::SwerveModuleState::Optimize(state, units::radian_t(getAbsolutePosition()));

  // frc::Rotation2d curAngle = units::radian_t(getAbsolutePosition());

  //idk i took this from 2363. heres what they said: 
  // Since we use relative encoder of steer motor, it is a field (doesn't wrap
  // from 2pi to 0 for example). We need to calculate delta to avoid taking a
  // longer route This is analagous to the EnableContinuousInput() function of
  // WPILib's PIDController classes
  // double delta = std::fmod(std::fmod((optimizedState.angle.Radians().value() -
  //                                   curAngle.Radians().value() + M_PI),
  //                                   2 * M_PI) +
  //                             2 * M_PI,
  //                         2 * M_PI) -
  //               M_PI;   // NOLINT  
  
  // double adjustedAngle = delta + curAngle.Radians().value();
  //
  // however, they didn't use "setPIDpositionwrapping," so maybe i can just do that instead
  double adjustedAngle = optimizedState.angle.Radians().value();
  // double adjustedPosition = optimizedState.angle.Degrees().value()/360; // turns it into a circle fraction
  
  frc::SmartDashboard::PutNumber("O "+getName(m_driveMotor.GetDeviceId()), adjustedAngle);
  frc::SmartDashboard::PutNumber("angle"+getName(m_driveMotor.GetDeviceId()), getSteerPosition());
  m_steerController.SetReference((adjustedAngle), CANSparkBase::ControlType::kPosition);

  // m_driveController.SetReference(optimizedState.speed.value(), CANSparkMax::ControlType::kVelocity); // 2363 version
  m_driveMotor.Set(optimizedState.speed / maxSpeed);
}

void SwerveModule::Periodic(){
}