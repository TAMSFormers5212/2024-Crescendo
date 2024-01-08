#include "subsystems/SwerveModule.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>
#include <iostream>

using namespace SwerveModuleConstants;
using namespace rev;

SwerveModule::SwerveModule(int driveMotor, int steerMotor, int absEncoder, double offset)
  : encoderOffset(offset),
    m_driveMotor(driveMotor, CANSparkLowLevel::MotorType::kBrushless),
    m_steerMotor(steerMotor, CANSparkLowLevel::MotorType::kBrushless),
    m_absoluteEncoder(absEncoder),
    m_moduleName(getName(driveMotor))
    {

    }