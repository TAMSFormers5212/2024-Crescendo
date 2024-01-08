#pragma once

#include <frc2/command/SubsystemBase.h>

#include <rev/CANSparkBase.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkMaxPIDController.h>

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/AnalogEncoder.h>


#include <Constants.h>

using namespace rev;

class SwerveModule : public frc2::SubsystemBase{

public:
    SwerveModule(int driveMotor, int turningMotor, int absEncoder, double offset);

    


    void Periodic() override;

private:

    double encoderOffset;

    CANSparkBase m_driveMotor;
    CANSparkBase m_steerMotor;

    SparkRelativeEncoder m_driveEncoder = m_driveMotor.GetEncoder(SparkRelativeEncoder::Type::kHallSensor, 42);
    SparkRelativeEncoder m_steerEncoder = m_steerMotor.GetEncoder(SparkRelativeEncoder::Type::kHallSensor, 42);

    SparkPIDController m_driveController = m_driveMotor.GetPIDController();
    SparkPIDController m_steerController = m_steerMotor.GetPIDController();

    frc::AnalogEncoder m_absoluteEncoder;

    std::string m_moduleName;
};