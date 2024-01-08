#pragma once

#include <frc2/command/SubsystemBase.h>

#include <rev/CANSparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkMaxPIDController.h>

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/AnalogEncoder.h>


#include <Constants.h>

using namespace rev;

class SwerveModule : public frc2::SubsystemBase{

public:
    SwerveModule(int driveMotor, int steerMotor, int absEncoder, double offset);

    void resetModule();
    void resetDriveMotor();
    void resetSteerMotor();
    void resetDriveEncoder();
    void resetSteerEncoder();
    double getDrivePosition();
    double getSteerPosition();
    double getDriveVelocity();
    double getAbsolutePosition(); 


    void Periodic() override;

private:

    double encoderOffset;

    CANSparkMax m_driveMotor;
    CANSparkMax m_steerMotor;

    SparkMaxRelativeEncoder m_driveEncoder = m_driveMotor.GetEncoder(SparkRelativeEncoder::Type::kHallSensor, 42);
    SparkMaxRelativeEncoder m_steerEncoder = m_steerMotor.GetEncoder(SparkRelativeEncoder::Type::kHallSensor, 42);

    SparkMaxPIDController m_driveController = m_driveMotor.GetPIDController();
    SparkMaxPIDController m_steerController = m_steerMotor.GetPIDController();

    frc::AnalogEncoder m_absoluteEncoder;

    std::string m_moduleName;
};