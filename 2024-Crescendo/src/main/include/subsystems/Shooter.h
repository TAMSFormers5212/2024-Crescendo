#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>

#include <frc/AnalogEncoder.h>

#include <rev/CANSparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkPIDController.h>
// #include <rev/ThroughBoreEncoder.h>

// clean up include list once subclasses are finished

#include <Constants.h>
#include <frc/controller/SimpleMotorFeedforward.h>

using namespace std;
using namespace rev;

class Shooter : public frc2::SubsystemBase{

public:
    Shooter(int leftMotor, int rightMotor);

    void resetMotors();

    void setPercent(double speed);
    void setSpeed(double speed);
    double getSpeed();
    void setleftSpeed(double speed);
    double getleftSpeed();
    void setrightSpeed(double speed);
    double getrightSpeed();

    void Periodic() override;

private:

    CANSparkMax m_leftMotor; 
    CANSparkMax m_rightMotor;

    SparkRelativeEncoder m_leftEncoder = m_leftMotor.GetEncoder(SparkRelativeEncoder::Type::kHallSensor, 42);
    SparkRelativeEncoder m_rightEncoder = m_rightMotor.GetEncoder(SparkRelativeEncoder::Type::kHallSensor, 42);

    SparkPIDController m_leftController = m_leftMotor.GetPIDController();
    SparkPIDController m_rightController = m_rightMotor.GetPIDController();

    frc::SimpleMotorFeedforward<units::meters> m_shooterFF(const double, const double, const double);

    double m_goalSpeed = 0;
};

