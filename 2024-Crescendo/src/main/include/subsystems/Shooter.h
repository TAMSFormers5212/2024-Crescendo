#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>

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
using namespace frc;
using namespace ShooterConstants;

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
    void exitAuto();
    void enterAuto();
    double getrightSpeed();

    void Periodic() override;
    // void TeleopPeriodic() override;
    // void AutonomousPeriodic() override;

private:

    CANSparkMax m_leftMotor; 
    CANSparkMax m_rightMotor;

    SparkRelativeEncoder m_leftEncoder = m_leftMotor.GetEncoder(SparkRelativeEncoder::Type::kHallSensor, 42);
    SparkRelativeEncoder m_rightEncoder = m_rightMotor.GetEncoder(SparkRelativeEncoder::Type::kHallSensor, 42);

    SparkPIDController m_leftController = m_leftMotor.GetPIDController();
    SparkPIDController m_rightController = m_rightMotor.GetPIDController();

    SimpleMotorFeedforward<units::meters> m_leftFF;
    SimpleMotorFeedforward<units::meters> m_rightFF;

    bool inAuto;
    bool shooterGood;
    double avgSho;
    units::meters_per_second_t m_goalSpeed{0};
};

