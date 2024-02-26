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

using namespace std;
using namespace rev;

class Shooter : public frc2::SubsystemBase{

public:
    Shooter(int topMotor, int bottomMotor);

    void resetMotors();

    void setSpeed(double speed);
    double getSpeed();
    void setTopSpeed(double speed);
    double getTopSpeed();
    void setBottomSpeed(double speed);
    double getBottomSpeed();

    void Periodic() override;

private:

    CANSparkMax m_topMotor; 
    CANSparkMax m_bottomMotor;

    SparkRelativeEncoder m_topEncoder = m_topMotor.GetEncoder(SparkRelativeEncoder::Type::kHallSensor, 42);
    SparkRelativeEncoder m_bottomEncoder = m_bottomMotor.GetEncoder(SparkRelativeEncoder::Type::kHallSensor, 42);

    SparkPIDController m_topController = m_topMotor.GetPIDController();
    SparkPIDController m_bottomController = m_bottomMotor.GetPIDController();

    double m_goalSpeed = 0;
};

