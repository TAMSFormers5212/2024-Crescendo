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

    static double calculateSpeed(double distance, double x, double y); // calculate the needed speed based on current speed
    static double calculateAngle(double distance, double x, double y); // angle to feed to arm

    void setSpeed(double speed);
    double getSpeed();

    void Periodic() override;

private:

    CANSparkMax m_topMotor; 
    CANSparkMax m_bottomMotor;

    // these may be modified if we don't use the design where the left and right sides can have 
    // different speeds to create spin. A high enough speed/rpm and it won't matter
    SparkRelativeEncoder m_topEncoder = m_topMotor.GetEncoder(SparkRelativeEncoder::Type::kHallSensor, 42);
    SparkRelativeEncoder m_bottomEncoder = m_bottomMotor.GetEncoder(SparkRelativeEncoder::Type::kHallSensor, 42);

    SparkPIDController m_topController = m_topMotor.GetPIDController();
    SparkPIDController m_bottomController = m_bottomMotor.GetPIDController();
};

