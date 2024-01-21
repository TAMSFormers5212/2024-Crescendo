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

    Shooter(int leftMotor, int rightMotor);


private:

    CANSparkMax m_leftMotor; // may change to top and bottom depending on the shooter design
    CANSparkMax m_rightMotor;

    // these may be modified if we don't use the design where the left and right sides can have 
    // different speeds to create spin. A high enough speed/rpm and it won't matter
    SparkRelativeEncoder m_leftEncoder = m_leftMotor.GetEncoder(SparkRelativeEncoder::Type::kHallSensor, 42);
    SparkRelativeEncoder m_rightEncoder = m_rightMotor.GetEncoder(SparkRelativeEncoder::Type::kHallSensor, 42);

    SparkPIDController m_leftController = m_leftMotor.GetPIDController();
    SparkPIDController m_rightController = m_rightMotor.GetPIDController();
};

