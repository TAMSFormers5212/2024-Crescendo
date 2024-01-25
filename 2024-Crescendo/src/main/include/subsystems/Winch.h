#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/ArmFeedForward.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <rev/CANSparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkPIDController.h>

#include <frc/AnalogEncoder.h>

#include <Constants.h>

using namespace rev;
using namespace frc;

class Winch : public frc2::SubsystemBase{
public:
    Winch();

    void climb();

private:
    CANSparkMax m_winchMotor; 

    SparkRelativeEncoder m_encoder = m_winchMotor.GetEncoder(SparkRelativeEncoder::Type::kHallSensor, 42);

    SparkPIDController m_winchController = m_winchMotor.GetPIDController();

};