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
    Winch(int motor);

    void resetMotor();

    void climb(double speed); // pull down
    void spring(); // act like a spring/slightly tension to prevent slack from bulding up
    void extend(double speed); // let go
    void hold(); // set speed to 0

    double getOutputCurrent();
    double getAppliedOutput();

    void setWinchPosition(double position); // position in linear distance from ground


    void Periodic() override;

private:
    CANSparkMax m_winchMotor; 

    SparkRelativeEncoder m_encoder = m_winchMotor.GetEncoder(SparkRelativeEncoder::Type::kHallSensor, 42);

    SparkPIDController m_winchController = m_winchMotor.GetPIDController();

};