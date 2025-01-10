#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/PIDController.h>
// #include <frc/controller/ProfiledPIDController.h>
// #include <frc/controller/ArmFeedForward.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkClosedLoopController.h>

#include <frc/AnalogEncoder.h>

#include <Constants.h>

using namespace rev::spark;
using namespace frc;

class Winch : public frc2::SubsystemBase{
public:
    Winch();
    Winch(int motor);

    void resetMotor();

    void climb(double speed); // move winch based on speed
    void spring(); // act like a spring/slightly tension to prevent slack from bulding up
    void extend(double speed); // let go
    void hold(); // set speed to 0, hold in place

    double getOutputCurrent();
    double getAppliedOutput();
    double getMotorTemp();
    double getPosition();

    void setWinchPosition(double position); // position in linear distance from ground
    double getWinchPosition();
    void setWinchSpeed(double speed);


    void Periodic() override;

private:
    SparkMax m_winchMotor; 

    SparkMaxConfig m_winchConfig; 

    SparkRelativeEncoder m_encoder = m_winchMotor.GetEncoder();

    SparkClosedLoopController m_winchController = m_winchMotor.GetClosedLoopController();

};