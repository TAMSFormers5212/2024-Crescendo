#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/ArmFeedForward.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/voltage.h>


#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkClosedLoopController.h>



#include <frc/AnalogEncoder.h>


#include <Constants.h>

using namespace rev::spark;
using namespace frc;
using namespace ArmConstants;

class Arm : public frc2::SubsystemBase{

public:
    Arm(int leftMotor, int rightMotor, int encoder, double encoderOffset);

    void setPosition(double armPose);
    void setNeoPosition(double armPose);
    double getPosition();
    double getVelocity();
    void resetMotors();
    void resetEncoder();
    double getRawPosition();
    double getRelativePosition();
    void set(double value);
    void setInitialPosition();
    double ampPreset();
    double speakerPreset();
    double groundPreset();
    void Periodic() override;

    double initalPosition =0;
    

private:

    SparkMax m_leftMotor;
    SparkMax m_rightMotor;

    SparkMaxConfig m_leftConfig;
    SparkMaxConfig m_rightConfig;

    SparkRelativeEncoder m_leftEncoder = m_leftMotor.GetEncoder();
    SparkRelativeEncoder m_rightEncoder = m_rightMotor.GetEncoder();

    SparkClosedLoopController m_leftController = m_leftMotor.GetClosedLoopController(); // leader
    SparkClosedLoopController m_rightController = m_rightMotor.GetClosedLoopController(); // follower


    // TrapezoidProfile<units::meters> profile{{maxVelo, maxAccel}};
    // TrapezoidProfile<units::radians> m_profile{{maxVelo, maxAccel}};

    // units::radian_t m_setpointDistance {0};
    // units::radians_per_second_t m_setpointSpeed {0};
    // TrapezoidProfile<units::radians>::State m_setpoint = {m_setpointDistance, m_setpointSpeed};

    //someone else will have to tune this. i've never done it before
    ArmFeedforward m_armFF ; // initalized in constructor


    DutyCycleEncoder m_absoluteEncoder{encoder};

    double position = 0.0;  
    bool commandGiven = false;

};