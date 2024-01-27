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
using namespace ArmConstants;

class Arm : public frc2::SubsystemBase{

public:
    Arm(int leftMotor, int rightMotor, int encoder, double encoderOffset);

    void setPosition(double armPose);
    double getPosition();
    void resetMotors();
    void resetEncoder();
    double getRawPosition();

    void Periodic() override;

private:

    CANSparkMax m_leftMotor;
    CANSparkMax m_rightMotor;

    SparkRelativeEncoder m_leftEncoder = m_leftMotor.GetEncoder(SparkRelativeEncoder::Type::kHallSensor, 42);
    SparkRelativeEncoder m_rightEncoder = m_rightMotor.GetEncoder(SparkRelativeEncoder::Type::kHallSensor, 42);

    SparkPIDController m_leftController = m_leftMotor.GetPIDController(); // leader
    SparkPIDController m_rightController = m_rightMotor.GetPIDController(); // follower


//failed attempt at using ProfiledPIDController. i'll just try smartmotion instead
    // units::velocity::feet_per_second maxVelocity{maxVelo};

    // TrapezoidProfile m_armConstraints{maxVelo, maxAccel};
    //  TrapezoidProfile::Cons
    //   {maxVelo, maxAccel};
    // ProfiledPIDController m_armController = new ProfiledPIDController(kaP, kaI, kaD, m_armConstraints, units::second_t(20_ms));
    // ArmFeedforward m_armFF = new ArmFeedforward(units::volt_t{kaS}, units::volt_t{kaG}, units::unit_t< kv_unit > kaV, units::unit_t< ka_unit > kaA);


    DutyCycleEncoder m_absoluteEncoder;

    double position = 0.0;

};