#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/ArmFeedForward.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/voltage.h>


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
    double getVelocity();
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


    // TrapezoidProfile<units::meters> profile{{maxVelo, maxAccel}};
    // TrapezoidProfile<units::degrees> m_profile{{maxRotatioV, maxRotationA}};

    // units::degree_t m_goalDistance = 0_deg;
    // units::degrees_per_second_t m_goalSpeed = 0_deg_per_s;
    // // TrapezoidProfile<units::degree_t>::State m_setpoint;
    // units::degree_t m_setpointDistance = 0_deg;
    // units::degrees_per_second_t m_setpointSpeed = 0_deg_per_s;
    // TrapezoidProfile<units::degrees>::State m_setpoint = {m_setpointDistance, m_setpointSpeed};

    //someone else will have to tune this. i've never done it before
    // ArmFeedforward m_armFF = ArmFeedforward(units::volt_t{kaS}, units::volt_t{kaG}, units::unit_t< ArmFeedforward::kv_unit >{kaV}, units::unit_t< ArmFeedforward::kv_unit >{kaA});


    DutyCycleEncoder m_absoluteEncoder;

    double position = 0.0;

};