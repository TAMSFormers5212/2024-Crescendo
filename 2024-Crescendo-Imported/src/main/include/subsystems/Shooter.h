#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>

#include <frc/AnalogEncoder.h>

#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkClosedLoopController.h>
// #include <rev/ThroughBoreEncoder.h>

// clean up include list once subclasses are finished
#include <frc2/command/sysid/SysIdRoutine.h>
#include <Constants.h>
#include <frc/controller/SimpleMotorFeedforward.h>

using namespace std;
using namespace rev::spark;
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
    bool isShooterGood();
    bool shooterGood;
    // frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
    // frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);

    void Periodic() override;
    // void TeleopPeriodic() override;
    // void AutonomousPeriodic() override;

private:

    SparkMax m_leftMotor; 
    SparkMax m_rightMotor;

    SparkRelativeEncoder m_leftEncoder = m_leftMotor.GetEncoder();
    SparkRelativeEncoder m_rightEncoder = m_rightMotor.GetEncoder();

    SparkClosedLoopController m_leftController = m_leftMotor.GetClosedLoopController();
    SparkClosedLoopController m_rightController = m_rightMotor.GetClosedLoopController();

    SimpleMotorFeedforward<units::meters> m_leftFF;
    SimpleMotorFeedforward<units::meters> m_rightFF;

    bool inAuto;
    
    double avgSho;
    units::meters_per_second_t m_goalSpeed{0};

    //  frc2::sysid::SysIdRoutine m_sysIdRoutine{
    //   frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt,
    //                       std::nullopt},
    //   frc2::sysid::Mechanism{
    //       [this](units::volt_t driveVoltage) {
    //         m_leftMotor.SetVoltage(driveVoltage);
    //         m_rightMotor.SetVoltage(driveVoltage);
    //       },
    //       [this](frc::sysid::SysIdRoutineLog* log) {
    //         log->Motor("shooter-wheel")
    //             .voltage(m_leftMotor.Get() *
    //                      frc::RobotContainer::GetBatteryVoltage())
    //             .position(units::turn_t{m_leftEncoder.GetDistance()})
    //             .velocity(
    //                 units::turns_per_second_t{m_leftEncoder.GetRate()});
    //       },
    //       this}};
    //  frc::PIDController m_shooterFeedback{ShooterConstants::ksP, 0, 0};
    //  frc::SimpleMotorFeedforward<units::turns> m_shooterFeedforward{
    // ShooterConstants::ksS, ShooterConstants::ksV, ShooterConstants::ksA};
    // frc::SimpleMotorFeedforward<units::turns> m_leftShooterFeedforward{
    //   ShooterConstants::KlsS, ShooterConstants::KlsV, 0};
    // frc::SimpleMotorFeedforward<units::turns> m_rightShooterFeedforward{
    // ShooterConstants::KrsS, ShooterConstants::KrsV, 0};
};

