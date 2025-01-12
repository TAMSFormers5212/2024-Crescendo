#include "subsystems/Winch.h"

using namespace WinchConstants;
using namespace MathConstants;

Winch::Winch(int motor)
    : m_winchMotor(motor, rev::spark::SparkLowLevel::MotorType::kBrushless) {
    resetMotor();
    m_winchMotor.Configure(m_winchConfig, SparkMax::ResetMode::kNoResetSafeParameters, SparkMax::PersistMode::kPersistParameters);
    
}

void Winch::resetMotor() {
    // m_winchMotor.RestoreFactoryDefaults();
    m_winchConfig.closedLoop
        .Pidf(kwP, kwI, kwD, kwFF)
        .IZone(kwIz);
    // m_winchController.SetP(kwP);
    // m_winchController.SetI(kwI);
    // m_winchController.SetD(kwD);
    // m_winchController.SetFF(kwFF);
    // m_winchController.SetIZone(kwIz);
    m_winchConfig
        .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .VoltageCompensation(12.0)
        .SmartCurrentLimit(20, 40);
    // m_winchController.SetOutputRange(kMinOutput, kMaxOutput);

    // m_winchController.SetSmartMotionMaxAccel(maxAccel);
    // m_winchController.SetSmartMotionMaxVelocity(maxVelo);
    // m_winchController.SetSmartMotionMinOutputVelocity(0);
    // m_winchController.SetSmartMotionAllowedClosedLoopError(allowedError);

    // m_winchMotor.SetIdleMode(CANSparkBase::IdleMode::kBrake);
    // m_winchMotor.EnableVoltageCompensation(12.0);
    // m_winchMotor.SetSmartCurrentLimit(20, 40);
    m_winchConfig.encoder
        .PositionConversionFactor((pi * winchDiameter.value()) / winchRatio);
    
    // m_encoder.SetPositionConversionFactor((pi * winchDiameter.value()) / winchRatio);
    m_winchMotor.Configure(m_winchConfig, SparkMax::ResetMode::kNoResetSafeParameters, SparkMax::PersistMode::kPersistParameters);  // turn it to linear distance
}

void Winch::climb(double speed) {  // spin winch based on speed
    m_winchController.SetReference(speed, rev::spark::SparkLowLevel::ControlType::kVelocity);
}
void Winch::spring() {  // act like a spring/slightly tension to prevent slack
                        // from bulding up
    if (m_encoder.GetPosition() < heightToTravel.value() / 2) {
        m_winchController.SetReference(0.1, rev::spark::SparkLowLevel::ControlType::kVelocity);
    } else {
        m_winchConfig
            .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
    }
    m_winchMotor.Configure(m_winchConfig, SparkMax::ResetMode::kNoResetSafeParameters, SparkMax::PersistMode::kPersistParameters);
}
void Winch::extend(double speed) {  // let go
    // m_winchController.SetReference(speed,
    // CANSparkLowLevel::ControlType::kVelocity);
    m_winchConfig
            .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
    m_winchMotor.Configure(m_winchConfig, SparkMax::ResetMode::kNoResetSafeParameters, SparkMax::PersistMode::kPersistParameters);
}
void Winch::hold() {  // set speed to 0
    m_winchConfig
            .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    m_winchController.SetReference(0, rev::spark::SparkLowLevel::ControlType::kVelocity);
    m_winchMotor.Configure(m_winchConfig, SparkMax::ResetMode::kNoResetSafeParameters, SparkMax::PersistMode::kPersistParameters);
}

double Winch::getOutputCurrent() { return m_winchMotor.GetOutputCurrent(); }
double Winch::getAppliedOutput() { return m_winchMotor.GetAppliedOutput(); }
double Winch::getMotorTemp() { return m_winchMotor.GetMotorTemperature(); }
double Winch::getPosition() { return m_encoder.GetPosition(); }

void Winch::setWinchPosition(double position) {  // position in linear distance from ground
    m_winchController.SetReference(position, rev::spark::SparkLowLevel::ControlType::kPosition);
}

double Winch::getWinchPosition() {
    return m_encoder.GetPosition();
}
void Winch::setWinchSpeed(double speed) {  // speed of climb
    m_winchController.SetReference(speed, rev::spark::SparkLowLevel::ControlType::kVelocity);
}

void Winch::Periodic() {}