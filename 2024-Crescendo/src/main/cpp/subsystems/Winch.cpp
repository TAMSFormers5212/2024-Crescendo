#include "subsystems/Winch.h"

using namespace WinchConstants;
using namespace MathConstants;

Winch::Winch(int motor)
    : m_winchMotor(motor, CANSparkLowLevel::MotorType::kBrushless)
{

}

void Winch::resetMotor(){
    m_winchMotor.RestoreFactoryDefaults();
  
    m_winchController.SetP(kwP);
    m_winchController.SetI(kwI);
    m_winchController.SetD(kwD);
    m_winchController.SetFF(kwFF);
    m_winchController.SetIZone(kwIz);
    // m_winchController.SetOutputRange(kMinOutput, kMaxOutput);

    // m_winchController.SetSmartMotionMaxAccel(maxAccel);
    // m_winchController.SetSmartMotionMaxVelocity(maxVelo);
    // m_winchController.SetSmartMotionMinOutputVelocity(0);
    // m_winchController.SetSmartMotionAllowedClosedLoopError(allowedError);

    m_winchMotor.SetIdleMode(CANSparkBase::IdleMode::kBrake);
    m_winchMotor.EnableVoltageCompensation(12.0);
    m_winchMotor.SetSmartCurrentLimit(20, 40);

    m_encoder.SetPositionConversionFactor((pi*winchDiameter.value())/winchRatio); // turn it to linear distance
}

void Winch::climb(double speed){ // pull down
    m_winchController.SetReference(speed, CANSparkLowLevel::ControlType::kVelocity);
}
void Winch::spring(){ // act like a spring/slightly tension to prevent slack from bulding up
    m_winchController.SetReference(0.1, CANSparkMax::ControlType::kVelocity);
}
void Winch::extend(double speed){ // let go
    m_winchController.SetReference(speed, CANSparkLowLevel::ControlType::kVelocity);
}
void Winch::hold(){ // set speed to 0
    m_winchController.SetReference(0, CANSparkLowLevel::ControlType::kVelocity);
}

double Winch::getOutputCurrent(){
    return m_winchMotor.GetOutputCurrent();
}
double Winch::getAppliedOutput(){   
    return m_winchMotor.GetAppliedOutput();
}

void Winch::setWinchPosition(double position){ // position in linear distance from ground
    m_winchController.SetReference(heightToTravel, CANSparkLowLevel::ControlType::kPosition);
}


void Winch:: Periodic(){

}