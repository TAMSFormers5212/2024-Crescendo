#include "subsystems/Intake.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <cmath>
#include <iostream>

using namespace IntakeConstants;
using namespace rev;
using namespace std;
using namespace MathConstants;


Intake::Intake(int motor, int sensor)
    :   m_intakeMotor(motor, CANSparkLowLevel::MotorType::kBrushless)  
{
    resetMotor();
}

void Intake::resetMotor(){
    m_intakeMotor.RestoreFactoryDefaults();
  
    // m_intakeController.SetP(kiP);
    // m_intakeController.SetI(kaI);
    // m_intakeController.SetD(kaD);
    // m_intakeController.SetFF(kaFF);
    // m_intakeController.SetIZone(kaIz);
    // m_intakeController.SetOutputRange(kMinOutput, kMaxOutput);

    // m_intakeController.SetSmartMotionMaxAccel(maxAccel);
    // m_leftController.SetSmartMotionMaxVelocity(maxVelo);
    // m_leftController.SetSmartMotionMinOutputVelocity(0);
    // m_leftController.SetSmartMotionAllowedClosedLoopError(allowedError);

    m_intakeMotor.SetIdleMode(CANSparkBase::IdleMode::kBrake);
    m_intakeMotor.EnableVoltageCompensation(12.0);
    m_intakeMotor.SetSmartCurrentLimit(20, 25);

    m_encoder.SetPositionConversionFactor(1.0/intakeRatio);
}

void Intake::intakeNote(){ // intake until note collected
    if(!holdingNote){
        m_intakeMotor.Set(0.5);
        if(m_intakeMotor.GetOutputCurrent()>10){
            indexNote();
            holdingNote = true;
            // state = 1;
        }
    }else if(holdingNote){
        indexNote();
    }
}
void Intake::indexNote(){ // move note to indexer spot
    if(holdingNote){
        // state = 1;
        //move the note to the beambreak
    }
}
void Intake::shootNote(){ // give note to shooter 
    if(holdingNote){
        m_intakeMotor.Set(1);
        if(m_intakeMotor.GetOutputCurrent()<5){
            holdingNote = false;
            // state = 0;
        }
    }
}
void Intake::reverseIntake(){ // in case of 2 notes and need to eject
    m_intakeMotor.Set(-0.5);
    if(m_intakeMotor.GetOutputCurrent()<5){
        holdingNote = false;
    }
}

void Intake::setSpeed(double speed){
    m_intakeMotor.Set(speed);
}
double Intake::getSpeed(){
    return m_encoder.GetVelocity();
}
double Intake::getPiecePosition(){
    
}

double Intake::getOutputCurrent(){
    return m_intakeMotor.GetOutputCurrent();
}


void Periodic(){

}
