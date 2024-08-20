#include "subsystems/Intake.h"

#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/LEDController.h"

#include <cmath>
#include <iostream>
#include <frc/DigitalInput.h>

using namespace IntakeConstants;
using namespace rev;
using namespace std;
using namespace MathConstants;

Intake::Intake(int motor, int sensor)
    : m_intakeMotor(motor, CANSparkLowLevel::MotorType::kBrushless) {
    resetMotor();

    
}

void Intake::resetMotor() {
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
    m_intakeMotor.SetInverted(true);

    m_encoder.SetPositionConversionFactor(1.0 / intakeRatio);
}

void Intake::intakeNote() {  // intake until note collected
    // if (!holdingNote) {
    //     m_intakeMotor.Set(0.4);
        
        cout<<m_intakeMotor.GetOutputCurrent()<<endl;
        if (m_intakeMotor.GetOutputCurrent() > loadedCurrent) { // monitor current to find loaded current
            indexNote();
            holdingNote = true;
            state = held;
        }
//     } else if (holdingNote) {
//         indexNote();
//     }
}
void Intake::indexNote() {  // move note to indexer spot (kinda useless without sensor)
    if (holdingNote||state==held) {
        // state = 1;
        // move the note to the beambreak if not already indexed
        // if (state != indexed) {
        //     if (beambreak == false) {
        //         m_intakeMotor.Set(0.1);
        //     } else {
        //         m_intakeMotor.Set(-0.1);
        //         state = indexed;
        //     }
        // }
        state = indexed;
    }
}
void Intake::shootNote() {  // give note to shooter
    if (holdingNote) {
        m_intakeMotor.Set(-0.4);
        if (m_intakeMotor.GetOutputCurrent() < freeCurrent) {
            holdingNote = false;
            state = IntakeConstants::empty;
        }
    }
}
void Intake::reverseIntake() {  // in case of 2 notes and need to eject 
    m_intakeMotor.Set(-0.35);
    if (m_intakeMotor.GetOutputCurrent() < freeCurrent) {
        holdingNote = false;
        IntakeConstants::empty;
    }
}
void Intake::stopIntake() {  // in case of 2 notes and need to eject 
    m_intakeMotor.Set(0);
    
}

void Intake::setSpeed(double speed) { m_intakeMotor.Set(speed); }

double Intake::getSpeed() { return m_encoder.GetVelocity(); }

bool Intake::getNote() { return holdingNote; }

double Intake::getOutputCurrent() { return m_intakeMotor.GetOutputCurrent(); }

int Intake::getState() { return state; }

void Intake::setState(int state) { this->state = state; }

void Intake::Periodic() {
    //failed attempt to get holding note working:
    
    if (m_beamBreak.Get()==0){
        noteHeld = true;
        //m_LEDs.setColor(0.65);
    }
    else{
        noteHeld= false;
        //m_LEDs.setColor(0.77);
    }
    frc::SmartDashboard::PutBoolean("holding note", noteHeld);
    // frc::SmartDashboard::PutNumber("intakeCurrent", m_intakeMotor.GetOutputCurrent());
}
