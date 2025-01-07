#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>

#include <frc/AnalogEncoder.h>

#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkClosedLoopController.h>
// #include <rev/ThroughBoreEncoder.h>
#include "LEDController.h"
// clean up include list once subclasses are finished

#include <Constants.h>
#include <frc/DigitalInput.h>

using namespace std;
using namespace rev::spark;

class Intake : public frc2::SubsystemBase{

public:
    Intake(int motor, int sensor);

    void resetMotor();

    void intakeNote(); // intake until note collected
    void indexNote(); // move note to indexer spot
    void shootNote(); // give note to shooter 
    void reverseIntake(); // in case of 2 notes and need to eject
    void stopIntake(); 

    void setSpeed(double speed);
    double getSpeed();
    bool getNote();
    double getOutputCurrent();
    int getState();
    void setState(int state);

    void Periodic() override;
    bool noteHeld = false;
private:

    SparkMax m_intakeMotor; // may need to switch to 775 if neo550 is not fixed in time

    SparkRelativeEncoder m_encoder = m_intakeMotor.GetEncoder();

    SparkClosedLoopController m_intakeController = m_intakeMotor.GetClosedLoopController();

    //sensor
    
    //beambreak? distance sensor? color sensor?
    frc::DigitalInput m_beamBreak{4};
    int state = IntakeConstants::empty;
    bool holdingNote = false;
    //LEDController m_LEDs;
    



};

