#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>

#include <frc/AnalogEncoder.h>

#include <rev/CANSparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkPIDController.h>
// #include <rev/ThroughBoreEncoder.h>

// clean up include list once subclasses are finished

#include <Constants.h>

using namespace std;
using namespace rev;

class Intake : public frc2::SubsystemBase{

public:
    Intake(int motor, int sensor);

    void resetMotor();

    void intakeNote(); // intake until note collected
    void indexNote(); // move note to indexer spot
    void shootNote(); // give note to shooter 
    void reverseIntake(); // in case of 2 notes and need to eject

    void setSpeed(double speed);
    double getSpeed();
    bool getNote();
    double getOutputCurrent();
    int getState();
    void setState(int state);

    void Periodic() override;

private:

    CANSparkMax m_intakeMotor; // may need to switch to 775 if neo550 is not fixed in time

    SparkRelativeEncoder m_encoder = m_intakeMotor.GetEncoder(SparkRelativeEncoder::Type::kHallSensor, 42);

    SparkPIDController m_intakeController = m_intakeMotor.GetPIDController();

    //sensor
    //beambreak? distance sensor? color sensor?

    int state = IntakeConstants::empty;
    bool holdingNote = false;



};

