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
    Intake(int topMotor, int bottomMotor);

    void intakeNote(); // intake until note collected
    void indexNote(); // move note to indexer spot
    void shootNote(); // give note to shooter 
    void reverseIntake(); // in case of 2 notes and need to eject

    void setSpeed(double speed);
    double getSpeed();
    double getPiecePosition();


private:

    CANSparkMax m_intakeMotor; 

    // these may be modified if we don't use the design where the left and right sides can have 
    // different speeds to create spin. A high enough speed/rpm and it won't matter
    SparkRelativeEncoder m_encoder = m_intakeMotor.GetEncoder(SparkRelativeEncoder::Type::kHallSensor, 42);

    SparkPIDController m_intakeController = m_intakeMotor.GetPIDController();

    //sensor
    //beambreak? distance sensor? color sensor?

    int state;


};

