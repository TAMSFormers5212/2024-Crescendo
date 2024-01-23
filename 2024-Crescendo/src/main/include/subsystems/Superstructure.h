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

class Superstructure : public frc2::SubsystemBase{

public:

    Superstructure(); // elevator, telescope, arm, intake, shooter



    void Periodic() override;

private: 

//winch
//2 motors total (1 motor per winch)

//arm
//2 motors

//intake
//1 motor

//shooter
//2 motors
// ^ maybe combining intake and shooter into one mechanism may make the handoff easier (3 motors)
//inter

//2+2+1+2+8 = 15

};

// superstructure design is not completed yet, but we can make some simple outlines for the probable mechanisms of the superstructure. 

