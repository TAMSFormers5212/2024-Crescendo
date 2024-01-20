#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/controller/PIDController.h>
#include <frc/AnalogEncoder.h>
// #include <frc/controller/

#include <rev/CANSparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkPIDController.h>

// clean up include list once subclasses are finished

#include <Constants.h>

using namespace std;

class superstructure : public frc2::SubsystemBase{

public:



private: 

//elevator/telescope (climb)
//1-2 motors

//elevator/telescope (truss)
//1-2 motors

//arm
//2 motors?

//intake
//1 motor

//shooter
//2 motors

// ^ need to either use pdh or reduce motor count
// pdp only has 16 slots, assuming 2 motors for both elevator systems, it comes out to 17 motors
// pdh IS in stock rn, 240$ + ~150-200$ in breakers (we could keep using old breakers)

};

// superstructure design is not completed yet, but we can make some simple outlines for the probable mechanisms of the superstructure. 

