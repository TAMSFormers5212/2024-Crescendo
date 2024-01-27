#include "subsystems/Intake.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <cmath>
#include <iostream>

using namespace IntakeConstants;
using namespace rev;
using namespace std;
using namespace MathConstants;


Intake::Intake(int motor)
    :   m_intakeMotor(motor, CANSparkLowLevel::MotorType::kBrushless)  
{

}

void intakeNote(); // intake until note collected
void indexNote(); // move note to indexer spot
void shootNote(); // give note to shooter 
void reverseIntake(); // in case of 2 notes and need to eject

void setSpeed(double speed);
double getSpeed();
double getPiecePosition();

void Periodic(){

}
