
#include "commands/stopIntake.h"

StopIntake::StopIntake(Intake* intake) 
    : m_intake(intake) {
  
  
  AddRequirements(intake);
  
}

void StopIntake::Initialize() {
    
    m_intake->setSpeed(0);
  
}

void StopIntake::End(bool interrupted) {}