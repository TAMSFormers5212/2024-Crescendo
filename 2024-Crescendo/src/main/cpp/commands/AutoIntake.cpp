
#include "commands/AutoIntake.h"

AutoIntake::AutoIntake(Intake* intake) 
    : m_intake(intake) {
  
  AddRequirements(intake);
}

void AutoIntake::Initialize() {
    m_intake->setSpeed(0.4);
}
void AutoIntake::End(bool interrupted) {}