#include "commands/deIntake.h"

DeIntake::DeIntake(Intake* intake) 
    : m_intake(intake) {
  
  AddRequirements(intake);
}

void DeIntake::Initialize() {
    m_intake->setSpeed(-0.10);
} 
void DeIntake::End(bool interrupted) {}