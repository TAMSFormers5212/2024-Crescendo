
#include "commands/AutoIntake.h"
#include <frc/smartdashboard/SmartDashboard.h>

AutoIntake::AutoIntake(Intake* intake) 
    : m_intake(intake) {
  
  AddRequirements(intake);
}

void AutoIntake::Initialize() {
    m_intake->setSpeed(0.8);
    frc::SmartDashboard::PutBoolean("autoIntaking", true);
}
void AutoIntake::End(bool interrupted) {}