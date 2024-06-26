
#include "commands/AutoIntake.h"
#include <frc/smartdashboard/SmartDashboard.h>

AutoIntake::AutoIntake(Intake* intake) 
    : m_intake(intake) {
  
  AddRequirements(intake);
}

void AutoIntake::Initialize() {
    m_intake->setSpeed(0.4);
    frc::SmartDashboard::PutBoolean("autoIntaking", !(frc::SmartDashboard::GetBoolean("autoIntaking",false)));
}
void AutoIntake::End(bool interrupted) {}