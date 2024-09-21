
#include "commands/AutoIntake.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

AutoIntake::AutoIntake(Intake* intake) 
    : m_intake(intake) {
  
  AddRequirements(intake);
}

void AutoIntake::Initialize() {
    m_intake->setSpeed(0.4);
// <<<<<<< cmfaw
    frc::SmartDashboard::PutBoolean("SmartDashboard/Main Tab/autoIntaking", !(frc::SmartDashboard::GetBoolean("SmartDashboard/Main Tab/autoIntaking",false)));
// =======
//     autoIntaking->SetBoolean(!(autoIntaking->GetBoolean(false)));
    // frc::Shuffleboard::GetTab("Main Tab").Add("autoIntaking", !(frc::SmartDashboard::GetBoolean("autoIntaking",false)));
}
// nt::GenericEntry *AutoIntake::getAutoI(){
//   return autoIntaking;
// >>>>>>> main
}
void AutoIntake::End(bool interrupted) {}