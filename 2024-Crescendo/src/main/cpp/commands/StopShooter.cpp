
#include "commands/StopShooter.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include "commands/ReadyShooter.h"
#include "commands/AutoIntake.h"
StopShooter::StopShooter(Shooter* grabber, Intake* intake) 
    : m_shooter(grabber), m_intake(intake) {
  
  AddRequirements(grabber);
  AddRequirements(intake);
  
}

void StopShooter::Initialize() {
    m_shooter->setSpeed(0);
    m_intake->setSpeed(0);
    //m_shooter->exitAuto();
// <<<<<<< cmfaw
    frc::SmartDashboard::PutBoolean("SmartDashboard/Main Tab/autoShooting",false);
    frc::SmartDashboard::PutBoolean("SmartDashboard/Main Tab/autoIntaking",false);
// =======
//     ReadyShooter myS = ReadyShooter(m_shooter);
//     AutoIntake myI = AutoIntake(m_intake);
//     myI.getAutoI()->SetBoolean(false);
//     myS.getAutoS()->SetBoolean(false);
    // frc::Shuffleboard::GetTab("Main Tab").Add("autoShooting",false);
    // frc::Shuffleboard::GetTab("Main Tab").Add("autoIntaking",false);
// >>>>>>> main
}

void StopShooter::End(bool interrupted) {}