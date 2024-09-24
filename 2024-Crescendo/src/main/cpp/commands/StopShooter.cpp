
#include "commands/StopShooter.h"
#include <frc/smartdashboard/SmartDashboard.h>

StopShooter::StopShooter(Shooter* grabber, Intake* intake) 
    : m_shooter(grabber), m_intake(intake) {
  
  AddRequirements(grabber);
  AddRequirements(intake);
  
}

void StopShooter::Initialize() {
    m_shooter->setSpeed(0);
    m_intake->setSpeed(0);
    //m_shooter->exitAuto();
    frc::SmartDashboard::PutBoolean("autoShooting",false);
    frc::SmartDashboard::PutBoolean("autoIntaking",false);
}

void StopShooter::End(bool interrupted) {}