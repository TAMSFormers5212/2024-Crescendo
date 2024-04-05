
#include "commands/StopShooter.h"

StopShooter::StopShooter(Shooter* grabber, Intake* intake) 
    : m_shooter(grabber), m_intake(intake) {
  
  AddRequirements(grabber);
  AddRequirements(intake);
  
}

void StopShooter::Initialize() {
    m_shooter->setSpeed(0);
    m_intake->setSpeed(0);
    m_shooter->exitAuto();
}

void StopShooter::End(bool interrupted) {}