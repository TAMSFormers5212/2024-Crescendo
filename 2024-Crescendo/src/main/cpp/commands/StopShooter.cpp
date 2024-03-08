
#include "commands/StopShooter.h"

StopShooter::StopShooter(Shooter* grabber, Intake* intake) 
    : m_shooter(grabber), m_intake(intake) {
  
  AddRequirements(grabber);
  AddRequirements(intake);
  
}

void StopShooter::Initialize() {
    m_shooter->setPercent(0);
    m_intake->setSpeed(0);
}

void StopShooter::End(bool interrupted) {}