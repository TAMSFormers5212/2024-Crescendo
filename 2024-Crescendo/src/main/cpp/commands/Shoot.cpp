
#include "commands/shoot.h"

Shoot::Shoot(Intake* wrist, Shooter* grabber, Arm* arm) 
    : m_wrist(wrist), m_shooter(grabber), m_arm(arm) {
  AddRequirements(wrist);
  AddRequirements(grabber);
  AddRequirements(arm);
}

void Shoot::Initialize() {
    
}

void Shoot::End(bool interrupted) {}