
#include "commands/ArmLower.h"

ArmLower::ArmLower(Arm* arm) 
    : m_arm(arm) {
  
  AddRequirements(arm);
}

void ArmLower::Initialize() {
    m_arm->setPosition(m_arm->getRelativePosition()+(-0.73+m_arm->getRawPosition())+1);
}

void ArmLower::End(bool interrupted) {
    
}