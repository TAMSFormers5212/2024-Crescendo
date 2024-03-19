
#include "commands/ArmLower.h"

ArmLower::ArmLower(Arm* arm) 
    : m_arm(arm) {
  
  AddRequirements(arm);
}

void ArmLower::Initialize() {
    m_arm->setPosition(m_arm->speakerPreset());//m_arm->getRelativePosition()-(-0.03+m_arm->getRawPosition()));
}

void ArmLower::End(bool interrupted) {
    
}