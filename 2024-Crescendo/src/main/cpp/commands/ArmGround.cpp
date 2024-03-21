#include "commands/armGround.h"

ArmGround::ArmGround(Arm* arm) 
    : m_arm(arm) {
  
  AddRequirements(arm);
}

void ArmGround::Initialize() {
    m_arm->setPosition(m_arm->groundPreset());//m_arm->getRelativePosition()-(-0.03+m_arm->getRawPosition()));
}

void ArmGround::End(bool interrupted) {
    
}