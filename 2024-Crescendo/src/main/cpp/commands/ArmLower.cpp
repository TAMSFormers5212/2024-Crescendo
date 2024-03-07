
#include "commands/ArmLower.h"

#include "Constants.h"
#include <frc2/command/WaitCommand.h>
#include <subsystems/Arm.h>
#include <frc2/command/SequentialCommandGroup.h>

ArmLower::ArmLower(Arm* arm) 
    : m_arm(arm) {
  
  AddRequirements(arm);
}

void ArmLower::Initialize() {
    m_arm->setPosition(m_arm->getRelativePosition()+(-0.73+m_arm->getRawPosition()));
}

void ArmLower::End(bool interrupted) {
    
}