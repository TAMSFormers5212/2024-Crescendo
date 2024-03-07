
#include "commands/shoot.h"

#include "Constants.h"
#include <frc2/command/WaitCommand.h>
#include <subsystems/Arm.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>

Shoot::Shoot(Intake* wrist, Shooter* grabber, Arm* arm) 
    : m_wrist(wrist), m_shooter(grabber), m_arm(arm) {
  AddRequirements(wrist);
  AddRequirements(grabber);
  AddRequirements(arm);
}

void Shoot::Initialize() {
    
}

void Shoot::End(bool interrupted) {}