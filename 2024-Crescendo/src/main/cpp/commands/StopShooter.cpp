
#include "commands/StopShooter.h"

#include "Constants.h"
#include <frc2/command/WaitCommand.h>
#include <subsystems/Arm.h>
#include <frc2/command/SequentialCommandGroup.h>

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