
#include "commands/AutoIntake.h"

#include "Constants.h"
#include <frc2/command/WaitCommand.h>
#include <subsystems/Arm.h>
#include <frc2/command/SequentialCommandGroup.h>

AutoIntake::AutoIntake(Intake* intake) 
    : m_intake(intake) {
  
  AddRequirements(intake);
}

void AutoIntake::Initialize() {
    m_intake->setSpeed(0.4);
}
void AutoIntake::End(bool interrupted) {}