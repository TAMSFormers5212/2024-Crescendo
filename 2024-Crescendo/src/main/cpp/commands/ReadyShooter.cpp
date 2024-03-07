
#include "commands/ReadyShooter.h"

#include "Constants.h"
#include <frc2/command/WaitCommand.h>
#include <subsystems/Arm.h>
#include <frc2/command/SequentialCommandGroup.h>

ReadyShooter::ReadyShooter(Shooter* grabber) 
    : m_shooter(grabber) {
  
  AddRequirements(grabber);
  
}

void ReadyShooter::Initialize() {
    m_shooter->setPercent(0.7);
    // frc2::WaitCommand(4_s);

}
// void ReadyShooter::Periodic(){
//     m_shooter->setPercent(1);
// }

void ReadyShooter::End(bool interrupted) {}