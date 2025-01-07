#include "commands/reverseShooter.h"

ReverseShooter::ReverseShooter(Shooter* grabber) 
    : m_shooter(grabber) {
  
  AddRequirements(grabber);
  
}

void ReverseShooter::Initialize() {
    m_shooter->setSpeed(-10);
    // frc2::WaitCommand(4_s);

}

  // Called every time the scheduler runs while the command is scheduled.
// void ReadyShooter::Execute() {
//     // arm.hold(state);
//     m_shooter->setSpeed(100);
//   }


// void ReadyShooter::Periodic(){
//     m_shooter->setPercent(1);
// }

void ReverseShooter::End(bool interrupted) {}