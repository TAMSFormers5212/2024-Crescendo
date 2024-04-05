
#include "commands/ReadyShooter.h"

ReadyShooter::ReadyShooter(Shooter* grabber) 
    : m_shooter(grabber) {
  
  AddRequirements(grabber);
  
}

void ReadyShooter::Initialize() {
    m_shooter->setPercent(0.95);
    m_shooter->setSpeed(500);
    m_shooter->enterAuto();
    // frc2::WaitCommand(4_s);

}

  // Called every time the scheduler runs while the command is scheduled.
// void ReadyShooter::Execute() {
//     // arm.hold(state);
//     m_shooter->setSpeed(100);
//   }


void ReadyShooter::Periodic(){
    m_shooter->setPercent(0.95);
    m_shooter->setSpeed(500);

}

void ReadyShooter::End(bool interrupted) {}