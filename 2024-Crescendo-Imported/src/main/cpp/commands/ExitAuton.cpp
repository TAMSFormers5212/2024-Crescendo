
#include "commands/ExitAuton.h"

ExitAuton::ExitAuton(Shooter* grabber) 
    : m_shooter(grabber) {
  
  AddRequirements(grabber);
  
}

void ExitAuton::Initialize() {
    m_shooter->exitAuto();
    // frc2::WaitCommand(4_s);

}

  // Called every time the scheduler runs while the command is scheduled.
// void ReadyShooter::Execute() {
//     // arm.hold(state);
//     m_shooter->setSpeed(100);
//   }


void ExitAuton::End(bool interrupted) {}