
#include "commands/ReadyShooter.h"
#include <frc/smartdashboard/SmartDashboard.h>

ReadyShooter::ReadyShooter(Shooter* grabber, double speedIn) 
    : m_shooter(grabber) {
  speed = speedIn;
  AddRequirements(grabber);
  
}

void ReadyShooter::Initialize() {
    //m_shooter->setPercent(0.95);
    m_shooter->setSpeed(speed);
    frc::SmartDashboard::PutBoolean("autoShooting",true);
    //m_shooter->enterAuto();
    // frc2::WaitCommand(4_s);

}

  // Called every time the scheduler runs while the command is scheduled.
// void ReadyShooter::Execute() {
//     // arm.hold(state);
//     m_shooter->setSpeed(100);
//   }


void ReadyShooter::Periodic(){
    //m_shooter->setPercent(0.95);
    m_shooter->setSpeed(speed);
    frc::SmartDashboard::PutBoolean("autoShooting",true);
}

void ReadyShooter::End(bool interrupted) {}