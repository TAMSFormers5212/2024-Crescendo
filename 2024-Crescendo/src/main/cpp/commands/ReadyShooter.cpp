
#include "commands/ReadyShooter.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/GenericEntry.h>

ReadyShooter::ReadyShooter(Shooter* grabber) 
    : m_shooter(grabber) {
  
  AddRequirements(grabber);
  
}

void ReadyShooter::Initialize() {
    //m_shooter->setPercent(0.95);
    m_shooter->setSpeed(300);
    
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
    m_shooter->setSpeed(300);
    autoShooting->SetBoolean(true);
    // frc::Shuffleboard::GetTab("Main Tab").Add("autoShooting",true);
}

nt::GenericEntry *ReadyShooter::getAutoS(){
  return autoShooting;
}

void ReadyShooter::End(bool interrupted) {}