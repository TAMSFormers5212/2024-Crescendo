
#include "commands/shoot.h"

#include "Constants.h"
#include <frc2/command/WaitCommand.h>

Shoot::Shoot(Intake* wrist, Shooter* grabber)
    : m_wrist(wrist), m_shooter(grabber) {
  AddRequirements(wrist);
  AddRequirements(grabber);
}

void Shoot::Initialize() {
    m_shooter->setSpeed(1);
    frc2::InstantCommand([this]() { frc2::WaitCommand(2_s);});
    m_wrist->setSpeed(0.4);
    frc2::InstantCommand([this]() { frc2::WaitCommand(1_s);});
    m_shooter->setSpeed(0);
    m_wrist->setSpeed(0);
}

void Shoot::End(bool interrupted) {}