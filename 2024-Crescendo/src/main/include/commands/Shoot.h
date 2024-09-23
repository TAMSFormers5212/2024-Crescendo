
#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include "subsystems/SwerveDrive.h"
#include <subsystems/Shooter.h>

#include <subsystems/Intake.h>
#include <subsystems/Arm.h>

class Shoot : public frc2::CommandHelper<frc2::Command, Shoot> {
public:
    explicit Shoot(Intake* wrist, Shooter* grabber, Arm* arm);

    void Initialize() override;

    void End(bool interrupted) override;

private:
    Shooter* m_shooter;
    Intake* m_wrist;
    Arm* m_arm;
};