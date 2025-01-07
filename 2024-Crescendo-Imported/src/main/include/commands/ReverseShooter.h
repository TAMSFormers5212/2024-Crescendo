
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/SwerveDrive.h"
#include <subsystems/Shooter.h>

#include <subsystems/Intake.h>
#include <subsystems/Arm.h>

class ReverseShooter : public frc2::CommandHelper<frc2::Command, ReverseShooter> {
public:
    explicit ReverseShooter(Shooter* grabber);

    void Initialize() override;
    // void Execute() override;
    // void Periodic() override;
    void End(bool interrupted) override;

private:
    Shooter* m_shooter;

};