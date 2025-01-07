
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/SwerveDrive.h"
#include <subsystems/Shooter.h>

#include <subsystems/Intake.h>
#include <subsystems/Arm.h>

class ExitAuton : public frc2::CommandHelper<frc2::Command, ExitAuton> {
public:
    explicit ExitAuton(Shooter* grabber);

    void Initialize() override;
    // void Execute() override;
    void Periodic();
    void End(bool interrupted) override;

private:
    Shooter* m_shooter;

};