
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/SwerveDrive.h"
#include <subsystems/Shooter.h>

#include <subsystems/Intake.h>
#include <subsystems/Arm.h>

class StopShooter : public frc2::CommandHelper<frc2::Command, StopShooter> {
public:
    explicit StopShooter(Shooter* grabber, Intake* intake);

    void Initialize() override;

    void End(bool interrupted) override;

private:
    Shooter* m_shooter;
    Intake* m_intake;

};