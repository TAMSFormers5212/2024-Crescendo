#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/SwerveDrive.h"
#include <subsystems/Shooter.h>

#include <subsystems/Intake.h>
#include <subsystems/Arm.h>
class AutoIntake : public frc2::CommandHelper<frc2::Command, AutoIntake> {
public:
    explicit AutoIntake(Intake* intake);

    void Initialize() override;

    void End(bool interrupted) override;

private:
    
    Intake* m_intake;
};