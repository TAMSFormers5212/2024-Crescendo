#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/SwerveDrive.h"
#include <subsystems/Shooter.h>

#include <subsystems/Intake.h>
#include <subsystems/Arm.h>
class DeIntake : public frc2::CommandHelper<frc2::Command, DeIntake> {
public:
    explicit DeIntake(Intake* intake);

    void Initialize() override;

    void End(bool interrupted) override;

private:
    
    Intake* m_intake;
};