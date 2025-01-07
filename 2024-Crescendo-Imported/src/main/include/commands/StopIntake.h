
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/SwerveDrive.h"
#include <subsystems/Shooter.h>

#include <subsystems/Intake.h>
#include <subsystems/Arm.h>

class StopIntake : public frc2::CommandHelper<frc2::Command, StopIntake> {
public:
    explicit StopIntake(Intake* intake);

    void Initialize() override;

    void End(bool interrupted) override;

private:
   
    Intake* m_intake;

};