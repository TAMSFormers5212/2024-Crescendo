#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/SwerveDrive.h"
#include <subsystems/Shooter.h>

#include <subsystems/Intake.h>
#include <subsystems/Arm.h>

class ArmGround : public frc2::CommandHelper<frc2::Command, ArmGround> {
public:
    explicit ArmGround(Arm* arm);

    void Initialize() override;

    void End(bool interrupted) override;

private:
    
    Arm* m_arm;
};