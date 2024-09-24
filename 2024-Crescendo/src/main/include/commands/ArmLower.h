
#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/SwerveDrive.h"
#include <subsystems/Shooter.h>

#include <subsystems/Intake.h>
#include <subsystems/Arm.h>

class ArmLower : public frc2::CommandHelper<frc2::Command, ArmLower> {
public:
    explicit ArmLower(Arm* arm);

    void Initialize() override;

    void End(bool interrupted) override;

private:
    
    Arm* m_arm;
};