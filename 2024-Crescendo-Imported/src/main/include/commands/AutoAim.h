
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/SwerveDrive.h"
#include <subsystems/Shooter.h>

#include <subsystems/Intake.h>
#include <subsystems/Arm.h>
#include <subsystems/VisionSubsystem.h>
#include <subsystems/Superstructure.h>

class AutoAim : public frc2::CommandHelper<frc2::Command, AutoAim> {
public:
    explicit AutoAim(Arm* arm, VisionSubsystem* vision, Superstructure* superstructure);

    void Initialize() override;

    void End(bool interrupted) override;

private:
    
    Arm* m_arm;
    VisionSubsystem* m_vision;
    Superstructure* m_superstructure;
};