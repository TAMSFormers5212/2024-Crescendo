
#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/SwerveDrive.h"

class StopDrive : public frc2::CommandHelper<frc2::Command, StopDrive> {
public:
    explicit StopDrive(SwerveDrive* drive);
    void Initialize() override;

    void End(bool interrupted) override;

private:
    SwerveDrive* m_drive;
};