#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/SwerveDrive.h"
#include <subsystems/Shooter.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <subsystems/Intake.h>
#include <subsystems/Arm.h>
class AutoIntake : public frc2::CommandHelper<frc2::Command, AutoIntake> {
public:
    explicit AutoIntake(Intake* intake);

    void Initialize() override;

    nt::GenericEntry *getAutoI();
    void End(bool interrupted) override;

private:
    nt::GenericEntry* autoIntaking = frc::Shuffleboard::GetTab("Main Tab").AddPersistent("autoIntaking",true).GetEntry("boolean");
    Intake* m_intake;
};