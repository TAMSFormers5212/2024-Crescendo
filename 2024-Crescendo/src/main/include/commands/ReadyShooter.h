
#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include "subsystems/SwerveDrive.h"
#include <subsystems/Shooter.h>

#include <subsystems/Intake.h>
#include <subsystems/Arm.h>

#include <networktables/GenericEntry.h>

class ReadyShooter : public frc2::CommandHelper<frc2::Command, ReadyShooter> {
public:
    explicit ReadyShooter(Shooter* grabber);
    

    void Initialize() override;
    // void Execute() override;
    void Periodic();
    nt::GenericEntry *getAutoS();
    void End(bool interrupted) override;

private:

    Shooter* m_shooter;
    // frc::SimpleWidget* autoShoot = frc::Shuffleboard::GetTab("Main Tab").Add("autoShooting",true);
    // nt::GenericPublisher autoShootingPub = autoShooting.GenericPublish("boolean");
    nt::GenericEntry* autoShooting = frc::Shuffleboard::GetTab("Main Tab").AddPersistent("autoShooting",true).GetEntry("boolean");
};