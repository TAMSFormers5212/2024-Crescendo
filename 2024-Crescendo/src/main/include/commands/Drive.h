//take in data from current arm position, distance by pose estimator and distance by vision
//calculate the nessecary arm/shooter angle and shooter rpm
//aim/rotate the robot using vision, pose, and heading

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/SwerveDrive.h"

class Drive : public frc2::CommandHelper<frc2::Command, Drive> {
public:
    explicit Drive(SwerveDrive* drive, double x, double y, double z, bool blue);

    void Initialize() override;

    void End(bool interrupted) override;

private:
    SwerveDrive* m_drive;
    bool blueAlliance;
    double x = 0;
    double y = 0.5;
    double z = 0;
};