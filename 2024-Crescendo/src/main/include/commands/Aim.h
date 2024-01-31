//take in data from current arm position, distance by pose estimator and distance by vision
//calculate the nessecary arm/shooter angle and shooter rpm
//aim/rotate the robot using vision, pose, and heading

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/SwerveDrive.h"
#include "subsystems/Arm.h"
#include "subsystems/Shooter.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"

class Aim : public frc2::CommandHelper<frc2::CommandBase, Aim> {
public:
    explicit Aim(SwerveDrive* drive, Arm* arm, double x, double y, double z);

    void Initialize() override;

    void End(bool interrupted) override;

private:
    SwerveDrive* m_drive;
    Arm* m_arm;
    double m_heading;
    double m_armPose;
};