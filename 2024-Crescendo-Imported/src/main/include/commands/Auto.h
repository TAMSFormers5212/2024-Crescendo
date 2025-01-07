#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/SwerveDrive.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"
#include "subsystems/Arm.h"
#include "Constants.h"

class Auto : public frc2::CommandHelper<frc2::SequentialCommandGroup, Auto> {
 public:
  Auto(SwerveDrive* drive, Arm* arm, Intake* intake, Shooter* shooter);// drives for mobility
  Auto(SwerveDrive* drive);
  // void CH(SwerveDrive* drive, Intake* wrist, Arm* arm, Shooter* grabber); // cone high

};