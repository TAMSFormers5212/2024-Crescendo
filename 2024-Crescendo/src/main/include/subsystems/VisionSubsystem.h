#pragma once

#include <frc2/command/SubsystemBase.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include "Constants.h"

#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <span>

using namespace SwerveModuleConstants;

class VisionSubsystem : public frc2::SubsystemBase {
 public:
  VisionSubsystem();

  /**
   * Vision command factory method.
   */
  frc2::CommandPtr VisionMethodCommand();

  /**
   * An Vision method querying a boolean state of the subsystem (for Vision, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  bool VisionCondition();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void setOutput(double op);
  double getOutput();

  void setDistanceError(double dist_error);
  double getDistanceError();
  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

 private:
  frc::PIDController pid;
  double output;
  double distError;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};