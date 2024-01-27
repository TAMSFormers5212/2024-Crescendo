#pragma once

#include <frc2/command/SubsystemBase.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

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

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};