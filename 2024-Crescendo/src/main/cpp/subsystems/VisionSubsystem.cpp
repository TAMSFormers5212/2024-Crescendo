#include "subsystems/VisionSubsystem.h"
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <span>

VisionSubsystem::VisionSubsystem()
    : output(),
      distError(),
      pid(kvP, kvI, kvD)
      {
  
  // Implementation of subsystem constructor goes here.
}

frc2::CommandPtr VisionSubsystem::VisionMethodCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return RunOnce([/* this */] { /* one-time action goes here */ });
}

bool VisionSubsystem::VisionCondition() {
  // Query some boolean state, such as a digital sensor.
  return false;
}

void VisionSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    frc::PIDController pid(kvP, kvI, kvD);
            double targetOffsetAngle_Vertical = table->GetNumber("ty", 0.0);

            double limelightMountAngleDegrees = 5;

            double limelightLensHeightInches = 8.5;
            
            double goalHeightInches = 12; 

            double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

            //calculate distance
            double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/tan(angleToGoalRadians);
            float KpDistance = -0.1f;  // Proportional control constant for distance
            float desired_distance = 10;
            float distance_error = abs(desired_distance-distanceFromLimelightToGoalInches)*KpDistance;
            setDistanceError(distance_error);
             
            frc::SmartDashboard::PutNumber("up angle", targetOffsetAngle_Vertical);
            frc::SmartDashboard::PutNumber("distance", distanceFromLimelightToGoalInches);
            double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
            double heading_error = targetOffsetAngle_Horizontal;
            // pid.SetSetpoint(0);
            frc::SmartDashboard::PutNumber("heading", heading_error);
            double output = pid.Calculate(heading_error, 0); 
            frc::SmartDashboard::PutNumber("pid", output);   
            setOutput(output);
            
            // pid.Calculate();

}

void VisionSubsystem::setOutput(double op){
  output = op;
}
double VisionSubsystem::getOutput(){
  return output;
}

void VisionSubsystem::setDistanceError(double dist_error){
  distError = dist_error;
}

double VisionSubsystem::getDistanceError(){
  return distError;
}

void VisionSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}