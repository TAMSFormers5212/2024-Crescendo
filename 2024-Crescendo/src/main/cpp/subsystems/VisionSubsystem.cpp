#include "subsystems/VisionSubsystem.h"

#include <span>

#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableValue.h"

using namespace VisionConstants;
using namespace MathConstants;

VisionSubsystem::VisionSubsystem() : output(), distError(), pid(kvP, kvI, kvD) {
}

frc2::CommandPtr VisionSubsystem::VisionMethodCommand() {
    return RunOnce([/* this */] { /* one-time action goes here */ });
}

bool VisionSubsystem::VisionCondition() {
    return false;
}

void VisionSubsystem::Periodic() {
    table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    table2 = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray("tid", std::vector<double>(6));
    // frc::PIDController pid(kvP, kvI, kvD); dont need to recreate pid every
    // periodic cycle
    double targetOffsetAngle_Vertical = table->GetNumber("ty", 0.0);
    double id = table2.at(0);
    double goalHeightInches = 12.0;

    if (id == 4.0 || id == 7.0){
      goalHeightInches = VisionConstants::speakerTagHeight;
    }
    else if (id == 10.0 || id == 9.0 || id == 1.0 || id == 2.0){
      goalHeightInches = VisionConstants::sourceTagHeight;
    }
    else if (id == 5.0 || id == 6.0){
      goalHeightInches = VisionConstants::ampTagHeight;
    }
    else if (id == 11.0 ||id == 12.0 ||id == 13.0 ||id == 14.0 ||id == 15.0 ||id == 16.0){
      goalHeightInches = VisionConstants::stageTagHeight;
    }
    
    
    double angleToGoalDegrees = VisionConstants::limelightAngleAboveHorizontal + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (pi / 180.0);

    // calculate distance
    double distanceFromLimelightToGoalInches = (VisionConstants::speakerTagHeight - VisionConstants::limelightHeight.value()) / tan(angleToGoalRadians);
    float KpDistance = -0.1f;  // Proportional control constant for distance 
    float desired_distance = 64;
    float distance_error = abs(desired_distance - distanceFromLimelightToGoalInches) * KpDistance;
    setDistanceError(distance_error);

    frc::SmartDashboard::PutNumber("up angle", targetOffsetAngle_Vertical);
    frc::SmartDashboard::PutNumber("distance", distanceFromLimelightToGoalInches);
    frc::SmartDashboard::PutNumber("id", id);
    double targetOffsetAngle_Horizontal = table->GetNumber("tx", 0.0);
    double heading_error = targetOffsetAngle_Horizontal;
    // pid.SetSetpoint(0);
    frc::SmartDashboard::PutNumber("heading", heading_error);
    double output = pid.Calculate(heading_error, 0);
    frc::SmartDashboard::PutNumber("pid", output);
    setOutput(output);

    // pid.Calculate();
}

void VisionSubsystem::setOutput(double op) { output = op; }

double VisionSubsystem::getOutput() { return output; }

void VisionSubsystem::setDistanceError(double dist_error) {
    distError = dist_error;
}

double VisionSubsystem::getDistanceError() { return distError; }

void VisionSubsystem::SimulationPeriodic() {
    // Implementation of subsystem simulation periodic method goes here.
}