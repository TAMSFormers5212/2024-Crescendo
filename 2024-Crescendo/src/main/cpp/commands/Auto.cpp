#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/WaitCommand.h>

#include <frc2/command/RunCommand.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>


#include "commands/Auto.h"
// #include "commands/Drive.h"
// #include "commands/SetPose.h"
#include "Constants.h"
#include <frc2/command/ParallelCommandGroup.h>
// #include "commands/Grab.h"
#include <commands/Drive.h>
#include <commands/Shoot.h>

using namespace PoseConstants;

Auto::Auto(SwerveDrive* drive, Intake* wrist, Arm* arm, Shooter* grabber) {    
    // AddCommands(
    //     frc2::ParallelCommandGroup{
    //     Shoot(wrist, grabber),
    //     frc2::WaitCommand(3_s)}
    // );
    AddCommands(
        frc2::ParallelCommandGroup{
        Drive(drive, 0, 0.5, 0, true),
        frc2::WaitCommand(2_s)}
    );
    
}