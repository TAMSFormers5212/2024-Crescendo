#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/CommandPtr.h>

#include "commands/Auto.h"
// #include "commands/Drive.h"
// #include "commands/SetPose.h"
#include "Constants.h"
// #include "commands/Grab.h"

using namespace PoseConstants;

Auto::Auto(SwerveDrive* drive, Intake* wrist, Arm* arm, Shooter* grabber) {    
    AddCommands(
        frc2::ParallelRaceGroup{
            
            frc2::WaitCommand(2_s)
        }
    );
    
}