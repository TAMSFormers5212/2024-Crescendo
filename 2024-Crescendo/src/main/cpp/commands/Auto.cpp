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
#include <commands/ArmLower.h>
#include <commands/ReadyShooter.h>
#include <commands/AutoIntake.h>
#include <commands/StopShooter.h>
#include <iostream>
using namespace std;
using namespace PoseConstants;

Auto::Auto(SwerveDrive* drive, Arm* arm, Intake* intake, Shooter* shooter) {    
    AddCommands(
        frc2::SequentialCommandGroup{
              frc2::ParallelRaceGroup{
                
                frc2::WaitCommand(4_s),
                ArmLower(arm)
             
              },
              
              //ReadyShooter(shooter),
              frc2::ParallelRaceGroup{
                frc2::SequentialCommandGroup{
                    frc2::WaitCommand(2_s),
                    frc2::ParallelRaceGroup{
                        AutoIntake(intake),
                        frc2::WaitCommand(2_s)
                    }
                },
                frc2::WaitCommand(4_s),
                ReadyShooter(shooter)
             
              },
            //   frc2::ParallelRac
              StopShooter(shooter, intake),
              frc2::ParallelRaceGroup{
                Drive(drive, 0, 0.1, 0, true),
                frc2::WaitCommand(2_s)},
             
              frc2::WaitCommand(1_s), 
                

            
            
        }
    );
    std::cout << "Auton Finished" << endl;
}
Auto::Auto(SwerveDrive* drive) {
    AddCommands(
       frc2::ParallelRaceGroup{
       Drive(drive, 0, 0.1, 0, true),
       frc2::WaitCommand(2_s)}
        
    );
} 