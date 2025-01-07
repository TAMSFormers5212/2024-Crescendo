
#include "commands/drive.h"

Drive::Drive(SwerveDrive* drive, double x, double y, double z, bool blue)
    : m_drive(drive), blueAlliance(blue) {
  AddRequirements(drive);
}

void Drive::Initialize() {
    if(blueAlliance){
        y*=-1;
    }
    m_drive->swerveDrive(x, y, z, false);
}

void Drive::End(bool interrupted) {}