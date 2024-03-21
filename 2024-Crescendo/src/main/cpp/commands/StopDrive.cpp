
#include "commands/stopDrive.h"

StopDrive::StopDrive(SwerveDrive* drive)
    : m_drive(drive) {
  AddRequirements(drive);
}

void StopDrive::Initialize() {
    
    m_drive->swerveDrive(0, 0, 0, false);
}

void StopDrive::End(bool interrupted) {}