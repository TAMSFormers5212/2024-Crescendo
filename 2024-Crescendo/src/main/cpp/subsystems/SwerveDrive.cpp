#include "subsystems/SwerveDrive.h"
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

using namespace SwerveModuleConstants;
using namespace MathConstants;
using namespace units;

SwerveDrive::SwerveDrive()
    : m_modules{{SwerveModule(topleft::driveMotor, topleft::steerMotor, topleft::absencoder, topleft::offset),
                 SwerveModule(topright::driveMotor, topright::steerMotor, topright::absencoder, topright::offset),
                 SwerveModule(bottomleft::driveMotor, bottomleft::steerMotor, bottomleft::absencoder, bottomleft::offset),
                 SwerveModule(bottomright::driveMotor, bottomright::steerMotor, bottomright::absencoder, bottomright::offset)}},
      m_driveKinematics{{frc::Translation2d{drivebase::WheelBase / 2, -drivebase::TrackWidth / 2},
                         frc::Translation2d{-drivebase::WheelBase / 2, -drivebase::TrackWidth / 2},
                         frc::Translation2d{drivebase::WheelBase / 2, drivebase::TrackWidth / 2},
                         frc::Translation2d{-drivebase::WheelBase / 2, drivebase::TrackWidth / 2}}},
      m_odometry{m_driveKinematics,
                 frc::Rotation2d(getGyroHeading()),
                 {m_modules[0].getPosition(), m_modules[1].getPosition(), m_modules[2].getPosition(), m_modules[3].getPosition()},
                 frc::Pose2d()},
      m_poseEstimator{m_driveKinematics,
                      frc::Rotation2d(getGyroHeading()),
                      {m_modules[0].getPosition(), m_modules[1].getPosition(), m_modules[2].getPosition(), m_modules[3].getPosition()},
                      frc::Pose2d()},
      thetaController(0, 0, 0)
{
  m_gyro.Calibrate();
  // m_gyro.ZeroYaw();
  heading = frc::Rotation2d(degree_t{-m_gyro.GetYaw()});
  lastAngle = -m_gyro.GetYaw();
  // resetOdometry(m_poseEstimator.GetEstimatedPosition());
  std::cout << "Swerve subsystem initalized correctly" << std::endl;
}

frc::Pose2d SwerveDrive::AveragePose(){
  return m_poseEstimator.GetEstimatedPosition();
}

frc::Pose2d SwerveDrive::OdometryPose(){
  return m_odometry.GetPose();
} // odometry pose

frc::Rotation2d SwerveDrive::getGyroHeading(){ // i have no f*cking clue how this works
  double newAngle = -m_gyro.GetYaw();
  double delta = std::fmod(std::fmod((newAngle - lastAngle + 180), 360) + 360, 360) - 180; // NOLINT
  lastAngle = newAngle;
  heading = heading + frc::Rotation2d(degree_t{delta * 1.02466666667});
  return heading;
}

void SwerveDrive::resetHeading(){
  m_gyro.Reset();
}

void SwerveDrive::resetOdometry(const frc::Pose2d pose){
  m_odometry.ResetPosition(getGyroHeading(), {m_modules[0].getPosition(), m_modules[1].getPosition(), m_modules[2].getPosition(), m_modules[3].getPosition()}, pose);
  m_poseEstimator.ResetPosition(getGyroHeading(), {m_modules[0].getPosition(), m_modules[1].getPosition(), m_modules[2].getPosition(), m_modules[3].getPosition()}, pose);
}

void SwerveDrive::swerveDrive(double x, double y, double theta, bool fieldCentric){
  frc::ChassisSpeeds speeds = fieldCentric ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                                                 x * SwerveModuleConstants::maxSpeed,
                                                 y * SwerveModuleConstants::maxSpeed,
                                                 theta * SwerveModuleConstants::maxRotation,
                                                 units::degree_t{-m_gyro.GetYaw()})
                                           : frc::ChassisSpeeds{
                                                 x * SwerveModuleConstants::maxSpeed,
                                                 y * SwerveModuleConstants::maxSpeed,
                                                 theta * SwerveModuleConstants::maxRotation};
  speeds = speeds.Discretize(speeds.vx, speeds.vy, speeds.omega, units::second_t(0.02)); // second order kinematics?!?! nani

  auto saturatedStates = m_driveKinematics.ToSwerveModuleStates(speeds);
  
  // // maybe desaturate wheel speeds here
  m_driveKinematics.DesaturateWheelSpeeds(&saturatedStates, maxSpeed);
  // 1. figure out the max speed modules can go
  // 2. figure out the max speed the modules are actually going 

  auto states = m_driveKinematics.ToSwerveModuleStates(speeds);


  for (size_t i = 0; i < states.size(); ++i)
  {
    m_modules[i].setState(states[i]);
  }
}

void SwerveDrive::brake(){
  swerveDrive(0, 0, 0.05, false);
}

void SwerveDrive::moveToAngle(double x, double y){ // basically crab drive
  double temp = x;
  x = -y;
  y = temp;
  double r = sqrt(pow(x, 2) + pow(y, 2));
  double angle = 0.0;
  if (x == 0 && y == 0)
  {
    r = 0;
    angle = 0;
  }
  else
  {
    if (x > 0 && y >= 0)
    {
      angle = atan(y / x) + pi / 2;
    }
    else if (x <= 0 && y > 0)
    {
      angle = atan(-x / y) + pi;
    }
    else if (x < 0 && y <= 0)
    {
      angle = atan(-y / -x) + 3 * pi / 2;
    }
    else if (x >= 0 && y < 0)
    {
      angle = atan(-x / y);
    }
  }
  // frc::SmartDashboard::PutNumber("Magnitude", r);
  frc::SmartDashboard::PutNumber("angle", angle);
  // frc::SmartDashboard::PutNumber("x", x);
  // frc::SmartDashboard::PutNumber("y", y);
  for (auto &module : m_modules)
  {
    module.setState(frc::SwerveModuleState{meters_per_second_t(r), frc::Rotation2d(radian_t(angle))});
  }
    // frc::SmartDashboard::PutNumber("x", x);
    // frc::SmartDashboard::PutNumber("y", y);
    // frc::SmartDashboard::PutNumber("")
}

void SwerveDrive::tankDrive(double x, double y){
  // implement differential drive later
  m_modules[0].setState(frc::SwerveModuleState{meters_per_second_t(x + y), frc::Rotation2d(radian_t(0))}); // tl
  m_modules[1].setState(frc::SwerveModuleState{meters_per_second_t(x - y), frc::Rotation2d(radian_t(0))}); // tr
  m_modules[2].setState(frc::SwerveModuleState{meters_per_second_t(x + y), frc::Rotation2d(radian_t(0))}); // bl
  m_modules[3].setState(frc::SwerveModuleState{meters_per_second_t(x - y), frc::Rotation2d(radian_t(0))}); // br
}

void SwerveDrive::Periodic(){
  m_odometry.Update(getGyroHeading(), {m_modules[0].getPosition(), m_modules[1].getPosition(),
                                       m_modules[2].getPosition(), m_modules[3].getPosition()});
  m_poseEstimator.Update(getGyroHeading(), {m_modules[0].getPosition(), m_modules[1].getPosition(),
                                            m_modules[2].getPosition(), m_modules[3].getPosition()});



}

void SwerveDrive::resetAbsoluteEncoders(){
  for (auto &module : m_modules)
  {
    module.resetDriveEncoder();
    module.resetSteerEncoder();
  }
}

void SwerveDrive::SyncAbsoluteEncoders(){
  for (auto &module : m_modules)
  {
    module.resetSteerEncoder();
  }
}

bool SwerveDrive::getOffsetToggle(){
  return offsetToggle;
}

void SwerveDrive::toggleOffset(){
  if(offsetToggle){
    offsetToggle = false;
    for(auto &module : m_modules){
      module.togglePositionOffset(offsetToggle);
    }
  }else{
    offsetToggle = true;
    for(auto &module : m_modules){
      module.togglePositionOffset(offsetToggle);
    }
  }
}
