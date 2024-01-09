#include "subsystems/SwerveDrive.h"
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

using namespace SwerveModuleConstants;
using namespace MathConstants;
using namespace units;

SwerveDrive::SwerveDrive()
   :m_modules{{ SwerveModule(topleft::driveMotor, topleft::steerMotor, topleft::absencoder, topleft::offset),
                SwerveModule(topright::driveMotor, topright::steerMotor, topright::absencoder, topright::offset),
                SwerveModule(bottomleft::driveMotor, bottomleft::steerMotor, bottomleft::absencoder, bottomleft::offset),
                SwerveModule(bottomright::driveMotor, bottomright::steerMotor, bottomright::absencoder, bottomright::offset)
                }},
    m_driveKinematics{{frc::Translation2d{drivebase::WheelBase/2, drivebase::TrackWidth/2},
                       frc::Translation2d{drivebase::WheelBase/2, -drivebase::TrackWidth/2},
                       frc::Translation2d{-drivebase::WheelBase/2, drivebase::TrackWidth/2},
                       frc::Translation2d{-drivebase::WheelBase/2,-drivebase::TrackWidth/2}
    }},
    m_odometry{m_driveKinematics, 
               frc::Rotation2d(getGyroHeading()), 
               {m_modules[0].getPosition(), m_modules[1].getPosition(), m_modules[2].getPosition(), m_modules[3].getPosition()},
               frc::Pose2d()
    },
    m_poseEstimator{m_driveKinematics,
                    frc::Rotation2d(getGyroHeading()),
                    {m_modules[0].getPosition(), m_modules[1].getPosition(), m_modules[2].getPosition(), m_modules[3].getPosition()},
                    frc::Pose2d()
    },
    thetaController(0, 0, 0)
    {
    m_gyro.Calibrate();
    //m_gyro.ZeroYaw();
    heading = frc::Rotation2d(degree_t{-m_gyro.GetYaw()});
    lastAngle = -m_gyro.GetYaw();
    // resetOdometry(m_poseEstimator.GetEstimatedPosition());
    std::cout<<"Swerve subsystem initalized correctly"<<std::endl;
  }

frc::Pose2d SwerveDrive::AveragePose(){
    return m_poseEstimator.GetEstimatedPosition();
}

frc::Pose2d SwerveDrive::OdometryPose(){
    return m_odometry.GetPose();
} //odometry pose

frc::Rotation2d SwerveDrive::getGyroHeading(){//i have no f*cking clue how this works
    double newAngle = -m_gyro.GetYaw();
    double delta = std::fmod(std::fmod((newAngle - lastAngle + 180), 360) + 360, 360) -
      180;  // NOLINT
    lastAngle = newAngle;
    heading = heading + frc::Rotation2d(degree_t{delta * 1.02466666667});
    return heading;
}

void SwerveDrive::resetHeading(){
    m_gyro.Reset();
}

void SwerveDrive::resetOdometry(const frc::Pose2d pose){
    m_odometry.ResetPosition(getGyroHeading(), {m_modules[0].getPosition(), m_modules[1].getPosition(), 
                             m_modules[2].getPosition(), m_modules[3].getPosition()}, pose);
    m_poseEstimator.ResetPosition(getGyroHeading(), {m_modules[0].getPosition(), m_modules[1].getPosition(), 
                                  m_modules[2].getPosition(), m_modules[3].getPosition()}, pose);
}

void SwerveDrive::swerveDrive(double x, double y, double theta, bool fieldCentric){

}

void SwerveDrive::brake(){

}

void SwerveDrive::moveToAngle(double x, double y){

}

void SwerveDrive::tankDrive(double x, double y, double theta){

}

void SwerveDrive::Periodic(){

}

void SwerveDrive::resetAbsoluteEncoders(){
    for(auto& module : m_modules){
      module.resetDriveEncoder();
      module.resetSteerEncoder();
    }
}

void SwerveDrive::SyncAbsoluteEncoders(){
    for(auto& module : m_modules){
      module.resetSteerEncoder();
    }
  }
