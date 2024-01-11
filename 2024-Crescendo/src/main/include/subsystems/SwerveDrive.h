#pragma once

#include <frc2/command/CommandPtr.h>

#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/controller/PIDController.h>

#include "SwerveModule.h"
#include <AHRS.h>

using namespace SwerveModuleConstants;
using namespace std;

class SwerveDrive : public frc2::SubsystemBase{

public:
    SwerveDrive();

    frc::Pose2d AveragePose();
    frc::Pose2d OdometryPose();
    frc::Rotation2d getGyroHeading();
    void resetHeading();
    void resetOdometry(const frc::Pose2d pose);
    void swerveDrive(double x, double y, double theta, bool fieldCentric);
    void brake();
    void tankDrive(double x, double y);
    void moveToAngle(double x, double y);
    void resetAbsoluteEncoders();
    void SyncAbsoluteEncoders();
    void Periodic() override; //update pose using gyro, vision, and odometry

private:

    array<SwerveModule, 4> m_modules;

    frc::SwerveDriveKinematics<4> m_driveKinematics;
    frc::SwerveDriveOdometry<4> m_odometry;
    frc::SwerveDrivePoseEstimator<4> m_poseEstimator;

    frc::PIDController thetaController; // closed loop control for heading
    // may be something we want to implement if we notice the drive slowly twisting as it drives
    
    AHRS m_gyro{frc::SPI::Port::kMXP};

    frc::Rotation2d heading;

    double lastAngle;
};