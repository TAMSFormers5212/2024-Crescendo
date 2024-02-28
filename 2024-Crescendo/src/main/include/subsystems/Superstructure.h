#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>

#include <frc/AnalogEncoder.h>

#include <rev/CANSparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkPIDController.h>
// #include <rev/ThroughBoreEncoder.h>

// clean up include list once subclasses are finished
#include <vector>
#include <Constants.h>
#include "Arm.h"
#include "Intake.h"
#include "Shooter.h"
#include "Winch.h"
#include "VisionSubsystem.h"

using namespace std;
using namespace PoseConstants;

class Superstructure : public frc2::SubsystemBase{

public:

    // Superstructure(Arm arm, Intake intake, Shooter shooter, Winch leftWinch, Winch rightWinch); // arm, intake, shooter, winch
    Superstructure();

    void resetSuperstructure();

    //set positions and state control
    void setToIntake();
    void intakeNote();
    void indexNote();
    double calculateSpeed(double distance, double x, double y);  // calculate the needed speed based on current speed
    double calculateAngle(double distance, double x, double y);  // angle to feed to arm
    void aim(double distance, double x, double y);
    void aim(double angle, double speed);
    void speakerShot();
    void ampShot();
    void raiseToClimb();
    void climb();
    void climb(double roll);

    //utility control
    void setIntake(double speed);
    void setShooter(double speed);
    void setArm(double position);
    void setLeftWinchPosition(double position);
    void setRightWinchPosition(double position);
    void setLeftWinchSpeed(double speed);
    void setRightWinchSpeed(double speed);

    void Periodic() override;

private: 

//winch
//2 motors total (1 motor per winch)
    Winch m_leftWinch;
    Winch m_rightWinch;
    PIDController rollPID;

//arm
//2 motors
    Arm m_arm;

//intake
//1 motor
    Intake m_intake;

//shooter
//2 motors
    Shooter m_shooter;

    VisionSubsystem m_limelight;

    // std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    vector<units::inch_t> distances = {12_in, 24_in, 36_in, 48_in, 60_in, 72_in, 84_in, 96_in, 108_in};
    vector<double> angles = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    vector<double> speeds = {0, 0, 0, 0, 0, 0, 0, 0, 0};
};

