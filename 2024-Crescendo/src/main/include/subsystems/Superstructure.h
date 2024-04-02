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
    void autonAim(double distance);
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

    //infomation about superstructure status
    double getArmPosition();
    double getLeftWinchPosition();
    double getRightWinchPosition();
    double getleftShooterSpeed();
    double getrightShooterSpeed();
    double getShooterSpeed();

    void Periodic() override;

    VisionSubsystem m_vision;    
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

   private:
    // winch
    // 2 motors total (1 motor per winch)



    // std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    //distances are in inches
    vector<double> distances = {24, 36, 41, 48, 60, 72, 84, 96, 111};
    vector<double> angles = { 0.255,0.278, 0.291, 0.328, 0.426, 0.493, 0.523, 0.582, 0.607};
    vector<double> speeds = { 500,540, 580, 600, 600, 600, 600, 600, 600};
};

