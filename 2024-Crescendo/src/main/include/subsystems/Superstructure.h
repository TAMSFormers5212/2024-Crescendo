// #pragma once

// #include <frc2/command/CommandPtr.h>
// #include <frc2/command/SubsystemBase.h>

// #include <frc/controller/PIDController.h>
// #include <frc/controller/ProfiledPIDController.h>

// #include <frc/AnalogEncoder.h>

// #include <rev/CANSparkMax.h>
// #include <rev/SparkRelativeEncoder.h>
// #include <rev/SparkPIDController.h>
// // #include <rev/ThroughBoreEncoder.h>

// // clean up include list once subclasses are finished

// #include <Constants.h>
// #include "Arm.h"
// #include "Intake.h"
// #include "Shooter.h"
// #include "Winch.h"

// using namespace std;

// class Superstructure : public frc2::SubsystemBase{

// public:

//     Superstructure(Arm arm, Intake intake, Shooter shooter); // arm, intake, shooter, winch

//     void resetSuperstructure();

//     //set positions and state control
//     void setToIntake();
//     void intakeNote();
//     void indexNote();
//     void aimShooter();
//     void speakerShot();
//     void ampShot();
//     void raiseToClimb();
//     void climb();

//     //utility control
//     void setIntake(double speed);
//     void setShooter(double speed);
//     void setArm(double position);
//     void setLeftWinchPosition(double position);
//     void setRightWinchPosition(double position);
//     void setLeftWinchSpeed(double speed);
//     void setRightWinchSpeed(double speed);

//     void Periodic() override;

// private: 

// //winch
// //2 motors total (1 motor per winch)
//     Winch leftWinch;
//     Winch rightWinch;
// //arm
// //2 motors
//     Arm arm;

// //intake
// //1 motor
//     Intake intake;

// //shooter
// //2 motors
//     Shooter shooter;

// //2+2+1+2+8 = 15
// // 1 for powerering coprocessor?

//     // std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

// };

