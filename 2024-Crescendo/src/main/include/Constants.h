#pragma once

#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation2d.h>

#include <units/length.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/acceleration.h>
#include <units/math.h>
#include <units/velocity.h>
#include <units/length.h>
#include <units/time.h>
// #include <units/base.h>
#include <numbers>
#include <string>
#include <vector>
#include <array>

namespace PortConstants {


    //LEDS?
    //vision or other accessories

    constexpr int kMXP = 2; // mxp port

    //digital io 


    //analog in - the only ones we're using rn are the swerve encoders and those are in SwerveConstants

} 

namespace MathConstants {
    constexpr double pi = 3.1415926535;
    constexpr double pi2 = 6.283185307;

    constexpr double gMeters = 9.81;
    constexpr double gFeet = 32.1741;
}


namespace OIConstants {//Controller buttons 

    constexpr int kDriverControllerPort = 0;
    constexpr int kOperatorControllerPort = 1;

// need to check these constants, previous code used xbox controller, now its hid controller
    //flight joystick controller
    namespace Joystick{
        //Axis
        constexpr int XAxis = 0; 
        constexpr int YAxis = 1; 
        constexpr int RotAxis = 2; 
        constexpr int ThrottleSlider = 3;
        constexpr int miniXAxis = 5; // mini axis only returns -1, 0, or 1
        constexpr int miniYAxis = 6;
        //Buttons
        constexpr int Trigger = 1; 
        constexpr int ButtonTwo = 2; // side thumb button
        constexpr int ButtonThree = 3; // 3-6 on top
        constexpr int ButtonFour = 4;
        constexpr int ButtonFive = 5;
        constexpr int ButtonSix = 6;
        constexpr int ButtonSeven = 7; // 7-12 on bottom
        constexpr int ButtonEight = 8;
        constexpr int ButtonNine = 9;
        constexpr int ButtonTen = 10;
        constexpr int ButtonEleven = 11;
        constexpr int ButtonTwelve = 12;

        constexpr double deadband = 0.15; 
    }

    namespace Controller{//console controller
        //axis
        constexpr int leftXAxis = 0;
        constexpr int leftYAxis = 1;
        constexpr int rightXAxis = 4;
        constexpr int rightYAxis = 5;
        constexpr int leftTrigger = 2;
        constexpr int rightTrigger = 3;
        //buttons
        constexpr int A = 1;
        constexpr int B = 2;
        constexpr int X = 3;
        constexpr int Y = 4;
        constexpr int leftBumper = 5;
        constexpr int rightBumper = 6;
        constexpr int View = 7;
        constexpr int Menu = 8;
        constexpr int LPress = 9;
        constexpr int RPress = 10;
    }



}

namespace SwerveModuleConstants {//per swerve module
    constexpr double ktP = 0.2; // Turning PID
    constexpr double ktI = 0.0;
    constexpr double ktD = 0.0;
    constexpr double ktFF = 0.00001;
    
    constexpr double kdP = 0.6; // Driving Speed PID
    constexpr double kdI = 0.0;
    constexpr double kdD = 0.0;
    constexpr double kdFF = 0.00;

    constexpr auto maxSpeed = 3.81_mps; // max free speed of SDS mk4 L1
    constexpr auto maxRotation = 2.0_rad_per_s; // figure this out later
    constexpr double driveRatio = 8.14; //SDS Mk4 L1
    constexpr double steerRatio = 12.8; //SDS Mk4 L1
    constexpr units::inch_t wheelDiameter = 4_in; // likely different based on tread wear
    constexpr units::inch_t wheelCircumfrence = units::inch_t{wheelDiameter.value()*MathConstants::pi}; // recalculate based on wheel diameter 
    constexpr units::inch_t centerDistance = 10.5_in;

    constexpr double khP = 0.0; // Heading PID
    constexpr double khI = 0.0;
    constexpr double khD = 0.0;
    constexpr double khFF = 0.00000;

    constexpr double kxP = 0.0; // X PID
    constexpr double kxI = 0.0;
    constexpr double kxD = 0.0;
    constexpr double kxFF = 0.00000;

    constexpr double kyP = 0.0; // Y PID
    constexpr double kyI = 0.0;
    constexpr double kyD = 0.0;
    constexpr double kyFF = 0.00000;

    //possible heaving, x, and y PID for auto/pathplanning

    constexpr double kRampTimeSeconds = 0.1; // slew rate limiter (delay on acceleration)

    namespace drivebase{
        constexpr units::meter_t WheelBase = 0.5207_m; // for kinematics
        constexpr units::meter_t TrackWidth = 0.5207_m; // aka 20.5 inches
        // 27x27 inch chassis from 2023, will update once 2024 chassis is assembed, but its just +1 inch 
    }
    
    
    namespace topleft{
        constexpr int driveMotor = 9; //CAN ID
        constexpr int steerMotor = 2; //CAN ID
        constexpr int absencoder = 3; //analogin Port

        constexpr double offset = 0.508-0.25;

    }
    namespace topright{
        constexpr int driveMotor = 5; //CAN ID
        constexpr int steerMotor = 6; //CAN ID
        constexpr int absencoder = 0; //analogin Port

        constexpr double offset = 0.170-0.25+1;
    }
    namespace bottomleft{
        constexpr int driveMotor = 4; //CAN ID
        constexpr int steerMotor = 3; //CAN ID
        constexpr int absencoder = 2; //analogin Port

        constexpr double offset = 0.322-0.25;
    }
    namespace bottomright{          
        constexpr int driveMotor = 7; //CAN ID
        constexpr int steerMotor = 8; //CAN ID
        constexpr int absencoder = 1; //analogin Port

        constexpr double offset = 0.834+0.25-1;
    }

}

namespace ArmConstants{

    constexpr int leftMotor = 10;
    constexpr int rightMotor = 11;

    constexpr int encoder = 2; // depends on what encoder. planning for a rev through bore 
    // constexpr int limitSwitch = 1;

    constexpr double encoderOffset = 0;

    constexpr double sprocketRatio = 5.5;
    constexpr int planetaryRatio = 20;
    constexpr int armRatio = 110; // 20:1 maxplanetary * 66:12 sprocket
    
    constexpr double kaP = 0.0;
    constexpr double kaI = 0.0;
    constexpr double kaD = 0.0;
    constexpr double kaFF = 0.0;
    constexpr double kaIz = 0.0;
    constexpr units::second_t kaT = 20_ms;

    constexpr int kMaxOutput = 1;
    constexpr int kMinOutput = -1;
    constexpr units::meters_per_second_squared_t maxAccel = 10.0_mps_sq;
    constexpr units::meters_per_second_t maxVelo = 20.0_mps;
    constexpr double allowedError = 0.0;

    constexpr double kaS = 0.0;
    constexpr double kaG = 0.0;
    constexpr double kaV = 0.0;
    constexpr double kaA = 0.0;
}

namespace ShooterConstants{
    constexpr int topMotor = 12;
    constexpr int bottomMotor = 13;
    constexpr double pulleyRatio = 30.0/18.0;
    constexpr units::inch_t wheelDiameter = 4_in; // may change based on rpm

    constexpr double ksP = 0.0; // shooter uses a velocity PID
    constexpr double ksI = 0.0;
    constexpr double ksD = 0.0;
    constexpr double ksFF = 0.0;

    constexpr units::revolutions_per_minute_t maxNeoRpm = 5700_rpm;
    constexpr units::revolutions_per_minute_t maxWheelRpm = 9000_rpm;//replace this value with whatever max andymark says
    constexpr units::feet_per_second_t maxExitVelocity = 90_fps;

}

namespace IntakeConstants{
    constexpr int motor = 1;

    constexpr int beamBreakIO = 0;  // digital io pins

    constexpr double intakeRatio = 2.0;

    constexpr double loadedCurrent = 10.0; // current when note is held
    constexpr double freeCurrent = 5.0; // current when no note is held
    //random guess numbers, experiement in order to prevent burning out the neo550
    
    constexpr int empty = 0;
    constexpr int held = 1;
    constexpr int indexed = 2;
}

namespace WinchConstants{
    constexpr int leftWinchMotor = 13;
    constexpr int rightWinchMotor = 14;


    constexpr double winchRatio = 60;
    constexpr units::inch_t winchDiameter = 0.5_in;
    constexpr units::inch_t heightToTravel = 27_in; // distance from chain to winch, aka max winchable distance

    constexpr double kwP = 0.0;
    constexpr double kwI = 0.0;
    constexpr double kwD = 0.0;
    constexpr double kwFF = 0.0;
    constexpr double kwIz = 0.0;
}

namespace PoseConstants{
    //normal setpoints

    //Amp
    constexpr double armAmp = 0;

    //Climb
    constexpr double armClimb = 0;
    constexpr double intakeTrapSpeed = 0;

    //lookup table design
    //proven setpoints and just interpolate inbetween those values

    //distance: 3ft
    constexpr units::inch_t distance3 = 36_in;
    constexpr double shooterRPM3 = 0;
    constexpr double armAngle3 = 0;
    
    //distance: 5ft
    constexpr units::inch_t distance5 = 60_in;
    constexpr double shooterRPM5 = 0;
    constexpr double armAngle5 = 0;

    //distance: 7ft
    constexpr units::inch_t distance7 = 84_in;
    constexpr double shooterRPM7 = 0;
    constexpr double armAngle7 = 0;

    // distance: 9ft
    constexpr units::inch_t distance9 = 108_in;
    constexpr double shooterRPM9 = 0;
    constexpr double armAngle9 = 0;

    // distance: 11ft
    constexpr units::inch_t distance11 = 132_in;
    constexpr double shooterRPM11 = 0;
    constexpr double armAngle11 = 0;

    // distance: 13ft
    constexpr units::inch_t distance13 = 156_in;
    constexpr double shooterRPM13 = 0;
    constexpr double armAngle13 = 0;
    //max distance of vision for now
}

namespace VisionConstants{

    //vision pid constants for aiming
    constexpr double kvP = 0.05;
    constexpr double kvI = 0.01;
    constexpr double kvD = 0.0001;
    constexpr double kvFF = 0.0;

    //vision pid constants for translating

    constexpr units::inch_t limelightHeight = 7_in;//14 on cart, 7 off

    constexpr double limelightAngleback = 60;//degrees tilted back
    constexpr double limelightAngleAboveHorizontal= 39.0;
    
    constexpr double TagHeight = 8;// height of the tags
    constexpr double TagCenterHeight = 4; // center of the tag from the middle

    // tags are measured from the carpet to the bottom of the tag
    constexpr double speakerTagHeight = 4*12+3+7.0/8; // height in inches
    constexpr double sourceTagHeight = 4*12+1.0/8;
    constexpr double ampTagHeight = 4 * 12 + 1.0 / 8;
    constexpr double stageTagHeight = 4*12-0.5;

    // 0. speaker center, 1. speaker side, 2. amp, 3. stage amp, 4. stage source, 5. stage center, 6. source close, 7. source far
    // constexpr std::array<int, 8> redTags = {4, 3, 5, 12, 11, 13, 10, 9};
    // constexpr std::array<int, 8> blueTags = {7, 8, 6, 15, 16, 14, 1, 2};
    constexpr int redSpeakerCenter = 4;
    constexpr int redSpeakerSide = 3;
    constexpr int redAmp = 5;
    constexpr int redStageAmp = 12;
    constexpr int redStageSource = 11;
    constexpr int redStageCenter = 13;
    constexpr int redSourceClose = 10;
    constexpr int redSourceFar = 9;
    constexpr int blueSpeakerCenter = 7;
    constexpr int blueSpeakerSide = 8;
    constexpr int blueAmp = 6;
    constexpr int blueStageAmp = 15;
    constexpr int blueStageSource = 16;
    constexpr int blueStageCenter = 14;
    constexpr int blueSourceClose = 1;
    constexpr int blueSourceFar = 2;
} 