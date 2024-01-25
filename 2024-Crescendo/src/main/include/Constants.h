#pragma once

#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation2d.h>

#include <units/length.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/math.h>
#include <units/velocity.h>
#include <string>
#include <vector>
#include <array>

namespace PortConstants {


//LEDS?
//vision or other accessories

constexpr int kMXP = 2;


} 

namespace MathConstants {
    constexpr double pi = 3.1415926535;
    constexpr double pi2 = 6.283185307;
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
        //Buttons
        constexpr int Trigger = 1; 
        constexpr int ButtonThree = 3;
        constexpr int ButtonFour = 4;

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

    constexpr auto maxSpeed = 4.0_mps; // arbitrary values for now
    constexpr auto maxRotation = 2.0_rad_per_s;
    constexpr double driveRatio = 8.14; //SDS Mk4 L1
    constexpr double steerRatio = 12.8; //SDS Mk4 L1
    constexpr units::inch_t wheelDiameter = 4_in;
    constexpr units::inch_t wheelCircumfrence = 12.57_in;
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

    constexpr float kvP = -0.3f;
    //possible heaving, x, and y PID for auto/pathplanning

    constexpr double kRampTimeSeconds = 0.1; // slew rate limiter (delay on acceleration)

    namespace drivebase{
        constexpr units::meter_t WheelBase = 0.5207_m; // for kinematics
        constexpr units::meter_t TrackWidth = 0.5207_m; 
        //tony 1/7 : wait isn't the frame a square? wheelbase and trackwidth should be the same wtf!
    }
    
    
    namespace topleft{
        constexpr int driveMotor = 9; //CAN ID
        constexpr int steerMotor = 2; //CAN ID
        constexpr int absencoder = 3; //analogin Port

        constexpr double offset = 0.508-0.25;//1-0.258;//-0.235;
//2.2009 

    }
    namespace topright{
        constexpr int driveMotor = 5; //CAN ID
        constexpr int steerMotor = 6; //CAN ID
        constexpr int absencoder = 0; //analogin Port

        constexpr double offset = 0.170-0.25+1;//-0.146;
    }
    namespace bottomleft{
        constexpr int driveMotor = 4; //CAN ID
        constexpr int steerMotor = 3; //CAN ID
        constexpr int absencoder = 2; //analogin Port

        constexpr double offset = 0.322-0.25;//1-0.744;//0.810;//(0.38+M_PI)/(2*M_PI);//0.6101363-0.58127+0.1;// (343.800190846-229.081278318-15)/360;
        //7.48
    }
    namespace bottomright{          
        constexpr int driveMotor = 7; //CAN ID
        constexpr int steerMotor = 8; //CAN ID
        constexpr int absencoder = 1; //analogin Port

        constexpr double offset = 0.834+0.25-1;//1.247;
    }

}

namespace ArmConstants{

    constexpr int leftMotor = 10;
    constexpr int rightMotor = 11;

    constexpr int encoder = 2; // depends on what encoder 
    constexpr int limitSwitch = 1;
    
    constexpr double kaP = 0.0;
    constexpr double kaI = 0.0;
    constexpr double kaD = 0.0;
    constexpr double kaFF = 0.0;

    constexpr int armRatio = 100; //idk just a filler
    constexpr int maxAccel = 1000;
    constexpr int maxVelo = 5000;
    constexpr int minVelo = 5000;
}

namespace PoseConstants{
    namespace stow{
        constexpr int intakeRPM = 0;
        constexpr int shooterRPM = 0;
        constexpr double armPose = 0;
        constexpr double elevatorPose = 0;
    }

    namespace groundIntake{
        constexpr int intakeRPM = 1000;
        constexpr int shooterRPM = 0;
        constexpr double armPose = 0; 
        constexpr double elevatorPose = 0;
    }

    namespace amp{
        constexpr int intakeRPM = 0;
        constexpr int shooterRPM = 100;
        constexpr double armPose = 0;
        constexpr double elevatorPose = 0;
    }

    namespace trap{
        constexpr int intakeRPM = 0;
        constexpr int shooterRPM = 0;
        constexpr double armPose = 0;
        constexpr double elevatorPose = 0;
    }
}