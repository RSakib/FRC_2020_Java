/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;




/**
 * Add your docs here.
 */
public class Constants {

    //position of elevator
    public static double kPosition = 0;

    //encoder ticks per inch on gear ratio
    public static final double kEncoderTicksPerInch = 574;

    //Measurements for positions on elevator
    public static final double CollectCargo = 1.0;
    public static final double RocketBottomHeightCargo = 3.3;
    public static final double RocketMidHeightCargo = 31.5;
    public static final double RocketTopHeightCargo = 57.5;
    public static final double ShipHeightCargo = 19.5;
    
    public static final double CollectHatch = 4.3;
    public static final double ShipHeightHatch = 3.3;
    public static final double RocketBottomHeightHatch = 4.3;
    public static final double RocketMidHeightHatch = 32.5;
    public static final double RocketTopHeightHatch = 58.5;
    
    //if it is loaded with ball
    public static final int CARGO_STATE_LOADED = 0;
    public static final int CARGO_STATE_UNLOADED = 1;
    public static int CARGO_STATE = CARGO_STATE_UNLOADED;

    //if it tray is extended or retreacted
    public static final int TRAY_STATE_EXTENDED = 0;
    public static final int TRAY_STATE_RETRACTED = 1;
    public static int TRAY_STATE = TRAY_STATE_RETRACTED;

    //if talons are hold disk
    public static final int TALON_STATE_HOLDING = 0;
    public static final int TALON_STATE_RELEASED = 1;
    public static int TALON_STATE = TALON_STATE_HOLDING;

    //pnuematics on driveTrain
    public static final int HIGH_GEAR = 0;
    public static final int LOW_GEAR = 1;
    public static int CURRENT_GEAR = HIGH_GEAR;

    //For command purposes to see if hatch is wanted
    public static boolean WantHatch = false;

    //Elevator PID settings
    public static final int ElevatorSlotIDx = 0;
    public static final double Elevator_kP = 0.40;
    public static final double Elevator_kI = 0.0;
    public static final double Elevator_kD = 30.0;
    public static final int Elevator_MotionAccel = 13000;
    public static final int Elevator_MotionCruiseVelo = 15000;
    public static final int kTimeoutms = 10;

    //PID Drive Settings not finished yet
 /*    public static double Drive_kP = 0.5;
    public static double Drive_kI = 0.0;
    public static double Drive_kD = 0.0;
    public static double Drive_kF = 0.0;
    public static int Drive_kIzone = 0;
    public static int kDriveCruiseVelo = 30000;
    public static int kDriveAccel = 25000;
    public static double kToleranceDistance = 1000;
    public static double TurnOutputMin = 0.2;
    public static double LeftDistanceTarget, RightDistanceTarget, TurnOutput; */
    public static int Drive_kIzone = 0;

    //Varibles for Turning on Drivetrain
    public static double kTurnrateCurve = 0.1;
    public static double kTurnrateLimit = 0.8;

    //NAVX PID settings for  turning to angle
    public static double kToleranceDegrees = 5.0;
    public static double Turn_kP = 0.1;
    public static double Turn_kI = 0.0;
    public static double Turn_kD = 0.35;
    public static double Turn_kF = 0.0;
    public static double DesiredDistance;
    public static double DesiredHeading;
    //(4096 / (10 * (20/64) * (12/36) * (Constants.kDriveWheelDiameterInches*Math.PI))); // 200.0263
    // roughly 20k units/100ms at ~103 ips

    //Kinematics Varibales
    public static double kTrackLengthInches = 8.265;
    public static double kTrackWidthInches = 23.8;
    public static double kTrackEffectiveDiameter = (kTrackWidthInches * kTrackWidthInches
            + kTrackLengthInches * kTrackLengthInches) / kTrackWidthInches;
    public static double kTrackScrubFactor = 0.5;
    public static double kDriveWheelDiameterInches = 6.25;
    public static double kRatioFactor = 43;

    // PID gains for drive velocity loop
    // Units: error is 4096 counts/rev. Max output is +/- 1023 units.
    public static double kDriveVelocityKp = 0.1; //1.0
    public static double kDriveVelocityKi = 0.0;
    public static double kDriveVelocityKd = 1.0; //6.0
    public static double kDriveVelocityKf = 0.5;
    public static int kDriveVelocityIZone = 0;
    public static double kDriveVelocityRampRate = 0.0;
    public static int kDriveVelocityAllowableError = 0;

        // PID gains for drive base lock loop
    // Units: error is 4096 counts/rev. Max output is +/- 1023 units.
    public static double kDriveBaseLockKp = 0.5;
    public static double kDriveBaseLockKi = 0;
    public static double kDriveBaseLockKd = 0;
    public static double kDriveBaseLockKf = 0;
    public static int kDriveBaseLockIZone = 0;
    public static double kDriveBaseLockRampRate = 0;
    public static int kDriveBaseLockAllowableError = 10;

    // PID gains for constant heading velocity control
    // Units: Error is degrees. Output is inches/second difference to
    // left/right.
    public static double kDriveHeadingVelocityKp = 4.0; // 6.0;
    public static double kDriveHeadingVelocityKi = 0.0;
    public static double kDriveHeadingVelocityKd = 50.0;

    public static double kPathFollowingLookahead = 72.0; // inches
    public static double kPathFollowingMaxVel = 120.0; // inches/sec
    public static double kPathFollowingMaxAccel = 100.0; // inches/sec^2
    
    public static double kLooperDt = 0.01;

    //yes just yes (easier this way)
    public static final boolean On = true;
    public static final boolean Off = false;

}
