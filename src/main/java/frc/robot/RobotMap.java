/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Author Fernando Matias (207)
 */


public class RobotMap {
    //DriveTrain
    //Left
    public static final int mDrive_Left_A_ID = 3;
    public static final int mDrive_Left_B_ID = 5;
    public static final int mDrive_Left_C_ID = 7;
    //Right
    public static final int mDrive_Right_A_ID = 4;
    public static final int mDrive_Right_B_ID = 6;
    public static final int mDrive_Right_C_ID = 8;
    //Shooter
    public static final int mShooter_ID = 10;
    //Elevator
    public static final int mElevator_Master_ID = 9;
    public static final int mElevator_Slave_ID = 11;

    //pnuematics
    //Tray
    public static final int mTrayExtended_ID = 1;
    public static final int mTrayRetract_ID = 1;
    //Shifters
    public static final int mShiftHigh_ID = 2;
    public static final int mShiftLow_ID = 2;
    //Talons
    public static final int mTalonHold_ID = 3;
    public static final int mtalonsRelease_ID = 3;

    //Sensors
    //Bump Switch
    public static final int mCargoLoadedSensor_ID = 9;
    //Limit Switch
    public static final int mHatchLoadedSensor_ID = 0;
    //Ultrasonic Sensor
    public static final int mUltrasonicSensorPing_ID = 5;
    public static final int mUltrasonicSensorEcho_ID = 6;

    //Controls 
    public static final int LeftJoystick_ID = 0;
    public static final int RightJoystick_ID = 1;
    public static final int Gampad_ID = 2;
    //PCM
    public static final int PCM_A = 2;
    public static final int PCM_B = 1;
    
}
