/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.PID.PIDGains;

/**
 * Add your docs here.
 */
public class Constants {

    public static final int HIGH_GEAR = 0;
    public static final int LOW_GEAR = 1;
    public static int CURRENT_GEAR = HIGH_GEAR;

    public static double kTurnrateCurve = 0.1;
    public static double kTurnrateLimit = 0.8;

    public static final boolean On = true;
    public static final boolean Off = false;

}
