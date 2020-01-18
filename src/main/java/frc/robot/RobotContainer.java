/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;


//import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Input.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...


  private static final LogitechAttack3Joystick LeftStick = new LogitechAttack3Joystick(RobotMap.LeftJoystick_ID);
  private static final LogitechAttack3Joystick RightStick = new LogitechAttack3Joystick(RobotMap.RightJoystick_ID);
  //private static final LogitechController Gamepad = new LogitechController(RobotMap.Gampad_ID);

  //private Elevator elevator = Elevator.getInstance();

  public static double getLeftThrottleInput() {
    return LeftStick.getYAxis();
  }
  public static double getRightThrottleInput() {
    return RightStick.getYAxis(); 
  }
  public static double getLeftSteeringInput() {
    return LeftStick.getXAxis();
  }
  public static double getRightSteeringInput() {
    return RightStick.getXAxis();
  }
  public static double getLeftThrottleInputInverted() {
    return LeftStick.getYAxisInverted();
  }
  public static double getRightThrottleInputInverted() {
    return RightStick.getYAxisInverted();
  }
  public static double getLeftSteeringInputInverted() {
    return LeftStick.getXAxisInverted();
  }
  public static double getRightSteeringInputInverted() {
    return RightStick.getXAxisInverted();
  }

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /* public Command () {
    // An ExampleCommand will run in autonomous
    
  } */
}
