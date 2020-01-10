/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Input;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * Add your docs here.
 */
public class LogitechController extends Joystick{

    private final Button[] mButtons = {
        new JoystickButton(this, 0),
        new JoystickButton(this, 1),
        new JoystickButton(this, 2),
        new JoystickButton(this, 3),
        new JoystickButton(this, 4),
        new JoystickButton(this, 5),
        new JoystickButton(this, 6),
        new JoystickButton(this, 7),
        new JoystickButton(this, 8),
        new JoystickButton(this, 9),
        new JoystickButton(this, 10),
        new JoystickButton(this, 11),
        new JoystickButton(this, 12),
    };

    public LogitechController(int port){
        super(port);
    }

    //Axes
    public double getLeftXAxis(){
        return getRawAxis(0);
    }

    public double getLeftYAxis(){
        return getRawAxis(1);
    }

    public double getRightXAxis(){
        return getRawAxis(4);
    }

    public double getRightYAxis(){
        return getRawAxis(5);
    }
    
    //buttons
    public Button getButtonA(){
        return mButtons[1];
    }

    public Button getButtonB(){
        return mButtons[2];
    }

    public Button getButtonX(){
        return mButtons[3];
    }
    public Button getButtonY(){
        return mButtons[4];
    }
    public Button getLeftBumper(){
        return mButtons[5];
    }

    public Button getRightBumper(){
        return mButtons[6];
    }
    
    //triggers
    public double getLeftTrigger(){
        return getRawAxis(2);
    }

    public double getRightTrigger(){
        return getRawAxis(3);
    }

    //extra buttons
    public Button getButtonBack(){
        return mButtons[7];
    }

    public Button getButtonStart(){
        return mButtons[8];
    }

    public Button getButtonLeftStick(){
        return mButtons[9];
    }

    public Button getButtonRightStick(){
        return mButtons[10];
    }
}
