/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Tray;
import frc.robot.subsystems.Elevator.ElevatorModes;
import frc.robot.subsystems.Elevator.ElevatorPositions;
import frc.robot.Constants;

public class CollectCargoCommand extends CommandBase {
  private Elevator elevator = Elevator.getInstance();
  private Tray tray = Tray.getInstance();

  public CollectCargoCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
    //isInterruptible();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tray.TalonsRelease();
    Constants.WantHatch = false;
    tray.UpdateLoadState();
    if (Constants.CARGO_STATE == Constants.CARGO_STATE_UNLOADED) {
        tray.ExtendTray();
        tray.IntakeCargo();
        elevator.SetElevatorPosition(ElevatorPositions.COLLECT, ElevatorModes.CARGO);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tray.UpdateLoadState();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Constants.CARGO_STATE == Constants.CARGO_STATE_LOADED) {
      tray.StopIntakeCargo();
      tray.RetractTray();
      elevator.SetElevatorPosition(ElevatorPositions.ROCKET_BOTTOM, ElevatorModes.CARGO);
      return true;
    }
    else {
      return false;
    }
    
  }
}
