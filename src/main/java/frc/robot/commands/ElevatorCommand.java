/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorModes;
import frc.robot.subsystems.Elevator.ElevatorPositions;

public class ElevatorCommand extends CommandBase {
  /**
   * Creates a new ElevatorCommand.
   */
  private ElevatorPositions position;
  private ElevatorModes mode;
  private Elevator elevator = Elevator.getInstance();

  public ElevatorCommand(ElevatorPositions Position, ElevatorModes Mode) {
    // Use addRequirements() here to declare subsystem dependencies.
    position = Position;
    mode = Mode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.SetElevatorPosition(position, mode);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
