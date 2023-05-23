// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Vision;
import frc.robot.subsystems.RobotState.RobotMode;
import frc.robot.subsystems.RobotState;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ChangeDriveSlowCommand extends CommandBase {
  private final RobotState mRobotState;
  private final boolean mIncrease;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ChangeDriveSlowCommand(RobotState robotState, boolean increase) {
    this.mIncrease = increase;
    this.mRobotState = robotState;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(mIncrease) {
      mRobotState.setDriveSlowPercent(mRobotState.getDriveSlowPercent() + .10);
    }
    else {
      mRobotState.setDriveSlowPercent(mRobotState.getDriveSlowPercent() - .10);  
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
