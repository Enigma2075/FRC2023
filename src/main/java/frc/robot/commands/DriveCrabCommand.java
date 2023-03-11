// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.geometry.Rotation2d;

import frc.robot.subsystems.Drive;

/** An example command that uses an example subsystem. */
public class DriveCrabCommand extends CommandBase {
  private final Drive mDrive;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCrabCommand(Drive drive) {
    mDrive = drive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mDrive.orientModules(List.of(
      Rotation2d.fromDegrees(45),
      Rotation2d.fromDegrees(-45),
      Rotation2d.fromDegrees(-45),
      Rotation2d.fromDegrees(45)
));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
