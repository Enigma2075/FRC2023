// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.geometry.Rotation2d;
import frc.lib.swerve.ChassisSpeeds;
import frc.lib.util.Util;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;

/** An example command that uses an example subsystem. */
public class ArmManualCommand extends CommandBase {
  private final Arm mArm;
  private final DoubleSupplier mShoulderSupplier;
  private final DoubleSupplier mElbowSupplier;
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmManualCommand(Arm arm, DoubleSupplier shoulderSupplier, DoubleSupplier elbowSupplier) {
    mArm = arm;
    mShoulderSupplier = shoulderSupplier;
    mElbowSupplier = elbowSupplier;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double elbow = Util.handleDeadband(-mElbowSupplier.getAsDouble(), Constants.DriverStation.kJoystickThreshold);
    double shoulder = Util.handleDeadband(-mShoulderSupplier.getAsDouble(), Constants.DriverStation.kJoystickThreshold);
    
    mArm.setShoulderVelocity(shoulder);
    mArm.setElbowVelocity(elbow);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
