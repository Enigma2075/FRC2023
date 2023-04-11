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
import frc.robot.subsystems.Arm.ArmPosition;

/** An example command that uses an example subsystem. */
public class ArmMoveCommand extends CommandBase {
  public enum CommandMode {NORMAL, WAIT, DONT_END}

  protected final Arm mArm;
  protected final double mHandOutput;
  protected final ArmPosition[] mPositions;
  protected final boolean mForce;

  //protected boolean mAtPos = false;
  protected CommandMode mMode = CommandMode.NORMAL;
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmMoveCommand(Arm arm, double handOutput, CommandMode mode, ArmPosition... positions) {
    mArm = arm;
    mHandOutput = handOutput;
    mMode = mode;
    mPositions = positions;
    mForce = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }
  
  public ArmMoveCommand(Arm arm, double handOutput, ArmPosition... positions) {
    mArm = arm;
    mHandOutput = handOutput;
    mMode = CommandMode.NORMAL;
    mPositions = positions;
    mForce = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  public ArmMoveCommand(Arm arm, CommandMode mode, ArmPosition... positions) {
    mArm = arm;
    mHandOutput = Double.MIN_VALUE;
    mMode = mode;
    mPositions = positions;
    mForce = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  public ArmMoveCommand(Arm arm, ArmPosition... positions) {
    mArm = arm;
    mHandOutput = Double.MIN_VALUE;
    mMode = CommandMode.NORMAL;
    mPositions = positions;
    mForce = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }
  
  public ArmMoveCommand(Arm arm, boolean force, ArmPosition... positions) {
    mArm = arm;
    mHandOutput = Double.MIN_VALUE;
    mMode = CommandMode.NORMAL;
    mPositions = positions;
    mForce = force;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
    if(mHandOutput != Double.MIN_VALUE) {
      mArm.setHandOutput(mHandOutput);
    }
    mArm.setPositions(mForce, mPositions);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(mMode == CommandMode.WAIT) {
      return mArm.isSequenceComplete();
    }
    else if(mMode == CommandMode.DONT_END) {
      return false;
    }
    else {
      return mArm.isSequenceComplete(false);
    }
  }
}
