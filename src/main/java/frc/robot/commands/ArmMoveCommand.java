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
  private final Arm mArm;
  private final double mHandOutput;
  private final boolean mWait;
  private final ArmPosition[] mPositions;

  private int mCurrentPos = 0;
  private boolean mAtPos = false;
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmMoveCommand(Arm arm, double handOutput, boolean wait, ArmPosition... positions) {
    mArm = arm;
    mHandOutput = handOutput;
    mWait = wait;
    mPositions = positions;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  public ArmMoveCommand(Arm arm, double handOutput, ArmPosition... positions) {
    mArm = arm;
    mHandOutput = handOutput;
    mWait = false;
    mPositions = positions;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  public ArmMoveCommand(Arm arm, boolean wait, ArmPosition... positions) {
    mArm = arm;
    mHandOutput = Double.MIN_VALUE;
    mWait = wait;
    mPositions = positions;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  public ArmMoveCommand(Arm arm, ArmPosition... positions) {
    mArm = arm;
    mHandOutput = Double.MIN_VALUE;
    mWait = false;
    mPositions = positions;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mAtPos = false;
    mCurrentPos = 0;
  
    if(mHandOutput != Double.MIN_VALUE) {
      mArm.setHandOutput(mHandOutput);
    }
    mArm.setPosition(mPositions[mCurrentPos]);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!mAtPos || mCurrentPos < mPositions.length) {
      if(mArm.atPosition()) {
        mAtPos = true;
        mCurrentPos++;
        if(mCurrentPos < mPositions.length) {
          mArm.setPosition(mPositions[mCurrentPos]);
        }
      }
      else {
        if(mCurrentPos < mPositions.length) {
          mArm.setPosition(mPositions[mCurrentPos]);
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(mWait) {
      return mCurrentPos == mPositions.length;
    }
    else {
      return mCurrentPos == mPositions.length - 1;
    }
  }
}
