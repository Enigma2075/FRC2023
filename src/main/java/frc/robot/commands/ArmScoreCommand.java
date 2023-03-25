// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.geometry.Rotation2d;
import frc.lib.swerve.ChassisSpeeds;
import frc.lib.util.Util;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmMotion;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Arm.ScoreMode;

/** An example command that uses an example subsystem. */
public class ArmScoreCommand extends CommandBase {
  protected final Arm mArm;

  protected ScoreMode mMode;
  protected double mTimer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmScoreCommand(Arm arm) {
    mArm = arm;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mTimer = Double.MIN_VALUE;
    switch (mArm.getTargetPosition()) {
      case HIGH_CONE:
      case HIGH_CUBE:
        mMode = ScoreMode.HIGH;
        break;
      case MEDIUM_CONE:
      case MEDIUM_CUBE:
        mMode = ScoreMode.MIDDLE;
        break;
      case HOLD:
        mMode = ScoreMode.LOW;
        break;
      default:
    }

    if (mMode == null) {
      return;
    }

    if (mArm.isConeMode()) {
      switch (mMode) {
        case LOW:
          mArm.setPosition(ArmPosition.LOW);
          break;
        case MIDDLE:
          mArm.setPosition(ArmPosition.SCORE_OFFSET_CONE_MID);
          break;
        case HIGH:
          mArm.setPosition(ArmPosition.SCORE_OFFSET_CONE_HIGH);
          break;
      }
    } else {
      switch (mMode) {
        case LOW:
          mArm.setPosition(ArmPosition.LOW);
          break;
        default:
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mMode == null) {
      return;
    }

    if (mArm.atPosition() && mTimer == Double.MIN_VALUE) {
      mTimer = Timer.getFPGATimestamp();
    
      if (mArm.isConeMode()) {
        switch (mMode) {
          case LOW:
            mArm.setHandOutput(-1);
            break;
          case MIDDLE:
            mArm.setHandOutput(-.2);
            break;
          case HIGH:
            mArm.setHandOutput(-.2);
            break;
        }
      } else {
        switch (mMode) {
          case LOW:
            mArm.setHandOutput(-.7);
            break;
          case MIDDLE:
            mArm.setHandOutput(-.7);
            break;
          case HIGH:
            mArm.setHandOutput(-.5);
            break;
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (mMode == null) {
      return;
    }

    if (mMode == ScoreMode.LOW) {
      mArm.setPosition(ArmPosition.DEFAULT);
      return;
    }

    if (mArm.isConeMode()) {
      double shoulderCondition = mArm.getShoulderAngle().getDegrees() + 5;
      mArm.setPositions(new ArmMotion(ArmPosition.HOLD, (s, e) -> {
        return s > shoulderCondition;
      }), new ArmMotion(ArmPosition.DEFAULT));
    } 
    else {
      double shoulderCondition = mArm.getShoulderAngle().getDegrees() + 2;
      if (mArm.getShoulderAngle().getDegrees() < 0) {
        mArm.setPositions(new ArmMotion(ArmPosition.DEFAULT, (s, e) -> {
          return s > shoulderCondition;
        }));
      } else {
        mArm.setPositions(new ArmMotion
        (ArmPosition.DEFAULT, (s, e) -> {
          return s > -2;
        }));
      }
    }

    mArm.setHandOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (mMode == null) {
      return true;
    }

    return mArm.isSequenceComplete() && mTimer != Double.MIN_VALUE && Timer.getFPGATimestamp() - mTimer > .25;
  }
}
