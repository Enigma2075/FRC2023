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
import frc.robot.subsystems.RobotState;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Arm.ScoreMode;

/** An example command that uses an example subsystem. */
public class ArmMoveToScoreCommand extends CommandBase {
  protected final Arm mArm;
  protected final RobotState mRobotState;
  protected final boolean mWait;

  protected ScoreMode mMode;
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmMoveToScoreCommand(Arm arm, RobotState robotState, ScoreMode mode, boolean wait) {
    mArm = arm;
    mRobotState = robotState;
    mWait = wait;
    mMode = mode;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
      if(mRobotState.isConeMode()) {
        switch(mMode) {
          case LOW:
          break;
          case MIDDLE:
            mArm.setPosition(new ArmMotion(ArmPosition.MEDIUM_CONE, null, (s, e) -> {return e < -10;}));
          break;
          case HIGH:
            mArm.setPosition(new ArmMotion(ArmPosition.HIGH_CONE, null, (s, e) -> {return e < -5;}));
          break;
        }
      }
      else {
        switch(mMode) {
          case LOW:
          break;
          case MIDDLE:
            mArm.setPosition(new ArmMotion(ArmPosition.MEDIUM_CUBE, null, (s, e) -> {return e < -10;}));
          break;
          case HIGH:
            mArm.setPosition(new ArmMotion(ArmPosition.HIGH_CUBE, null, (s, e) -> {return e < -10;}));
          break;
        }
      }
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
    return mWait ? mArm.isSequenceComplete() : true;
  }
}
