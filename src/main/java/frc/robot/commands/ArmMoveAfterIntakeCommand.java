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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Intake.IntakeMode;
import frc.robot.subsystems.Intake.PivotPosition;

/** An example command that uses an example subsystem. */
public class ArmMoveAfterIntakeCommand extends ArmMoveCommand {
  private final Intake mIntake;
  private boolean initialized = false;
  private boolean mHasCustomEndPosition;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmMoveAfterIntakeCommand(Arm arm, Intake intake) {
    this(arm, intake, ArmPosition.DEFAULT);
    mHasCustomEndPosition = false;
  }

  public ArmMoveAfterIntakeCommand(Arm arm, Intake intake, ArmPosition endPosition) {
    super(arm, .9, CommandMode.WAIT, ArmPosition.HANDOFF_CONE2, ArmPosition.HANDOFF_CONE3, ArmPosition.HANDOFF_CONE4, endPosition);

    mHasCustomEndPosition = true;
    mIntake = intake;
    addRequirements(mIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialized = false;
    mIntake.setPivotPosition(PivotPosition.HANDOFF_CONE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!initialized && Math.abs(mIntake.getPivotAngle().getDegrees() - PivotPosition.HANDOFF_CONE.mAngle) < 2) {//mIntake.getIntakePosition().x > -23) {
      mIntake.setIntake(IntakeMode.CONE_HANDOFF);
      super.initialize();
      initialized = true;
    }
    else if(initialized) {
      if(mArm.getArmPosition().y < 14) {
        mIntake.setPivotPosition(PivotPosition.UP);    
      }
      super.execute();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(mMode == CommandMode.WAIT && !interrupted && mArm.handHasGamePeice()) {
      if(!mHasCustomEndPosition) {
        mArm.setPosition(ArmPosition.HOLD);
      }
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished() && initialized;
  }
}
