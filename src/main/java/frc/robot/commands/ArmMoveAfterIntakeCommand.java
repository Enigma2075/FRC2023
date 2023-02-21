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
import frc.robot.subsystems.Intake.PivotPosition;

/** An example command that uses an example subsystem. */
public class ArmMoveAfterIntakeCommand extends ArmMoveCommand {
  private final Intake mIntake;
  private boolean initialized = false;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmMoveAfterIntakeCommand(Arm arm, Intake intake) {
    super(arm, .9, CommandMode.WAIT, ArmPosition.HANDOFF2_CONE2, ArmPosition.HANDOFF2_CONE3, ArmPosition.DEFAULT_ELBOW);
    
    mIntake = intake;
    addRequirements(mIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialized = false;
    mIntake.setPivotPosition(PivotPosition.HANDOFF_CONE);
    reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!initialized && mIntake.getIntakePosition().x > -26) {
      super.initialize();
      initialized = true;
    }
    else if(initialized) {
      if(mArm.getArmPosition().y < 17) {
        mIntake.setPivotPosition(PivotPosition.UP);    
      }
      super.execute();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished();
  }
}
