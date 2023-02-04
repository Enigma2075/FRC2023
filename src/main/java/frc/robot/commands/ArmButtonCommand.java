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
public class ArmButtonCommand extends CommandBase {
  private final Arm mArm;
  private final double mShoulderAngle;
  private final double mElbowAngle;
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmButtonCommand(Arm arm, double shoulderAngle, double elbowAngle) {
    mArm = arm;
    mShoulderAngle = shoulderAngle;
    mElbowAngle = elbowAngle;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mArm.setElbowAngle(mElbowAngle);
    mArm.setShoulderAngle(mShoulderAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double output = mArm.calcShoulderArbFF();
    //mArm.setShoulderOutput(output);
  
    //double elbow = Util.handleDeadband(-mElbowSupplier.getAsDouble(), Constants.DriverStation.kJoystickThreshold);
    //double shoulder = Util.handleDeadband(-mShoulderSupplier.getAsDouble(), Constants.DriverStation.kJoystickThreshold);
    
    //mArm.setShoulderVelocity(shoulder);
    //mArm.setElbowVelocity(elbow);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mArm.setElbowOutput(0);
    mArm.setShoulderOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
