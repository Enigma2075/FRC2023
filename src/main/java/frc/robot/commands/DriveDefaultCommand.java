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
import frc.robot.subsystems.Drive;

/** An example command that uses an example subsystem. */
public class DriveDefaultCommand extends CommandBase {
  private final Drive mDrive;
  private final DoubleSupplier mThrottledSupplier;
  private final DoubleSupplier mStrafeSupplier;
  private final DoubleSupplier mRotationSupplier;
  private final BooleanSupplier mRequestCrabModeSupplier;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveDefaultCommand(Drive drive, DoubleSupplier throttleSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier, Trigger requestCrabModeTrigger) {
    mDrive = drive;
    mThrottledSupplier = throttleSupplier;
    mStrafeSupplier = strafeSupplier;
    mRotationSupplier = rotationSupplier;
    mRequestCrabModeSupplier = requestCrabModeTrigger;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean wantSmoothMode = false;

    //Adjust inputs
    double throttle = Util.handleDeadband(-mThrottledSupplier.getAsDouble(), Constants.DriverStation.kDriveJoystickThreshold);
    double strafe = Util.handleDeadband(-mStrafeSupplier.getAsDouble(), Constants.DriverStation.kDriveJoystickThreshold);
    double rot = Util.handleDeadband(-mRotationSupplier.getAsDouble(), Constants.DriverStation.kJoystickThreshold);
  

    throttle = Math.signum(throttle) * throttle * throttle;
    strafe = Math.signum(strafe) * strafe * strafe;
    rot = Math.signum(rot) * rot * rot;
    if (mRequestCrabModeSupplier.getAsBoolean() ) {//&&
            //(RobotState.getInstance().getMeasuredVelocity().norm() < 0.25)) {
        mDrive.orientModules(List.of(
                Rotation2d.fromDegrees(45),
                Rotation2d.fromDegrees(-45),
                Rotation2d.fromDegrees(-45),
                Rotation2d.fromDegrees(45)
        ));
    } else {
        mDrive.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                throttle * Constants.Drive.kMaxVelocityMetersPerSecond * Constants.Drive.kScaleTranslationInputs,
                strafe * Constants.Drive.kMaxVelocityMetersPerSecond * Constants.Drive.kScaleTranslationInputs,
                rot * Constants.Drive.kMaxAngularVelocityRadiansPerSecond * Constants.Drive.kScaleRotationInputs,
                mDrive.getFieldRelativeGyroscopeRotation()));
    }

    //TODO: Do we need smooth?
    // if (mControlBoard.getSmoothMode()) {
    //     wantSmoothMode = true;
    // }

    // if (wantSmoothMode) {
    //   mDrive.setKinematicLimits(Constants.kSmoothKinematicLimits);
    // } else {
    //     mDrive.setKinematicLimits(Constants.kUncappedKinematicLimits);
    // }
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
