// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.geometry.Rotation2d;
import frc.lib.swerve.ChassisSpeeds;
import frc.lib.util.Util;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.DriveControlState;

/** An example command that uses an example subsystem. */
public class DriveBalanceCommand extends CommandBase {
  private final Drive mDrive;
  private final boolean mIsReverse;

  private final double mTarget1Fwd = -20;
  private final double mTarget1Rev = 13;
  private boolean mTarget1Hit = false;
  private final double mDropTarget1 = 14;
  private boolean mDropTarget1Hit = false;
  private final double mTarget2 = 23;
  private boolean mTarget2Hit = false;

  private double mTimer = Double.MIN_VALUE;
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveBalanceCommand(Drive drive, Boolean isReverse) {
    mDrive = drive;
    mIsReverse = isReverse;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mTarget1Hit = false;
    mTarget2Hit = false;
    mDropTarget1Hit = false;
    mTimer = Double.MIN_VALUE;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(mIsReverse) {
    //   if (Math.abs(mDrive.getPitch().getDegrees()) > mTarget1 && !mTarget1Hit) {
    //     mTarget1Hit = true;
    //   }
    //   if(mTarget1Hit && Math.abs(mDrive.getPitch().getDegrees()) < mDropTarget1) {
    //     mDropTarget1Hit = true;
    //   }
    //   else if(Math.abs(mDrive.getPitch().getDegrees()) > mTarget2 && mTarget1Hit && mDropTarget1Hit) {
    //     mTarget2Hit = true;
    //   }
    // }
    // else {
      if (((mDrive.getPitch().getDegrees() < mTarget1Fwd && !mIsReverse) || (mDrive.getPitch().getDegrees() > mTarget1Rev && mIsReverse)) && !mTarget1Hit) {
        mTarget1Hit = true;
        mTarget2Hit = true;
        mTimer = Timer.getFPGATimestamp();
      }
    //}

    double throttle = 0;
    double strafe = 0;
    double rot = 0;
    double requestedOrientation = 0;

    System.out.println(mDrive.getPitch().getDegrees());

    if (!mTarget1Hit || !mTarget2Hit) {
      if (mIsReverse) {
        throttle = -.3;
      } else {
        throttle = .3;
      }
    //} else if ((mDrive.getPitch().getDegrees() < -14 && !mIsReverse) || (mDrive.getPitch().getDegrees() > 18 && mIsReverse)) {
    } else if (Timer.getFPGATimestamp() - mTimer < 1.85) {
      if (mIsReverse) {
        throttle = -.2;
      } else {
        throttle = .2;
      }
    } else {
      throttle = 0;
    }

    System.out.println(throttle);

    Rotation2d robotOrientation = mDrive.getFieldRelativeGyroscopeRotation();
    double dist = Double.MIN_VALUE;

    dist = robotOrientation.distance(Rotation2d.fromDegrees(requestedOrientation));
    if (Math.abs(Math.toDegrees(dist)) > 1.5) {
      double percent = dist / Math.PI;

      double minSrc = -1;
      double maxSrc = 1;
      double minDest = -1;
      double maxDest = 1;
      percent = (((percent - minSrc) / (maxSrc - minSrc)) * (maxDest - minDest)) + minDest;
      double withF = Math.abs(percent) + .08;
      if (withF > 1) {
        withF = 1;
      }
      percent = withF * Math.signum(percent);

      // double error = cardinal -
      // mDrive.getFieldRelativeGyroscopeRotation().getDegrees();

      // double tmpRot = 180.0 / error;
      // SmartDashboard.putNumber("O-Change", percent);
      // SmartDashboard.putNumber("O-RobotRot", mDrive.getFieldRelativeGyroscopeRotation().getDegrees());
      // SmartDashboard.putNumber("O-Desire", requestedOrientation);

      rot = percent;
    }

    System.out.println("Run");
    mDrive.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
      throttle * Constants.Drive.kMaxVelocityMetersPerSecond * Constants.Drive.kScaleTranslationInputs,
      strafe * Constants.Drive.kMaxVelocityMetersPerSecond * Constants.Drive.kScaleTranslationInputs,
      rot * Constants.Drive.kMaxAngularVelocityRadiansPerSecond * Constants.Drive.kScaleRotationInputs,
      mDrive.getFieldRelativeGyroscopeRotation()));
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
