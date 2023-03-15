// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final DoubleSupplier mRotationXSupplier;
  private final DoubleSupplier mRotationYSupplier;
  private final BooleanSupplier mRequestCrabModeSupplier;
  private final BooleanSupplier mRequestOrientTrigger;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveDefaultCommand(Drive drive, DoubleSupplier throttleSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationXSupplier, DoubleSupplier rotationYSupplier, Trigger requestCrabModeTrigger, Trigger requestOrientTrigger) {
    mDrive = drive;
    mThrottledSupplier = throttleSupplier;
    mStrafeSupplier = strafeSupplier;
    mRotationXSupplier = rotationXSupplier;
    mRotationYSupplier = rotationYSupplier;
    mRequestCrabModeSupplier = requestCrabModeTrigger;
    mRequestOrientTrigger = requestOrientTrigger;

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
    double rot = Util.handleDeadband(-mRotationXSupplier.getAsDouble(), Constants.DriverStation.kJoystickThreshold);
    double rotY = Util.handleDeadband(-mRotationYSupplier.getAsDouble(), Constants.DriverStation.kJoystickThreshold);
    
  

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
    } 
    else {
      if(mRequestOrientTrigger.getAsBoolean()) {
        if(rotY != 0 || rot != 0) {
          Rotation2d robot = mDrive.getFieldRelativeGyroscopeRotation();
          double cardinal = Double.MIN_VALUE;
          double dist = Double.MIN_VALUE;
          if((rot < rotY && Math.signum(rot) == -1 && Math.signum(rotY) == -1) || (Math.signum(rot) == -1 && (Math.signum(rotY) == 1 || Math.signum(rotY) == 0) && Math.abs(rot) > Math.abs(rotY))){
            // Right
            cardinal = -90;
          }
          else if(rotY < rot && Math.signum(rotY) == -1){
            // Reverse
            cardinal = 180;
          }
          else if(rot > rotY && Math.signum(rot) == 1){
            // Left
            cardinal = 90;
          }
          else if(rotY > rot && Math.signum(rotY) == 1){
            // Forward
            cardinal = 0;
          }

          dist = robot.distance(Rotation2d.fromDegrees(cardinal));

          //double error = cardinal - mDrive.getFieldRelativeGyroscopeRotation().getDegrees();
          
          //double tmpRot = 180.0 / error;
          SmartDashboard.putNumber("O-Change", dist);
          SmartDashboard.putNumber("O-RobotRot", mDrive.getFieldRelativeGyroscopeRotation().getDegrees());
          SmartDashboard.putNumber("O-Desire", cardinal);  
          SmartDashboard.putNumber("O-X", rot);  
          SmartDashboard.putNumber("O-Y", rotY);

          rot = 0;
        }
      }
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
    //   mDrive.setKinematicLimits(Constants.Drive.kSmoothKinematicLimits);
    // } else {
    //     mDrive.setKinematicLimits(Constants.Drive.kUncappedKinematicLimits);
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
