// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.other.LimelightHelpers;
import frc.lib.other.Subsystem;
import frc.robot.Constants;

public class Vision extends Subsystem {
  private final PeriodicIO mPeriodicIO = new PeriodicIO();

  public static class PeriodicIO {
    LimelightHelpers.Results resultsRight = null;
    LimelightHelpers.Results resultsLeft = null;
    Pose2d poseRight = new Pose2d();
    Pose2d poseLeft = new Pose2d();
    Pose2d bestPose = new Pose2d();
    LimelightHelpers.Results bestPoseResults = null;
    double bestPoseError = Double.MIN_VALUE;
  }

  public Vision() {
  }

  public Pose2d getBestPose(Pose2d currentPose) {
    var rightError = currentPose.getTranslation().getDistance(mPeriodicIO.poseRight.getTranslation());
    var leftError = currentPose.getTranslation().getDistance(mPeriodicIO.poseLeft.getTranslation());

    mPeriodicIO.bestPose = mPeriodicIO.poseLeft;
    mPeriodicIO.bestPoseError = leftError;
    mPeriodicIO.bestPoseResults = mPeriodicIO.resultsLeft;
    if(Math.abs(rightError) < Math.abs(leftError)) {
      mPeriodicIO.bestPose = mPeriodicIO.poseRight;
      mPeriodicIO.bestPoseError = rightError;
      mPeriodicIO.bestPoseResults = mPeriodicIO.resultsRight;
    }

    if(mPeriodicIO.bestPose.getX() == 0 && mPeriodicIO.bestPose.getY() == 0) {
      return null;
    }
    else if(mPeriodicIO.bestPoseError < .33) {
      return mPeriodicIO.bestPose;
    }
    
    return null;
  }

  public double getTimestamp() {
    return Timer.getFPGATimestamp() - (mPeriodicIO.bestPoseResults.latency_capture / 1000.0) - (mPeriodicIO.bestPoseResults.latency_pipeline / 1000.0);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    
  }

  /** 
   step 1: get tx?, ty?, and rx from limelight 
   tx - 8.265 = 0 in pathplanner
   ty - 4 = 0 in pathplanner
   rx - 180 should be ok????
   that should just set the stuff from limelight to be good in pathplanner
   tx, calculated x, ty, calculated y, rx, calculated rotation go in shuffleboard?
   should all be good then but idk
  **/
  
  @Override
  public synchronized void readPeriodicInputs() {
    if(Constants.Robot.kPracticeBot) {
      return;
    }

    mPeriodicIO.resultsLeft = getResults("limelight-r");
    mPeriodicIO.resultsRight = getResults("limelight-l");

    mPeriodicIO.poseLeft = getVisionPose2d(mPeriodicIO.resultsLeft);
    mPeriodicIO.poseRight = getVisionPose2d(mPeriodicIO.resultsRight);
  }

  public LimelightHelpers.Results getResults(String limelightName) {
    return LimelightHelpers.getLatestResults(limelightName).targetingResults;
  }

  public Pose2d getVisionPose2d(LimelightHelpers.Results results) {
    if(!(results.botpose[0] == 0 && results.botpose[1] == 0) && results.targets_Fiducials.length > 1) {
      if(DriverStation.getAlliance() == Alliance.Blue) {
        return LimelightHelpers.toPose2D(results.botpose_wpiblue);
      }
      else {
        return LimelightHelpers.toPose2D(results.botpose_wpired);
      }
    }
    else {
      return new Pose2d();
    }
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub
    
  }
  @Override
  public boolean checkSystem() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public void outputTelemetry() {
    SmartDashboard.putString("V Raw", String.format("X:%f, Y:%f, Rot:%f",
    mPeriodicIO.bestPose.getX(), mPeriodicIO.bestPose.getY(), mPeriodicIO.bestPose.getRotation().getDegrees()));
  }
}
