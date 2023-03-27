// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.other.LimelightHelpers;
import frc.lib.other.Subsystem;
import frc.lib.other.LimelightHelpers.LimelightResults;

public class Vision extends Subsystem {
  private final PeriodicIO mPeriodicIO = new PeriodicIO();

  public static class PeriodicIO {
    LimelightHelpers.Results results = null;
    boolean hasValidTarget = false;
    Pose2d rawPose = new Pose2d();
  }

  public Vision() {
  }

  public Pose2d getBestPose(Pose2d currentPose) {
    if(currentPose.getTranslation().getDistance(mPeriodicIO.rawPose.getTranslation()) < .33) {
      return mPeriodicIO.rawPose;
    }
    return null;
  }

  public double getTimestamp() {
    return Timer.getFPGATimestamp() - (mPeriodicIO.results.latency_capture / 1000.0) - (mPeriodicIO.results.latency_pipeline / 1000.0);
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
    mPeriodicIO.results = LimelightHelpers.getLatestResults("limelight-r").targetingResults;
    
    if(!(mPeriodicIO.results.botpose[0] == 0 && mPeriodicIO.results.botpose[1] == 0) && mPeriodicIO.results.targets_Fiducials.length > 1) {
      if(DriverStation.getAlliance() == Alliance.Blue) {
        mPeriodicIO.rawPose = LimelightHelpers.toPose2D(mPeriodicIO.results.botpose_wpiblue);    
      }
      else {
        mPeriodicIO.rawPose = LimelightHelpers.toPose2D(mPeriodicIO.results.botpose_wpired);
      }
    }
    else {
      mPeriodicIO.hasValidTarget = false;
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
    mPeriodicIO.rawPose.getX(), mPeriodicIO.rawPose.getY(), mPeriodicIO.rawPose.getRotation().getDegrees()));
    SmartDashboard.putBoolean("V Has-Target", mPeriodicIO.hasValidTarget);
  }
}
