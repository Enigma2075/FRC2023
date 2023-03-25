// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.other.Subsystem;

public class Vision extends Subsystem {
  private final NetworkTableEntry blueEntry; 
  private final NetworkTableEntry targetIdEntry;

  private final PeriodicIO mPeriodicIO = new PeriodicIO();

  public static class PeriodicIO {
    boolean hasValidTarget = false;
    Pose2d rawPose = new Pose2d();

    Pose2d calcPose = new Pose2d();
  }

  public Vision() {
    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    blueEntry = limelight.getEntry("botpose_wpiblue");
    targetIdEntry = limelight.getEntry("tid");
  }

  public Pose2d getRawPose() {
    return mPeriodicIO.rawPose;
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
    double tid = targetIdEntry.getDouble(Double.MIN_VALUE);

    SmartDashboard.putNumber("tid", tid);

    if(tid < 0) {
      mPeriodicIO.hasValidTarget = false;
      return;
    }
    
    mPeriodicIO.hasValidTarget = true;

    double[] robotPosition = blueEntry.getDoubleArray(new double[6]);

    double rawX = robotPosition[0]; //- 8.265;
    double rawY = robotPosition[1]; //- 4;
    double rawRot = robotPosition[5]; //- 180;

    //double rawX = (double)Array.getDouble(botpose_wpired, 0);
    //double rawY = (double)Array.getDouble(botpose_wpired, 1);
    //double rawRotate = (double)Array.getDouble(botpose_wpired, 5);
    //double calcX = rawX - 8.265;
    //double calcY = rawY - 4;
    //double calcRot = rawRotate - 180;

    mPeriodicIO.rawPose = new Pose2d(rawX, rawY, Rotation2d.fromDegrees(rawRot));
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
