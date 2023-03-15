// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Array;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

public double rawX;
public double rawY;
public double rawRotate;
public double calcX;
public double calcY;
public double calcRot;
  /** Creates a new ExampleSubsystem. */
  public Vision() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpired").getDoubleArray(new double[6]);
    /*
    rawX = (double)Array.getDouble(botpose_wpired, 0);
    rawY = (double)Array.getDouble(botpose_wpired, 1);
    rawRotate = (double)Array.getDouble(botpose_wpired, 5);
    calcX = rawX - 8.265;
    calcY = rawY - 4;
    calcRot = rawRotate - 180; 
    */
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
   double[] robotPosition = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpired").getDoubleArray(new double[6]);

    //double rawX = (double)Array.getDouble(botpose_wpired, 0);
    //double rawY = (double)Array.getDouble(botpose_wpired, 1);
    //double rawRotate = (double)Array.getDouble(botpose_wpired, 5);
    //double calcX = rawX - 8.265;
    double calcX = robotPosition[0] - 8.265;
    //double calcY = rawY - 4;
    double calcY = robotPosition[1] - 4;
    //double calcRot = rawRotate - 180;
    double calcRot = robotPosition[5] - 180;

    //SmartDashboard.putNumber("Raw X", rawX);
    //SmartDashboard.putNumber("Raw Y", rawY);
    //SmartDashboard.putNumber("Raw Rotation", rawRotate);
    SmartDashboard.putNumber("Calculated X", calcX);
    SmartDashboard.putNumber("Calculated Y", calcY);
    SmartDashboard.putNumber("Calculated Rotation", calcRot);
    // This method will be called once per scheduler run
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
}
